#include "syncsolver.h"

#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include <QDebug>

static constexpr double PI = 3.14159265358979323846;
static constexpr double CORR_STEP = 0.002;       // 2 ms cross-correlation step
static constexpr double CORR_RANGE = 0.5;         // ±0.5 s search range
static constexpr int MIN_WINDOWS = 3;             // minimum valid windows
static constexpr double PEAK_THRESHOLD = 0.3;     // minimum normalized peak
// Two crystal oscillators at +-50 ppm each disagree by at most ~1e-4 s/s;
// even sloppy ones stay inside ~2e-4. 5e-4 is already generous. The previous
// 5e-3 was 25x past anything physical, which let a 13 s clip come back with
// "0.45 %/s of clock drift" that was really a line fitted to 3 ms of window
// scatter over a 5 s lever arm.
static constexpr double MAX_DRIFT = 5e-4;         // s/s
// The cross-correlation only searches ±CORR_RANGE, so an absolute offset well
// outside that range can only have come from the line fit extrapolating past
// its own data. Reject rather than store.
static constexpr double MAX_ABS_OFFSET = 1.0;     // s
// A good per-window fit lands within a few frames; beyond this the windows
// disagree so much that the line through them is meaningless.
static constexpr double MAX_RESIDUAL_MS = 120.0;  // ms
static constexpr int TARGET_WINDOWS = 12;         // target number of windows
static constexpr int MIN_WINDOWS_COUNT = 8;
static constexpr int MAX_WINDOWS_COUNT = 16;
static constexpr double WINDOW_DURATION = 1.0;    // 1-second sliding window for variance
// Per-window lag search. The global stage has already aligned the streams to
// within a few frames, so a window only has to resolve the local residual.
// Searching ±0.5 s inside a 1 s window (the old range) meant that at the
// extremes half the window had slid off the end of its own data.
static constexpr double WINDOW_LAG_RANGE = 0.10;  // ±100 ms
// A window needs enough samples for a correlation to mean anything.
static constexpr int MIN_WINDOW_SAMPLES = 5;
// How many standard errors a fitted slope must clear before it is treated as a
// measurement rather than as noise in the per-window offsets.
static constexpr double kDriftSignificance = 3.0;

// Decide whether a fitted slope is a measurement. It has to be both
// physically possible AND resolvable against the scatter of the windows it was
// fitted through. When it is neither, zero it and re-estimate the intercept as
// the weighted mean of the offsets — the per-window refinement of the OFFSET is
// still worth keeping even when the clip is too short to see any drift.
template <typename Offsets>
static void acceptSlope(const Offsets &offsets, double &o0, double &drift, double se)
{
    const bool physical   = std::abs(drift) <= MAX_DRIFT;
    const bool resolvable = std::abs(drift) > kDriftSignificance * se;
    if (physical && resolvable)
        return;

    qDebug("SyncSolver: fitted drift %.4g s/s rejected (%s, %s; stderr %.3g) "
           "- using a constant offset", drift,
           physical ? "physical" : "IMPLAUSIBLE",
           resolvable ? "resolvable" : "within noise", se);

    double W = 0.0, Wo = 0.0;
    for (const auto &lo : offsets) { W += lo.weight; Wo += lo.weight * lo.offsetLocal; }
    if (W > 0.0)
        o0 = Wo / W;
    drift = 0.0;
}

// ---------------------------------------------------------------------------
// GyroIntegral
// ---------------------------------------------------------------------------
void GyroIntegral::build(const QVector<ImuSample> &samples, double step)
{
    m_prefix.clear();
    if (samples.size() < 2 || step <= 0.0)
        return;

    m_t0 = samples.first().timestamp;
    m_step = step;
    const double span = samples.last().timestamp - m_t0;
    const int n = static_cast<int>(span / m_step);
    if (n < 2)
        return;

    m_prefix.resize(n + 1);
    m_prefix[0] = QVector3D(0.0f, 0.0f, 0.0f);

    // Walk the raw samples once alongside the grid (no per-point binary search).
    int si = 0;
    QVector3D acc(0.0f, 0.0f, 0.0f);
    QVector3D prevG = samples.first().gyro;
    for (int i = 1; i <= n; i++) {
        const double t = m_t0 + i * m_step;
        while (si + 1 < samples.size() - 1 && samples[si + 1].timestamp < t)
            si++;
        // Linear interpolation between samples[si] and samples[si+1].
        QVector3D g;
        const double ta = samples[si].timestamp;
        const double tb = samples[si + 1].timestamp;
        if (tb - ta < 1e-12) {
            g = samples[si].gyro;
        } else {
            const float f = static_cast<float>(qBound(0.0, (t - ta) / (tb - ta), 1.0));
            g = samples[si].gyro + (samples[si + 1].gyro - samples[si].gyro) * f;
        }
        // Trapezoid: the integral is in degrees, so meanOver is deg/s.
        acc += (prevG + g) * static_cast<float>(m_step * 0.5);
        prevG = g;
        m_prefix[i] = acc;
    }
}

QVector3D GyroIntegral::integralAt(double t) const
{
    const double x = (t - m_t0) / m_step;
    const int i = static_cast<int>(std::floor(x));
    if (i < 0) return m_prefix.first();
    if (i >= m_prefix.size() - 1) return m_prefix.last();
    const float f = static_cast<float>(x - i);
    return m_prefix[i] + (m_prefix[i + 1] - m_prefix[i]) * f;
}

bool GyroIntegral::meanOver(double a, double b, QVector3D &out) const
{
    if (!isValid())
        return false;
    if (b < a) std::swap(a, b);
    if (a < tBegin() || b > tEnd())
        return false;

    const double dt = b - a;
    if (dt < 1e-9) {
        // Degenerate interval: fall back to the local slope.
        const double h = m_step;
        if (a - h * 0.5 < tBegin() || a + h * 0.5 > tEnd())
            return false;
        out = (integralAt(a + h * 0.5) - integralAt(a - h * 0.5)) / static_cast<float>(h);
        return true;
    }
    out = (integralAt(b) - integralAt(a)) / static_cast<float>(dt);
    return true;
}

// ---------------------------------------------------------------------------
// Quaternion → axis-angle (degrees)
// ---------------------------------------------------------------------------
QVector3D SyncSolver::quaternionToAxisAngle(const QQuaternion &q)
{
    QQuaternion qn = q.normalized();
    // Ensure w >= 0 for shortest path
    if (qn.scalar() < 0.0)
        qn = QQuaternion(-qn.scalar(), -qn.x(), -qn.y(), -qn.z());

    double w = qn.scalar();
    // Clamp to avoid acos domain errors
    w = qBound(-1.0, w, 1.0);

    double angleRad = 2.0 * std::acos(w);
    double sinHalf = std::sin(angleRad * 0.5);

    if (sinHalf < 1e-10) {
        // Near-zero rotation
        return QVector3D(0.0f, 0.0f, 0.0f);
    }

    double angleDeg = angleRad * 180.0 / PI;
    double ax = qn.x() / sinHalf;
    double ay = qn.y() / sinHalf;
    double az = qn.z() / sinHalf;

    return QVector3D(static_cast<float>(ax * angleDeg),
                     static_cast<float>(ay * angleDeg),
                     static_cast<float>(az * angleDeg));
}

// ---------------------------------------------------------------------------
// Step 2: Compute visual rotation rates
// ---------------------------------------------------------------------------
QVector<SyncSolver::VisualRateSample> SyncSolver::computeVisualRates(
    const QVector<VisualRotationPair> &pairs)
{
    QVector<VisualRateSample> rates;
    rates.reserve(pairs.size());

    for (const auto &pair : pairs) {
        double dt = pair.t1 - pair.t0;
        if (dt < 1e-6)
            continue;

        QVector3D axisAngle = quaternionToAxisAngle(pair.deltaR);
        QVector3D omega(axisAngle.x() / static_cast<float>(dt),
                        axisAngle.y() / static_cast<float>(dt),
                        axisAngle.z() / static_cast<float>(dt));

        rates.append({pair.t0, pair.t1, (pair.t0 + pair.t1) * 0.5,
                      omega, (double)omega.length()});
    }

    std::sort(rates.begin(), rates.end(),
              [](const VisualRateSample &a, const VisualRateSample &b) {
                  return a.tMid < b.tMid;
              });
    return rates;
}

// ---------------------------------------------------------------------------
// The one correlation primitive (see the header for why magnitudes, why
// Pearson, and why the normalisation is recomputed per lag).
// ---------------------------------------------------------------------------
double SyncSolver::correlate(const QVector<VisualRateSample> &vis, int lo, int hi,
                             const GyroIntegral &gyro, double drift, double lag)
{
    double sv = 0.0, sg = 0.0, svv = 0.0, sgg = 0.0, svg = 0.0;
    int n = 0;
    for (int i = lo; i < hi; i++) {
        const double a = vis[i].t0 * (1.0 + drift) + lag;
        const double b = vis[i].t1 * (1.0 + drift) + lag;
        QVector3D g;
        if (!gyro.meanOver(a, b, g))
            continue;                       // outside the IMU stream
        const double gm = g.length();
        const double vm = vis[i].mag;
        sv += vm; sg += gm;
        svv += vm * vm; sgg += gm * gm; svg += vm * gm;
        n++;
    }
    if (n < 3)
        return -2.0;

    const double dn = static_cast<double>(n);
    const double covar = svg - sv * sg / dn;
    const double varV  = svv - sv * sv / dn;
    const double varG  = sgg - sg * sg / dn;
    if (varV <= 1e-12 || varG <= 1e-12)
        return -2.0;
    return covar / std::sqrt(varV * varG);
}

// ---------------------------------------------------------------------------
// Scan a lag range and parabolically refine the peak from the cached scores.
// ---------------------------------------------------------------------------
double SyncSolver::bestLag(const QVector<VisualRateSample> &vis, int lo, int hi,
                           const GyroIntegral &gyro, double drift,
                           double centre, double range, double step,
                           double *scoreOut)
{
    const int steps = qMax(2, static_cast<int>(std::round(2.0 * range / step)));
    QVector<double> score(steps + 1);
    int bestIdx = 0;
    double best = -1e18;
    for (int s = 0; s <= steps; ++s) {
        const double lag = centre - range + s * step;
        score[s] = correlate(vis, lo, hi, gyro, drift, lag);
        if (score[s] > best) { best = score[s]; bestIdx = s; }
    }
    if (scoreOut) *scoreOut = best;
    if (best <= -1.5)
        return centre;

    double lag = centre - range + bestIdx * step;
    // Parabolic refinement, reusing the scores already computed rather than
    // re-evaluating three full correlations.
    if (bestIdx > 0 && bestIdx < steps) {
        const double ym1 = score[bestIdx - 1], y0 = score[bestIdx], yp1 = score[bestIdx + 1];
        const double denom = ym1 - 2.0 * y0 + yp1;
        if (std::abs(denom) > 1e-12)
            lag += qBound(-1.0, 0.5 * (ym1 - yp1) / denom, 1.0) * step;
    }
    return lag;
}

// ---------------------------------------------------------------------------
// Per-window local offsets for the joint drift/offset line fit.
// ---------------------------------------------------------------------------
QVector<SyncSolver::LocalOffset> SyncSolver::crossCorrelateWindows(
    const QVector<VisualRateSample> &visualRates,
    const GyroIntegral &gyro, double drift, double baseLag)
{
    QVector<LocalOffset> results;
    const int n = visualRates.size();
    if (n < MIN_WINDOW_SAMPLES)
        return results;

    const double tMin = visualRates.first().tMid;
    const double tMax = visualRates.last().tMid;
    if (tMax - tMin < 2.0)
        return results;

    // Sliding windows, ranked by how much the rotation RATE varies inside them.
    // Motion-rich windows give sharp correlation peaks. Using the visual
    // magnitude (rather than the gyro) keeps window selection independent of
    // the very alignment being solved for.
    struct WindowInfo { double tCenter; double variance; int startIdx; int endIdx; };
    QVector<WindowInfo> windows;
    const double step = WINDOW_DURATION * 0.5;
    for (double tStart = tMin; tStart + WINDOW_DURATION <= tMax + 1e-9; tStart += step) {
        const double tEnd = tStart + WINDOW_DURATION;
        int startIdx = -1, endIdx = -1;
        for (int i = 0; i < n; ++i) {
            const double t = visualRates[i].tMid;
            if (t >= tStart - 1e-9 && t <= tEnd + 1e-9) {
                if (startIdx < 0) startIdx = i;
                endIdx = i + 1;
            }
        }
        if (startIdx < 0 || (endIdx - startIdx) < MIN_WINDOW_SAMPLES)
            continue;
        const int count = endIdx - startIdx;
        double mean = 0.0;
        for (int i = startIdx; i < endIdx; ++i) mean += visualRates[i].mag;
        mean /= count;
        double variance = 0.0;
        for (int i = startIdx; i < endIdx; ++i) {
            const double d = visualRates[i].mag - mean;
            variance += d * d;
        }
        variance /= count;
        windows.append({(tStart + tEnd) * 0.5, variance, startIdx, endIdx});
    }

    if (windows.isEmpty())
        return results;

    std::sort(windows.begin(), windows.end(),
        [](const WindowInfo &a, const WindowInfo &b) { return a.variance > b.variance; });
    // qMin, not qBound: qBound(8, size, 16) RETURNS 8 when fewer than 8 windows
    // exist, and the resize below then APPENDED zero-initialised windows.
    const int numWindows = qMin(qMin(windows.size(), MAX_WINDOWS_COUNT), TARGET_WINDOWS);
    windows.resize(numWindows);
    std::sort(windows.begin(), windows.end(),
        [](const WindowInfo &a, const WindowInfo &b) { return a.tCenter < b.tCenter; });
    Q_UNUSED(MIN_WINDOWS_COUNT);

    for (const auto &win : windows) {
        double peak = -2.0;
        const double lag = bestLag(visualRates, win.startIdx, win.endIdx, gyro,
                                   drift, baseLag, WINDOW_LAG_RANGE, CORR_STEP,
                                   &peak);
        if (peak < PEAK_THRESHOLD)
            continue;
        // Report the offset RELATIVE to baseLag, which is what the line fit
        // models as tau(t) = o0 + drift * t.
        results.append({win.tCenter, lag - baseLag, win.variance});
    }

    return results;
}

// ---------------------------------------------------------------------------
// Step 5: Joint line fit (weighted least squares)
// ---------------------------------------------------------------------------
bool SyncSolver::fitLine(const QVector<LocalOffset> &offsets,
                         double &o0, double &drift, double &residualMs,
                         double &driftStdErr)
{
    if (offsets.size() < MIN_WINDOWS)
        return false;

    double W = 0.0, Wt = 0.0, Wt2 = 0.0, Wo = 0.0, Wto = 0.0;

    for (const auto &lo : offsets) {
        double w = lo.weight;
        double t = lo.tWindow;
        double o = lo.offsetLocal;

        W   += w;
        Wt  += w * t;
        Wt2 += w * t * t;
        Wo  += w * o;
        Wto += w * t * o;
    }

    double det = W * Wt2 - Wt * Wt;
    if (std::abs(det) < 1e-15)
        return false;

    o0    = (Wt2 * Wo - Wt * Wto) / det;
    drift = (W * Wto - Wt * Wo) / det;

    if (!std::isfinite(o0) || !std::isfinite(drift))
        return false;

    // Compute RMS residual
    double sumSqResid = 0.0;
    for (const auto &lo : offsets) {
        double predicted = o0 + drift * lo.tWindow;
        double resid = lo.offsetLocal - predicted;
        sumSqResid += lo.weight * resid * resid;
    }
    residualMs = std::sqrt(sumSqResid / W) * 1000.0; // convert to ms

    if (!std::isfinite(residualMs) || residualMs > MAX_RESIDUAL_MS)
        return false;

    // Standard error of the slope: sigma / sqrt(Stt). A short clip has almost
    // no lever arm, so a few ms of window scatter produces a large apparent
    // slope -- which is how a 13 s clip came back with 0.4%/s of "clock drift",
    // 3 orders of magnitude above anything two crystals can do.
    // Var(slope) = sigma^2 / Stt with sigma^2 = sum(w r^2) / (n - 2) -- the
    // residual variance estimate uses the DEGREES OF FREEDOM, not the weight
    // total; dividing by W leaves the weight units uncancelled and makes the
    // error look far smaller than it is.
    const double Stt = Wt2 - Wt * Wt / W;
    const int dof = offsets.size() - 2;
    const double sigma2 = (dof > 0) ? sumSqResid / dof
                                    : std::numeric_limits<double>::infinity();
    driftStdErr = (Stt > 1e-12 && std::isfinite(sigma2))
                  ? std::sqrt(sigma2 / Stt)
                  : std::numeric_limits<double>::infinity();

    return true;
}

// ---------------------------------------------------------------------------
// Main solve method
// ---------------------------------------------------------------------------
SyncSolver::SyncSolver(QObject *parent)
    : QObject(parent)
{
}

void SyncSolver::solve(const QVector<VisualRotationPair> &visualPairs,
                       const QVector<ImuSample> &imuSamples,
                       double initialDrift,
                       double initialOffset)
{
    // Validate inputs
    if (visualPairs.size() < 5) {
        emit solveFailed(QStringLiteral("Need at least 5 visual rotation pairs, got %1").arg(visualPairs.size()));
        return;
    }
    if (imuSamples.isEmpty()) {
        emit solveFailed(QStringLiteral("No IMU samples provided"));
        return;
    }

    // Step 2: Compute visual rates
    emit progressChanged(0.0, QStringLiteral("Computing visual rates..."));
    QVector<VisualRateSample> visualRates = computeVisualRates(visualPairs);
    if (visualRates.size() < 5) {
        emit solveFailed(QStringLiteral("Insufficient valid visual rotation pairs after filtering"));
        return;
    }

    GyroIntegral gyro;
    gyro.build(imuSamples, CORR_STEP);
    if (!gyro.isValid()) {
        emit solveFailed(QStringLiteral("IMU stream too short to correlate"));
        return;
    }

    // Step 3: Coarse global lag. The search is centred so that it covers BOTH
    // zero and the caller's current estimate — `initialOffset` used to be
    // accepted and never read, which meant a re-run could not refine a previous
    // result and a true offset outside ±CORR_RANGE was unreachable.
    emit progressChanged(0.2, QStringLiteral("Cross-correlating gyro and visual rate..."));
    const double lo = qMin(0.0, initialOffset) - CORR_RANGE;
    const double hi = qMax(0.0, initialOffset) + CORR_RANGE;
    const double centre = (lo + hi) * 0.5;
    const double range  = (hi - lo) * 0.5;

    const int nAll = visualRates.size();
    double coarseScore = -2.0;
    double globalLag = bestLag(visualRates, 0, nAll, gyro, initialDrift,
                               centre, range, CORR_STEP, &coarseScore);
    // Fine pass around the coarse peak.
    globalLag = bestLag(visualRates, 0, nAll, gyro, initialDrift,
                        globalLag, 0.05, CORR_STEP * 0.25, &coarseScore);

    // Step 4: Per-window local offsets for the joint offset+drift line fit.
    emit progressChanged(0.4, QStringLiteral("Refining per-window sync..."));
    QVector<LocalOffset> offsets = crossCorrelateWindows(visualRates, gyro,
                                                         initialDrift, globalLag);

    // Absolute model so far: tImu = t*(1 + driftAbs) + offsetAbs.
    double driftAbs = initialDrift;
    double offsetAbs = globalLag;
    double residualMs = 0.0;
    int windowsUsed = offsets.size();

    if (offsets.size() >= MIN_WINDOWS) {
        double o0, dResid, resid, dSe;
        if (fitLine(offsets, o0, dResid, resid, dSe)) {
            acceptSlope(offsets, o0, dResid, dSe);
            // The windows were measured against a mapping that already used
            // driftAbs, so the fitted slope is a RESIDUAL and must be ADDED.
            // Overwriting driftAbs with it (the old code) silently discarded
            // the caller's estimate on one path while returning it verbatim on
            // the fallback paths — the same field meaning two different things.
            offsetAbs += o0;
            driftAbs  += dResid;
            residualMs = resid;

            // Step 5: second pass around the refined model.
            emit progressChanged(0.6, QStringLiteral("Second pass refinement..."));
            QVector<LocalOffset> offsets2 = crossCorrelateWindows(visualRates, gyro,
                                                                  driftAbs, offsetAbs);
            if (offsets2.size() >= MIN_WINDOWS) {
                double o0b, dResidB, residB, dSeB;
                if (fitLine(offsets2, o0b, dResidB, residB, dSeB)) {
                    acceptSlope(offsets2, o0b, dResidB, dSeB);
                    offsetAbs += o0b;
                    driftAbs  += dResidB;
                    residualMs = residB;
                    windowsUsed = offsets2.size();
                }
            }
        }
    }

    SyncResult result;
    result.syncOffset = offsetAbs;
    result.drift = driftAbs;
    result.residualMs = residualMs;
    result.windowsUsed = windowsUsed;

    // Final plausibility gate. The searched lag range is bounded, so an
    // absolute offset far outside it is the line fit extrapolating, not a
    // measurement — and a stored bad offset poisons every later stage
    // (calibration pairs the wrong gyro samples with each visual rotation) as
    // well as playback.
    if (!std::isfinite(result.syncOffset) || !std::isfinite(result.drift)) {
        emit solveFailed(QStringLiteral("Sync solve produced a non-finite result"));
        return;
    }
    if (std::abs(result.syncOffset) > MAX_ABS_OFFSET) {
        emit solveFailed(
            QStringLiteral("Solved sync offset %1 s is outside the plausible ±%2 s "
                           "range (search range is only ±%3 s) — rejected.")
                .arg(result.syncOffset, 0, 'f', 3)
                .arg(MAX_ABS_OFFSET).arg(CORR_RANGE));
        return;
    }
    if (std::abs(result.drift) > MAX_DRIFT) {
        emit solveFailed(
            QStringLiteral("Solved clock drift %1 s/s exceeds the plausible ±%2 s/s "
                           "for two crystal clocks — rejected.")
                .arg(result.drift, 0, 'g', 3).arg(MAX_DRIFT));
        return;
    }

    emit progressChanged(1.0, QStringLiteral("Done"));
    emit syncSolved(result);
}
