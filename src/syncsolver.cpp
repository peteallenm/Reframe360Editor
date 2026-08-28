#include "syncsolver.h"

#include <cmath>
#include <algorithm>
#include <numeric>

static constexpr double PI = 3.14159265358979323846;
static constexpr double CORR_STEP = 0.002;       // 2 ms cross-correlation step
static constexpr double CORR_RANGE = 0.5;         // ±0.5 s search range
static constexpr int MIN_WINDOWS = 3;             // minimum valid windows
static constexpr double PEAK_THRESHOLD = 0.3;     // minimum normalized peak
// The IMU and video clocks are both crystal-derived, so their relative rate
// error is parts-per-thousand at worst (measured on these clips: ~6e-4).
// 0.05 s/s — the old bound — is three orders of magnitude past anything
// physical, so a fit that reached it was never clamped back to plausibility,
// it was rubber-stamped. Drift outside this band now falls back to the
// caller's initial estimate (which App derives from the stream durations)
// rather than being stored as a measurement.
static constexpr double MAX_DRIFT = 0.005;        // ±0.5% s/s
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
// Interpolate gyro at arbitrary time from IMU samples (binary search + lerp)
// ---------------------------------------------------------------------------
QVector3D SyncSolver::interpolateGyro(const QVector<ImuSample> &samples, double t)
{
    if (samples.isEmpty())
        return QVector3D(0.0f, 0.0f, 0.0f);

    if (t <= samples.first().timestamp)
        return samples.first().gyro;
    if (t >= samples.last().timestamp)
        return samples.last().gyro;

    // Binary search for bracketing index
    auto it = std::lower_bound(samples.constBegin(), samples.constEnd(), t,
        [](const ImuSample &s, double val) { return s.timestamp < val; });

    int hi = static_cast<int>(std::distance(samples.constBegin(), it));
    if (hi <= 0) hi = 1;
    if (hi >= samples.size()) hi = samples.size() - 1;
    int lo = hi - 1;

    double dt = samples[hi].timestamp - samples[lo].timestamp;
    if (dt < 1e-12)
        return samples[lo].gyro;

    double frac = (t - samples[lo].timestamp) / dt;
    const QVector3D &g0 = samples[lo].gyro;
    const QVector3D &g1 = samples[hi].gyro;

    return QVector3D(static_cast<float>(g0.x() + (g1.x() - g0.x()) * frac),
                     static_cast<float>(g0.y() + (g1.y() - g0.y()) * frac),
                     static_cast<float>(g0.z() + (g1.z() - g0.z()) * frac));
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

        double tMid = (pair.t0 + pair.t1) * 0.5;
        QVector3D axisAngle = quaternionToAxisAngle(pair.deltaR);
        QVector3D omega(axisAngle.x() / static_cast<float>(dt),
                        axisAngle.y() / static_cast<float>(dt),
                        axisAngle.z() / static_cast<float>(dt));

        rates.append({tMid, omega});
    }

    return rates;
}

// ---------------------------------------------------------------------------
// Step 3: Sample gyro rates at visual times
// ---------------------------------------------------------------------------
QVector<SyncSolver::GyroRateSample> SyncSolver::sampleGyroRates(
    const QVector<VisualRateSample> &visualRates,
    const QVector<ImuSample> &imuSamples,
    double drift, double offset)
{
    QVector<GyroRateSample> gyroRates;
    gyroRates.reserve(visualRates.size());

    for (const auto &vr : visualRates) {
        // Map video time to IMU time
        double tImu = vr.tMid * (1.0 + drift) + offset;
        QVector3D gyro = interpolateGyro(imuSamples, tImu);
        gyroRates.append({vr.tMid, gyro});
    }

    return gyroRates;
}

// ---------------------------------------------------------------------------
// Step 4a: Global cross-correlation of full visual-rate vs gyro-rate signals.
// The gyro is resampled from the raw IMU at (tMid + lag) for each candidate
// lag, which is a true temporal shift of the IMU stream. Returns the lag that
// maximizes the summed 3-axis correlation.
// ---------------------------------------------------------------------------
double SyncSolver::globalCrossCorrelate(const QVector<VisualRateSample> &visualRates,
                                        const QVector<ImuSample> &imuSamples,
                                        double drift, double *bestLagOut) const
{
    if (visualRates.size() < 3 || imuSamples.size() < 2)
        return 0.0;

    const double lagMin = -CORR_RANGE;
    const double lagMax =  CORR_RANGE;
    int steps = static_cast<int>(std::round((lagMax - lagMin) / CORR_STEP));

    double bestScore = -1e18;
    double bestLag = 0.0;

    // Correlate rotation-rate MAGNITUDES, as a properly normalised (Pearson)
    // correlation. Both details matter and both were wrong:
    //
    //  - FRAME: omegaVisual is expressed in the camera/bearing frame while
    //    interpolateGyro returns RAW SENSOR axes. A component-wise dot product
    //    between two different frames does not peak at the true lag at all.
    //    Magnitude is frame-independent, so it sidesteps the question entirely.
    //
    //  - NORMALISATION: the old score was a bare inner product with only the
    //    visual side zero-meaned. That is maximised wherever the gyro happens
    //    to be biggest, not where the two signals line up, so the "peak" drifted
    //    toward the most energetic part of the clip.
    //
    // Measured against a ground-truth sweep, the old code returned 0.395 s on
    // YIVR_0830 (truth 0.165) and 0.272 s on YIVR_0845 (truth 0.150) — a
    // quarter-second error, which at 91-169 deg/s is far more shake than it
    // removes.
    const int nv = visualRates.size();
    QVector<double> vmag(nv);
    double vMean = 0.0;
    for (int i = 0; i < nv; i++) { vmag[i] = visualRates[i].omegaVisual.length(); vMean += vmag[i]; }
    vMean /= nv;
    double vVar = 0.0;
    for (int i = 0; i < nv; i++) vVar += (vmag[i]-vMean) * (vmag[i]-vMean);
    if (vVar <= 0.0) return 0.0;

    QVector<double> gmag(nv);
    for (int s = 0; s <= steps; ++s) {
        double lag = lagMin + s * CORR_STEP;
        double gMean = 0.0;
        for (int i = 0; i < nv; i++) {
            const double tImu = visualRates[i].tMid * (1.0 + drift) + lag;
            gmag[i] = interpolateGyro(imuSamples, tImu).length();
            gMean += gmag[i];
        }
        gMean /= nv;
        double num = 0.0, gVar = 0.0;
        for (int i = 0; i < nv; i++) {
            const double dv = vmag[i] - vMean, dg = gmag[i] - gMean;
            num += dv * dg;
            gVar += dg * dg;
        }
        const double score = (gVar > 0.0) ? num / std::sqrt(vVar * gVar) : -1e18;
        if (score > bestScore) {
            bestScore = score;
            bestLag = lag;
        }
    }

    // Refine the peak with parabolic interpolation for sub-ms precision.
    int bestIdx = static_cast<int>(std::round((bestLag - lagMin) / CORR_STEP));
    if (bestIdx > 0 && bestIdx < steps) {
        double c0 = -1e18, cm = -1e18, cp = -1e18;
        for (int j = bestIdx - 1; j <= bestIdx + 1; ++j) {
            double lag = lagMin + j * CORR_STEP;
            double gMean = 0.0;
            for (int i = 0; i < nv; i++) {
                const double tImu = visualRates[i].tMid * (1.0 + drift) + lag;
                gmag[i] = interpolateGyro(imuSamples, tImu).length();
                gMean += gmag[i];
            }
            gMean /= nv;
            double num = 0.0, gVar = 0.0;
            for (int i = 0; i < nv; i++) {
                const double dv = vmag[i] - vMean, dg = gmag[i] - gMean;
                num += dv * dg; gVar += dg * dg;
            }
            const double score = (gVar > 0.0) ? num / std::sqrt(vVar * gVar) : -1e18;
            if (j == bestIdx) c0 = score;
            else if (j < bestIdx) cm = score;
            else cp = score;
        }
        double denom = cm - 2.0 * c0 + cp;
        if (std::abs(denom) > 1e-12) {
            double delta = 0.5 * (cm - cp) / denom;
            bestLag += qBound(-1.0, delta, 1.0) * CORR_STEP;
        }
    }

    if (bestLagOut)
        *bestLagOut = bestLag;
    return bestScore;
}

// ---------------------------------------------------------------------------
// Step 4b: Per-window local offsets (for the joint drift/offset line fit).
// The gyro is resampled from the raw IMU at the window's visual times plus a
// small local refinement around the global offset, giving per-window lags.
// ---------------------------------------------------------------------------
QVector<SyncSolver::LocalOffset> SyncSolver::crossCorrelateWindows(
    const QVector<VisualRateSample> &visualRates,
    const QVector<GyroRateSample> &gyroRates)
{
    QVector<LocalOffset> results;
    if (visualRates.isEmpty() || visualRates.size() != gyroRates.size())
        return results;

    int n = visualRates.size();
    double tMin = visualRates.first().tMid;
    double tMax = visualRates.last().tMid;
    double totalDuration = tMax - tMin;
    if (totalDuration < 2.0)
        return results;

    // Sliding variance windows over the gyro-rate signal to find motion-rich
    // segments (sharp motion gives sharp correlation peaks).
    struct WindowInfo {
        double tCenter;
        double variance;
        int startIdx;
        int endIdx;
    };
    QVector<WindowInfo> windows;
    double step = WINDOW_DURATION * 0.5;
    for (double tStart = tMin; tStart + WINDOW_DURATION <= tMax + 1e-9; tStart += step) {
        double tEnd = tStart + WINDOW_DURATION;
        int startIdx = -1, endIdx = -1;
        for (int i = 0; i < n; ++i) {
            double t = visualRates[i].tMid;
            if (t >= tStart - 1e-9 && t <= tEnd + 1e-9) {
                if (startIdx < 0) startIdx = i;
                endIdx = i + 1;
            }
        }
        if (startIdx < 0 || (endIdx - startIdx) < 5)
            continue;
        int count = endIdx - startIdx;
        QVector3D mean(0,0,0);
        for (int i = startIdx; i < endIdx; ++i) mean += gyroRates[i].omegaGyro;
        mean /= static_cast<float>(count);
        double variance = 0.0;
        for (int i = startIdx; i < endIdx; ++i) {
            QVector3D diff = gyroRates[i].omegaGyro - mean;
            variance += diff.x()*diff.x() + diff.y()*diff.y() + diff.z()*diff.z();
        }
        variance /= static_cast<double>(count);
        windows.append({(tStart + tEnd) * 0.5, variance, startIdx, endIdx});
    }

    if (windows.isEmpty())
        return results;

    std::sort(windows.begin(), windows.end(),
        [](const WindowInfo &a, const WindowInfo &b) { return a.variance > b.variance; });
    int numWindows = qBound(MIN_WINDOWS_COUNT,
                            static_cast<int>(windows.size()),
                            MAX_WINDOWS_COUNT);
    numWindows = qMin(numWindows, TARGET_WINDOWS);
    windows.resize(numWindows);
    std::sort(windows.begin(), windows.end(),
        [](const WindowInfo &a, const WindowInfo &b) { return a.tCenter < b.tCenter; });

    // For each window, cross-correlate the small local lag range using the
    // paired gyro samples (already aligned with visual times).
    int maxShift = static_cast<int>(std::round(CORR_RANGE / CORR_STEP));
    for (const auto &win : windows) {
        int count = win.endIdx - win.startIdx;
        if (count < 5)
            continue;

        QVector<QVector3D> visData, gyroData;
        QVector<double> times;
        visData.reserve(count); gyroData.reserve(count); times.reserve(count);
        for (int i = win.startIdx; i < win.endIdx; ++i) {
            visData.append(visualRates[i].omegaVisual);
            gyroData.append(gyroRates[i].omegaGyro);
            times.append(visualRates[i].tMid);
        }

        QVector3D visMean(0,0,0), gyroMean(0,0,0);
        for (int i = 0; i < count; ++i) { visMean += visData[i]; gyroMean += gyroData[i]; }
        visMean /= static_cast<float>(count);
        gyroMean /= static_cast<float>(count);
        for (int i = 0; i < count; ++i) { visData[i] -= visMean; gyroData[i] -= gyroMean; }

        QVector<double> corr(static_cast<size_t>(2 * maxShift + 1), 0.0);
        double windowDuration = times.last() - times.first();
        if (windowDuration < 1e-6)
            continue;

        for (int s = -maxShift; s <= maxShift; ++s) {
            double tau = s * CORR_STEP;
            double sum = 0.0;
            for (int i = 0; i < count; ++i) {
                double tShifted = times[i] + tau;
                if (tShifted < times.first() || tShifted > times.last())
                    continue;
                int lo = 0, hi = count - 1;
                while (lo < hi - 1) { int mid = (lo + hi) / 2; if (times[mid] <= tShifted) lo = mid; else hi = mid; }
                double dt = times[hi] - times[lo];
                QVector3D gyroInterp;
                if (dt < 1e-12) gyroInterp = gyroData[lo];
                else {
                    double frac = (tShifted - times[lo]) / dt;
                    gyroInterp = QVector3D(
                        static_cast<float>(gyroData[lo].x() + (gyroData[hi].x() - gyroData[lo].x()) * frac),
                        static_cast<float>(gyroData[lo].y() + (gyroData[hi].y() - gyroData[lo].y()) * frac),
                        static_cast<float>(gyroData[lo].z() + (gyroData[hi].z() - gyroData[lo].z()) * frac));
                }
                sum += visData[i].x() * gyroInterp.x()
                     + visData[i].y() * gyroInterp.y()
                     + visData[i].z() * gyroInterp.z();
            }
            corr[static_cast<size_t>(s + maxShift)] = sum;
        }

        int peakIdx = maxShift;
        double peakVal = corr[static_cast<size_t>(maxShift)];
        for (int s = -maxShift; s <= maxShift; ++s) {
            double val = corr[static_cast<size_t>(s + maxShift)];
            if (val > peakVal) { peakVal = val; peakIdx = s; }
        }

        double visNorm = 0.0, gyroNorm = 0.0;
        for (int i = 0; i < count; ++i) {
            visNorm += visData[i].x()*visData[i].x() + visData[i].y()*visData[i].y() + visData[i].z()*visData[i].z();
            gyroNorm += gyroData[i].x()*gyroData[i].x() + gyroData[i].y()*gyroData[i].y() + gyroData[i].z()*gyroData[i].z();
        }
        double normFactor = std::sqrt(visNorm * gyroNorm);
        double peakNormalized = (normFactor > 1e-10) ? peakVal / normFactor : 0.0;

        if (peakNormalized < PEAK_THRESHOLD)
            continue;

        double refinedShift = static_cast<double>(peakIdx);
        if (peakIdx > -maxShift && peakIdx < maxShift) {
            double ym1 = corr[static_cast<size_t>(peakIdx - 1 + maxShift)];
            double y0  = corr[static_cast<size_t>(peakIdx + maxShift)];
            double yp1 = corr[static_cast<size_t>(peakIdx + 1 + maxShift)];
            double denom = ym1 - 2.0 * y0 + yp1;
            if (std::abs(denom) > 1e-12)
                refinedShift = peakIdx + 0.5 * (ym1 - yp1) / denom;
        }

        results.append({win.tCenter, refinedShift * CORR_STEP, win.variance});
    }

    return results;
}

// ---------------------------------------------------------------------------
// Step 5: Joint line fit (weighted least squares)
// ---------------------------------------------------------------------------
bool SyncSolver::fitLine(const QVector<LocalOffset> &offsets,
                         double &o0, double &drift, double &residualMs)
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

    // Drift outside the physically plausible band means the per-window offsets
    // are not lying on a line at all — the slope is fitting noise. Report the
    // fit as failed so the caller keeps its own drift estimate, instead of
    // clamping to the rail and passing the rail off as a measurement.
    if (std::abs(drift) > MAX_DRIFT)
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

    // Step 3: Global cross-correlation to find the coarse sync offset (robust
    // even for short / continuously-moving clips). Gyro is resampled from the
    // raw IMU at a candidate lag for each step, so this is a true temporal
    // shift of the IMU stream — not bounded by the per-window pairing.
    emit progressChanged(0.2, QStringLiteral("Cross-correlating gyro and visual rate..."));
    double globalLag = 0.0;
    globalCrossCorrelate(visualRates, imuSamples, initialDrift, &globalLag);

    // Refine the global estimate with a second pass centered on the found lag,
    // using a finer step so the outcome is robust to drift*time over the clip.
    double refineCenter = globalLag;
    const double refineRange = 0.05;                 // ±50 ms around coarse peak
    const double refineStep = refineRange * 0.05;    // ~2.5 ms
    {
        int steps = static_cast<int>(std::round(2.0 * refineRange / refineStep));
        double bestScore = -1e18, bestLag = refineCenter;
        // Same normalised magnitude correlation as globalCrossCorrelate — see
        // the note there on why the frame-mixing dot product was wrong.
        const int nv = visualRates.size();
        QVector<double> vmag(nv), gmag(nv);
        double vMean = 0.0;
        for (int i = 0; i < nv; i++) { vmag[i] = visualRates[i].omegaVisual.length(); vMean += vmag[i]; }
        vMean /= nv;
        double vVar = 0.0;
        for (int i = 0; i < nv; i++) vVar += (vmag[i]-vMean)*(vmag[i]-vMean);
        for (int s = 0; s <= steps && vVar > 0.0; ++s) {
            double lag = refineCenter - refineRange + s * refineStep;
            double gMean = 0.0;
            for (int i = 0; i < nv; i++) {
                const double tImu = visualRates[i].tMid * (1.0 + initialDrift) + lag;
                gmag[i] = interpolateGyro(imuSamples, tImu).length();
                gMean += gmag[i];
            }
            gMean /= nv;
            double num = 0.0, gVar = 0.0;
            for (int i = 0; i < nv; i++) {
                const double dv = vmag[i]-vMean, dg = gmag[i]-gMean;
                num += dv*dg; gVar += dg*dg;
            }
            const double score = (gVar > 0.0) ? num/std::sqrt(vVar*gVar) : -1e18;
            if (score > bestScore) { bestScore = score; bestLag = lag; }
        }
        globalLag = bestLag;
    }

    // Step 4: Sample gyro at the coarse-aligned visual times, then per-window
    // local offsets for the joint offset+drift line fit.
    emit progressChanged(0.4, QStringLiteral("Refining per-window sync..."));
    QVector<GyroRateSample> gyroRates = sampleGyroRates(visualRates, imuSamples,
                                                        initialDrift, globalLag);
    QVector<LocalOffset> offsets = crossCorrelateWindows(visualRates, gyroRates);

    if (offsets.size() < MIN_WINDOWS) {
        // Too few per-window offsets (e.g. a short, continuously moving clip):
        // fall back to the global offset with a flat drift.
        emit progressChanged(1.0, QStringLiteral("Done (global peak only)"));
        SyncResult result;
        result.syncOffset = globalLag;
        result.drift = initialDrift;
        result.residualMs = 0.0;
        result.windowsUsed = (int)offsets.size();
        emit syncSolved(result);
        return;
    }

    // Step 5: Joint line fit of the per-window residuals (around globalLag):
    // tau(t) = o0 + drift*t. The absolute sync offset is globalLag + tau(t).
    double o0, drift, residualMs;
    if (!fitLine(offsets, o0, drift, residualMs)) {
        // Fit degenerate: use global offset, keep initial drift.
        emit progressChanged(1.0, QStringLiteral("Done (global + flat drift)"));
        SyncResult result;
        result.syncOffset = globalLag;
        result.drift = initialDrift;
        result.residualMs = 0.0;
        result.windowsUsed = (int)offsets.size();
        emit syncSolved(result);
        return;
    }
    // Absolute offset at t=0 for the refined second pass.
    const double absOffset0 = globalLag + o0;

    // Step 6: Second pass with the refined absolute offset and drift.
    emit progressChanged(0.6, QStringLiteral("Second pass refinement..."));
    QVector<GyroRateSample> gyroRates2 = sampleGyroRates(visualRates, imuSamples,
                                                         drift, absOffset0);
    QVector<LocalOffset> offsets2 = crossCorrelateWindows(visualRates, gyroRates2);
    if (offsets2.size() >= MIN_WINDOWS) {
        double o0b, driftb, residb;
        if (fitLine(offsets2, o0b, driftb, residb)) {
            // o0b is a further residual around absOffset0; accumulate it.
            o0 += o0b; drift = driftb; residualMs = residb;
        }
    }
    // (Second pass not possible: keep first-pass fit.)

    // Absolute sync offset: globalLag + accumulated residual.
    SyncResult result;
    result.syncOffset = globalLag + o0;
    result.drift = drift;
    result.residualMs = residualMs;
    result.windowsUsed = (int)(offsets2.isEmpty() ? offsets.size() : offsets2.size());

    // Final plausibility gate. The searched lag range is ±CORR_RANGE, so an
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

    emit progressChanged(1.0, QStringLiteral("Done"));
    emit syncSolved(result);
}
