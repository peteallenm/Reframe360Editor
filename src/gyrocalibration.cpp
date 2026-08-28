#include "gyrocalibration.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <numeric>
#include <utility>

static constexpr double PI = 3.14159265358979323846;
// The visual front-end decodes every 4th frame and then hops up to 4 decoded
// frames at a time, so a 5-7 s calibration clip yields only ~15 pairs in total
// (measured: JustRoll 13, JustPitch 17). The fit has 12 free parameters and
// each pair contributes 3 equations, so ~12 pairs is a 3:1 ratio — thin, but
// the acceptance gates downstream are what actually decide whether the answer
// is usable. A floor of 20 simply made every short clip unfittable.
static constexpr int MIN_VISUAL_PAIRS = 12;
static constexpr double HUBER_DELTA = 5.0;          // deg/s
static constexpr double REGULARIZATION_LAMBDA = 0.01;
static constexpr int IRLS_ITERATIONS = 3;
// Acceptance band for the solved matrix. These are REJECTION bounds, not
// clamps: a fit that lands outside them is not a slightly-wrong calibration
// that can be trimmed back into shape, it is evidence that the solve did not
// converge on anything meaningful. Clamping such a result and storing it
// anyway is how a run of AutoSync used to overwrite a perfectly good gyro with
// a rail-clamped 0.5*I (halving every rate) or, worse, a matrix with
// off-diagonals of +-34.
static constexpr double MIN_DIAGONAL = 0.7;
static constexpr double MAX_DIAGONAL = 1.4;
// A real gyro's axis misalignment is a few degrees, so cross-axis terms of
// more than ~0.2 mean the solve has started mixing axes to absorb noise.
static constexpr double MAX_OFF_DIAGONAL = 0.20;
// A plausible bias for this sensor is single-digit deg/s. Anything larger is
// the fit absorbing an unmodelled sync error into the constant term.
static constexpr double MAX_BIAS_DEG_S = 8.0;
// RMS residual above this means the visual and gyro rates never really agreed.
static constexpr double MAX_RESIDUAL_DEG_S = 8.0;
// Admitted pairs needed before the fit is trusted. A COUNT, deliberately, not
// a sum of normalised weights: weights are scaled so the best pair in the clip
// is 1.0, which makes their sum depend on how good that one pair happened to
// be — a clip of uniformly decent pairs would score lower than a clip with one
// excellent pair and the rest rubbish. Exactly backwards for a "do I have
// enough evidence" test.
static constexpr int MIN_ADMITTED_PAIRS = 12;
// Visual pairs weaker than this contribute nothing but noise to the fit.
// Set from the measured distribution of this pipeline (--pair-stats): inliers
// run p05=3, p25=17-22, median 44-212; RMS runs p25=5.6, median 7.2-7.8,
// p95=9-10. A cut at RMS<=8 would therefore discard about half of all pairs on
// a good clip. These bounds trim the tail, not the body.
static constexpr int MIN_PAIR_INLIERS = 15;
static constexpr double MAX_PAIR_RMS_DEG = 15.0;
// Sanity bound only. Long hops used to be a problem because the visual rate
// was an average over the hop while the gyro was sampled instantaneously at
// the midpoint; buildPairedSamples now averages the gyro over the same window,
// so a long hop is good data rather than biased data. The remaining error is
// the rotation of the body axes within the window, which is second-order.
static constexpr double MAX_PAIR_DT = 1.0;          // s

// ---------------------------------------------------------------------------
// 4x4 linear solver via Gaussian elimination with partial pivoting
// ---------------------------------------------------------------------------
static bool solve4x4(double A[4][4], double b_vec[4], double x[4])
{
    double aug[4][5];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            aug[i][j] = A[i][j];
        aug[i][4] = b_vec[i];
    }

    for (int col = 0; col < 4; col++) {
        // Find pivot
        int maxRow = col;
        double maxVal = std::abs(aug[col][col]);
        for (int row = col + 1; row < 4; row++) {
            if (std::abs(aug[row][col]) > maxVal) {
                maxVal = std::abs(aug[row][col]);
                maxRow = row;
            }
        }
        if (maxVal < 1e-15)
            return false;

        // Swap rows
        if (maxRow != col) {
            for (int j = 0; j < 5; j++)
                std::swap(aug[col][j], aug[maxRow][j]);
        }

        // Eliminate below
        for (int row = col + 1; row < 4; row++) {
            double factor = aug[row][col] / aug[col][col];
            for (int j = col; j < 5; j++)
                aug[row][j] -= factor * aug[col][j];
        }
    }

    // Back substitution
    for (int i = 3; i >= 0; i--) {
        x[i] = aug[i][4];
        for (int j = i + 1; j < 4; j++)
            x[i] -= aug[i][j] * x[j];
        x[i] /= aug[i][i];
    }
    return true;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
GyroCalibrator::GyroCalibrator(QObject *parent)
    : QObject(parent)
{
}

// ---------------------------------------------------------------------------
// Quaternion -> axis-angle (degrees)
// ---------------------------------------------------------------------------
QVector3D GyroCalibrator::quaternionToAxisAngle(const QQuaternion &q)
{
    QQuaternion qn = q.normalized();
    if (qn.scalar() < 0.0)
        qn = QQuaternion(-qn.scalar(), -qn.x(), -qn.y(), -qn.z());

    double w = qn.scalar();
    w = qBound(-1.0, w, 1.0);

    double angleRad = 2.0 * std::acos(w);
    double sinHalf = std::sin(angleRad * 0.5);

    if (sinHalf < 1e-10)
        return QVector3D(0.0f, 0.0f, 0.0f);

    double angleDeg = angleRad * 180.0 / PI;
    double ax = qn.x() / sinHalf;
    double ay = qn.y() / sinHalf;
    double az = qn.z() / sinHalf;

    return QVector3D(static_cast<float>(ax * angleDeg),
                     static_cast<float>(ay * angleDeg),
                     static_cast<float>(az * angleDeg));
}

// ---------------------------------------------------------------------------
// Interpolate gyro at arbitrary time (binary search + lerp)
// ---------------------------------------------------------------------------
QVector3D GyroCalibrator::interpolateGyro(const QVector<ImuSample> &samples, double t)
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
// Mean gyro over [t0, t1], trapezoidal over the samples inside the window plus
// the interpolated end points. Matches what the visual measurement is: an
// average across the hop, not a value at an instant.
// ---------------------------------------------------------------------------
QVector3D GyroCalibrator::meanGyroOverWindow(const QVector<ImuSample> &samples,
                                             double t0, double t1)
{
    if (samples.isEmpty() || t1 <= t0)
        return interpolateGyro(samples, 0.5 * (t0 + t1));

    // Window entirely outside the sample range, or so short it spans no sample
    // boundary: fall back to a midpoint interpolation.
    if (t1 <= samples.first().timestamp || t0 >= samples.last().timestamp)
        return interpolateGyro(samples, 0.5 * (t0 + t1));

    const double a = std::max(t0, samples.first().timestamp);
    const double b = std::min(t1, samples.last().timestamp);
    if (b <= a)
        return interpolateGyro(samples, 0.5 * (t0 + t1));

    // Index of the first sample strictly after a.
    auto it = std::upper_bound(samples.constBegin(), samples.constEnd(), a,
        [](double val, const ImuSample &s) { return val < s.timestamp; });
    int i = static_cast<int>(std::distance(samples.constBegin(), it));

    QVector3D acc(0.0f, 0.0f, 0.0f);
    double prevT = a;
    QVector3D prevG = interpolateGyro(samples, a);

    for (; i < samples.size() && samples[i].timestamp < b; i++) {
        const double curT = samples[i].timestamp;
        const QVector3D curG = samples[i].gyro;
        const double h = curT - prevT;
        if (h > 0.0)
            acc += (prevG + curG) * static_cast<float>(0.5 * h);
        prevT = curT;
        prevG = curG;
    }

    // Final partial interval up to b.
    const QVector3D endG = interpolateGyro(samples, b);
    const double h = b - prevT;
    if (h > 0.0)
        acc += (prevG + endG) * static_cast<float>(0.5 * h);

    return acc / static_cast<float>(b - a);
}

// ---------------------------------------------------------------------------
// Build paired samples: visual rates matched with raw gyro readings
// ---------------------------------------------------------------------------
QVector<GyroCalibrator::PairedSample> GyroCalibrator::buildPairedSamples(
    const QVector<VisualRotationPair> &visualPairs,
    const QVector<ImuSample> &imuSamples,
    double syncOffset, double drift,
    const QQuaternion &imuToCamera)
{
    // Sensor -> camera -> DISPLAY. The extra 180-deg roll about Z is
    // GyroscopeIntegrator's kFlipRoll, which it post-multiplies onto every
    // stored orientation to un-flip the YI camera's inherently upside-down
    // fisheye. The visual bearings come from pixelToBearing on that displayed
    // video, so they live in the flipped frame while the raw gyro does not.
    //
    // Omitting it is why this solve kept returning a matrix the gates rejected:
    // it came out as diag(-1, -1, +1) with negligible off-diagonals on all three
    // Just* clips, which is not a broken fit at all — that IS a 180-deg rotation
    // about Z. The solver was correctly recovering the missing flip and being
    // thrown away for it.
    const QQuaternion kFlipRoll(0.0f, 0.0f, 0.0f, 1.0f);
    const QQuaternion qInv = kFlipRoll * imuToCamera.conjugated();

    QVector<PairedSample> paired;
    paired.reserve(visualPairs.size());

    for (const auto &pair : visualPairs) {
        double dt = pair.t1 - pair.t0;
        if (dt < 1e-6)
            continue;

        // Reject pairs the visual solver itself had little confidence in. The
        // hop search accepts a rotation from as few as 3 inliers at up to 30
        // deg RMS so that fast sections stay tracked at all; such a pair is a
        // usable hint for chaining but is nowhere near good enough to serve as
        // ground truth for a calibration.
        if (pair.inliers < MIN_PAIR_INLIERS || pair.rmsDeg > MAX_PAIR_RMS_DEG)
            continue;

        if (dt > MAX_PAIR_DT)
            continue;

        // Visual rotation rate (deg/s): the NET rotation over [t0, t1] divided
        // by the hop, i.e. the mean angular velocity across the window.
        QVector3D axisAngle = quaternionToAxisAngle(pair.deltaR);
        QVector3D omegaVisual(axisAngle.x() / static_cast<float>(dt),
                              axisAngle.y() / static_cast<float>(dt),
                              axisAngle.z() / static_cast<float>(dt));

        // Gyro over the SAME window, mean-averaged — not sampled at the
        // midpoint. This is the whole ballgame for the scale estimate. The
        // model being fitted is linear, omega_visual = M * omega_raw + b, so
        // it survives averaging intact provided BOTH sides are averaged over
        // the same interval. Pairing a window-averaged regressand with an
        // instantaneous regressor instead is a textbook errors-in-variables
        // setup: the regressor carries all the high-frequency variance the
        // regressand has had removed, and the fitted slope is attenuated by
        // var_signal / (var_signal + var_highfreq). With hops of 0.13-0.53 s
        // on handheld footage that factor is brutal, which is why the solve
        // landed on the 0.5 diagonal rail for every clip that was ever run
        // through it.
        double tImu0 = pair.t0 * (1.0 + drift) + syncOffset;
        double tImu1 = pair.t1 * (1.0 + drift) + syncOffset;
        // ...rotated into camera axes so both sides of the regression live in
        // the same frame (see the imuToCamera note on calibrate()).
        QVector3D omegaRaw = qInv.rotatedVector(
            meanGyroOverWindow(imuSamples, tImu0, tImu1));

        paired.append({omegaVisual, omegaRaw,
                       pair.inliers / (1.0 + pair.rmsDeg)});
    }

    // Normalise weights so the best pair in the clip carries 1.0. The absolute
    // scale is arbitrary (it cancels in the weighted normal equations) but a
    // fixed maximum keeps MIN_EFFECTIVE_SAMPLES and REGULARIZATION_LAMBDA
    // meaningful across clips of different quality.
    double maxWeight = 0.0;
    for (const auto &s : paired)
        maxWeight = std::max(maxWeight, s.weight);
    if (maxWeight > 0.0) {
        for (auto &s : paired)
            s.weight /= maxWeight;
    }

    return paired;
}

// ---------------------------------------------------------------------------
// Solve one row of M with regularization and optional weights
// ---------------------------------------------------------------------------
std::array<double, 4> GyroCalibrator::solveRowRegularized(
    const QVector<PairedSample> &samples,
    int component,
    const QVector<double> &weights,
    const std::array<double, 4> &priorRow,
    double lambda)
{
    // Build normal equations: (A^T W A + lambda*I) x = A^T W y + lambda*prior
    // A is N x 4, each row: [omegaRaw.x, omegaRaw.y, omegaRaw.z, 1]
    // y is the visual omega component

    double ATA[4][4] = {};
    double ATy[4] = {};

    for (int i = 0; i < samples.size(); i++) {
        double w = (i < weights.size()) ? weights[i] : 1.0;
        const auto &s = samples[i];

        // Visual component for this row
        double yi;
        switch (component) {
        case 0: yi = s.omegaVisual.x(); break;
        case 1: yi = s.omegaVisual.y(); break;
        default: yi = s.omegaVisual.z(); break;
        }

        // Raw gyro row: [gx, gy, gz, 1]
        double a[4] = {
            static_cast<double>(s.omegaRaw.x()),
            static_cast<double>(s.omegaRaw.y()),
            static_cast<double>(s.omegaRaw.z()),
            1.0
        };

        // Accumulate A^T W A
        for (int r = 0; r < 4; r++) {
            for (int c = 0; c < 4; c++)
                ATA[r][c] += w * a[r] * a[c];
            ATy[r] += w * a[r] * yi;
        }
    }

    // Add regularization: lambda * I to ATA, lambda * prior to ATy
    for (int i = 0; i < 4; i++) {
        ATA[i][i] += lambda;
        ATy[i] += lambda * priorRow[i];
    }

    // Solve
    std::array<double, 4> result = {};
    if (!solve4x4(ATA, ATy, result.data())) {
        // Fallback to prior if singular
        result = priorRow;
    }

    return result;
}

// ---------------------------------------------------------------------------
// Compute Huber weights from residuals
// ---------------------------------------------------------------------------
QVector<double> GyroCalibrator::computeHuberWeights(
    const QVector<PairedSample> &samples,
    const QMatrix3x3 &M, const QVector3D &b,
    double delta)
{
    QVector<double> weights;
    weights.reserve(samples.size());

    for (const auto &s : samples) {
        // Predicted = M * omegaRaw + b
        double rx = static_cast<double>(s.omegaRaw.x());
        double ry = static_cast<double>(s.omegaRaw.y());
        double rz = static_cast<double>(s.omegaRaw.z());

        double predX = M(0, 0) * rx + M(0, 1) * ry + M(0, 2) * rz + b.x();
        double predY = M(1, 0) * rx + M(1, 1) * ry + M(1, 2) * rz + b.y();
        double predZ = M(2, 0) * rx + M(2, 1) * ry + M(2, 2) * rz + b.z();

        double resX = static_cast<double>(s.omegaVisual.x()) - predX;
        double resY = static_cast<double>(s.omegaVisual.y()) - predY;
        double resZ = static_cast<double>(s.omegaVisual.z()) - predZ;

        double resNorm = std::sqrt(resX * resX + resY * resY + resZ * resZ);

        // Huber weight: if |r| < delta, w = 1; else w = delta / |r|
        double w = (resNorm < delta) ? 1.0 : (delta / resNorm);
        weights.append(w);
    }

    return weights;
}

// ---------------------------------------------------------------------------
// Robust per-axis diagonal scale + bias fit
// omega_visual = s * omega_raw + b  (per axis, independent)
// Returns a vector of (scale, scale, bias) per axis for use by the parser.
// ---------------------------------------------------------------------------
QVector<QVector3D> GyroCalibrator::fitDiagonalScales(
    const QVector<PairedSample> &samples,
    const QVector3D &priorScale,
    const QVector3D &priorBias) const
{
    // Result[i] = (s_i, unused_y, b_i) for axis i
    QVector<QVector3D> result(3, QVector3D(1.0f, 0.0f, 0.0f));

    if (samples.isEmpty())
        return result;

    // Solve each axis independently: [omegaRaw_axis, 1] * [s, b] = omegaVisual_axis
    // Normal equations with a tiny prior toward (1, 0).
    const double lambda = REGULARIZATION_LAMBDA * 1e-3; // very light prior

    auto priorS = [&](int axis) {
        switch (axis) { case 0: return (double)priorScale.x(); case 1: return (double)priorScale.y(); default: return (double)priorScale.z(); }
    };
    auto priorB = [&](int axis) {
        switch (axis) { case 0: return (double)priorBias.x(); case 1: return (double)priorBias.y(); default: return (double)priorBias.z(); }
    };

    for (int axis = 0; axis < 3; axis++) {
        double bestS = 1.0, bestB = 0.0;

        // IRLS with Huber weights, seeded from the visual measurement
        // confidence so weak pairs never dominate the first iteration.
        QVector<double> weights(samples.size(), 1.0);
        for (int i = 0; i < samples.size(); i++)
            weights[i] = samples[i].weight;
        for (int iter = 0; iter < IRLS_ITERATIONS; iter++) {
            double ATA00 = lambda, ATA01 = 0.0, ATA11 = lambda;
            double ATy0 = lambda * priorS(axis), ATy1 = lambda * priorB(axis);

            for (int i = 0; i < samples.size(); i++) {
                double gr;
                switch (axis) { case 0: gr = samples[i].omegaRaw.x(); break; case 1: gr = samples[i].omegaRaw.y(); break; default: gr = samples[i].omegaRaw.z(); break; }
                double vv;
                switch (axis) { case 0: vv = samples[i].omegaVisual.x(); break; case 1: vv = samples[i].omegaVisual.y(); break; default: vv = samples[i].omegaVisual.z(); break; }

                double w = weights[i];
                ATA00 += w * gr * gr;
                ATA01 += w * gr;
                ATA11 += w;
                ATy0 += w * gr * vv;
                ATy1 += w * vv;
            }

            double det = ATA00 * ATA11 - ATA01 * ATA01;
            if (std::abs(det) < 1e-12)
                break;
            double s = (ATy0 * ATA11 - ATy1 * ATA01) / det;
            double b = (ATA00 * ATy1 - ATA01 * ATy0) / det;
            bestS = s; bestB = b;

            // Recompute Huber weights from residuals
            double sumAbs = 0.0;
            for (int i = 0; i < samples.size(); i++) {
                double gr;
                switch (axis) { case 0: gr = samples[i].omegaRaw.x(); break; case 1: gr = samples[i].omegaRaw.y(); break; default: gr = samples[i].omegaRaw.z(); break; }
                double vv;
                switch (axis) { case 0: vv = samples[i].omegaVisual.x(); break; case 1: vv = samples[i].omegaVisual.y(); break; default: vv = samples[i].omegaVisual.z(); break; }
                double r = vv - (s * gr + b);
                double huber = (std::abs(r) < HUBER_DELTA) ? 1.0 : HUBER_DELTA / std::abs(r);
                weights[i] = huber * samples[i].weight;
                sumAbs += std::abs(r);
            }
        }

        // Sanity: keep the scale within [0.5, 2.0]
        bestS = qBound(0.5, bestS, 2.0);
        result[axis] = QVector3D((float)bestS, 0.0f, (float)bestB);
    }

    return result;
}

// ---------------------------------------------------------------------------
// 3x3 multiply (QMatrix3x3 has no operator*)
// ---------------------------------------------------------------------------
static QMatrix3x3 mat3Mul(const QMatrix3x3 &A, const QMatrix3x3 &B)
{
    QMatrix3x3 C;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            float acc = 0.0f;
            for (int k = 0; k < 3; k++)
                acc += A(r, k) * B(k, c);
            C(r, c) = acc;
        }
    return C;
}

static QMatrix3x3 mat3Transpose(const QMatrix3x3 &A)
{
    QMatrix3x3 T;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            T(r, c) = A(c, r);
    return T;
}

// ---------------------------------------------------------------------------
// Main calibration routine
// ---------------------------------------------------------------------------
void GyroCalibrator::calibrate(
    const QVector<VisualRotationPair> &visualPairs,
    const QVector<ImuSample> &imuSamples,
    double syncOffset,
    double drift,
    const QMatrix3x3 &priorM,
    const QVector3D &priorB,
    const QQuaternion &imuToCamera)
{
    // The whole solve runs in CAMERA axes (see the header note). R maps sensor
    // -> camera; a correction M_s in sensor axes is the same correction as
    // M_c = R * M_s * R^T in camera axes, and a bias b_s as b_c = R * b_s.
    const QQuaternion kFlipRollC(0.0f, 0.0f, 0.0f, 1.0f);
    const QQuaternion qSensorToVisual = kFlipRollC * imuToCamera.conjugated();
    const QMatrix3x3 R = qSensorToVisual.toRotationMatrix();
    const QMatrix3x3 Rt = mat3Transpose(R);
    const QMatrix3x3 priorMcam = mat3Mul(mat3Mul(R, priorM), Rt);
    const QVector3D priorBcam = qSensorToVisual.rotatedVector(priorB);

    // Check minimum sample count
    if (visualPairs.size() < MIN_VISUAL_PAIRS) {
        emit calibrationFailed(
            QString("Too few visual rotation pairs: %1 (need at least %2)")
                .arg(visualPairs.size()).arg(MIN_VISUAL_PAIRS));
        return;
    }

    emit progressChanged(0.1, "Building paired samples");

    // Build paired visual/gyro samples
    QVector<PairedSample> paired = buildPairedSamples(visualPairs, imuSamples,
                                                       syncOffset, drift,
                                                       imuToCamera);
    if (paired.size() < MIN_ADMITTED_PAIRS) {
        // Say how many were dropped and why, so a failure here points at the
        // visual front-end (short clip, weak features, hops too wide) rather
        // than looking like a mystery.
        emit calibrationFailed(
            QString("Only %1 of %2 visual pairs passed quality admission "
                    "(need %3; require >=%4 inliers, <=%5 deg RMS, <=%6 s hop)")
                .arg(paired.size()).arg(visualPairs.size()).arg(MIN_ADMITTED_PAIRS)
                .arg(MIN_PAIR_INLIERS).arg(MAX_PAIR_RMS_DEG).arg(MAX_PAIR_DT));
        return;
    }

    emit progressChanged(0.3, "Solving initial least-squares");

    // Set up prior rows
    std::array<std::array<double, 4>, 3> priorRows;
    for (int row = 0; row < 3; row++) {
        double bv;
        switch (row) {
        case 0: bv = priorBcam.x(); break;
        case 1: bv = priorBcam.y(); break;
        default: bv = priorBcam.z(); break;
        }
        priorRows[row] = {
            static_cast<double>(priorMcam(row, 0)),
            static_cast<double>(priorMcam(row, 1)),
            static_cast<double>(priorMcam(row, 2)),
            bv
        };
    }

    // Initial solve weighted by visual measurement confidence. Weighting here
    // rather than uniformly matters: a 25-inlier pair and a 400-inlier pair
    // are not equally good evidence, and the Huber pass below cannot recover
    // from an initial fit that the weak pairs already dragged off course.
    QVector<double> qualityWeights;
    qualityWeights.reserve(paired.size());
    for (const auto &s : paired)
        qualityWeights.append(s.weight);

    QMatrix3x3 M;
    QVector3D biasVec;

    for (int row = 0; row < 3; row++) {
        auto sol = solveRowRegularized(paired, row, qualityWeights,
                                        priorRows[row], REGULARIZATION_LAMBDA);
        M(row, 0) = static_cast<float>(sol[0]);
        M(row, 1) = static_cast<float>(sol[1]);
        M(row, 2) = static_cast<float>(sol[2]);
        switch (row) {
        case 0: biasVec.setX(static_cast<float>(sol[3])); break;
        case 1: biasVec.setY(static_cast<float>(sol[3])); break;
        default: biasVec.setZ(static_cast<float>(sol[3])); break;
        }
    }

    emit progressChanged(0.5, "Robust fitting (IRLS)");

    // IRLS iterations
    for (int iter = 0; iter < IRLS_ITERATIONS; iter++) {
        QVector<double> weights = computeHuberWeights(paired, M, biasVec, HUBER_DELTA);
        // Robust weight and measurement confidence are independent reasons to
        // trust a sample, so they multiply.
        for (int i = 0; i < weights.size() && i < paired.size(); i++)
            weights[i] *= paired[i].weight;

        for (int row = 0; row < 3; row++) {
            auto sol = solveRowRegularized(paired, row, weights,
                                            priorRows[row], REGULARIZATION_LAMBDA);
            M(row, 0) = static_cast<float>(sol[0]);
            M(row, 1) = static_cast<float>(sol[1]);
            M(row, 2) = static_cast<float>(sol[2]);
            switch (row) {
            case 0: biasVec.setX(static_cast<float>(sol[3])); break;
            case 1: biasVec.setY(static_cast<float>(sol[3])); break;
            default: biasVec.setZ(static_cast<float>(sol[3])); break;
            }
        }
    }

    emit progressChanged(0.8, "Computing residual");

    // Compute weighted RMS residual, using the same measurement confidence the
    // fit used. An unweighted residual is dominated by the weakest pairs and
    // so reports a bad number for a good fit (and vice versa).
    double sumSqRes = 0.0;
    double sumWeight = 0.0;
    for (const auto &s : paired) {
        double rx = static_cast<double>(s.omegaRaw.x());
        double ry = static_cast<double>(s.omegaRaw.y());
        double rz = static_cast<double>(s.omegaRaw.z());

        double predX = M(0, 0) * rx + M(0, 1) * ry + M(0, 2) * rz + biasVec.x();
        double predY = M(1, 0) * rx + M(1, 1) * ry + M(1, 2) * rz + biasVec.y();
        double predZ = M(2, 0) * rx + M(2, 1) * ry + M(2, 2) * rz + biasVec.z();

        double resX = static_cast<double>(s.omegaVisual.x()) - predX;
        double resY = static_cast<double>(s.omegaVisual.y()) - predY;
        double resZ = static_cast<double>(s.omegaVisual.z()) - predZ;

        sumSqRes += s.weight * (resX * resX + resY * resY + resZ * resZ);
        sumWeight += s.weight;
    }

    if (sumWeight <= 0.0) {
        emit calibrationFailed(
            QStringLiteral("All paired samples had zero weight"));
        return;
    }
    double rmsResidual = std::sqrt(sumSqRes / sumWeight);

    // Always log the raw solve before the gates judge it. A rejection that
    // names one element tells you nothing about WHY; the shape of the whole
    // matrix does (near-identity? a permutation? rank-deficient?).
    qInfo().noquote() << QString("GyroCalibration raw solve (residual %1 deg/s, %2 pairs):")
        .arg(rmsResidual, 0, 'f', 2).arg(paired.size());
    for (int r = 0; r < 3; r++)
        qInfo().noquote() << QString("  [%1 %2 %3]")
            .arg(M(r,0),8,'f',3).arg(M(r,1),8,'f',3).arg(M(r,2),8,'f',3);
    qInfo().noquote() << QString("  bias [%1 %2 %3] deg/s")
        .arg(biasVec.x(),0,'f',2).arg(biasVec.y(),0,'f',2).arg(biasVec.z(),0,'f',2);

    // --- Acceptance gates -------------------------------------------------
    // Everything below REJECTS. Nothing here trims a bad answer into the
    // acceptable band and passes it on: a calibration that misses these bounds
    // is not a calibration, and storing one silently overwrites a gyro that
    // was working fine. Returning calibrationFailed() leaves the previous
    // (or default) calibration untouched, which is the safe outcome.

    // Non-finite: a singular normal-equation solve can produce NaN/Inf.
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            if (!std::isfinite(M(r, c))) {
                emit calibrationFailed(
                    QStringLiteral("Calibration solve produced a non-finite matrix"));
                return;
            }
        }
    }
    if (!std::isfinite(biasVec.x()) || !std::isfinite(biasVec.y())
            || !std::isfinite(biasVec.z()) || !std::isfinite(rmsResidual)) {
        emit calibrationFailed(
            QStringLiteral("Calibration solve produced a non-finite bias or residual"));
        return;
    }

    if (paired.size() < MIN_ADMITTED_PAIRS) {
        emit calibrationFailed(
            QStringLiteral("Only %1 visual pairs passed quality admission (need %2). "
                           "The clip is too short, too fast or too featureless to "
                           "calibrate against.")
                .arg(paired.size()).arg(MIN_ADMITTED_PAIRS));
        return;
    }

    for (int i = 0; i < 3; i++) {
        const double d = M(i, i);
        if (d < MIN_DIAGONAL || d > MAX_DIAGONAL) {
            emit calibrationFailed(
                QStringLiteral("Gyro scale on axis %1 solved to %2, outside the "
                               "trusted band [%3, %4]. Rejected — the gyro is "
                               "left as it was.")
                    .arg(i).arg(d, 0, 'f', 3)
                    .arg(MIN_DIAGONAL).arg(MAX_DIAGONAL));
            return;
        }
    }

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            if (r == c)
                continue;
            if (std::abs(M(r, c)) > MAX_OFF_DIAGONAL) {
                emit calibrationFailed(
                    QStringLiteral("Cross-axis term M(%1,%2) solved to %3, above the "
                                   "%4 limit — the fit is mixing axes to absorb "
                                   "noise. Rejected.")
                        .arg(r).arg(c).arg(M(r, c), 0, 'f', 3)
                        .arg(MAX_OFF_DIAGONAL));
                return;
            }
        }
    }

    if (biasVec.length() > MAX_BIAS_DEG_S) {
        emit calibrationFailed(
            QStringLiteral("Solved gyro bias is %1 deg/s, above the %2 deg/s limit "
                           "— most likely an unmodelled sync error absorbed into "
                           "the constant term. Rejected.")
                .arg(biasVec.length(), 0, 'f', 2).arg(MAX_BIAS_DEG_S));
        return;
    }

    if (rmsResidual > MAX_RESIDUAL_DEG_S) {
        emit calibrationFailed(
            QStringLiteral("Visual and gyro rates disagree by %1 deg/s RMS after "
                           "fitting (limit %2). No calibration can be trusted from "
                           "this clip — check the sync offset first.")
                .arg(rmsResidual, 0, 'f', 2).arg(MAX_RESIDUAL_DEG_S));
        return;
    }

    // Build result
    // Convert the accepted camera-frame correction back into SENSOR axes,
    // because that is where GyroscopeIntegrator::integrate() applies it — it
    // computes M*gyro + b on the raw sample and only then rotates by qInv.
    //   M_s = R^T * M_c * R      b_s = R^T * b_c
    GyroCalibration result;
    result.matrix = mat3Mul(mat3Mul(Rt, M), R);
    result.bias = qSensorToVisual.conjugated().rotatedVector(biasVec);
    QVector3D priorScale(1,1,1), priorBiasQ(0,0,0);
    QVector<QVector3D> diagFit = fitDiagonalScales(paired, priorScale, priorBiasQ);
    result.diagScale = QVector3D(diagFit.size() > 0 ? diagFit[0].x() : 1.0f,
                                 diagFit.size() > 1 ? diagFit[1].x() : 1.0f,
                                 diagFit.size() > 2 ? diagFit[2].x() : 1.0f);
    result.diagBias = QVector3D(diagFit.size() > 0 ? diagFit[0].z() : 0.0f,
                                diagFit.size() > 1 ? diagFit[1].z() : 0.0f,
                                diagFit.size() > 2 ? diagFit[2].z() : 0.0f);
    result.residualDeg = rmsResidual;
    result.samplesUsed = paired.size();

    emit progressChanged(1.0, "Complete");
    emit calibrationComputed(result);
}
