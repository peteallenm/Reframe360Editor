#include "visualfusion.h"
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <QDebug>
#include <QMap>

// ---------------------------------------------------------------------------
// sampleImuOrientation — slerp interpolation at arbitrary IMU time
// ---------------------------------------------------------------------------
QQuaternion VisualFusion::sampleImuOrientation(const QVector<QQuaternion> &orientations,
                                                const QVector<double> &timestamps,
                                                double time)
{
    if (orientations.isEmpty())
        return QQuaternion(1, 0, 0, 0); // identity

    if (time <= timestamps.first())
        return orientations.first();
    if (time >= timestamps.last())
        return orientations.last();

    // Binary search for the bracketing index
    const auto it = std::lower_bound(timestamps.begin(), timestamps.end(), time);
    int i = (it == timestamps.end()) ? timestamps.size() - 1
                                     : static_cast<int>(it - timestamps.begin());
    if (i > 0 && timestamps[i] > time)
        i--;

    if (i + 1 >= orientations.size())
        return orientations.last();

    double dt = timestamps[i + 1] - timestamps[i];
    float t = (dt > 0.0) ? static_cast<float>((time - timestamps[i]) / dt) : 0.0f;
    t = qBound(0.0f, t, 1.0f);

    return QQuaternion::slerp(orientations[i], orientations[i + 1], t);
}

// ---------------------------------------------------------------------------
// correctionAt — Gaussian-weighted quaternion average over correction knots
// ---------------------------------------------------------------------------
QQuaternion VisualFusion::correctionAt(double videoTime) const
{
    if (m_knots.isEmpty())
        return QQuaternion(1, 0, 0, 0);

    const double threeSigma = 3.0 * m_sigma;

    // Find knots within ±3σ and compute weights
    struct WeightedKnot {
        const CorrectionKnot *knot;
        double weight;
    };
    QVector<WeightedKnot> nearby;
    nearby.reserve(m_knots.size());

    for (const auto &k : m_knots) {
        double dt = videoTime - k.time;
        if (std::abs(dt) > threeSigma)
            continue;
        double gauss = std::exp(-0.5 * (dt / m_sigma) * (dt / m_sigma));
        double w = gauss * k.quality;
        if (w > 1e-12)
            nearby.append({&k, w});
    }

    if (nearby.isEmpty()) {
        // Fall back to nearest knot
        double bestDist = 1e30;
        int bestIdx = 0;
        for (int i = 0; i < m_knots.size(); i++) {
            double d = std::abs(videoTime - m_knots[i].time);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        return m_knots[bestIdx].correction;
    }

    // Markley-style Gaussian-weighted quaternion average:
    // Use the highest-weighted knot as reference, sign-flip all others to
    // stay within 180° of reference, accumulate weighted components, normalize.
    const QQuaternion &ref = nearby[0].knot->correction;
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float wsum = 0.0f;

    for (const auto &nk : nearby) {
        QQuaternion q = nk.knot->correction;
        float weight = static_cast<float>(nk.weight);
        // Sign-flip to stay within 180° of reference
        if (QQuaternion::dotProduct(q, ref) < 0.0f)
            q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
        x += q.x() * weight;
        y += q.y() * weight;
        z += q.z() * weight;
        w += q.scalar() * weight;
        wsum += weight;
    }

    if (wsum <= 0.0f)
        return ref;

    QQuaternion avg(w, x, y, z);
    avg /= wsum;
    avg.normalize();
    return avg;
}

// ---------------------------------------------------------------------------
// fuse — main entry point
// ---------------------------------------------------------------------------
void VisualFusion::fuse(const QVector<VisualRotationPair> &visualPairs,
                         const QVector<QQuaternion> &imuOrientations,
                         const QVector<double> &imuTimestamps,
                         double syncOffset,
                         double drift,
                         double sigmaSeconds)
{
    m_fusedOrientations.clear();
    m_fusedTimestamps.clear();
    m_knots.clear();
    m_syncOffset = syncOffset;
    m_drift = drift;
    m_sigma = sigmaSeconds;

    // --- Edge case: not enough visual pairs ---
    if (visualPairs.size() < 10) {
        qWarning() << "VisualFusion: only" << visualPairs.size()
                    << "visual pairs (need >= 10), skipping fusion";
        return;
    }

    if (imuOrientations.isEmpty() || imuTimestamps.isEmpty()) {
        qWarning() << "VisualFusion: empty IMU orientations, skipping fusion";
        return;
    }

    // --- Step 1: Chain visual rotations into Q_vis(t) ---
    // Collect all unique times from the visual pairs and build a chain.
    // Visual pairs are (t0, t1, deltaR). We chain: Q_vis(t1) = deltaR * Q_vis(t0).
    //
    // Build a time-indexed map. The anchor is the IMU orientation at the first
    // pair's t0 (mapped to IMU time).

    // First, collect all unique times and sort them
    QMap<double, int> timeToIndex; // video time -> index in visual chain
    QVector<double> chainTimes;
    for (const auto &pair : visualPairs) {
        if (!timeToIndex.contains(pair.t0)) {
            timeToIndex[pair.t0] = chainTimes.size();
            chainTimes.append(pair.t0);
        }
        if (!timeToIndex.contains(pair.t1)) {
            timeToIndex[pair.t1] = chainTimes.size();
            chainTimes.append(pair.t1);
        }
    }

    // chainTimes is already sorted (QMap iterates in order)
    const int chainSize = chainTimes.size();
    QVector<QQuaternion> qVis(chainSize, QQuaternion(1, 0, 0, 0));

    // Anchor: IMU orientation at the first chain time (mapped to IMU time)
    double anchorImuTime = chainTimes.first() * (1.0 + drift) + syncOffset;
    qVis[0] = sampleImuOrientation(imuOrientations, imuTimestamps, anchorImuTime);

    // Chain the visual rotations as world-frame axis-angle (rotation-vector)
    // accumulation instead of composing quaternions per pair. The bearing solve
    // returns R with Q_B = Q_A * R^-1 (a body-frame incremental rotation);
    // its world-frame incremental rotation vector is -Q_A (log R) Q_A^-1,
    // conjugate-transformed by the current absolute orientation. Accumulating
    // these rotation vectors linearly is exact for single-axis paths (e.g. a
    // full 360° roll sums to exactly 360°) and robust to per-pair axis wobble,
    // which otherwise causes quaternion composition to eat into the total
    // (measured: a 360° clip composed to ~54°). Unreliable pairs (poor inliers
    // / high RMS) are rejected rather than scaled, so the total is not
    // distorted.
    QVector<VisualRotationPair> sortedPairs = visualPairs;
    std::sort(sortedPairs.begin(), sortedPairs.end(),
              [](const VisualRotationPair &a, const VisualRotationPair &b) {
                  return a.t0 < b.t0;
              });

    // Track which pair provides the quality for each chain time
    // (use the pair where this time appears as t1, or t0 for the anchor)
    QVector<int> pairIndexForTime(chainSize, -1);
    pairIndexForTime[0] = -2; // anchor (no pair)

    QVector3D worldRotVec(0.0f, 0.0f, 0.0f);
    for (int pi = 0; pi < sortedPairs.size(); pi++) {
        const auto &pair = sortedPairs[pi];
        int idx0 = timeToIndex.value(pair.t0, -1);
        int idx1 = timeToIndex.value(pair.t1, -1);
        if (idx0 < 0 || idx1 < 0)
            continue;

        // Reject unreliable pairs: too few inliers or too high a fit residual.
        if (pair.inliers < 50 || pair.rmsDeg > 15.0) {
            pairIndexForTime[idx1] = pi;
            continue;
        }

        // log(R): body-frame incremental rotation vector (degrees).
        QQuaternion qn = pair.deltaR.normalized();
        if (qn.scalar() < 0.0f)
            qn = QQuaternion(-qn.scalar(), -qn.x(), -qn.y(), -qn.z());
        double w = qBound(-1.0, (double)qn.scalar(), 1.0);
        double angleRad = 2.0 * std::acos(w);
        double sinHalf = std::sin(angleRad * 0.5);
        QVector3D bodyVec(0.0f, 0.0f, 0.0f);
        if (sinHalf > 1e-10) {
            bodyVec = QVector3D(
                (float)(qn.x() / sinHalf * angleRad * 180.0 / M_PI),
                (float)(qn.y() / sinHalf * angleRad * 180.0 / M_PI),
                (float)(qn.z() / sinHalf * angleRad * 180.0 / M_PI));
        }

        // Q_B = Q_A * R^-1  =>  world-frame incremental rotation vector is
        // Q_A * (-log R) * Q_A^-1  (rotate body vector into the anchor frame).
        QVector3D worldInc = qVis[idx0].rotatedVector(-bodyVec);
        worldRotVec += worldInc;

        // World-frame rotation vector -> quaternion.
        float len = worldRotVec.length();
        if (len > 1e-6f) {
            QVector3D ax = worldRotVec / len;
            qVis[idx1] = QQuaternion::fromAxisAndAngle(ax, len).normalized();
        } else {
            qVis[idx1] = QQuaternion(1, 0, 0, 0);
        }
        pairIndexForTime[idx1] = pi;
    }

    // --- Step 2: Compute correction knots ---
    // At each chain time t (video time):
    //   tImu = t * (1 + drift) + syncOffset
    //   Q_gyro = sampleImuOrientation at tImu
    //   C(t) = Q_vis(t) * Q_gyro^-1
    //   quality = inliers / (1 + rmsDeg) from the associated visual pair

    m_knots.reserve(chainSize);
    for (int i = 0; i < chainSize; i++) {
        double tVideo = chainTimes[i];
        double tImu = tVideo * (1.0 + drift) + syncOffset;
        QQuaternion qGyro = sampleImuOrientation(imuOrientations, imuTimestamps, tImu);

        // C(t) = Q_vis(t) * Q_gyro^-1
        QQuaternion qGyroInv = qGyro.conjugated();
        QQuaternion correction = (qVis[i] * qGyroInv).normalized();

        // Quality from the associated visual pair
        double quality = 1.0;
        int pi = pairIndexForTime[i];
        if (pi >= 0 && pi < sortedPairs.size()) {
            const auto &pair = sortedPairs[pi];
            quality = pair.inliers / (1.0 + pair.rmsDeg);
        } else if (pi == -2) {
            // Anchor time: use quality from the first pair
            if (!sortedPairs.isEmpty()) {
                const auto &pair = sortedPairs.first();
                quality = pair.inliers / (1.0 + pair.rmsDeg);
            }
        }

        m_knots.append({tVideo, correction, quality});
    }

    // Normalize qualities so max = 1.0
    double maxQuality = 0.0;
    for (const auto &k : m_knots)
        maxQuality = qMax(maxQuality, k.quality);
    if (maxQuality > 0.0) {
        for (auto &k : m_knots)
            k.quality /= maxQuality;
    }

    // --- Step 3: Edge case checks ---

    // Check for large discontinuities between adjacent knots
    bool hasDiscontinuity = false;
    for (int i = 1; i < m_knots.size(); i++) {
        double dot = std::abs(QQuaternion::dotProduct(m_knots[i].correction,
                                                       m_knots[i - 1].correction));
        dot = qBound(0.0, dot, 1.0);
        double angleDeg = qRadiansToDegrees(2.0 * std::acos(dot));
        if (angleDeg > 30.0) {
            hasDiscontinuity = true;
            break;
        }
    }
    if (hasDiscontinuity) {
        qWarning() << "VisualFusion: large discontinuity in correction spline,"
                    << "reducing sigma from" << m_sigma << "to 0.5";
        m_sigma = 0.5;
    }

    // Check if visual chain drifts significantly from IMU (>45° at any knot)
    for (const auto &k : m_knots) {
        double w = std::abs(k.correction.scalar());
        w = qBound(0.0, w, 1.0);
        double angleDeg = qRadiansToDegrees(2.0 * std::acos(w));
        if (angleDeg > 45.0) {
            qWarning() << "VisualFusion: visual chain drifts" << angleDeg
                        << "degrees from IMU at t=" << k.time
                        << "(> 45° threshold), skipping fusion";
            m_knots.clear();
            return;
        }
    }

    qDebug() << "VisualFusion:" << m_knots.size() << "correction knots,"
             << "sigma =" << m_sigma << "s";

    // --- Step 4: Generate fused orientations at IMU timestamps ---
    const int n = imuTimestamps.size();
    m_fusedOrientations.reserve(n);
    m_fusedTimestamps.reserve(n);

    for (int i = 0; i < n; i++) {
        double tImu = imuTimestamps[i];
        QQuaternion qGyro = imuOrientations[i];

        // Map IMU time to video time for correction lookup
        double tVideo = (tImu - syncOffset) / (1.0 + drift);
        QQuaternion C = correctionAt(tVideo);

        // Q_fused = C * Q_gyro
        QQuaternion qFused = (C * qGyro).normalized();
        m_fusedOrientations.append(qFused);
        m_fusedTimestamps.append(tImu);
    }

    qDebug() << "VisualFusion: generated" << m_fusedOrientations.size()
             << "fused orientations";
}

// ---------------------------------------------------------------------------
// orientationAt — query fused orientation at arbitrary time (IMU time domain)
// ---------------------------------------------------------------------------
QQuaternion VisualFusion::orientationAt(double time) const
{
    if (m_fusedOrientations.isEmpty())
        return QQuaternion(1, 0, 0, 0);

    if (time <= m_fusedTimestamps.first())
        return m_fusedOrientations.first();
    if (time >= m_fusedTimestamps.last())
        return m_fusedOrientations.last();

    // Binary search for bracketing index
    const auto it = std::lower_bound(m_fusedTimestamps.begin(),
                                      m_fusedTimestamps.end(), time);
    int i = (it == m_fusedTimestamps.end())
                ? m_fusedTimestamps.size() - 1
                : static_cast<int>(it - m_fusedTimestamps.begin());
    if (i > 0 && m_fusedTimestamps[i] > time)
        i--;

    // Gaussian-weighted quaternion average with 33ms minimum window
    // (same as GyroscopeIntegrator::orientationAt)
    const float minWindowMs = 33.0f;
    const float windowMs = minWindowMs; // no extra smoothing for fused
    const float halfMs = windowMs / 2.0f;
    const float sigmaMs = windowMs / 4.0f;

    const double t0 = time - halfMs;
    const double t1 = time + halfMs;
    int lo = i, hi = i;
    while (lo > 0 && m_fusedTimestamps[lo] > t0) lo--;
    while (hi < m_fusedTimestamps.size() - 1 && m_fusedTimestamps[hi] < t1) hi++;

    // Markley-style Gaussian-weighted quaternion average
    const QQuaternion &ref = m_fusedOrientations[i];
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float wsum = 0.0f;
    const float invSigma2 = 1.0f / (sigmaMs * sigmaMs);

    for (int j = lo; j <= hi; j++) {
        double dt = (m_fusedTimestamps[j] - time) * 1000.0; // ms
        float weight = std::exp(-0.5f * static_cast<float>(dt * dt) * invSigma2);
        QQuaternion q = m_fusedOrientations[j];
        if (QQuaternion::dotProduct(q, ref) < 0.0f)
            q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
        x += q.x() * weight;
        y += q.y() * weight;
        z += q.z() * weight;
        w += q.scalar() * weight;
        wsum += weight;
    }

    if (wsum <= 0.0f)
        return m_fusedOrientations[i];

    QQuaternion avg(w, x, y, z);
    avg /= wsum;
    avg.normalize();
    return avg;
}

QVector<QQuaternion> VisualFusion::fusedOrientations() const
{
    return m_fusedOrientations;
}

QVector<double> VisualFusion::fusedTimestamps() const
{
    return m_fusedTimestamps;
}
