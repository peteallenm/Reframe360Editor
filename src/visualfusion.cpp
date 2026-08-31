// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include "visualfusion.h"
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <QDebug>
#include <QMap>

// Pair-quality bar for the fusion chain. Matched to GyroCalibrator's
// MIN_PAIR_INLIERS / MAX_PAIR_RMS_DEG: the two stages fit the same measurements
// against the same IMU, so admitting different subsets meant the calibration
// and the drift correction disagreed about which pairs were real. The old
// fusion-only bar of 50 inliers rejected most pairs on shaky footage and left
// identity holes in the chain.
static constexpr int    kFusionMinInliers = 15;
static constexpr double kFusionMaxRmsDeg  = 15.0;

// Erratic-spline gate: 90th-percentile knot-to-knot step, in degrees. Real
// drift moves the correction by a fraction of a degree between adjacent knots
// (measured: <= 2 deg/pair on YIVR_0845 at every frame); a broken chain jumps
// tens of degrees.
static constexpr double kMaxKnotStepDeg = 20.0;

// Ceiling on how fast the drift correction may change. Real drift on the
// fastest clip measured moves ~20 deg/s; anything above this is a bad visual
// pair, and letting it through puts a visible flick in the output.
static constexpr double kMaxCorrectionSlewDegPerSec = 30.0;
// Ceiling on how fast the APPLIED correction may change per IMU sample. Real
// yaw drift on this sensor is ~1-2 deg/s even on a fast orbit; 4 keeps up with
// it while making the visual chain's frame-to-frame noise invisible.
static constexpr double kMaxAppliedSlewDegPerSec = 4.0;

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

    // Knots are time-sorted, so binary-search the +-3 sigma window instead of
    // scanning them all. This is called once per IMU sample when generating the
    // fused chain, so the old full scan was O(N_imu x N_knots) -- tens of
    // millions of exp() calls on a minute of footage -- plus a heap allocation
    // the size of the whole knot list on every call.
    const auto lo = std::lower_bound(m_knots.begin(), m_knots.end(),
                                     videoTime - threeSigma,
                                     [](const CorrectionKnot &k, double t) {
                                         return k.time < t;
                                     });
    const auto hi = std::upper_bound(m_knots.begin(), m_knots.end(),
                                     videoTime + threeSigma,
                                     [](double t, const CorrectionKnot &k) {
                                         return t < k.time;
                                     });

    if (lo >= hi) {
        // Outside every knot's support: fall back to the nearest knot. lo is
        // the first knot at or after the window, so the nearest is lo or lo-1.
        if (lo == m_knots.begin())
            return m_knots.first().correction;
        if (lo == m_knots.end())
            return m_knots.last().correction;
        const auto prev = lo - 1;
        return (videoTime - prev->time <= lo->time - videoTime)
               ? prev->correction : lo->correction;
    }

    // Markley-style Gaussian-weighted quaternion average: sign-flip each knot
    // to stay within 180 deg of the reference, accumulate weighted components,
    // normalize. The reference is the knot nearest videoTime, so the sign
    // disambiguation is anchored on the most influential sample rather than on
    // whichever knot happened to come first in the window.
    const CorrectionKnot *refKnot = &(*lo);
    double bestDist = std::abs(videoTime - lo->time);
    for (auto it = lo; it != hi; ++it) {
        const double d = std::abs(videoTime - it->time);
        if (d < bestDist) { bestDist = d; refKnot = &(*it); }
    }
    const QQuaternion &ref = refKnot->correction;

    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float wsum = 0.0f;
    const double invSigma2 = 1.0 / (m_sigma * m_sigma);

    for (auto it = lo; it != hi; ++it) {
        const double dt = videoTime - it->time;
        const double weight = std::exp(-0.5 * dt * dt * invSigma2) * it->quality;
        if (weight <= 1e-12)
            continue;
        QQuaternion q = it->correction;
        if (QQuaternion::dotProduct(q, ref) < 0.0f)
            q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
        const float fw = (float)weight;
        x += q.x() * fw;
        y += q.y() * fw;
        z += q.z() * fw;
        w += q.scalar() * fw;
        wsum += fw;
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
                         double sigmaSeconds,
                         const QVector<float> &imuTrust)
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

    // Collect the unique pair endpoints. The QMap orders them, so emitting
    // chainTimes FROM the map (rather than in pair-arrival order) is what
    // actually makes chainTimes sorted — correctionAt() binary-searches the
    // knots, and step 1 walks the pairs in time order assuming the chain is
    // monotonic.
    QMap<double, int> timeToIndex; // video time -> index in visual chain
    for (const auto &pair : visualPairs) {
        timeToIndex.insert(pair.t0, 0);
        timeToIndex.insert(pair.t1, 0);
    }
    QVector<double> chainTimes;
    chainTimes.reserve(timeToIndex.size());
    for (auto it = timeToIndex.begin(); it != timeToIndex.end(); ++it) {
        it.value() = chainTimes.size();
        chainTimes.append(it.key());
    }

    const int chainSize = chainTimes.size();
    QVector<QQuaternion> qVis(chainSize, QQuaternion(1, 0, 0, 0));
    // Which chain times the visual chain actually reaches. Times behind a
    // rejected pair are NOT reached and must not become correction knots —
    // leaving identity in qVis there (the previous behaviour) injected a
    // full-magnitude bogus correction that tripped both the discontinuity and
    // the 45 deg drift check, which is why fusion was skipped on every clip.
    QVector<bool> qVisSet(chainSize, false);

    auto imuAt = [&](double tVideo) {
        return sampleImuOrientation(imuOrientations, imuTimestamps,
                                    tVideo * (1.0 + drift) + syncOffset);
    };

    // Anchor: the IMU orientation at the first chain time. Every subsequent
    // knot is this anchor advanced by the visual increments, so C(anchor) is
    // identity by construction and C(t) measures pure accumulated disagreement.
    qVis[0] = imuAt(chainTimes.first());
    qVisSet[0] = true;

    // Chain the visual rotations as world-frame axis-angle (rotation-vector)
    // accumulation instead of composing quaternions per pair. The bearing solve
    // returns R with Q_B = Q_A * R^-1 (a body-frame incremental rotation);
    // its world-frame incremental rotation vector is -Q_A (log R) Q_A^-1,
    // conjugate-transformed by the current absolute orientation. Accumulating
    // these rotation vectors linearly is exact for single-axis paths (e.g. a
    // full 360° roll sums to exactly 360°) and robust to per-pair axis wobble,
    // which otherwise causes quaternion composition to eat into the total
    // (measured: a 360° clip composed to ~54°).
    //
    // The accumulator is relative to qBase, the absolute orientation at the
    // start of the current unbroken run of pairs, and the result is composed
    // back onto it: qVis[i] = exp(worldRotVec) * qBase. Omitting that final
    // composition (the original bug) left qVis holding the DELTA since the
    // anchor, so C = qVis * qGyro^-1 carried the camera's absolute attitude —
    // ~180 deg on every clip, because the stored IMU chain is flipped.
    QVector<VisualRotationPair> sortedPairs = visualPairs;
    std::sort(sortedPairs.begin(), sortedPairs.end(),
              [](const VisualRotationPair &a, const VisualRotationPair &b) {
                  return a.t0 < b.t0;
              });

    // Track which pair provides the quality for each chain time
    // (use the pair where this time appears as t1, or t0 for the anchor)
    QVector<int> pairIndexForTime(chainSize, -1);
    pairIndexForTime[0] = -2; // anchor (no pair)

    // Rotation vector (degrees) -> quaternion.
    auto expDeg = [](const QVector3D &v) {
        const float len = v.length();
        if (len <= 1e-6f)
            return QQuaternion(1, 0, 0, 0);
        return QQuaternion::fromAxisAndAngle(v / len, len).normalized();
    };

    QQuaternion qBase = qVis[0];
    QVector3D worldRotVec(0.0f, 0.0f, 0.0f);
    int lastIdx = 0;            // last chain index the visual chain reached
    int rejected = 0, gaps = 0;
    double totalRotationDeg = 0.0;   // how far the camera actually turned

    for (int pi = 0; pi < sortedPairs.size(); pi++) {
        const auto &pair = sortedPairs[pi];
        int idx0 = timeToIndex.value(pair.t0, -1);
        int idx1 = timeToIndex.value(pair.t1, -1);
        if (idx0 < 0 || idx1 < 0)
            continue;

        // Reject unreliable pairs: too few inliers or too high a fit residual.
        if (!pair.isReliable(kFusionMinInliers, kFusionMaxRmsDeg)) {
            pairIndexForTime[idx1] = pi;
            rejected++;
            continue;
        }

        // The producer drops a pair entirely when no hop length solves, so the
        // chain can have holes. Across a hole there is no visual information:
        // re-base on the IMU while CARRYING the correction accumulated so far,
        // which lets the gyro fill the gap without discarding the drift
        // correction earned before it.
        if (!qVisSet[idx0]) {
            const QQuaternion C = qVis[lastIdx] * imuAt(chainTimes[lastIdx]).conjugated();
            qVis[idx0] = (C * imuAt(chainTimes[idx0])).normalized();
            qVisSet[idx0] = true;
            qBase = qVis[idx0];
            worldRotVec = QVector3D(0.0f, 0.0f, 0.0f);
            gaps++;
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

        // Q_B = Q_A * R  =>  world-frame incremental rotation vector is
        // Q_A * (log R) * Q_A^-1 (rotate the body vector into the world frame,
        // using the ABSOLUTE orientation at t0 — the un-anchored delta that
        // used to be here made the transport frame wrong for every pair after
        // the first).
        //
        // The sign is measured, not derived. The old code used -bodyVec on the
        // reasoning that the bearing solve returns R with Q_B = Q_A * R^-1;
        // comparing each pair's R against the IMU's own increment over the same
        // interval in the same frame (dImu = S(t0)^-1 * S(t1), `--fusion`
        // diagnostic) says otherwise, and not marginally:
        //
        //     JustYaw   |dImu vs R| = 0.42 deg   |dImu vs R^-1| = 10.72 deg
        //     YIVR_0845 |dImu vs R| = 4.00 deg   |dImu vs R^-1| = 20.73 deg
        //     (mean per-pair motion 5.3 deg and 10.8 deg respectively)
        //
        // R^-1 is wrong by twice the motion, i.e. it rotates the visual chain
        // BACKWARDS. That is what drove the chain ~180 deg from the IMU once a
        // clip had turned ~90 deg, and it is the real reason fusion was skipped
        // on every clip. It is also, almost certainly, the "360 deg clip
        // composes to ~54 deg" measurement that motivated the rotation-vector
        // accumulation in the first place.
        worldRotVec += qVis[idx0].rotatedVector(bodyVec);
        totalRotationDeg += bodyVec.length();

        qVis[idx1] = (expDeg(worldRotVec) * qBase).normalized();
        qVisSet[idx1] = true;
        lastIdx = idx1;
        pairIndexForTime[idx1] = pi;
    }

    qDebug() << "VisualFusion: chained" << chainSize << "times,"
             << rejected << "pairs rejected," << gaps << "chain gaps re-based on IMU";

    // --- Step 1b: anchor the visual chain where the IMU can be BELIEVED -----
    //
    // The visual chain only measures RELATIVE rotation; its absolute attitude
    // is whatever it was anchored to. Anchoring at the first pair (above) ties
    // it to the IMU at t~0 -- and on a clip that starts mid-motion, t~0 is
    // precisely where the IMU has no gravity reference and its attitude is
    // least trustworthy. On YIVR_0845 that inherited a ~90 deg error into
    // every fused frame of the first half minute.
    //
    // The IMU's attitude IS trustworthy wherever the Mahony gravity gate was
    // open (imuTrust ~ 1). So: measure the constant world-frame offset between
    // the two chains as a trust-weighted average over the whole clip, and move
    // the visual chain onto it. Where the IMU is verified the two now agree;
    // where it is not, the visual chain carries the gravity-verified attitude
    // there by relative rotation -- backwards through the start of the clip if
    // that is where the trust is missing.
    if (!imuTrust.isEmpty() && imuTrust.size() == imuOrientations.size()) {
        auto trustAt = [&](double tImu) -> float {
            const auto it = std::lower_bound(imuTimestamps.begin(), imuTimestamps.end(), tImu);
            int i = (it == imuTimestamps.end()) ? imuTimestamps.size() - 1
                                                : (int)(it - imuTimestamps.begin());
            return imuTrust[qBound(0, i, imuTrust.size() - 1)];
        };
        // Trust-weighted Markley mean of D_i = qGyro_i * qVis_i^-1.
        QQuaternion ref; bool haveRef = false; float bestW = 0.0f;
        QVector<QQuaternion> D(chainSize); QVector<float> W(chainSize, 0.0f);
        for (int i = 0; i < chainSize; i++) {
            if (!qVisSet[i]) continue;
            const double tImu = chainTimes[i] * (1.0 + drift) + syncOffset;
            const float w = trustAt(tImu);
            if (w <= 0.01f) continue;
            D[i] = (sampleImuOrientation(imuOrientations, imuTimestamps, tImu)
                    * qVis[i].conjugated()).normalized();
            W[i] = w;
            if (w > bestW) { bestW = w; ref = D[i]; haveRef = true; }
        }
        double x = 0, y = 0, z = 0, ww = 0, wsum = 0;
        for (int i = 0; i < chainSize; i++) {
            if (W[i] <= 0.0f) continue;
            QQuaternion q = D[i];
            if (QQuaternion::dotProduct(q, ref) < 0.0f)
                q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
            x += q.x() * W[i]; y += q.y() * W[i]; z += q.z() * W[i]; ww += q.scalar() * W[i];
            wsum += W[i];
        }
        // Need a real body of trusted knots (equivalent of >= 10 fully trusted).
        if (haveRef && wsum >= 10.0) {
            QQuaternion Dmean((float)ww, (float)x, (float)y, (float)z);
            Dmean.normalize();
            const double ang = 2.0 * std::acos(qBound(-1.0, (double)std::abs(Dmean.scalar()), 1.0))
                               * 180.0 / M_PI;
            for (int i = 0; i < chainSize; i++)
                if (qVisSet[i]) qVis[i] = (Dmean * qVis[i]).normalized();
            qDebug() << "VisualFusion: re-anchored the visual chain to the gravity-trusted IMU"
                     << "(trust mass" << wsum << "knots); moved it" << ang << "deg from the t0 anchor";
        } else {
            qDebug() << "VisualFusion: not enough gravity-trusted IMU to re-anchor (trust mass"
                     << wsum << "); keeping the t0 anchor";
        }
    }

    // --- Step 2: Compute correction knots ---
    // At each chain time t (video time):
    //   tImu = t * (1 + drift) + syncOffset
    //   Q_gyro = sampleImuOrientation at tImu
    //   C(t) = Q_vis(t) * Q_gyro^-1
    //   quality = inliers / (1 + rmsDeg) from the associated visual pair

    m_knots.reserve(chainSize);
    for (int i = 0; i < chainSize; i++) {
        // Skip times the visual chain never reached (behind a rejected pair).
        if (!qVisSet[i])
            continue;

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

    if (m_knots.size() < 10) {
        qWarning() << "VisualFusion: only" << m_knots.size()
                   << "usable correction knots (need >= 10), skipping fusion";
        m_knots.clear();
        return;
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
    //
    // Both checks measure the correction RELATIVE TO THE FIRST KNOT. C is a
    // correction, not an attitude, so only its change is meaningful; testing
    // |C| absolutely (the previous behaviour) also measured any constant frame
    // offset between the two chains and could never pass.
    auto angleBetween = [](const QQuaternion &a, const QQuaternion &b) {
        double dot = std::abs(QQuaternion::dotProduct(a, b));
        dot = qBound(0.0, dot, 1.0);
        return qRadiansToDegrees(2.0 * std::acos(dot));
    };

    // A LARGE correction is not a broken one. Measured against hand-authored
    // reference keyframes on YIVR_0845 (`--groundtruth`), over 33 s the gyro
    // chain's horizon error averages 96.5 deg while the visual chain's averages
    // 31.2 deg — the visual chain is closer at every single keyframe. The
    // correction needed there is therefore genuinely ~100 deg, and any gate
    // that rejects on magnitude throws away precisely the drift correction this
    // class exists to provide.
    //
    // What actually distinguishes a usable correction from a broken one is
    // whether it is SMOOTH. Real drift accumulates gradually; a broken visual
    // chain (mismatched features, a bad pair slipping through) jumps. So gate
    // on the distribution of adjacent-knot steps, not on the total.
    // Drop isolated outlier knots before judging the spline. One bad visual
    // pair puts a single knot far off the trajectory its neighbours agree on;
    // smoothing does not remove it, it smears it across +-3 sigma. Detect it as
    // "far from BOTH neighbours while the neighbours agree with each other".
    if (m_knots.size() >= 3) {
        QVector<CorrectionKnot> kept;
        kept.reserve(m_knots.size());
        kept.append(m_knots.first());
        int dropped = 0;
        for (int i = 1; i < m_knots.size() - 1; i++) {
            const double toPrev = angleBetween(m_knots[i].correction, kept.last().correction);
            const double toNext = angleBetween(m_knots[i].correction, m_knots[i + 1].correction);
            const double across = angleBetween(kept.last().correction,
                                               m_knots[i + 1].correction);
            if (toPrev > kMaxKnotStepDeg && toNext > kMaxKnotStepDeg
                && across < 0.5 * (toPrev + toNext)) {
                dropped++;
                continue;
            }
            kept.append(m_knots[i]);
        }
        kept.append(m_knots.last());
        if (dropped > 0) {
            qDebug() << "VisualFusion: dropped" << dropped
                     << "outlier knots that disagreed with both neighbours";
            m_knots = kept;
        }
    }

    // Slew-limit the correction. It represents DRIFT, which is slow by
    // definition: on the most violent clip measured the horizon error moves at
    // about 20 deg/s. A step of 36 deg between knots 0.07 s apart is 500 deg/s
    // -- a bad visual pair, not drift, and it reaches the screen as a one-frame
    // flick (measured 5.2 deg/sample against the gyro chain's 1.6). Clamping
    // the rate keeps every genuine correction while making a single bad pair
    // cost only a fraction of a degree.
    {
        int limited = 0;
        for (int i = 1; i < m_knots.size(); i++) {
            const double dt = qMax(1e-3, m_knots[i].time - m_knots[i - 1].time);
            const double maxStepDeg = kMaxCorrectionSlewDegPerSec * dt;
            const double step = angleBetween(m_knots[i].correction,
                                             m_knots[i - 1].correction);
            if (step > maxStepDeg) {
                const float t = (float)(maxStepDeg / step);
                m_knots[i].correction = QQuaternion::slerp(m_knots[i - 1].correction,
                                                           m_knots[i].correction, t)
                                        .normalized();
                limited++;
            }
        }
        if (limited > 0)
            qDebug() << "VisualFusion: slew-limited" << limited << "of"
                     << m_knots.size() << "knots to"
                     << kMaxCorrectionSlewDegPerSec << "deg/s";
    }

    QVector<double> steps;
    steps.reserve(m_knots.size());
    for (int i = 1; i < m_knots.size(); i++)
        steps.append(angleBetween(m_knots[i].correction, m_knots[i - 1].correction));

    double medStep = 0.0, p90Step = 0.0, maxStep = 0.0;
    if (!steps.isEmpty()) {
        QVector<double> sorted = steps;
        std::sort(sorted.begin(), sorted.end());
        medStep = sorted[sorted.size() / 2];
        p90Step = sorted[qMin(sorted.size() - 1, (int)(sorted.size() * 0.9))];
        maxStep = sorted.last();
    }

    if (p90Step > kMaxKnotStepDeg) {
        qWarning() << "VisualFusion: correction spline is erratic (90th-percentile"
                   << "knot-to-knot step" << p90Step << "deg, median" << medStep
                   << "deg, max" << maxStep << "deg) -- the visual chain is"
                   << "unreliable here; skipping fusion";
        m_knots.clear();
        return;
    }

    // Anything still large after outlier removal is a genuine kink in the
    // trajectory. Shorten sigma so the spline does not smear it sideways --
    // and only ever shorten, never lengthen (the old code assigned 0.5
    // unconditionally, which RAISED sigma once the default came down).
    if (maxStep > 30.0 && m_sigma > 0.3) {
        qWarning() << "VisualFusion: residual discontinuity of" << maxStep
                   << "deg in the correction spline, reducing sigma from"
                   << m_sigma << "to 0.3";
        m_sigma = 0.3;
    }

    const QQuaternion C0 = m_knots.first().correction;
    double maxDev = 0.0, maxDevTime = 0.0;
    for (const auto &k : m_knots) {
        const double dev = angleBetween(k.correction, C0);
        if (dev > maxDev) { maxDev = dev; maxDevTime = k.time; }
    }
    qDebug() << "VisualFusion: correction reaches" << maxDev << "deg at t=" << maxDevTime
             << "; knot steps median" << medStep << "p90" << p90Step << "max" << maxStep
             << "deg over" << totalRotationDeg << "deg turned";

    // --- Step 3b: keep only the YAW component of each correction --------------
    //
    // The IMU chain's roll and pitch are now anchored to gravity (bias measured
    // only when still, gyro-only integration, slow world-frame re-level) and
    // render level end to end on the hardest clip. The visual chain, by
    // contrast, accumulates axis error at speed (20-36 deg/s of divergence in
    // the fastest sections of YIVR_0845) — so a full 3-axis correction would
    // write the VISUAL chain's tilt error into frames that were already right.
    //
    // Yaw is the one axis gravity cannot observe, so that is where the visual
    // chain has something to add. Decompose each correction about world +Y
    // (swing-twist) and keep the twist. The stored chain carries kFlipRoll on
    // the right, which does not change a WORLD-frame (left-multiplied) yaw.
    if (!qgetenv("RENDER360_FUSION_FULL").isEmpty()) {
        qDebug() << "VisualFusion: FULL 3-axis correction (measurement hook)";
    } else {
        const QVector3D up(0.0f, 1.0f, 0.0f);
        for (auto &k : m_knots) {
            const QQuaternion &q = k.correction;
            const QVector3D v(q.x(), q.y(), q.z());
            const QVector3D proj = up * QVector3D::dotProduct(v, up);
            QQuaternion twist(q.scalar(), proj.x(), proj.y(), proj.z());
            if (twist.length() < 1e-8f) twist = QQuaternion(1, 0, 0, 0);
            k.correction = twist.normalized();
        }
        qDebug() << "VisualFusion: correction restricted to yaw (roll/pitch come from gravity)";
    }

    // --- Step 4: Generate fused orientations at IMU timestamps ---
    const int n = imuTimestamps.size();
    m_fusedOrientations.reserve(n);
    m_fusedTimestamps.reserve(n);

    // The correction is yaw DRIFT: on this sensor it accumulates at roughly a
    // degree or two per second at most. Anything faster is the visual chain's
    // noise, and applied to the output it reads as camera motion -- measured
    // before this limit: mean 6.7 deg/s, p99 40 deg/s, kicks >10 deg/s every
    // 0.4 s, which the user saw as a ~1 Hz jitter. Slerp each sample's
    // correction toward the previous one so its rate never exceeds
    // kMaxAppliedSlewDegPerSec.
    QQuaternion prevC; bool havePrev = false; double prevT = 0.0;
    int limitedCount = 0; double maxPostRate = 0.0;
    for (int i = 0; i < n; i++) {
        double tImu = imuTimestamps[i];
        QQuaternion qGyro = imuOrientations[i];

        // Map IMU time to video time for correction lookup
        double tVideo = (tImu - syncOffset) / (1.0 + drift);
        QQuaternion C = correctionAt(tVideo);
        if (havePrev) {
            const double dt = qMax(1e-4, tImu - prevT);
            const QQuaternion dC = (C * prevC.conjugated()).normalized();
            // Angle from the VECTOR part. For a 0.01 deg step the scalar part
            // is 1 - 4e-9, which float rounds to exactly 1 -- acos then says
            // the step is zero and the limit never engages until the step is
            // ~0.04 deg/sample (16 deg/s). asin(|v|) keeps full precision.
            const double sinHalf = qMin(1.0, (double)QVector3D(dC.x(), dC.y(), dC.z()).length());
            const double step = 2.0 * std::asin(sinHalf) * 180.0 / M_PI;
            const double maxStep = kMaxAppliedSlewDegPerSec * dt;
            static const bool kNoSlew = !qgetenv("RENDER360_FUSION_NOSLEW").isEmpty();   // measurement hook
            if (step > maxStep && !kNoSlew) {
                C = QQuaternion::slerp(prevC, C, (float)(maxStep / step)).normalized();
                limitedCount++;
            }
            {   // post-limit step, for the diagnostic below
                const QQuaternion d2 = (C * prevC.conjugated()).normalized();
                const double s2 = 2.0 * std::asin(qMin(1.0, (double)QVector3D(d2.x(), d2.y(), d2.z()).length())) * 180.0 / M_PI / dt;
                maxPostRate = qMax(maxPostRate, s2);
            }
        }
        prevC = C; prevT = tImu; havePrev = true;

        // Q_fused = C * Q_gyro
        QQuaternion qFused = (C * qGyro).normalized();
        m_fusedOrientations.append(qFused);
        m_fusedTimestamps.append(tImu);
    }

    qDebug() << "VisualFusion: generated" << m_fusedOrientations.size()
             << "fused orientations; slew limit engaged on" << limitedCount
             << "samples, max post-limit correction rate" << maxPostRate << "deg/s";
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

    const double halfSec = halfMs / 1000.0;   // timestamps are seconds
    const double t0 = time - halfSec;
    const double t1 = time + halfSec;
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
