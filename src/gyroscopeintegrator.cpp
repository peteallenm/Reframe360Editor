// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include "gyroscopeintegrator.h"
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <QDebug>
#include <QElapsedTimer>
#include <iostream>

// Mahony-style complementary filter gains. accelKp is the proportional gain
// (deg/s per rad of tilt error); accelKi is the integral gain, both passed in
// via integrate() so the user can tune them. The integral term accumulates the
// gravity-alignment error while the camera is still and feeds it back into the
// body rate, estimating and cancelling the gyro bias that would otherwise
// integrate into slow, unbounded orientation drift (most visible in roll).
// accelKi too high causes oscillation/overshoot during fast motion.
static const float kGyroGate   = 60.0f;   // deg/s: below this the accel is trusted
// IIR low-pass filter removed: quaternion integration is itself an integrator;
// sensor noise at 400 Hz integrates to almost nothing. The one-pole IIR
// (alpha=0.55, ~50 Hz cutoff) added ~2-3 ms of group delay which at 300°/s
// shake rate caused ~0.9° of instantaneous error — visible in 360 renders.
// World frame convention: +Y is the world "up" vector used to seed and correct
// the orientation (the Mahony filter drives current^-1 * kWorldUp toward the
// measured acceleration). For a level camera the accelerometer reads
// specific-force ≈ +Y in camera axes (after the IMU→camera quaternion), so the
// seed rotationTo(accel, +Y) is near-identity and the default view is level.
//
// kFlipRoll is a constant 180° roll about the forward (Z) axis that un-flips
// the YI camera's inherently 180°-flipped fisheye video (with stabilization
// OFF the video is upside down). It is applied ONLY when storing each
// orientation (current * kFlipRoll), never inside the seed or the gyro
// integration: baking it into the seed would rotate the body frame the gyro
// integrates in, which flips the pitch/yaw stabilization direction and makes
// the accelerometer correction fight the gyro.
static const QVector3D kWorldUp(0.0f, 1.0f, 0.0f);
// Frame tag on the stored chain (see kFlipRollQ() in the header, which is now
// the single definition; the composition applies it once, on the outside).
static const QQuaternion &kFlipRoll = kFlipRollQ();
// Seed cost penalty for a full clip-length of distance from the start.
// Measurement hook: RENDER360_GYRO_ROT="x,y,z" (degrees, camera frame) applies a
// small fixed rotation to the gyro vector to probe cross-axis / mounting error.
// Time constant of the accelerometer-derived tilt datum. Long enough that
// centripetal/linear acceleration lasting seconds averages out; short enough
// to follow gyro tilt drift, which is ~1-2 deg/min on this sensor.
static const double kRelevelSigmaSec = 30.0;

static double earlyWeight()
{
    static const double v = []() {
        const QByteArray e = qgetenv("RENDER360_EARLY_WEIGHT");   // measurement hook
        return e.isEmpty() ? 5.0 : e.toDouble();
    }();
    return v;
}
// Length of the WORLD-frame accelerometer average feeding the Mahony gravity
// correction. Gravity is constant in the world frame; linear and centripetal
// acceleration are not, so averaging there recovers gravity -- but only over a
// window long enough for the offending acceleration to turn. On an orbiting
// camera the centripetal vector only completes a turn over the ORBIT period,
// so the old 0.5 s never cancelled it and the magnitude gate stayed shut.
//
// Measured as the growth in chain-vs-gravity tilt error over each clip
// (tracking_tests --tilt), 0.5 s -> 5 s:
//     YIVR_0845   +62.4 deg  ->  -16.6 deg
//     YIVR_0807   +36.9      ->  +30.8
//     JustRoll    +46.1      ->  +31.4
//     JustYaw      +6.1      ->   +5.2
//     YIVR_0830   15.1 abs   ->    8.6 abs
// Uniformly better, with the start-of-clip attitude unchanged (chain-vs-accel
// at t=0 stays 0.0-0.3 deg on every clip). Overridable for measurement.
static double accelAvgSeconds()
{
    static const double v = []() {
        const double d = qgetenv("RENDER360_ACCEL_AVG").toDouble();
        return (d > 0.0) ? d : 5.0;
    }();
    return v;
}

GyroscopeIntegrator::GyroscopeIntegrator(QObject *parent)
    : QObject(parent)
    , m_sampleRate(400.0)
{
}

QVector3D GyroscopeIntegrator::computeGyroBias(const QVector<ImuSample> &samples) const
{
    // The zero-rate offset of a MEMS gyro is a fraction of a degree per second
    // (this sensor: |b| ~ 0.3-0.8 deg/s on the clips that do have a still
    // moment). It can only be READ when the camera is genuinely still, because
    // the mean gyro over a window that is moving is the motion, not the bias.
    //
    // The previous version took the quietest 0.5 s window in the first 20 s
    // and used its mean unconditionally. On YIVR_0845 -- an orbit that never
    // stops -- that window was doing 17 deg/s, so 17 deg/s was subtracted from
    // every gyro sample for the whole clip: ~1000 deg of fabricated rotation
    // per minute, which is most of the "drift that gets bad after a minute".
    // Below, a window only counts as still if it is still in BOTH senses:
    // low mean rate (not moving) AND low rate variability (not shaking about a
    // moving mean). If nothing in the clip qualifies, the honest answer is
    // that the bias is unmeasurable here; return zero and let the Mahony
    // integral term (accelKi) estimate it slowly where gravity is visible.
    const int winSamples = qMax(1, (int)(0.5 * m_sampleRate));  // 0.5 s window
    if (samples.size() < winSamples)
        return QVector3D(0.0f, 0.0f, 0.0f);

    // Whole recording, not the first 20 s: the still moment on a clip that
    // starts mid-motion can be anywhere (or nowhere).
    const int n = samples.size();
    double bestScore = 1e30;
    QVector3D bestMean(0.0f, 0.0f, 0.0f);
    double bestRate = 0.0, bestStd = 0.0;

    QVector3D sum(0, 0, 0);
    double sumRate = 0.0, sumRate2 = 0.0;
    for (int i = 0; i < n; i++) {
        const float r = samples[i].gyro.length();
        sum += samples[i].gyro; sumRate += r; sumRate2 += (double)r * r;
        if (i >= winSamples) {
            const int j = i - winSamples;
            const float rj = samples[j].gyro.length();
            sum -= samples[j].gyro; sumRate -= rj; sumRate2 -= (double)rj * rj;
        }
        if (i + 1 < winSamples)
            continue;
        const double meanRate = sumRate / winSamples;
        const double var = qMax(0.0, sumRate2 / winSamples - meanRate * meanRate);
        const double score = meanRate + std::sqrt(var);   // still AND steady
        if (score < bestScore) {
            bestScore = score;
            bestMean = sum / (float)winSamples;
            bestRate = meanRate;
            bestStd = std::sqrt(var);
        }
    }

    // A still camera reads its bias plus noise: a few deg/s of mean rate at
    // most, with almost no spread. Anything above this is motion.
    static const double kMaxStillRateDegS = 3.0;
    static const double kMaxStillStdDegS  = 1.5;
    if (bestRate > kMaxStillRateDegS || bestStd > kMaxStillStdDegS) {
        qWarning() << "Gyro bias: no still window in clip (quietest 0.5 s has mean rate"
                   << bestRate << "deg/s, spread" << bestStd
                   << "deg/s) -- bias unmeasurable, using 0";
        return QVector3D(0.0f, 0.0f, 0.0f);
    }
    return bestMean;
}

GyroscopeIntegrator::PassResult
GyroscopeIntegrator::integratePass(const QVector<ImuSample> &samples, double sampleRate,
                                   const QQuaternion &imuToCamera,
                                   float accelKp, float accelKi,
                                   const QVector3D &bias, bool forward,
                                   const QQuaternion &seedAdjust)
{
    PassResult result;
    const int n = samples.size();
    if (n == 0) return result;

    result.orientations.resize(n);
    result.gateWeights.resize(n, 0.0f);

    const QQuaternion qInv = imuToCamera.conjugated();

    // Seed the orientation from the STILLEST moment anywhere in the clip.
    //
    // This used to scan only the first second, take the first sample that
    // passed rather than the best one, and otherwise fall back to "any of the
    // first 10 samples with |accel| > 0.3". On a clip that is already moving at
    // frame 0 — a bullet-time orbit, say — nothing passes the gate, so the
    // fallback seeded from an accelerometer reading dominated by CENTRIPETAL
    // acceleration. That is not gravity, and it produced a starting attitude
    // measured 156 deg from level on YIVR_0845 (against hand-authored reference
    // keyframes). Everything downstream then inherits that error, and the
    // gravity feedback spends the whole clip fighting it.
    //
    // Scanning the whole recording is safe because the pass integrates in BOTH
    // directions from the seed, so anchoring mid-clip is exactly what the
    // design already supports. Gravity is averaged over a short window whose
    // rotation rate is low, which is what makes body-frame averaging valid.
    int seedIdx = -1;
    QQuaternion seed;
    {
        const int winLen = qMax(1, (int)(0.25 * sampleRate));   // 250 ms
        const double nDur = (n > 1)
            ? (samples[n - 1].timestamp - samples[0].timestamp) : 0.0;
        double bestCost = 1e30;
        QVector3D bestAccel(0.0f, 1.0f, 0.0f);

        for (int k = 0; k + winLen <= n; k += qMax(1, winLen / 4)) {
            QVector3D accSum(0, 0, 0);
            double gyroSum = 0.0;
            for (int j = k; j < k + winLen; j++) {
                accSum += samples[j].accel;
                gyroSum += samples[j].gyro.length();
            }
            const QVector3D accMean = accSum / (float)winLen;
            const double gyroMean = gyroSum / winLen;

            // Reject windows that are rotating enough for the accelerometer to
            // be measuring anything other than gravity.
            if (gyroMean >= kGyroGate)
                continue;

            const QVector3D a0 = qInv.rotatedVector(accMean);
            const double mag = a0.length();
            if (mag < 0.85 || mag > 1.15)
                continue;

            // Prefer near-exactly 1 g (little linear acceleration), as close to
            // motionless as the clip offers, AND as early as possible.
            //
            // Earliness matters because everything before the seed is reached by
            // integrating BACKWARDS from it, accumulating drift on the way — and
            // hold-world-steady mode pins the virtual camera to the orientation
            // at t=0, so error there contaminates the entire clip. Seeding at
            // t=79 s on a 140 s clip (which the unweighted cost did on
            // YIVR_0845) means the start is 79 s of backward integration away.
            // The weight has to be heavy, not a tie-breaker. On YIVR_0845 a
            // weight of 1.0 still chose t=38 s (nothing earlier is as still),
            // and 38 s of backward integration left the chain 122 deg away from
            // what the accelerometer says at the video start. A merely
            // acceptable window at t=0.2 s beats an excellent one 38 s later.
            const double whenFrac = (nDur > 0.0)
                ? (samples[k + winLen / 2].timestamp - samples[0].timestamp) / nDur : 0.0;
            const double cost = std::abs(mag - 1.0) * 10.0
                              + gyroMean / kGyroGate
                              + whenFrac * earlyWeight();
            if (cost < bestCost) {
                bestCost = cost;
                bestAccel = a0 / (float)mag;
                seedIdx = k + winLen / 2;
            }
        }

        if (seedIdx >= 0) {
            seed = (seedAdjust * QQuaternion::rotationTo(bestAccel, QVector3D(0.0f, 1.0f, 0.0f)))
                       .normalized();
            qDebug() << "IMU seed: sample" << seedIdx
                     << QString("(t=%1s)").arg(samples[seedIdx].timestamp, 0, 'f', 2).toUtf8().constData()
                     << "cost" << bestCost << "up_cam"
                     << bestAccel.x() << bestAccel.y() << bestAccel.z();
        } else {
            // No window anywhere in the clip looks like still gravity. Use the
            // single sample closest to 1 g rather than an arbitrary early one.
            double best = 1e30;
            for (int k = 0; k < n; k++) {
                const QVector3D a0 = qInv.rotatedVector(samples[k].accel);
                const double d = std::abs((double)a0.length() - 1.0);
                if (d < best && a0.length() > 0.3f) {
                    best = d;
                    seed = QQuaternion::rotationTo(a0.normalized(), QVector3D(0.0f, 1.0f, 0.0f));
                    seedIdx = k;
                }
            }
            if (seedIdx < 0) { seedIdx = 0; seed = QQuaternion(1, 0, 0, 0); }
            seed = (seedAdjust * seed).normalized();
            qWarning() << "IMU seed: no still window in clip; closest-to-1g sample"
                       << seedIdx << "|accel|-1 =" << best;
        }
    }

    // Mahony integral term + world-frame accel averaging, shared across the
    // pass (both the backward and forward sweeps use the same state so the
    // bias estimate and gravity observation are continuous through the seed).
    QVector3D integralFB(0.0f, 0.0f, 0.0f);
    const int accelWindowSamples = qMax(1, (int)(accelAvgSeconds() * sampleRate));
    // Ring buffer with a running sum. This used to be a QVector that re-summed
    // all ~200 entries AND did a removeFirst() (an O(window) memmove) on every
    // one of ~62 000 samples -- about 12 M vector adds and 148 MB of memmove
    // per integration, on the GUI thread, and integrate() re-runs on every
    // Drift-corr slider movement.
    // The sum is kept in DOUBLE and re-derived exactly every few thousand
    // samples. The Mahony gate is exp(-((|a|-1)/0.08)^2) feeding back into the
    // orientation that produces the next |a|, so the chain amplifies tiny
    // perturbations: a float running sum drifting by 1e-5 moved the measured
    // start attitude by a few tenths of a degree. Doubles plus a periodic
    // resync keep the result identical run to run.
    QVector<QVector3D> accelRing(accelWindowSamples, QVector3D(0.0f, 0.0f, 0.0f));
    double accelSumX = 0.0, accelSumY = 0.0, accelSumZ = 0.0;
    int accelRingPos = 0;
    int accelRingCount = 0;
    int accelSinceResync = 0;
    constexpr int kAccelResyncInterval = 4096;

    // Per-sample Mahony correction given the current camera orientation `cur`
    // at (the neighborhood of) sample i. Outputs the measured body-rate
    // gyroCam (deg/s), the correction corr (deg/s incl. integral feedback) and
    // the trust gate in [0,1]. `dt` scales the integral accumulation.
    auto mahonySample = [&](int i, const QQuaternion &cur, float dt,
                            QVector3D &gyroCam, QVector3D &corr, float &gate) {
        QVector3D gyroCorr = samples[i].gyro - bias;
        gyroCam = qInv.rotatedVector(gyroCorr);
        float cx = gyroCam.x(), cy = gyroCam.y(), cz = gyroCam.z();

        // World-frame accel averaging: gravity is constant in world frame while
        // linear/centripetal acceleration averages toward zero.
        QVector3D accelCam = qInv.rotatedVector(samples[i].accel);
        QVector3D accelWorld = cur.rotatedVector(accelCam);
        if (accelRingCount == accelWindowSamples) {
            const QVector3D &old = accelRing[accelRingPos];   // evict the oldest
            accelSumX -= old.x(); accelSumY -= old.y(); accelSumZ -= old.z();
        } else {
            accelRingCount++;
        }
        accelRing[accelRingPos] = accelWorld;
        accelSumX += accelWorld.x(); accelSumY += accelWorld.y(); accelSumZ += accelWorld.z();
        accelRingPos = (accelRingPos + 1) % accelWindowSamples;
        if (++accelSinceResync >= kAccelResyncInterval) {
            accelSinceResync = 0;
            accelSumX = accelSumY = accelSumZ = 0.0;
            for (int k = 0; k < accelRingCount; k++) {
                const QVector3D &v = accelRing[k];
                accelSumX += v.x(); accelSumY += v.y(); accelSumZ += v.z();
            }
        }
        const double invN = 1.0 / accelRingCount;
        QVector3D accelWorldAvg((float)(accelSumX * invN),
                                (float)(accelSumY * invN),
                                (float)(accelSumZ * invN));
        QVector3D accelAvg = cur.conjugated().rotatedVector(accelWorldAvg);
        float accelMag = accelAvg.length();

        // Combined gate: Gaussian magnitude weight * spin-rate soft gate.
        float magWeight = std::exp(-std::pow((accelMag - 1.0f) / 0.08f, 2));
        float spinRate = std::sqrt(cx * cx + cy * cy + cz * cz);
        float spinGate = 0.0f;
        if (spinRate < kGyroGate)
            spinGate = 1.0f;
        else if (spinRate < 2.0f * kGyroGate)
            spinGate = 1.0f - (spinRate - kGyroGate) / kGyroGate;
        gate = magWeight * spinGate;

        QVector3D err(0.0f, 0.0f, 0.0f);
        if (gate > 0.0f) {
            QVector3D upPred = cur.conjugated().rotatedVector(kWorldUp);
            QVector3D upMeas = (accelMag > 1e-6f) ? (accelAvg / accelMag) : kWorldUp;
            float dotPU = QVector3D::dotProduct(upPred, upMeas);
            if (dotPU < -0.999f) {
                QVector3D perp = (std::abs(upPred.x()) < 0.9f)
                                 ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0);
                err = QVector3D::crossProduct(upPred, perp).normalized();
            } else {
                err = QVector3D::crossProduct(upPred, upMeas);
            }
            const float radToDeg = 180.0f / M_PI;
            integralFB += err * (accelKi * radToDeg) * dt;
        }

        // corr = proportional (gated) + integral (unconditional) correction.
        const float radToDeg = 180.0f / M_PI;
        corr = err * (accelKp * gate * radToDeg);
        corr += integralFB;
    };

    // === Backward sweep: from the seed anchor down to sample 0 ===
    // Orientation[i] is derived from orientation[i+1] by reversing the body
    // rotation over [i, i+1]: a backward step uses R((-gyro + corr) * dt), so
    // the +corr drives the older orientation toward measured gravity (the same
    // sensitivity as the forward sweep).
    QQuaternion cur = seed;
    result.orientations[seedIdx] = cur;
    QVector3D thPrevB(0, 0, 0), thPrevF(0, 0, 0);
    if (seedIdx > 0) {
        for (int i = seedIdx - 1; i >= 0; --i) {
            float dt = (float)(samples[i + 1].timestamp - samples[i].timestamp);
            if (dt <= 0.0f) dt = (float)(1.0 / sampleRate);
            QVector3D gyroCam, corr; float gate;
            mahonySample(i, cur, dt, gyroCam, corr, gate);
            result.gateWeights[i] = gate;
            QVector3D th((-gyroCam.x() + corr.x()) * dt,
                         (-gyroCam.y() + corr.y()) * dt,
                         (-gyroCam.z() + corr.z()) * dt);
            th += QVector3D::crossProduct(thPrevB, th) * (1.0f / 12.0f);
            thPrevB = QVector3D((-gyroCam.x() + corr.x()) * dt,
                                (-gyroCam.y() + corr.y()) * dt,
                                (-gyroCam.z() + corr.z()) * dt);
            float mag = th.length();
            if (mag > 1e-5f)
                cur = (cur * QQuaternion::fromAxisAndAngle(th / mag, mag)).normalized();
            result.orientations[i] = cur;
        }
    }

    float peakIntegral = 0.0f;
    // === Forward sweep: from the seed anchor to the end of the clip ===
    // (A forward step uses R((+gyro + corr) * dt).) Two-sided coverage from the
    // single seed means this pass is correct for the WHOLE clip — no more
    // garbage before/after the seed, which was what the blend was snapping on.
    cur = seed;
    for (int i = seedIdx; i < n - 1; ++i) {
        float dt = (float)(samples[i + 1].timestamp - samples[i].timestamp);
        if (dt <= 0.0f) dt = (float)(1.0 / sampleRate);
        QVector3D gyroCam, corr; float gate;
        mahonySample(i, cur, dt, gyroCam, corr, gate);
        result.gateWeights[i] = gate; // (overwrites seedIdx's default of 0)
        peakIntegral = qMax(peakIntegral, integralFB.length());
        QVector3D th((gyroCam.x() + corr.x()) * dt,
                     (gyroCam.y() + corr.y()) * dt,
                     (gyroCam.z() + corr.z()) * dt);
        {
            // Second-order (two-sample) coning correction. Composing one
            // small rotation per sample is exact only when the axis is fixed
            // between samples; when two axes oscillate together the true
            // rotation over the step picks up (1/12) th_prev x th, which
            // first-order integration silently drops and which accumulates as
            // a systematic drift about the third axis.
            th += QVector3D::crossProduct(thPrevF, th) * (1.0f / 12.0f);
        }
        thPrevF = QVector3D((gyroCam.x() + corr.x()) * dt,
                            (gyroCam.y() + corr.y()) * dt,
                            (gyroCam.z() + corr.z()) * dt);
        float mag = th.length();
        if (mag > 1e-5f)
            cur = (cur * QQuaternion::fromAxisAndAngle(th / mag, mag)).normalized();
        result.orientations[i + 1] = cur;
    }
    qDebug() << "Mahony integral term: peak" << peakIntegral << "deg/s, at end of clip"
             << integralFB.length() << "deg/s  (a real gyro bias is < 1)";

    return result;
}

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate,
                                    const QQuaternion &imuToCamera,
                                    float accelKp, float accelKi,
                                    const QMatrix3x3 &gyroMatrix,
                                    const QVector3D &gyroBias)
{
    m_sampleRate = sampleRate;
    m_orientations.clear();
    m_timestamps.clear();
    m_trust.clear();

    if (samples.isEmpty()) return;

    // Apply gyro calibration if non-trivial (matrix != identity or bias != zero).
    // ω_corrected = M · ω_raw + b
    // If M is identity and b is zero, use original samples (no copy needed).
    bool hasGyroCal = (gyroMatrix != QMatrix3x3()) || (gyroBias != QVector3D());
    QVector<ImuSample> correctedSamples;
    const QVector<ImuSample> *workSamples = &samples;
    if (hasGyroCal) {
        correctedSamples.reserve(samples.size());
        for (const auto &s : samples) {
            ImuSample cs = s;
            // ω_corrected = M · ω_raw + b
            float gx = s.gyro.x(), gy = s.gyro.y(), gz = s.gyro.z();
            cs.gyro = QVector3D(
                gyroMatrix(0,0)*gx + gyroMatrix(0,1)*gy + gyroMatrix(0,2)*gz + gyroBias.x(),
                gyroMatrix(1,0)*gx + gyroMatrix(1,1)*gy + gyroMatrix(1,2)*gz + gyroBias.y(),
                gyroMatrix(2,0)*gx + gyroMatrix(2,1)*gy + gyroMatrix(2,2)*gz + gyroBias.z()
            );
            correctedSamples.append(cs);
        }
        workSamples = &correctedSamples;
        qDebug() << "GyroIntegrator: applying gyro calibration matrix + bias";
    }

    const int n = workSamples->size();
    m_orientations.reserve(n);
    m_timestamps.reserve(n);

    // Single-pass integration. The forward Mahony pass is anchored at the
    // earliest still sample and integrates BOTH directions from that anchor
    // (backward into the pre-seed region, forward to the end), so the entire
    // chain is continuous and gravity-aligned at the seed.
    //
    // A two-pass (forward+backward) blend was tried here, but the accelerometer-
    // derived seed attitudes and the gyro-integrated trajectory disagree by up
    // to ~170° on these handheld clips (426° gyro rotation vs 44° gravity
    // change between anchors), so the two passes sit in different absolute
    // attitudes and any blend between them snaps — the source of the one-frame
    // "flickers". A single continuous pass cannot flicker. Slow drift that this
    // sacrifices is recovered by the visual-fusion correction (q_virtual).
    QVector3D bias = m_haveForcedBias ? m_forcedBias : computeGyroBias(*workSamples);
    qDebug() << "Gyro bias:" << bias.x() << bias.y() << bias.z() << "deg/s";

    PassResult fwd = integratePass(*workSamples, sampleRate, imuToCamera,
                                   accelKp, accelKi, bias, /*forward=*/true);

    // World-frame re-levelling from the accelerometer, at a time scale of tens
    // of seconds. This is the ONLY place the accelerometer touches the chain.
    //
    // The per-sample Mahony feedback that used to do this job was the cause of
    // the "horizon rolled 45-90 deg for the first 30 s" failure on YIVR_0845:
    // on a pole-swung camera the accelerometer reads centripetal force most of
    // the time, and whenever the gate cracked open the proportional term
    // (0.35 * 180/pi = 20 deg/s per radian of error) yanked the chain toward a
    // false "up". Rendering with the feedback disabled produced level frames
    // from 0.5 s to the end; with a tenth of the gain they were rolled 45 deg.
    // Gyro-only, the chain holds to within 2 deg of gravity over 100 s on that
    // clip, and 0807's residual drift drops from 10 to 2.6 deg.
    //
    // What the gyro cannot do is hold tilt over many minutes, so the absolute
    // datum comes from the accelerometer averaged where the gate trusts it,
    // over a window long enough (kRelevelSigmaSec) that short-lived
    // contamination averages out while genuine slow drift is still followed.
    // Each orientation is left-multiplied by the rotation taking the smoothed
    // world-frame gravity onto +Y at its own time.
    QElapsedTimer relevelClock; relevelClock.start();
    if (!m_relevel) {
        qDebug() << "IMU world re-level: disabled (reproduction mode)";
    } else {
        const QQuaternion qInvCam = imuToCamera.conjugated();
        // Trusted world-frame gravity samples, decimated to ~10 Hz for speed.
        struct GSample { double t; QVector3D g; float w; };
        QVector<GSample> gs;
        const int stride = qMax(1, (int)(sampleRate / 10.0));
        for (int i = 0; i < n; i += stride) {
            const float w = (i < fwd.gateWeights.size()) ? fwd.gateWeights[i] : 0.0f;
            if (w <= 0.01f) continue;
            const QVector3D a = qInvCam.rotatedVector((*workSamples)[i].accel);
            const float m = a.length();
            if (m < 1e-6f) continue;
            gs.append({(*workSamples)[i].timestamp, fwd.orientations[i].rotatedVector(a / m), w});
        }
        double wsum = 0.0;
        for (const auto &g : gs) wsum += g.w;
        const double trustedSeconds = wsum * stride / sampleRate;

        if (trustedSeconds < 2.0) {
            qWarning() << "IMU world re-level: only" << trustedSeconds
                       << "s of trusted gravity in the clip; keeping the seed as measured";
        } else {
            // Gaussian-smoothed gravity direction.
            //
            // Smoothed ON THE GRAVITY SAMPLES' OWN GRID (~10 Hz) and then
            // interpolated to each orientation, rather than evaluated afresh
            // for every one of them. The window is thirty seconds wide, so the
            // smoothed direction is band-limited to about 1/30 Hz -- sampling
            // it at the IMU's 400 Hz asks for a thousand times more detail than
            // it contains, and doing so was the whole cost of opening a clip
            // (1541 ms of a 1575 ms integrate on a 139 s clip). On the coarse
            // grid it is a few milliseconds, and linear interpolation between
            // points a tenth of a second apart is far finer than the signal.
            const double sigma = kRelevelSigmaSec;
            const double inv2s2 = 1.0 / (2.0 * sigma * sigma);
            QVector<QVector3D> smoothed(gs.size());
            {
                int lo = 0;
                for (int j = 0; j < gs.size(); ++j) {
                    const double t = gs[j].t;
                    while (lo < gs.size() && gs[lo].t < t - 3.0 * sigma) lo++;
                    QVector3D acc(0, 0, 0); double aw = 0.0;
                    for (int k = lo; k < gs.size() && gs[k].t <= t + 3.0 * sigma; k++) {
                        const double dt = gs[k].t - t;
                        const double w = gs[k].w * std::exp(-dt * dt * inv2s2);
                        acc += gs[k].g * (float)w; aw += w;
                    }
                    // Every point has at least itself in its own window, so aw
                    // is only ever zero for a zero-weight sample.
                    smoothed[j] = (aw > 1e-6 && acc.length() > 1e-4f) ? acc.normalized()
                                                                     : gs[j].g;
                }
            }

            int lo = 0;
            double maxTilt = 0.0, meanTilt = 0.0; int nt = 0;
            for (int i = 0; i < n; i++) {
                const double t = (*workSamples)[i].timestamp;
                // Bracketing pair on the coarse grid; the cursor only moves
                // forward because both series are sorted by time.
                while (lo + 1 < gs.size() && gs[lo + 1].t < t) lo++;
                QVector3D acc;
                if (t <= gs.first().t) {
                    acc = smoothed.first();          // before the trusted span
                } else if (t >= gs.last().t) {
                    acc = smoothed.last();           // after it
                } else {
                    const double span = gs[lo + 1].t - gs[lo].t;
                    const double f = (span > 1e-9) ? qBound(0.0, (t - gs[lo].t) / span, 1.0) : 0.0;
                    acc = smoothed[lo] * (float)(1.0 - f) + smoothed[lo + 1] * (float)f;
                }
                if (acc.length() < 1e-4f)
                    acc = smoothed[qBound(0, lo, smoothed.size() - 1)];
                acc.normalize();
                const double tilt = std::acos(qBound(-1.0,
                    (double)QVector3D::dotProduct(acc, kWorldUp), 1.0)) * 180.0 / M_PI;
                maxTilt = qMax(maxTilt, tilt); meanTilt += tilt; nt++;
                if (tilt > 0.05)
                    fwd.orientations[i] = (QQuaternion::rotationTo(acc, kWorldUp)
                                           * fwd.orientations[i]).normalized();
            }
            qDebug() << "IMU world re-level: gravity-trusted" << trustedSeconds
                     << "s; applied correction mean" << (nt ? meanTilt / nt : 0.0)
                     << "deg, max" << maxTilt << "deg, smoothed over sigma" << sigma << "s";
        }
    }

    m_trust.clear();
    m_trust.reserve(n);
    for (int i = 0; i < n; i++) {
        m_timestamps.append((*workSamples)[i].timestamp);
        QQuaternion q = (i < fwd.orientations.size())
                        ? fwd.orientations[i] : QQuaternion();
        // Apply the video un-flip (180° roll about Z) after integration.
        m_orientations.append(q * kFlipRoll);
        m_trust.append(i < fwd.gateWeights.size() ? fwd.gateWeights[i] : 0.0f);
    }


    qInfo("IMU timing: re-level %lld ms", relevelClock.elapsed());
    qDebug() << "Gyro integrator (single pass):" << m_orientations.size() << "keyframes";
}

QQuaternion GyroscopeIntegrator::orientationAtTime(double time, float smoothingMs) const
{
    // q_virtual (the deliberately smooth "virtual tripod" path). When visual
    // fusion is available its slow yaw/pitch/roll-drift correction belongs HERE
    // — the fused chain is a low-frequency-corrected version of the gyro chain,
    // and it is Gaussian-smoothed before use, so its knot noise is averaged
    // out. Falls back to the pure gyro chain when fusion is unavailable.
    if (m_useFused)
        return orientationAt(m_fusedOrientations, m_fusedTimestamps, time, smoothingMs);
    return orientationAt(m_orientations, m_timestamps, time, smoothingMs);
}

QQuaternion GyroscopeIntegrator::orientationAtTimeUnsmoothed(double time) const
{
    // q_actual (full-bandwidth, unsmoothed). This MUST be sampled from the SAME
    // chain as q_virtual (orientationAtTime): the two-signal correction is
    // q_virtual^{-1} * q_actual, so if one comes from the fused chain and the
    // other from the raw gyro chain, the high-frequency shake (present in both)
    // cancels out and only the slow visual drift correction survives — making
    // the stabilization look like a no-op. The fused chain is the gyro chain
    // with a low-frequency correction applied, so its shake content is
    // identical to the raw chain; sampling both paths from it keeps the shake
    // cancellation intact while q_virtual carries the drift correction.
    const QVector<QQuaternion> &oris = m_useFused ? m_fusedOrientations : m_orientations;
    const QVector<double> &ts = m_useFused ? m_fusedTimestamps : m_timestamps;

    if (oris.isEmpty()) return QQuaternion();

    if (time <= ts.first())
        return oris.first();
    if (time >= ts.last())
        return oris.last();

    // Binary search for the bracketing index
    const auto it = std::lower_bound(ts.begin(), ts.end(), time);
    int i = (it == ts.end()) ? ts.size() - 1
                             : (int)(it - ts.begin());
    if (i > 0 && ts[i] > time) i--;

    // Slerp between the two bracketing quaternions (no Gaussian smoothing)
    if (i + 1 >= oris.size())
        return oris.last();

    double dt = ts[i + 1] - ts[i];
    float t = (dt > 0.0) ? (float)((time - ts[i]) / dt) : 0.0f;
    t = qBound(0.0f, t, 1.0f);

    return QQuaternion::slerp(oris[i], oris[i + 1], t);
}

QQuaternion GyroscopeIntegrator::orientationAt(const QVector<QQuaternion> &orientations,
                                               const QVector<double> &timestamps,
                                               double time, float smoothingMs)
{
    if (orientations.isEmpty()) return QQuaternion();

    if (time <= timestamps.first())
        return orientations.first();
    if (time >= timestamps.last())
        return orientations.last();

    // Find the bracketing index with a binary search. This runs on every
    // preview/export frame and the IMU chain can be tens of thousands of
    // samples long, so a linear scan (O(n) per call) stalls playback even
    // though it never maxes a core.
    int i;
    {
        const auto it = std::lower_bound(timestamps.begin(), timestamps.end(), time);
        i = (it == timestamps.end()) ? timestamps.size() - 1
                                     : (int)(it - timestamps.begin());
        if (i > 0 && timestamps[i] > time) i--;
    }

    // Minimum window = one 30 fps video frame (~33 ms, ~13 IMU samples at 400 Hz)
    // so even at smoothingMs = 0 we average over the frame exposure time, giving
    // ~11 dB jitter reduction for free. Larger windows use a Gaussian profile
    // (sigma = window/4) so the frequency response rolls off monotonically with
    // no box-filter side lobes. All orientations are pre-computed, so this
    // centered window looks ahead in the array without any real-time latency.
    //
    // The user's smoothingMs sets the window and the 33 ms exposure average is
    // the floor. (The old rate-adaptive scaling between them was removed: it
    // starved slow pans of smoothing while offering no benefit on fast shakes.)
    const float minWindowMs = 33.0f;
    const float windowMs = qMax(minWindowMs, smoothingMs);
    const float halfMs = windowMs / 2.0f;
    const float sigmaMs = windowMs / 4.0f;

    // halfMs is MILLISECONDS; timestamps are SECONDS. Subtracting it directly
    // (the old code) made the bracket +-150 s at smoothing=1 and +-16.5 s even
    // at the 33 ms floor, so both walks ran to the ends of the array and the
    // average below iterated EVERY sample in the clip -- ~62 k instead of ~120
    // on a 154 s clip, once per preview frame and twice per exported frame.
    // The result was still correct (out-of-window weights underflow to zero);
    // only the cost was wrong, and it grew with clip length.
    const double halfSec = halfMs / 1000.0;
    const double t0 = time - halfSec;
    const double t1 = time + halfSec;
    int lo = i, hi = i;
    while (lo > 0 && timestamps[lo] > t0) lo--;
    while (hi < timestamps.size() - 1 && timestamps[hi] < t1) hi++;

    // Markley-style Gaussian-weighted quaternion average: accumulate the
    // quaternion vectors (sign-flipped so each is within 180° of the reference)
    // weighted by a Gaussian of the time offset, then normalize.
    const QQuaternion &ref = orientations[i];
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float wsum = 0.0f;
    const float invSigma2 = 1.0f / (sigmaMs * sigmaMs);
    for (int j = lo; j <= hi; j++) {
        double dt = (timestamps[j] - time) * 1000.0; // ms
        float weight = std::exp(-0.5f * (float)(dt * dt) * invSigma2);
        QQuaternion q = orientations[j];
        if (QQuaternion::dotProduct(q, ref) < 0.0f)
            q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
        x += q.x() * weight; y += q.y() * weight;
        z += q.z() * weight; w += q.scalar() * weight;
        wsum += weight;
    }
    if (wsum <= 0.0f)
        return orientations[i];

    QQuaternion avg(w, x, y, z);
    avg /= wsum;
    avg.normalize();
    return avg;
}

void GyroscopeIntegrator::setFusedOrientations(const QVector<QQuaternion> &orientations,
                                                const QVector<double> &timestamps)
{
    m_fusedOrientations = orientations;
    m_fusedTimestamps = timestamps;
    m_useFused = !orientations.isEmpty();
    if (m_useFused) {
        qDebug() << "GyroscopeIntegrator: using" << orientations.size()
                 << "fused orientations from VisualFusion";
    }
}

void GyroscopeIntegrator::clearFusedOrientations()
{
    m_fusedOrientations.clear();
    m_fusedTimestamps.clear();
    m_useFused = false;
}

QQuaternion GyroscopeIntegrator::firstOrientation() const
{
    const QVector<QQuaternion> &oris = m_useFused ? m_fusedOrientations : m_orientations;
    return oris.isEmpty() ? QQuaternion() : oris.first();
}
