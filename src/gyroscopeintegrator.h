// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef GYROSCOPEINTEGRATOR_H
#define GYROSCOPEINTEGRATOR_H

#include <QObject>
#include <QVector>
#include <QQuaternion>
#include <QMatrix3x3>
#include <QVector3D>
#include "imuparser.h"
#include <QtMath>
#include <cmath>

// The ONE definition of the video un-flip. The fisheye mapping in
// project.frag is rotated 180 deg from the physical lens (screen up maps to the
// BOTTOM of the fisheye circle), so the shader needs this on top of the IMU
// correction or every render comes out upside down. It used to be declared
// separately in gyroscopeintegrator.cpp, app.cpp and inline in app.h.
inline const QQuaternion &kFlipRollQ()
{
    static const QQuaternion q(0.0f, 0.0f, 0.0f, 1.0f);   // 180 deg about Z
    return q;
}

// HORIZON LOCK. Reduce a virtual-camera orientation to its heading alone: a
// pure rotation about world up (+Y), discarding pitch and roll.
//
// Working through the shader composition, the world direction a pixel shows is
// A_true(t) * A(t)^-1 * V * E * ray, where A is the gyro chain, A_true the real
// attitude, and V the UNFLIPPED virtual orientation. The view is level when
// V is a pure rotation about +Y AND the chain is accurate.
//
// So this fixes the DATUM, not the drift: it guarantees the virtual camera the
// output is measured against is level, at every instant rather than only at
// t=0 (which is all the old one-shot `levelling()` did -- in high-pass mode the
// operator's own tilt then passed straight through it). Residual horizon drift
// from chain error A_true * A^-1 is NOT removed by this; that is the
// accelerometer's job inside the Mahony filter, and the visual fusion's.
//
// Deliberate tilt is still available through pitch/roll and keyframes, which
// act in the stabilised frame on top of this.
inline QQuaternion yawOnly(const QQuaternion &v)
{
    // The view looks along -Z, so V * (0,0,-1) is where it points in the world.
    const QVector3D f = v.rotatedVector(QVector3D(0.0f, 0.0f, -1.0f));
    QVector3D fh(f.x(), 0.0f, f.z());
    if (fh.lengthSquared() < 1e-8f) {
        // Pointing at the pole: forward carries no heading. The view's up axis
        // is horizontal there, so use it instead -- this keeps the heading
        // continuous as the camera tips through vertical.
        const QVector3D u = v.rotatedVector(QVector3D(0.0f, 1.0f, 0.0f));
        fh = QVector3D(u.x(), 0.0f, u.z());
        if (f.y() > 0.0f) fh = -fh;
        if (fh.lengthSquared() < 1e-8f)
            return QQuaternion();
    }
    fh.normalize();
    // R_y(theta) * (0,0,-1) = (-sin theta, 0, -cos theta)
    const float deg = std::atan2(-fh.x(), -fh.z()) * 180.0f / float(M_PI);
    return QQuaternion::fromAxisAndAngle(0.0f, 1.0f, 0.0f, deg);
}

// The ONE definition of the quaternion handed to the shader. Preview
// (App::imuOrientationAt) and export (ExportSnapshot::stateAt) both call this,
// so they cannot drift apart.
//
//   q = kFlipRoll * (yawOnly(V) * F)^-1 * (A * F) = yawOnly(V)^-1 * A * F
//
// The chain stores A*F, so the leading kFlipRoll cancels one of the two baked
// copies and the un-flip survives on the right, applied last in camera frame.
// Applying it inside the chain alone does NOT work: it appears on both factors
// and cancels completely, which is what left every render 180 deg rotated.
inline QQuaternion composeStabilisation(const QQuaternion &qActualStored,
                                        const QQuaternion &qVirtualStored,
                                        bool lockHorizon = true)
{
    const QQuaternion &F = kFlipRollQ();
    QQuaternion vStored = qVirtualStored;
    if (lockHorizon)
        vStored = yawOnly(qVirtualStored * F.conjugated()) * F;
    return F * vStored.conjugated() * qActualStored;
}

class GyroscopeIntegrator : public QObject
{
    Q_OBJECT
public:
    explicit GyroscopeIntegrator(QObject *parent = nullptr);

    // Integrate gyro+accel samples into a gravity-aligned orientation chain.
    // The accelerometer is fused (Mahony-style complementary filter) while the
    // camera is nearly still. The integration runs in the camera frame, where
    // the level-camera accel rotated by imuToCamera^-1 ≈ +Y, so the seed is
    // near-identity and the gyro/Mahony correction share one frame (no axis
    // flip). Each stored orientation is post-multiplied by a constant 180° roll
    // about the forward axis (kFlipRoll) to un-flip the YI camera's inherently
    // 180°-flipped fisheye video for display; the shader applies the conjugate,
    // so it samples kFlipRoll * Q_cam^-1 * ray. imuToCamera is the header's
    // IMU->camera quaternion (config[4..7]).
    // accelKp/accelKi default to ZERO: per-sample accelerometer feedback is
    // harmful on this camera's footage (see the re-level note in integrate()).
    // The parameters remain for experiments and for reproducing old chains.
    void integrate(const QVector<ImuSample> &samples, double sampleRate,
                   const QQuaternion &imuToCamera,
                   float accelKp = 0.0f, float accelKi = 0.0f,
                   const QMatrix3x3 &gyroMatrix = QMatrix3x3(),
                   const QVector3D &gyroBias = QVector3D());
    // smoothingMs: same semantics as orientationAt(); applied at the caller's
    // (video) frame rate to avoid 400 Hz -> 30 fps aliasing of high-freq jitter.
    // This returns the "virtual camera" path — a deliberately smooth trajectory
    // that the output video should follow.
    QQuaternion orientationAtTime(double time, float smoothingMs = 0.0f) const;

    // Returns the full-bandwidth, unsmoothed orientation at the given time
    // using slerp interpolation between the two bracketing integrated
    // quaternions. This is the actual camera orientation at the exposure
    // midpoint — never smooth this. The stabilization correction is:
    //   q_applied = q_virtual^{-1} * q_actual
    // where q_virtual comes from orientationAtTime() (smoothed) and q_actual
    // comes from this method (unsmoothed). High-frequency shake cancels
    // exactly while intentional motion follows the smooth virtual path.
    QQuaternion orientationAtTimeUnsmoothed(double time) const;

    // Pure interpolation over copied sample data, so orientations can be
    // evaluated from a worker thread (e.g. the exporter) without racing
    // against integrate() reallocating the vectors in the GUI thread.
    //
    // smoothingMs > 0: Gaussian-weighted quaternion average over a centered
    // time window of +-smoothingMs/2 around `time` (sigma = window/4). Applied
    // at the caller's sample rate (video fps), NOT the IMU rate, so no
    // high-frequency jitter aliases back into the 0-15 Hz band when
    // sub-sampled from 400 Hz to 30 fps. The window is clamped to >= 33 ms
    // (one 30 fps frame = ~13 IMU samples) at all settings — the frame
    // exposure time, giving ~11 dB jitter reduction for free with no lag (all
    // orientations are pre-computed, so a centered window looks ahead in the
    // array). Larger windows up to the caller's value add progressively more
    // high-frequency attenuation via the Gaussian profile (no box-filter side
    // lobes).
    static QQuaternion orientationAt(const QVector<QQuaternion> &orientations,
                                     const QVector<double> &timestamps,
                                     double time, float smoothingMs = 0.0f);

    QVector<QQuaternion> orientations() const { return m_orientations; }
    QVector<double> timestamps() const { return m_timestamps; }

    // Active chain (fused if available, else raw IMU). Sample from these when
    // the result must match orientationAtTime*() — e.g. the export snapshot,
    // which otherwise would stabilise against the raw gyro chain while the
    // preview uses the fused chain (the same no-op-looking mismatch).
    const QVector<QQuaternion> &activeOrientations() const
    { return m_useFused ? m_fusedOrientations : m_orientations; }
    const QVector<double> &activeTimestamps() const
    { return m_useFused ? m_fusedTimestamps : m_timestamps; }

    // Set fused orientations from VisualFusion. When set, orientationAtTime()
    // and orientationAtTimeUnsmoothed() use these instead of the IMU-only
    // orientations. This corrects yaw drift by fusing visual rotation
    // measurements with the IMU chain.
    void setFusedOrientations(const QVector<QQuaternion> &orientations,
                              const QVector<double> &timestamps);
    void clearFusedOrientations();
    bool hasFusedOrientations() const { return m_useFused; }

    // First orientation of the ACTIVE chain (fused if available, else raw IMU).
    // Hold-world-steady pins q_virtual to this so it is sampled from the same
    // chain as q_actual (orientationAtTimeUnsmoothed); otherwise the shake
    // cancels out and the correction looks like a no-op.
    QQuaternion firstOrientation() const;

    // Per-sample gravity trust in [0,1] from the Mahony gate (1 = the
    // accelerometer was reading clean gravity here, 0 = pure motion). Consumers
    // use it to decide WHERE the IMU's absolute attitude can be believed.
    const QVector<float> &gravityTrust() const { return m_trust; }

    void setForcedBias(const QVector3D &b) { m_forcedBias = b; m_haveForcedBias = true; }
    void setRelevelEnabled(bool on) { m_relevel = on; }


private:
    // Result of a single integration pass (forward or backward).
    struct PassResult {
        QVector<QQuaternion> orientations; // without kFlipRoll applied
        QVector<double> gateWeights;       // combined gate weight per sample
    };

    QVector3D computeGyroBias(const QVector<ImuSample> &samples) const;

    // Run a single Mahony filter pass over the samples, anchored at a
    // camera-still seed and integrating BOTH directions from it (backward into
    // the pre-seed region, forward to the end), so the whole chain is
    // continuous. forward==true scans the seed from the start of the array,
    // forward==false from the end (used only for diagnostic/debug comparisons —
    // the production path uses a single forward pass).
    // Orientations stored do NOT include kFlipRoll — that is applied after
    // integration.
    // seedAdjust is a WORLD-frame rotation applied on top of the
    // accelerometer-derived seed (identity = trust the seed as measured).
    PassResult integratePass(const QVector<ImuSample> &samples, double sampleRate,
                             const QQuaternion &imuToCamera,
                             float accelKp, float accelKi,
                             const QVector3D &bias, bool forward,
                             const QQuaternion &seedAdjust = QQuaternion());

    QVector<QQuaternion> m_orientations;
    QVector<double> m_timestamps;
    QVector<float> m_trust;
    double m_sampleRate;

    // Reproduction hooks (tests only): force the bias instead of measuring it,
    // and/or skip the whole-clip re-level, so a chain from an earlier build can
    // be rebuilt bit-for-bit to interpret keyframes authored against it.
    bool m_haveForcedBias = false;
    QVector3D m_forcedBias;
    bool m_relevel = true;


    // Fused orientations from VisualFusion (drop-in replacement for IMU chain)
    QVector<QQuaternion> m_fusedOrientations;
    QVector<double> m_fusedTimestamps;
    bool m_useFused = false;
};

#endif // GYROSCOPEINTEGRATOR_H
