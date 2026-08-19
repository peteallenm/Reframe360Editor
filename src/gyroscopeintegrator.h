#ifndef GYROSCOPEINTEGRATOR_H
#define GYROSCOPEINTEGRATOR_H

#include <QObject>
#include <QVector>
#include <QQuaternion>
#include "imuparser.h"

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
    // accelKp/accelKi are the Mahony complementary-filter gains. accelKi
    // (>0) accumulates the gravity-alignment error while the camera is still
    // and feeds it back to cancel gyro bias, eliminating slow roll drift.
    void integrate(const QVector<ImuSample> &samples, double sampleRate,
                   const QQuaternion &imuToCamera,
                   float accelKp = 0.35f, float accelKi = 0.005f);
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

private:
    QVector3D computeGyroBias(const QVector<ImuSample> &samples) const;

    QVector<QQuaternion> m_orientations;
    QVector<double> m_timestamps;
    double m_sampleRate;
};

#endif // GYROSCOPEINTEGRATOR_H
