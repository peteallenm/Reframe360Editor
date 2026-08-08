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
    void integrate(const QVector<ImuSample> &samples, double sampleRate,
                   const QQuaternion &imuToCamera);
    QQuaternion orientationAtTime(double time) const;

    // Pure interpolation over copied sample data, so orientations can be
    // evaluated from a worker thread (e.g. the exporter) without racing
    // against integrate() reallocating the vectors in the GUI thread.
    static QQuaternion orientationAt(const QVector<QQuaternion> &orientations,
                                     const QVector<double> &timestamps,
                                     double time);

    QVector<QQuaternion> orientations() const { return m_orientations; }
    QVector<double> timestamps() const { return m_timestamps; }

private:
    QVector3D computeGyroBias(const QVector<ImuSample> &samples) const;

    QVector<QQuaternion> m_orientations;
    QVector<double> m_timestamps;
    double m_sampleRate;
};

#endif // GYROSCOPEINTEGRATOR_H
