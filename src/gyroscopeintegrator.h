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
    // camera is nearly still, so the world frame is fixed with -Y = gravity up
    // (the display "up" axis) and gyro pitch/roll drift is corrected. The
    // first sample seeds the orientation, making the default view level.
    // imuToCamera is the header's IMU->camera quaternion (config[4..7]); it is
    // used to rotate the raw accelerometer (which reads specific force) into
    // camera axes, where gravity-up is +Y for a level camera.
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
