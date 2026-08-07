#include "gyroscopeintegrator.h"
#include <QtMath>
#include <cmath>
#include <QDebug>
#include <iostream>

// Mahony-style complementary filter gains.
static const float kAccelKp    = 0.35f;   // proportional gain, rad/s per rad of tilt
static const float kGyroGate   = 60.0f;   // deg/s: below this the accel is trusted
static const float kMagLow     = 0.9f;    // |accel| window (g) for "camera still"
static const float kMagHigh    = 1.1f;
// World frame convention: -Y is gravity-up (inverted display "up" axis).
static const QVector3D kWorldUp(0.0f, -1.0f, 0.0f);

GyroscopeIntegrator::GyroscopeIntegrator(QObject *parent)
    : QObject(parent)
    , m_sampleRate(400.0)
{
}

QVector3D GyroscopeIntegrator::computeGyroBias(const QVector<ImuSample> &samples) const
{
    // Compute mean gyro during the first samples with low total rate
    // (camera should be still at the start).
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    int count = 0;
    const int maxBiasSamples = 400;  // 1 second at 400 Hz

    for (int i = 0; i < qMin(maxBiasSamples, samples.size()); i++) {
        float rate = std::abs(samples[i].gyro.x())
                   + std::abs(samples[i].gyro.y())
                   + std::abs(samples[i].gyro.z());
        if (rate < 30.0f) {  // low total rate -> stationary
            sumX += samples[i].gyro.x();
            sumY += samples[i].gyro.y();
            sumZ += samples[i].gyro.z();
            count++;
        }
    }

    if (count > 0)
        return QVector3D((float)(sumX / count), (float)(sumY / count), (float)(sumZ / count));
    return QVector3D(0.0f, 0.0f, 0.0f);
}

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate,
                                    const QQuaternion &imuToCamera)
{
    m_sampleRate = sampleRate;
    m_orientations.clear();
    m_timestamps.clear();

    if (samples.isEmpty()) return;

    m_orientations.reserve(samples.size());
    m_timestamps.reserve(samples.size());

    QVector3D bias = computeGyroBias(samples);
    qDebug() << "Gyro bias:" << bias.x() << bias.y() << bias.z() << "deg/s";

    // The parser emits s.gyro = (roll, pitch, yaw) = (gx, gy, gz), where
    // gy = pitch, gz = yaw, gx = roll. Map these to camera axes:
    // camera X <- pitch, camera Y <- yaw, camera Z <- roll.
    //
    // Accelerometer fusion: the accelerometer at rest reads the specific force
    // (~1 g, pointing away from gravity), which gives an absolute "which way is
    // up" reference that the gyro integration (which drifts) lacks. The raw
    // sensor accel is rotated into camera axes with the header's IMU->camera
    // quaternion: q^-1 * accel == +Y (up) for a level camera. The world frame
    // is fixed so that -Y is gravity-up (the display "up" axis), matching what
    // LensViewer/the shader feed on. The first accelerometer reading seeds the
    // orientation so the default 0/0/0 view is level, and a Mahony-style
    // proportional correction continuously nudges the predicted gravity
    // direction toward the measured one (gated to only trust the accel while
    // the camera is nearly still, so linear/centripetal acceleration during
    // fast motion does not corrupt the estimate).
    const QQuaternion qInv = imuToCamera.conjugated();

    QQuaternion current;
    {
        QVector3D a0 = qInv.rotatedVector(samples[0].accel);
        float m0 = a0.length();
        if (m0 > 0.5f && m0 < 2.0f)
            current = QQuaternion::rotationTo(a0 / m0, kWorldUp);
    }

    for (int i = 0; i < samples.size(); i++) {
        m_timestamps.append(samples[i].timestamp);
        m_orientations.append(current);

        if (i + 1 >= samples.size()) break;

        float dt = (float)(samples[i + 1].timestamp - samples[i].timestamp);
        if (dt <= 0.0f) dt = (float)(1.0 / m_sampleRate);

        float gx = samples[i].gyro.x() - bias.x();  // roll
        float gy = samples[i].gyro.y() - bias.y();  // pitch
        float gz = samples[i].gyro.z() - bias.z();  // yaw

        // Constant IMU->camera axis mapping (all signs positive)
        float cx = -gy;  // camera X <- pitch
        float cy = gz;  // camera Y <- yaw
        float cz = -gx;  // camera Z <- roll

        // Accelerometer gravity correction (body-frame angular velocity).
        QVector3D accel = qInv.rotatedVector(samples[i].accel);  // camera axes
        float accelMag = accel.length();
        float spinRate = std::sqrt(cx * cx + cy * cy + cz * cz);
        bool still = (accelMag > kMagLow && accelMag < kMagHigh
                      && spinRate < kGyroGate);
        if (still) {
            // Predicted gravity-up in the body frame vs measured. The accel
            // reads specific force (up), so up is +accel in camera axes.
            QVector3D upPred = current.conjugated().rotatedVector(kWorldUp);
            QVector3D upMeas = accel / accelMag;
            QVector3D err = QVector3D::crossProduct(upPred, upMeas); // rad
            // Add the correction to the body-frame rate (rad/s -> deg/s).
            const float radToDeg = 180.0f / M_PI;
            cx += err.x() * kAccelKp * radToDeg;
            cy += err.y() * kAccelKp * radToDeg;
            cz += err.z() * kAccelKp * radToDeg;
        }

        float gxRad = qDegreesToRadians(cx);
        float gyRad = qDegreesToRadians(cy);
        float gzRad = qDegreesToRadians(cz);

        float mag = std::sqrt(gxRad * gxRad + gyRad * gyRad + gzRad * gzRad);
        if (mag < 0.0001f) continue;

        QVector3D axis(gxRad / mag, gyRad / mag, gzRad / mag);
        float angle = mag * dt;
        QQuaternion delta = QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle));
        current = (current * delta).normalized();
    }

    qDebug() << "Gyro integrator:" << m_orientations.size() << "keyframes";
}

QQuaternion GyroscopeIntegrator::orientationAtTime(double time) const
{
    return orientationAt(m_orientations, m_timestamps, time);
}

QQuaternion GyroscopeIntegrator::orientationAt(const QVector<QQuaternion> &orientations,
                                               const QVector<double> &timestamps,
                                               double time)
{
    if (orientations.isEmpty()) return QQuaternion();

    if (time <= timestamps.first())
        return orientations.first();
    if (time >= timestamps.last())
        return orientations.last();

    for (int i = 0; i < timestamps.size() - 1; i++) {
        if (time >= timestamps[i] && time < timestamps[i + 1]) {
            float t = (float)((time - timestamps[i]) /
                              (timestamps[i + 1] - timestamps[i]));
            return QQuaternion::slerp(orientations[i], orientations[i + 1], t);
        }
    }

    return orientations.last();
}