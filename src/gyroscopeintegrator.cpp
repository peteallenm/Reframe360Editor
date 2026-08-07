#include "gyroscopeintegrator.h"
#include <QtMath>
#include <cmath>
#include <QDebug>
#include <iostream>

GyroscopeIntegrator::GyroscopeIntegrator(QObject *parent)
    : QObject(parent)
    , m_sampleRate(200.0)
{
}

QVector3D GyroscopeIntegrator::computeGyroBias(const QVector<ImuSample> &samples) const
{
    // Compute mean gyro during the first samples with low total rate
    // (camera should be still at the start).
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    int count = 0;
    const int maxBiasSamples = 400;  // 2 seconds at 200 Hz

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

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate)
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
    QQuaternion current; // identity start; initial world orientation is irrelevant

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
        //std::cout << samples[i].timestamp << ": gx: " << gx << ", gy: " << gy << ", gz: " << gz << "\n";
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
    if (m_orientations.isEmpty()) return QQuaternion();

    if (time <= m_timestamps.first())
        return m_orientations.first();
    if (time >= m_timestamps.last())
        return m_orientations.last();

    for (int i = 0; i < m_timestamps.size() - 1; i++) {
        if (time >= m_timestamps[i] && time < m_timestamps[i + 1]) {
            float t = (float)((time - m_timestamps[i]) /
                              (m_timestamps[i + 1] - m_timestamps[i]));
            return QQuaternion::slerp(m_orientations[i], m_orientations[i + 1], t);
        }
    }

    return m_orientations.last();
}