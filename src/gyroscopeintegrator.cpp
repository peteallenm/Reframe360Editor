#include "gyroscopeintegrator.h"
#include <QtMath>

GyroscopeIntegrator::GyroscopeIntegrator(QObject *parent)
    : QObject(parent)
    , m_initialOrientation(1.0f, 0.0f, 0.0f, 0.0f)
    , m_sampleRate(370.0)
    , m_smoothing(0.5)
{
}

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate)
{
    m_sampleRate = sampleRate;
    m_orientations.clear();
    m_timestamps.clear();
    m_orientations.reserve(samples.size());
    m_timestamps.reserve(samples.size());

    if (samples.isEmpty()) return;

    QQuaternion current = m_initialOrientation;
    double dt = 1.0 / m_sampleRate;

    for (int i = 0; i < samples.size(); i++) {
        m_timestamps.append(samples[i].timestamp);
        m_orientations.append(current);

        if (i + 1 >= samples.size()) break;

        QVector3D gyro = samples[i].gyro;
        QVector3D gyroRemapped(-gyro.y(), gyro.x(), gyro.z());
        float gyroRadPerSec = qDegreesToRadians(gyroRemapped.length());

        if (gyroRadPerSec > 0.001f) {
            QVector3D axis = gyroRemapped.normalized();
            float angle = gyroRadPerSec * dt;
            QQuaternion delta = QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle));
            current = (current * delta).normalized();
        }
    }
}

QQuaternion GyroscopeIntegrator::orientationAtTime(double time) const
{
    if (m_orientations.isEmpty()) return m_initialOrientation;

    for (int i = 0; i < m_timestamps.size() - 1; i++) {
        if (time >= m_timestamps[i] && time < m_timestamps[i + 1]) {
            float t = (time - m_timestamps[i]) / (m_timestamps[i + 1] - m_timestamps[i]);
            return QQuaternion::slerp(m_orientations[i], m_orientations[i + 1], t);
        }
    }

    return m_orientations.last();
}
