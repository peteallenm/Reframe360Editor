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

    void integrate(const QVector<ImuSample> &samples, double sampleRate);
    QQuaternion orientationAtTime(double time) const;
    void setSmoothing(double factor) { m_smoothing = factor; }

private:
    QVector<QQuaternion> m_orientations;
    QVector<double> m_timestamps;
    QQuaternion m_initialOrientation;
    double m_sampleRate;
    double m_smoothing;
};

#endif // GYROSCOPEINTEGRATOR_H
