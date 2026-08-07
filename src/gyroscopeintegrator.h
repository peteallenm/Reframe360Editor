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

private:
    QVector3D computeGyroBias(const QVector<ImuSample> &samples) const;

    QVector<QQuaternion> m_orientations;
    QVector<double> m_timestamps;
    double m_sampleRate;
};

#endif // GYROSCOPEINTEGRATOR_H
