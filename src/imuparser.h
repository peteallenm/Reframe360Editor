#ifndef IMUPARSER_H
#define IMUPARSER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QQuaternion>

struct ImuSample {
    double timestamp;
    QVector3D gyro;   // deg/s
    QVector3D accel;  // g
};

class ImuParser : public QObject
{
    Q_OBJECT
public:
    explicit ImuParser(QObject *parent = nullptr);

    bool loadFile(const QString &path);

    QQuaternion initialQuaternion() const { return m_initialQuaternion; }
    double imuSampleRate() const { return m_imuSampleRate; }
    QVector<ImuSample> samples() const { return m_rawData; }

    double gyroScaleX() const { return m_gyroScaleX; }
    double gyroScaleY() const { return m_gyroScaleY; }
    double gyroScaleZ() const { return m_gyroScaleZ; }

private:
    QQuaternion m_initialQuaternion;
    double m_imuSampleRate;
    double m_gyroScaleX;
    double m_gyroScaleY;
    double m_gyroScaleZ;
    QVector<ImuSample> m_rawData;
};

#endif // IMUPARSER_H
