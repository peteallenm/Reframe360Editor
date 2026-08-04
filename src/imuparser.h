#ifndef IMUPARSER_H
#define IMUPARSER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QQuaternion>

struct ImuSample {
    double timestamp;
    QVector3D gyro;
    QVector3D accel;
    QVector3D mag;
};

class ImuParser : public QObject
{
    Q_OBJECT
public:
    explicit ImuParser(QObject *parent = nullptr);

    bool loadFile(const QString &path);

    QQuaternion initialQuaternion() const { return m_initialQuaternion; }
    double sampleRate() const { return m_sampleRate; }
    QVector<ImuSample> rawGyroData() const { return m_rawData; }

private:
    QQuaternion m_initialQuaternion;
    double m_sampleRate;
    QVector<ImuSample> m_rawData;
};

#endif // IMUPARSER_H
