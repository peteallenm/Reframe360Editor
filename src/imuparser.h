#ifndef IMUPARSER_H
#define IMUPARSER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QQuaternion>
#include <cstdint>

struct ImuSample {
    double timestamp;
    QVector3D gyro;   // deg/s
    QVector3D accel;  // g
    uint32_t counter; // hardware counter from t2 record (~1 MHz, ~2500 counts/sample)
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
    bool isLoaded() const { return m_loaded; }
    // Duration spanned by the parsed sample stream (seconds).
    double duration() const { return m_imuSampleRate > 0.0 ? m_rawData.size() / m_imuSampleRate : 0.0; }

    // Per-axis gyro scales in LSB/(deg/s), calibrated so a 360° rotation on
    // each axis integrates to exactly ±360°: X=roll 32.18, Y=pitch 33.51,
    // Z=yaw 33.64 (vs the nominal header 32.8).
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
    bool m_loaded = false;
};

#endif // IMUPARSER_H
