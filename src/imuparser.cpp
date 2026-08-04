#include "imuparser.h"
#include <QFile>
#include <QDataStream>
#include <QDebug>

ImuParser::ImuParser(QObject *parent)
    : QObject(parent)
    , m_sampleRate(370.0)
{
}

bool ImuParser::loadFile(const QString &path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open IMU file:" << path;
        return false;
    }

    QByteArray data = file.readAll();
    file.close();

    if (data.size() < 8) {
        qWarning() << "IMU file too small";
        return false;
    }

    const char *ptr = data.constData();

    uint32_t recordSize = *reinterpret_cast<const uint32_t*>(ptr);
    uint32_t headerField2 = *reinterpret_cast<const uint32_t*>(ptr + 4);
    Q_UNUSED(recordSize);
    Q_UNUSED(headerField2);

    ptr += 8;

    double timestamp = *reinterpret_cast<const double*>(ptr); ptr += 8;
    QVector3D gyroBias(*reinterpret_cast<const double*>(ptr),
                       *reinterpret_cast<const double*>(ptr + 8),
                       *reinterpret_cast<const double*>(ptr + 16));
    ptr += 24;
    QVector3D accelBias(*reinterpret_cast<const double*>(ptr),
                        *reinterpret_cast<const double*>(ptr + 8),
                        *reinterpret_cast<const double*>(ptr + 16));
    ptr += 24;
    QVector3D magField(*reinterpret_cast<const double*>(ptr),
                       *reinterpret_cast<const double*>(ptr + 8),
                       *reinterpret_cast<const double*>(ptr + 16));
    ptr += 24;
    double correction = *reinterpret_cast<const double*>(ptr); ptr += 8;
    ptr += 8;
    double qx = *reinterpret_cast<const double*>(ptr); ptr += 8;
    double qy = *reinterpret_cast<const double*>(ptr); ptr += 8;
    double qz = *reinterpret_cast<const double*>(ptr); ptr += 8;
    double qw = *reinterpret_cast<const double*>(ptr); ptr += 8;
    double confidence = *reinterpret_cast<const double*>(ptr); ptr += 8;

    Q_UNUSED(timestamp);
    Q_UNUSED(gyroBias);
    Q_UNUSED(accelBias);
    Q_UNUSED(magField);
    Q_UNUSED(correction);
    Q_UNUSED(confidence);

    m_initialQuaternion = QQuaternion(qw, qx, qy, qz);
    qDebug() << "Initial quaternion:" << qw << qx << qy << qz
             << "norm:" << m_initialQuaternion.length();

    ptr += 16;
    uint32_t indexCount = *reinterpret_cast<const uint32_t*>(ptr);
    Q_UNUSED(indexCount);
    ptr += 4 + indexCount * 16;

    ptr += 16;
    uint32_t tsPairCount = *reinterpret_cast<const uint32_t*>(ptr + 12);
    ptr += 16;
    ptr += tsPairCount * 16;

    int remaining = data.constData() + data.size() - ptr;
    int sampleSize = 24;
    int numSamples = remaining / sampleSize;

    m_rawData.clear();
    m_rawData.reserve(numSamples);

    double dt = 1.0 / m_sampleRate;
    for (int i = 0; i < numSamples; i++) {
        ImuSample sample;
        sample.timestamp = i * dt;
        sample.gyro = QVector3D(
            *reinterpret_cast<const int16_t*>(ptr),
            *reinterpret_cast<const int16_t*>(ptr + 2),
            *reinterpret_cast<const int16_t*>(ptr + 4)
        );
        sample.accel = QVector3D(
            *reinterpret_cast<const int16_t*>(ptr + 6),
            *reinterpret_cast<const int16_t*>(ptr + 8),
            *reinterpret_cast<const int16_t*>(ptr + 10)
        );
        sample.mag = QVector3D(
            *reinterpret_cast<const int16_t*>(ptr + 12),
            *reinterpret_cast<const int16_t*>(ptr + 14),
            *reinterpret_cast<const int16_t*>(ptr + 16)
        );
        m_rawData.append(sample);
        ptr += sampleSize;
    }

    qDebug() << "Parsed" << m_rawData.size() << "IMU samples";
    return true;
}
