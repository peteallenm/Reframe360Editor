#include "imuparser.h"
#include <QFile>
#include <QDebug>
#include <cmath>

ImuParser::ImuParser(QObject *parent)
    : QObject(parent)
    , m_imuSampleRate(400.0)
    , m_gyroScaleX(32.8)
    , m_gyroScaleY(32.8)
    , m_gyroScaleZ(32.8)
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

    if (data.size() < 160) {
        qWarning() << "IMU file too small:" << data.size();
        return false;
    }

    const char *ptr = data.constData();

    uint32_t sampleRate = *reinterpret_cast<const uint32_t*>(ptr); ptr += 4;
    uint32_t dataOffset = *reinterpret_cast<const uint32_t*>(ptr); ptr += 4;

    double calib[8];
    for (int i = 0; i < 8; i++) {
        calib[i] = *reinterpret_cast<const double*>(ptr); ptr += 8;
    }

    double config[11];
    for (int i = 0; i < 11; i++) {
        config[i] = *reinterpret_cast<const double*>(ptr); ptr += 8;
    }

    double qx = config[4], qy = config[5], qz = config[6], qw = config[7];
    m_initialQuaternion = QQuaternion((float)qw, (float)qx, (float)qy, (float)qz).normalized();
    qDebug() << "IMU init quat:" << qw << qx << qy << qz << "norm:" << m_initialQuaternion.length();

    // Header sample rate is the per-stream gyro/accel rate (400 Hz), not
    // combined across two IMUs.
    m_imuSampleRate = (sampleRate > 0) ? (double)sampleRate : 400.0;

    // calib[0] is the gyro scale factor in LSB/(deg/s) (32.8 in all files).
    if (calib[0] > 0.0 && calib[0] < 1000.0) {
        m_gyroScaleX = m_gyroScaleY = m_gyroScaleZ = calib[0];
    }

    // dataOffset must be even so the int16 record reads stay aligned.
    if (dataOffset < 160 || (dataOffset % 2) != 0 || (qint64)dataOffset + 8 > data.size()) {
        qWarning() << "Bad IMU data offset:" << dataOffset;
        return false;
    }

    // Data section: 8-byte records (4 x little-endian int16) in a fixed
    // 3-record cycle (24-byte packet per 400 Hz sample). Packet layout in
    // the file is t2, t3, t1:
    //   t2: 0, 0, counterLo, counterHi
    //   t3: 0, 0, accelY, accelZ
    //   t1: accelX, gyroPitch, gyroYaw, gyroRoll
    const int recordSize = 8;
    const int n = (data.size() - dataOffset) / recordSize;
    const int16_t *rec = reinterpret_cast<const int16_t*>(data.constData() + dataOffset);

    // 1) Locate the start of the real sensor cycle. Files begin with a
    //    variable-length pre-roll region (a few hundred to >10,000 records in
    //    real camera files, 0-350 in the hand-made test files) of counter-less
    //    records shaped [0,0,X,Y] and/or [X,Y,0,0]. The real t2/t3/t1 cycle is
    //    the first place where the 32-bit counter in the t2 position (fields
    //    [2]/[3]) increases monotonically (~2500 counts per sample). The scan
    //    covers the whole file because the pre-roll length is unbounded in
    //    practice (a real YIVR_0847 file had a 10452-record pre-roll).
    const int kNeed = 50;             // consecutive monotonic counters
    const uint32_t kMaxDelta = 20000; // sane per-sample counter step
    int t2 = -1;
    const int scanMax = n - 3 * (kNeed - 1); // scan whole file (early-exits at first cycle)
    for (int s = 0; s < scanMax && t2 < 0; s++) {
        uint32_t prev = 0;
        bool first = true, ok = true;
        for (int j = 0; j < kNeed && ok; j++) {
            const int16_t *r = rec + (s + 3*j) * 4;
            uint32_t c = (uint32_t)(uint16_t)r[2]
                       | ((uint32_t)(uint16_t)r[3] << 16);
            if (first) { prev = c; first = false; }
            else {
                uint32_t d = c - prev;   // unsigned; wraps only at 2^32
                if (d == 0 || d > kMaxDelta) ok = false;
                prev = c;
            }
        }
        if (ok && !first) t2 = s;
    }

    int start;
    if (t2 >= 0) {
        // Packet layout in the file is t2, t3, t1, so the t1 (sensor) record
        // is two records after the t2 record found by the counter scan.
        start = t2 + 2;
    } else {
        // Fallback (no monotonic counter found): skip a leading run of
        // (x, y, 0, 0) counter-less records, then detect the cycle phase by
        // accelX presence (t1 records carry accelX at field [0]).
        start = 0;
        while (start < n && rec[start*4+2] == 0 && rec[start*4+3] == 0
               && rec[start*4+0] != 0)
            start++;
        int phase = 0;
        for (int p = 0; p < 3; p++) {
            int cnt = 0, tot = 0;
            for (int i = start + p; i < n; i += 3) { tot++; if (rec[i*4+0] != 0) cnt++; }
            if (tot > 0 && (double)cnt / tot > 0.5) { phase = p; break; }
        }
        start += phase;
    }

    // 2) One sample per 24-byte packet at m_imuSampleRate (400 Hz).
    //    No gyro saturation filter is needed: with int16 input and scale
    //    32.8 LSB/(deg/s) the maximum representable rate is ~999 deg/s.
    const double accelScale = 4320.0;
    const double dt = 1.0 / m_imuSampleRate;
    double t = 0.0;
    m_rawData.clear();
    m_rawData.reserve((n - start) / 3 + 1);
    for (int i = start; i < n; i += 3) {
        const int16_t *t1 = rec + i * 4;   // accelX, gyroPitch, gyroYaw, gyroRoll
        // 0, 0, accelY, accelZ (same packet, one record before t1)
        const int16_t *t3 = (i - 1 >= 0) ? rec + (i - 1) * 4 : nullptr;

        ImuSample s;
        s.timestamp = t;
        s.gyro  = QVector3D((float)(t1[3] / m_gyroScaleX),   // X=roll
                            (float)(t1[1] / m_gyroScaleY),   // Y=pitch
                            (float)(t1[2] / m_gyroScaleZ));  // Z=yaw
        s.accel = QVector3D((float)(t1[0] / accelScale),
                            (float)(t3 ? t3[2] : 0) / accelScale,
                            (float)(t3 ? t3[3] : 0) / accelScale);  // X,Y,Z in g
        m_rawData.append(s);
        t += dt;
    }

    qDebug() << "IMU parsed:" << m_rawData.size() << "samples from" << n << "records";
    if (!m_rawData.isEmpty())
        qDebug() << "IMU time range:" << m_rawData.first().timestamp
                 << "to" << m_rawData.last().timestamp;
    return true;
}
