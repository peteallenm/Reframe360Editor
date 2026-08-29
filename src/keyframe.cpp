#include "keyframe.h"

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

KeyframeModel::KeyframeModel(QObject *parent) : QAbstractListModel(parent) {}

int KeyframeModel::rowCount(const QModelIndex &) const { return (int)m_keyframes.size(); }

QVariant KeyframeModel::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() >= (int)m_keyframes.size())
        return {};
    const auto &k = m_keyframes[index.row()];
    switch (role) {
    case TimeRole:  return k.time;
    case YawRole:   return k.yaw;
    case PitchRole: return k.pitch;
    case RollRole:  return k.roll;
    case FovRole:   return k.fov;
    }
    return {};
}

QHash<int, QByteArray> KeyframeModel::roleNames() const
{
    return {{TimeRole, "kfTime"}, {YawRole, "kfYaw"}, {PitchRole, "kfPitch"},
            {RollRole, "kfRoll"}, {FovRole, "kfFov"}};
}

void KeyframeModel::addKeyframe(double time, double yaw, double pitch, double roll, double fov)
{
    // Don't add duplicate at same time — update existing
    for (int i = 0; i < (int)m_keyframes.size(); i++) {
        if (qFuzzyCompare(m_keyframes[i].time, time)) {
            m_keyframes[i].yaw = yaw;
            m_keyframes[i].pitch = pitch;
            m_keyframes[i].roll = roll;
            m_keyframes[i].fov = fov;
            emit dataChanged(index(i), index(i));
            emit keyframesChanged();
            return;
        }
    }
    int pos = 0;
    while (pos < (int)m_keyframes.size() && m_keyframes[pos].time < time) pos++;
    beginInsertRows(QModelIndex(), pos, pos);
    m_keyframes.insert(pos, Keyframe{time, yaw, pitch, roll, fov});
    endInsertRows();
    emit keyframesChanged();
}

void KeyframeModel::setTrimIn(double t)
{
    if (qFuzzyCompare(m_trimIn, t))
        return;
    m_trimIn = t;
    emit trimChanged();
}

void KeyframeModel::setTrimOut(double t)
{
    if (qFuzzyCompare(m_trimOut, t))
        return;
    m_trimOut = t;
    emit trimChanged();
}

void KeyframeModel::setExportWidth(int w)
{
    w = qMax(16, w & ~1);
    if (m_exportWidth == w) return;
    m_exportWidth = w;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportHeight(int h)
{
    h = qMax(16, h & ~1);
    if (m_exportHeight == h) return;
    m_exportHeight = h;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportFps(double fps)
{
    if (qFuzzyCompare(m_exportFps, fps) || fps <= 0.0) return;
    m_exportFps = fps;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportCodec(const QString &codec)
{
    if (m_exportCodec == codec) return;
    m_exportCodec = codec;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportCrf(int crf)
{
    crf = qBound(0, crf, 51);
    if (m_exportCrf == crf) return;
    m_exportCrf = crf;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportBitrate(int bitrate)
{
    bitrate = qBound(1, bitrate, 100);
    if (m_exportBitrate == bitrate) return;
    m_exportBitrate = bitrate;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportVidstab(bool vidstab)
{
    if (m_exportVidstab == vidstab) return;
    m_exportVidstab = vidstab;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportVidstabInformed(bool informed)
{
    if (m_exportVidstabInformed == informed) return;
    m_exportVidstabInformed = informed;
    emit exportSettingsChanged();
}

void KeyframeModel::setExportFileName(const QString &name)
{
    if (m_exportFileName == name) return;
    m_exportFileName = name;
    emit exportSettingsChanged();
}

void KeyframeModel::removeKeyframe(int i)
{
    if (i < 0 || i >= (int)m_keyframes.size()) return;
    beginRemoveRows(QModelIndex(), i, i);
    m_keyframes.removeAt(i);
    endRemoveRows();
    emit keyframesChanged();
}

void KeyframeModel::updateKeyframe(int i, double time, double yaw, double pitch, double roll, double fov)
{
    if (i < 0 || i >= (int)m_keyframes.size()) return;
    m_keyframes[i].time = time;
    m_keyframes[i].yaw = yaw;
    m_keyframes[i].pitch = pitch;
    m_keyframes[i].roll = roll;
    m_keyframes[i].fov = fov;
    sortAndNotify();
    emit keyframesChanged();
}

Keyframe KeyframeModel::keyframeAt(int i) const
{
    if (i >= 0 && i < (int)m_keyframes.size()) return m_keyframes[i];
    return {};
}

void KeyframeModel::sortAndNotify()
{
    std::sort(m_keyframes.begin(), m_keyframes.end());
    if (!m_keyframes.isEmpty()) {
        emit dataChanged(index(0), index((int)m_keyframes.size() - 1));
    }
}

void KeyframeModel::interpolate(double time, double &yaw, double &pitch, double &roll, double &fov) const
{
    interpolate(m_keyframes, time, yaw, pitch, roll, fov);
}

void KeyframeModel::interpolate(const QVector<Keyframe> &kfs, double time,
                                double &yaw, double &pitch, double &roll, double &fov)
{
    if (kfs.isEmpty()) return;

    // Before first keyframe
    if (time <= kfs.first().time) {
        yaw = kfs.first().yaw;
        pitch = kfs.first().pitch;
        roll = kfs.first().roll;
        fov = kfs.first().fov;
        return;
    }

    // After last keyframe
    if (time >= kfs.last().time) {
        yaw = kfs.last().yaw;
        pitch = kfs.last().pitch;
        roll = kfs.last().roll;
        fov = kfs.last().fov;
        return;
    }

    // Between two keyframes — linear interpolation. Yaw/roll are angles, so
    // interpolate along the *shortest* path: a 350° -> 10° move must sweep the
    // 20° across the 360/0 wrap, not spin 340° the long way around.
    for (int i = 0; i < (int)kfs.size() - 1; i++) {
        if (time >= kfs[i].time && time <= kfs[i + 1].time) {
            double t1 = kfs[i].time;
            double t2 = kfs[i + 1].time;
            double alpha = (time - t1) / (t2 - t1);

            // Wrap the delta into [-180, 180] and keep the result in range.
            auto lerpAngle = [](double a, double b, double t) {
                double d = b - a;
                if (d > 180.0) d -= 360.0;
                else if (d < -180.0) d += 360.0;
                double r = a + d * t;
                if (r > 180.0) r -= 360.0;
                else if (r < -180.0) r += 360.0;
                return r;
            };

            yaw   = lerpAngle(kfs[i].yaw,   kfs[i + 1].yaw,   alpha);
            roll  = lerpAngle(kfs[i].roll,  kfs[i + 1].roll,  alpha);
            // Pitch is bounded to [-89.5, 89.5] (never wraps) — plain lerp is fine.
            pitch = kfs[i].pitch + (kfs[i + 1].pitch - kfs[i].pitch) * alpha;
            fov   = kfs[i].fov   + (kfs[i + 1].fov   - kfs[i].fov)   * alpha;
            return;
        }
    }
}

void KeyframeModel::saveToFile(const QString &path) const
{
    QJsonArray arr;
    for (const auto &k : m_keyframes) {
        arr.append(QJsonObject{
            {"time", k.time}, {"yaw", k.yaw}, {"pitch", k.pitch},
            {"roll", k.roll}, {"fov", k.fov},
        });
    }
    QJsonObject root{
        {"version", 5},
        {"in", m_trimIn},        // export trim start marker
        {"out", m_trimOut},      // export trim end marker
        {"export", QJsonObject{
            {"width", m_exportWidth},
            {"height", m_exportHeight},
            {"fps", m_exportFps},
            {"codec", m_exportCodec},
            {"crf", m_exportCrf},
            {"bitrateMbps", m_exportBitrate},
            {"vidstab", m_exportVidstab},
            {"vidstabInformed", m_exportVidstabInformed},
            {"fileName", m_exportFileName},
        }},
        {"keyframes", arr},
    };
    // Per-video IMU clock drift is optional; only stored once the user has
    // tuned it (or a sidecar was created with one).
    if (hasImuDrift())
        root.insert(QStringLiteral("imuDrift"), m_imuDrift);

    // Version 5: per-video sync offset and gyro calibration
    if (hasSyncOffset())
        root.insert(QStringLiteral("syncOffset"), m_syncOffset);

    if (m_hasGyroCalibration) {
        QJsonArray matArr;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                matArr.append((double)m_gyroMatrix(i, j));
        root.insert(QStringLiteral("gyroMatrix"), matArr);

        QJsonArray biasArr;
        biasArr.append((double)m_gyroBias.x());
        biasArr.append((double)m_gyroBias.y());
        biasArr.append((double)m_gyroBias.z());
        root.insert(QStringLiteral("gyroBias"), biasArr);
    }

    QFile f(path);
    if (f.open(QIODevice::WriteOnly))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void KeyframeModel::loadFromFile(const QString &path)
{
    QVector<Keyframe> loaded;
    double trimIn = 0.0;
    double trimOut = 0.0;
    double imuDrift = -1e9;   // absent in old sidecars -> not stored
    double syncOffset = -1e9; // absent in old sidecars -> not stored
    QMatrix3x3 gyroMatrix;    // default identity
    QVector3D gyroBias;       // default zero
    bool hasGyroCalibration = false;
    int expWidth = m_exportWidth;
    int expHeight = m_exportHeight;
    double expFps = m_exportFps;
    QString expCodec = m_exportCodec;
    int expCrf = m_exportCrf;
    int expBitrate = m_exportBitrate;
    bool expVidstab = m_exportVidstab;
    bool expVidstabInformed = m_exportVidstabInformed;
    QString expFileName = m_exportFileName;

    QFile f(path);
    if (f.open(QIODevice::ReadOnly)) {
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
        if (err.error == QJsonParseError::NoError && doc.isObject()) {
            const QJsonObject root = doc.object();
            const QJsonArray arr = root.value("keyframes").toArray();
            for (const QJsonValue &v : arr) {
                const QJsonObject o = v.toObject();
                Keyframe k;
                k.time  = o.value("time").toDouble(0.0);
                k.yaw   = o.value("yaw").toDouble(0.0);
                k.pitch = o.value("pitch").toDouble(0.0);
                k.roll  = o.value("roll").toDouble(0.0);
                k.fov   = o.value("fov").toDouble(90.0);
                loaded.append(k);
            }
            // Optional export trim markers (absent in v1 sidecars -> 0/0).
            trimIn  = root.value("in").toDouble(0.0);
            trimOut = root.value("out").toDouble(0.0);
            // Optional per-video IMU clock drift (absent -> not stored).
            if (root.contains("imuDrift"))
                imuDrift = root.value("imuDrift").toDouble(-1e9);
            // Version 5: optional sync offset and gyro calibration
            if (root.contains("syncOffset"))
                syncOffset = root.value("syncOffset").toDouble(-1e9);
            if (root.contains("gyroMatrix") && root.contains("gyroBias")) {
                const QJsonArray matArr = root.value("gyroMatrix").toArray();
                const QJsonArray biasArr = root.value("gyroBias").toArray();
                if (matArr.size() == 9 && biasArr.size() == 3) {
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 3; j++)
                            gyroMatrix(i, j) = (float)matArr[i * 3 + j].toDouble();
                    gyroBias.setX((float)biasArr[0].toDouble());
                    gyroBias.setY((float)biasArr[1].toDouble());
                    gyroBias.setZ((float)biasArr[2].toDouble());
                    hasGyroCalibration = true;
                }
            }
            // Optional last-used export options (absent in older sidecars ->
            // keep the current in-memory defaults).
            const QJsonObject exp = root.value("export").toObject();
            const int w = exp.value("width").toInt(expWidth);
            const int h = exp.value("height").toInt(expHeight);
            if (w >= 16 && w % 2 == 0) expWidth = w;
            if (h >= 16 && h % 2 == 0) expHeight = h;
            const double fps = exp.value("fps").toDouble(expFps);
            if (fps > 0.0) expFps = fps;
            if (exp.contains("codec")) expCodec = exp.value("codec").toString();
            expCrf = qBound(0, exp.value("crf").toInt(expCrf), 51);
            expBitrate = qBound(1, exp.value("bitrateMbps").toInt(expBitrate), 100);
            if (exp.contains("vidstab")) expVidstab = exp.value("vidstab").toBool();
            if (exp.contains("vidstabInformed")) expVidstabInformed = exp.value("vidstabInformed").toBool();
            if (exp.contains("fileName") && !exp.value("fileName").toString().isEmpty())
                expFileName = exp.value("fileName").toString();
        }
    }
    std::sort(loaded.begin(), loaded.end());

    // Keep the same invariant addKeyframe() maintains: at most one keyframe
    // per time (hand-edited sidecars could contain duplicates).
    if (loaded.size() > 1) {
        int write = 1;
        for (int i = 1; i < loaded.size(); ++i) {
            if (!qFuzzyCompare(loaded[i].time, loaded[write - 1].time))
                loaded[write++] = loaded[i];
        }
        loaded.resize(write);
    }

    const bool exportUnchanged = expWidth == m_exportWidth
                              && expHeight == m_exportHeight
                              && qFuzzyCompare(expFps, m_exportFps)
                              && expCodec == m_exportCodec
                              && expCrf == m_exportCrf
                              && expBitrate == m_exportBitrate
                              && expVidstab == m_exportVidstab
                              && expVidstabInformed == m_exportVidstabInformed
                              && expFileName == m_exportFileName;

    // No-op load (same contents + trim, or a missing file with an empty
    // model): skip the model reset so opening videos doesn't churn the
    // timeline. The trim/export checks matter too: two videos can share
    // identical keyframes but different markers or export options.
    if (loaded == m_keyframes
        && qFuzzyCompare(trimIn, m_trimIn)
        && qFuzzyCompare(trimOut, m_trimOut)
        && qFuzzyCompare(imuDrift, m_imuDrift)
        && qFuzzyCompare(syncOffset, m_syncOffset)
        && hasGyroCalibration == m_hasGyroCalibration
        && exportUnchanged)
        return;

    // Replace the model contents (empty if the file is missing/invalid) so
    // switching videos never leaks the previous video's keyframes.
    beginResetModel();
    m_keyframes = loaded;
    endResetModel();
    m_trimIn = trimIn;
    m_trimOut = trimOut;
    m_imuDrift = imuDrift;
    m_syncOffset = syncOffset;
    m_gyroMatrix = gyroMatrix;
    m_gyroBias = gyroBias;
    m_hasGyroCalibration = hasGyroCalibration;
    m_exportWidth = expWidth;
    m_exportHeight = expHeight;
    m_exportFps = expFps;
    m_exportCodec = expCodec;
    m_exportCrf = expCrf;
    m_exportBitrate = expBitrate;
    m_exportVidstab = expVidstab;
    m_exportVidstabInformed = expVidstabInformed;
    m_exportFileName = expFileName;
    emit trimChanged();
    emit exportSettingsChanged();
}
