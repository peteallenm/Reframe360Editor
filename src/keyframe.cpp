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
        {"version", 2},
        {"in", m_trimIn},        // export trim start marker
        {"out", m_trimOut},      // export trim end marker
        {"keyframes", arr},
    };

    QFile f(path);
    if (f.open(QIODevice::WriteOnly))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void KeyframeModel::loadFromFile(const QString &path)
{
    QVector<Keyframe> loaded;
    double trimIn = 0.0;
    double trimOut = 0.0;

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

    // No-op load (same contents + trim, or a missing file with an empty
    // model): skip the model reset so opening videos doesn't churn the
    // timeline. The trim check matters too: two videos can share identical
    // keyframes but different in/out markers.
    if (loaded == m_keyframes
        && qFuzzyCompare(trimIn, m_trimIn)
        && qFuzzyCompare(trimOut, m_trimOut))
        return;

    // Replace the model contents (empty if the file is missing/invalid) so
    // switching videos never leaks the previous video's keyframes.
    beginResetModel();
    m_keyframes = loaded;
    endResetModel();
    m_trimIn = trimIn;
    m_trimOut = trimOut;
    emit trimChanged();
}
