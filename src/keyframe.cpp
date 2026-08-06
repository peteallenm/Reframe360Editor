#include "keyframe.h"

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
            return;
        }
    }
    int pos = 0;
    while (pos < (int)m_keyframes.size() && m_keyframes[pos].time < time) pos++;
    beginInsertRows(QModelIndex(), pos, pos);
    m_keyframes.insert(pos, Keyframe{time, yaw, pitch, roll, fov});
    endInsertRows();
}

void KeyframeModel::removeKeyframe(int i)
{
    if (i < 0 || i >= (int)m_keyframes.size()) return;
    beginRemoveRows(QModelIndex(), i, i);
    m_keyframes.removeAt(i);
    endRemoveRows();
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
    if (m_keyframes.isEmpty()) return;

    // Before first keyframe
    if (time <= m_keyframes.first().time) {
        yaw = m_keyframes.first().yaw;
        pitch = m_keyframes.first().pitch;
        roll = m_keyframes.first().roll;
        fov = m_keyframes.first().fov;
        return;
    }

    // After last keyframe
    if (time >= m_keyframes.last().time) {
        yaw = m_keyframes.last().yaw;
        pitch = m_keyframes.last().pitch;
        roll = m_keyframes.last().roll;
        fov = m_keyframes.last().fov;
        return;
    }

    // Between two keyframes — linear interpolation
    for (int i = 0; i < (int)m_keyframes.size() - 1; i++) {
        if (time >= m_keyframes[i].time && time <= m_keyframes[i + 1].time) {
            double t1 = m_keyframes[i].time;
            double t2 = m_keyframes[i + 1].time;
            double alpha = (time - t1) / (t2 - t1);

            yaw   = m_keyframes[i].yaw   + (m_keyframes[i + 1].yaw   - m_keyframes[i].yaw)   * alpha;
            pitch = m_keyframes[i].pitch + (m_keyframes[i + 1].pitch - m_keyframes[i].pitch) * alpha;
            roll  = m_keyframes[i].roll  + (m_keyframes[i + 1].roll  - m_keyframes[i].roll)  * alpha;
            fov   = m_keyframes[i].fov   + (m_keyframes[i + 1].fov   - m_keyframes[i].fov)   * alpha;
            return;
        }
    }
}
