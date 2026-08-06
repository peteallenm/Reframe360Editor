#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <QObject>
#include <QAbstractListModel>
#include <QVector>
#include <algorithm>

struct Keyframe {
    double time = 0.0;
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double fov = 90.0;

    bool operator<(const Keyframe &o) const { return time < o.time; }
    bool operator==(const Keyframe &o) const {
        return qFuzzyCompare(time, o.time) && qFuzzyCompare(yaw, o.yaw)
            && qFuzzyCompare(pitch, o.pitch) && qFuzzyCompare(roll, o.roll)
            && qFuzzyCompare(fov, o.fov);
    }
};

class KeyframeModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles { TimeRole = Qt::UserRole + 1, YawRole, PitchRole, RollRole, FovRole };

    explicit KeyframeModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    Q_INVOKABLE void addKeyframe(double time, double yaw, double pitch, double roll, double fov);
    Q_INVOKABLE void removeKeyframe(int index);
    Q_INVOKABLE void updateKeyframe(int index, double time, double yaw, double pitch, double roll, double fov);

    Keyframe keyframeAt(int index) const;
    int count() const { return (int)m_keyframes.size(); }

    void interpolate(double time, double &yaw, double &pitch, double &roll, double &fov) const;
    bool hasKeyframes() const { return !m_keyframes.isEmpty(); }

private:
    void sortAndNotify();
    QVector<Keyframe> m_keyframes;
};

#endif // KEYFRAME_H
