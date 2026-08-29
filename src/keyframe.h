#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <QObject>
#include <QAbstractListModel>
#include <QVector>
#include <QMatrix3x3>
#include <QVector3D>
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

    // Export trim range (in/out markers). Stored in the same sidecar file as
    // the keyframes so each video remembers where its exported range starts
    // and ends, independent of any keyframes.
    Q_INVOKABLE double trimIn() const { return m_trimIn; }
    Q_INVOKABLE double trimOut() const { return m_trimOut; }
    Q_INVOKABLE void setTrimIn(double t);
    Q_INVOKABLE void setTrimOut(double t);

    // Last-used export output options, persisted in the same sidecar file so
    // each video remembers its own encoder/quality choices.
    Q_PROPERTY(int exportWidth READ exportWidth WRITE setExportWidth NOTIFY exportSettingsChanged)
    Q_PROPERTY(int exportHeight READ exportHeight WRITE setExportHeight NOTIFY exportSettingsChanged)
    Q_PROPERTY(double exportFps READ exportFps WRITE setExportFps NOTIFY exportSettingsChanged)
    Q_PROPERTY(QString exportCodec READ exportCodec WRITE setExportCodec NOTIFY exportSettingsChanged)
    Q_PROPERTY(int exportCrf READ exportCrf WRITE setExportCrf NOTIFY exportSettingsChanged)
    Q_PROPERTY(int exportBitrate READ exportBitrate WRITE setExportBitrate NOTIFY exportSettingsChanged)
    Q_PROPERTY(bool exportVidstab READ exportVidstab WRITE setExportVidstab NOTIFY exportSettingsChanged)
    Q_PROPERTY(bool exportVidstabInformed READ exportVidstabInformed WRITE setExportVidstabInformed NOTIFY exportSettingsChanged)
    Q_PROPERTY(QString exportFileName READ exportFileName WRITE setExportFileName NOTIFY exportSettingsChanged)
    Q_INVOKABLE int exportWidth() const { return m_exportWidth; }
    Q_INVOKABLE void setExportWidth(int w);
    Q_INVOKABLE int exportHeight() const { return m_exportHeight; }
    Q_INVOKABLE void setExportHeight(int h);
    Q_INVOKABLE double exportFps() const { return m_exportFps; }
    Q_INVOKABLE void setExportFps(double fps);
    Q_INVOKABLE QString exportCodec() const { return m_exportCodec; }
    Q_INVOKABLE void setExportCodec(const QString &codec);
    Q_INVOKABLE int exportCrf() const { return m_exportCrf; }
    Q_INVOKABLE void setExportCrf(int crf);
    Q_INVOKABLE int exportBitrate() const { return m_exportBitrate; }
    Q_INVOKABLE void setExportBitrate(int bitrate);
    Q_INVOKABLE bool exportVidstab() const { return m_exportVidstab; }
    Q_INVOKABLE void setExportVidstab(bool vidstab);
    Q_INVOKABLE bool exportVidstabInformed() const { return m_exportVidstabInformed; }
    Q_INVOKABLE void setExportVidstabInformed(bool informed);
    Q_INVOKABLE QString exportFileName() const { return m_exportFileName; }
    Q_INVOKABLE void setExportFileName(const QString &name);

    Keyframe keyframeAt(int index) const;
    int count() const { return (int)m_keyframes.size(); }

    // Per-video IMU<->video clock drift (s/s) persisted in the same sidecar
    // file so each video remembers its own tuned value. -1e9 = not stored;
    // the caller then falls back to the auto-calculated value.
    void setImuDrift(double d) { m_imuDrift = d; }
    double imuDrift() const { return m_imuDrift; }
    // Sentinel -1e9 = not stored. It must sit outside the value range, not at
    // its edge: the old ">= 0.0" test made every NEGATIVE drift (video clock
    // running faster than the IMU — an ordinary, expected case) indistinguishable
    // from "absent", so such a value was silently dropped on save.
    bool hasImuDrift() const { return m_imuDrift > -1e8; }

    // Per-video IMU sync offset (s) persisted in the sidecar.
    // Sentinel -1e9 = not stored; the caller falls back to manual/auto value.
    void setSyncOffset(double s) { m_syncOffset = s; }
    double syncOffset() const { return m_syncOffset; }
    bool hasSyncOffset() const { return m_syncOffset > -1e8; }

    // Per-video gyro calibration (matrix + bias) persisted in the sidecar.
    void setGyroCalibration(const QMatrix3x3 &M, const QVector3D &b) {
        m_gyroMatrix = M;
        m_gyroBias = b;
        m_hasGyroCalibration = true;
    }
    void clearGyroCalibration() {
        m_gyroMatrix = QMatrix3x3();
        m_gyroBias = QVector3D();
        m_hasGyroCalibration = false;
    }
    QMatrix3x3 gyroMatrix() const { return m_gyroMatrix; }
    QVector3D gyroBias() const { return m_gyroBias; }
    bool hasGyroCalibration() const { return m_hasGyroCalibration; }

    void interpolate(double time, double &yaw, double &pitch, double &roll, double &fov) const;
    bool hasKeyframes() const { return !m_keyframes.isEmpty(); }

    // Snapshot of the keyframe set, safe to hand to a worker thread (e.g. the
    // exporter) without racing against GUI-thread edits.
    QVector<Keyframe> keyframes() const { return m_keyframes; }

    // Pure interpolation over a snapshot, so it can be evaluated from another
    // thread. Mirrors the member interpolate() exactly (incl. shortest-path
    // wrapping for yaw/roll).
    static void interpolate(const QVector<Keyframe> &kfs, double time,
                            double &yaw, double &pitch, double &roll, double &fov);

    // Persistence: write/read the whole keyframe set as JSON. loadFromFile()
    // replaces the current contents (clearing them if the file is missing).
    void saveToFile(const QString &path) const;
    void loadFromFile(const QString &path);

signals:
    // Emitted after any keyframe is added, removed or updated, so callers can
    // persist the (possibly new) state.
    void keyframesChanged();
    // Emitted after the export trim in/out markers change.
    void trimChanged();
    // Emitted after any last-used export output option changes.
    void exportSettingsChanged();

private:
    void sortAndNotify();
    QVector<Keyframe> m_keyframes;
    double m_trimIn = 0.0;
    double m_trimOut = 0.0;
    int m_exportWidth = 1920;
    int m_exportHeight = 1080;
    double m_exportFps = 30.0;
    QString m_exportCodec = QStringLiteral("libx264");
    int m_exportCrf = 19;
    int m_exportBitrate = 12;
    bool m_exportVidstab = false;
    bool m_exportVidstabInformed = false;
    QString m_exportFileName;  // last-used output MP4 path (persisted per-video)
    double m_imuDrift = -1e9;  // -1e9 = not stored in the sidecar
    double m_syncOffset = -1e9; // sentinel = not stored in the sidecar
    QMatrix3x3 m_gyroMatrix;   // default identity
    QVector3D m_gyroBias;      // default zero
    bool m_hasGyroCalibration = false;
};

#endif // KEYFRAME_H
