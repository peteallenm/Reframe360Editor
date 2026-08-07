#ifndef APP_H
#define APP_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QQuaternion>
#include "videodecoder.h"
#include "imuparser.h"
#include "gyroscopeintegrator.h"
#include "calibration.h"
#include "keyframe.h"

class App : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString videoPath READ videoPath WRITE setVideoPath NOTIFY videoPathChanged)
    Q_PROPERTY(bool isPlaying READ isPlaying WRITE setIsPlaying NOTIFY isPlayingChanged)
    Q_PROPERTY(double currentTime READ currentTime WRITE setCurrentTime NOTIFY currentTimeChanged)
    Q_PROPERTY(double duration READ duration NOTIFY durationChanged)
    Q_PROPERTY(double yaw READ yaw WRITE setYaw NOTIFY yawChanged)
    Q_PROPERTY(double pitch READ pitch WRITE setPitch NOTIFY pitchChanged)
    Q_PROPERTY(double roll READ roll WRITE setRoll NOTIFY rollChanged)
    Q_PROPERTY(double fov READ fov WRITE setFov NOTIFY fovChanged)
    Q_PROPERTY(bool imuStabilize READ imuStabilize WRITE setImuStabilize NOTIFY imuStabilizeChanged)
    Q_PROPERTY(double imuSmoothing READ imuSmoothing WRITE setImuSmoothing NOTIFY imuSmoothingChanged)
    Q_PROPERTY(double imuSyncOffset READ imuSyncOffset WRITE setImuSyncOffset NOTIFY imuSyncOffsetChanged)
    Q_PROPERTY(QString previewThumbnailPath READ previewThumbnailPath NOTIFY videoPathChanged)
    Q_PROPERTY(bool usePreviewThumbnail READ usePreviewThumbnail WRITE setUsePreviewThumbnail NOTIFY usePreviewThumbnailChanged)
    Q_PROPERTY(int activeLens READ activeLens WRITE setActiveLens NOTIFY activeLensChanged)
    Q_PROPERTY(int projection READ projection WRITE setProjection NOTIFY projectionChanged)
    Q_PROPERTY(VideoDecoder* videoDecoder READ videoDecoder CONSTANT)
    Q_PROPERTY(CalibrationPresetModel* calibrationPresets READ calibrationPresets CONSTANT)
    Q_PROPERTY(CalibrationProfile* currentCalibration READ currentCalibration NOTIFY currentCalibrationChanged)
    Q_PROPERTY(QQuaternion imuOrientation READ imuOrientation NOTIFY imuOrientationChanged)
    Q_PROPERTY(KeyframeModel* keyframes READ keyframes CONSTANT)

public:
    explicit App(QObject *parent = nullptr);
    ~App();

    QString videoPath() const;
    void setVideoPath(const QString &path);

    QString previewThumbnailPath() const;
    bool usePreviewThumbnail() const;
    void setUsePreviewThumbnail(bool use);

    bool isPlaying() const;
    void setIsPlaying(bool playing);

    double currentTime() const;
    void setCurrentTime(double time);

    double duration() const;

    double yaw() const;
    void setYaw(double yaw);

    double pitch() const;
    void setPitch(double pitch);

    double roll() const;
    void setRoll(double roll);

    double fov() const;
    void setFov(double fov);

    bool imuStabilize() const;
    void setImuStabilize(bool stabilize);

    double imuSmoothing() const;
    void setImuSmoothing(double smoothing);

    double imuSyncOffset() const;
    void setImuSyncOffset(double offset);

    int activeLens() const;
    void setActiveLens(int lens);

    int projection() const;
    void setProjection(int projection);

    VideoDecoder* videoDecoder() const;

    CalibrationPresetModel* calibrationPresets() const;
    CalibrationProfile* currentCalibration() const;

    QQuaternion imuOrientation() const;

    KeyframeModel* keyframes() const { return m_keyframes; }

    Q_INVOKABLE void exportFrame(const QString &path, int width, int height);
    Q_INVOKABLE void exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime);
    Q_INVOKABLE QString grabStill(int lens);
    Q_INVOKABLE void dragLook(double angleAboutUp, double angleAboutRight);

    Q_INVOKABLE void addKeyframeAtCurrent();
    Q_INVOKABLE void applyKeyframeInterpolation();

private:
    QQuaternion viewQuatFromEuler() const;
    void extractEulerFromQuat(const QQuaternion &q, double &yaw, double &pitch, double &roll) const;
    void setViewFromQuat(const QQuaternion &q);

    void loadSettings();
    void saveSettings() const;
    void computeGravityAlignment();

signals:
    void videoPathChanged();
    void isPlayingChanged();
    void currentTimeChanged();
    void durationChanged();
    void yawChanged();
    void pitchChanged();
    void rollChanged();
    void fovChanged();
    void imuStabilizeChanged();
    void imuSmoothingChanged();
    void imuSyncOffsetChanged();
    void usePreviewThumbnailChanged();
    void activeLensChanged();
    void projectionChanged();
    void currentCalibrationChanged();
    void imuOrientationChanged();
    void videoLoaded();
    void errorOccurred(const QString &message);

private:
    VideoDecoder *m_decoder;
    ImuParser *m_imuParser;
    GyroscopeIntegrator *m_gyroIntegrator;
    CalibrationPresetModel *m_calibrationPresets;
    CalibrationProfile *m_currentCalibration;
    KeyframeModel *m_keyframes;

    QString m_videoPath;
    bool m_isPlaying;
    double m_currentTime;
    double m_yaw;
    double m_pitch;
    double m_roll;
    double m_fov;
    bool m_imuStabilize;
    double m_imuSmoothing;
    double m_imuSyncOffset;
    bool m_usePreview;
    int m_activeLens;
    int m_projection;
    QQuaternion m_viewQuat;
    QQuaternion m_gravityAlign;
};

#endif // APP_H
