#ifndef APP_H
#define APP_H

#include <QObject>
#include <QString>
#include <QUrl>
#include <QQuaternion>
#include <QImage>
#include <algorithm>

class QTimer;
class QThread;
#include "videodecoder.h"
#include "imuparser.h"
#include "gyroscopeintegrator.h"
#include "calibration.h"
#include "colorgrade.h"
#include "keyframe.h"
#include "exporter.h"
#include "flowrenderer.h"
#include "imudriftcalibrator.h"

// Immutable snapshot of everything the exporter needs, captured at export
// start so the export worker thread never touches live GUI state. Per-time
// variation (keyframes, IMU) is folded into stateAt().
struct ExportSnapshot {
    ExportFrameState base;
    QVector<Keyframe> keyframes;
    bool imuStabilize = false;
    // Copied IMU samples (by value) so the worker never reads the live
    // integrator, which the GUI thread may re-populate by loading a video
    // while an export is running.
    QVector<QQuaternion> imuOrientations;
    QVector<double> imuTimestamps;
    double syncOffset = 0.0;
    // Linear clock-drift between the IMU and video clocks: the effective sync
    // offset at video time t is syncOffset + drift * t (s/s, ~0.0038 measured
    // on this camera). A constant offset cannot be optimal for the whole clip
    // because the two clocks run at slightly different rates.
    double drift = 0.0;
    // Gaussian smoothing window (ms) applied at the export frame rate by
    // orientationAt(); 0 = min window (33 ms exposure average).
    float imuSmoothingMs = 0.0f;

    ExportFrameState stateAt(double t) const
    {
        ExportFrameState s = base;
        if (!keyframes.isEmpty()) {
            double y, p, r, f;
            KeyframeModel::interpolate(keyframes, t, y, p, r, f);
            s.yaw = y;
            s.pitch = p;
            s.roll = r;
            s.fov = f;
        }
        if (imuStabilize && !imuOrientations.isEmpty()) {
            const double tImu = t * (1.0 + drift) + syncOffset;
            // Two-signal architecture: q_actual^{-1} * q_virtual
            // q_actual: full-bandwidth slerp (unsmoothed) — shake cancels exactly
            // q_virtual: Gaussian-smoothed — smooth intentional motion path
            // The shader applies imuOrientation.conjugated(), giving
            // q_virtual^{-1} * q_actual as the stabilization correction.
            QQuaternion qActual = GyroscopeIntegrator::orientationAt(
                imuOrientations, imuTimestamps, tImu, 0.0f);
            // For q_actual we need unsmoothed slerp, not the 33ms-minimum
            // Gaussian window. Compute it inline:
            if (imuOrientations.size() >= 2 && tImu > imuTimestamps.first()
                && tImu < imuTimestamps.last()) {
                const auto it = std::lower_bound(imuTimestamps.begin(),
                                                 imuTimestamps.end(), tImu);
                int idx = (it == imuTimestamps.end()) ? imuTimestamps.size() - 1
                                                      : (int)(it - imuTimestamps.begin());
                if (idx > 0 && imuTimestamps[idx] > tImu) idx--;
                if (idx + 1 < imuOrientations.size()) {
                    double dtIdx = imuTimestamps[idx + 1] - imuTimestamps[idx];
                    float frac = (dtIdx > 0.0) ? (float)((tImu - imuTimestamps[idx]) / dtIdx) : 0.0f;
                    frac = qBound(0.0f, frac, 1.0f);
                    qActual = QQuaternion::slerp(imuOrientations[idx],
                                                 imuOrientations[idx + 1], frac);
                }
            }
            QQuaternion qVirtual = GyroscopeIntegrator::orientationAt(
                imuOrientations, imuTimestamps, tImu, imuSmoothingMs);
            s.imuOrientation = qActual.conjugated() * qVirtual;
        } else {
            s.imuOrientation = QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }
        return s;
    }
};

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
    Q_PROPERTY(double imuDrift READ imuDrift WRITE setImuDrift NOTIFY imuDriftChanged)
    Q_PROPERTY(double imuAccelKi READ imuAccelKi WRITE setImuAccelKi NOTIFY imuAccelKiChanged)
    Q_PROPERTY(bool flowStitch READ flowStitch WRITE setFlowStitch NOTIFY flowStitchChanged)
    Q_PROPERTY(double flowStrength READ flowStrength WRITE setFlowStrength NOTIFY flowStrengthChanged)
    Q_PROPERTY(int flowIterations READ flowIterations WRITE setFlowIterations NOTIFY flowIterationsChanged)
    Q_PROPERTY(double flowAlpha READ flowAlpha WRITE setFlowAlpha NOTIFY flowAlphaChanged)
    Q_PROPERTY(QImage flowImage READ flowImage NOTIFY flowImageReady)
    Q_PROPERTY(double flowEncode READ flowEncode NOTIFY flowImageReady)
    Q_PROPERTY(bool seamStitch READ seamStitch WRITE setSeamStitch NOTIFY seamStitchChanged)
    Q_PROPERTY(double seamStrength READ seamStrength WRITE setSeamStrength NOTIFY seamStrengthChanged)
    Q_PROPERTY(QImage seamImage READ seamImage NOTIFY seamImageChanged)
    Q_PROPERTY(QString previewThumbnailPath READ previewThumbnailPath NOTIFY videoPathChanged)
    Q_PROPERTY(bool usePreviewThumbnail READ usePreviewThumbnail WRITE setUsePreviewThumbnail NOTIFY usePreviewThumbnailChanged)
    Q_PROPERTY(int activeLens READ activeLens WRITE setActiveLens NOTIFY activeLensChanged)
    Q_PROPERTY(int projection READ projection WRITE setProjection NOTIFY projectionChanged)
    Q_PROPERTY(VideoDecoder* videoDecoder READ videoDecoder CONSTANT)
    Q_PROPERTY(CalibrationPresetModel* calibrationPresets READ calibrationPresets CONSTANT)
    Q_PROPERTY(CalibrationProfile* currentCalibration READ currentCalibration NOTIFY currentCalibrationChanged)
    Q_PROPERTY(ColorGrade* colorGrade READ colorGrade CONSTANT)
    Q_PROPERTY(QQuaternion imuOrientation READ imuOrientation NOTIFY imuOrientationChanged)
    Q_PROPERTY(KeyframeModel* keyframes READ keyframes CONSTANT)
    Q_PROPERTY(bool exportRunning READ exportRunning NOTIFY exportRunningChanged)
    Q_PROPERTY(double exportProgress READ exportProgress NOTIFY exportProgressChanged)
    Q_PROPERTY(QString exportStatus READ exportStatus NOTIFY exportStatusChanged)
    Q_PROPERTY(double exportStart READ exportStart WRITE setExportStart NOTIFY exportStartChanged)
    Q_PROPERTY(double exportEnd READ exportEnd WRITE setExportEnd NOTIFY exportEndChanged)
    Q_PROPERTY(int exportWidth READ exportWidth WRITE setExportWidth NOTIFY exportWidthChanged)
    Q_PROPERTY(int exportHeight READ exportHeight WRITE setExportHeight NOTIFY exportHeightChanged)
    Q_PROPERTY(double exportFps READ exportFps WRITE setExportFps NOTIFY exportFpsChanged)
    Q_PROPERTY(QString exportCodec READ exportCodec WRITE setExportCodec NOTIFY exportCodecChanged)
    Q_PROPERTY(int exportCrf READ exportCrf WRITE setExportCrf NOTIFY exportCrfChanged)
    Q_PROPERTY(int exportBitrate READ exportBitrate WRITE setExportBitrate NOTIFY exportBitrateChanged)
    Q_PROPERTY(bool exportVidstab READ exportVidstab WRITE setExportVidstab NOTIFY exportVidstabChanged)
    Q_PROPERTY(bool exportVidstabInformed READ exportVidstabInformed WRITE setExportVidstabInformed NOTIFY exportVidstabInformedChanged)
    Q_PROPERTY(QString exportFileName READ exportFileName WRITE setExportFileName NOTIFY exportFileNameChanged)
    Q_PROPERTY(bool calibrationRunning READ calibrationRunning NOTIFY calibrationRunningChanged)
    Q_PROPERTY(double calibrationProgress READ calibrationProgress NOTIFY calibrationProgressChanged)
    Q_PROPERTY(QString calibrationStatus READ calibrationStatus NOTIFY calibrationStatusChanged)

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

    double imuDrift() const;
    void setImuDrift(double drift);

    double imuAccelKi() const;
    void setImuAccelKi(double ki);

    bool flowStitch() const { return m_flowStitch; }
    void setFlowStitch(bool v);
    double flowStrength() const { return m_flowStrength; }
    void setFlowStrength(double v);
    int flowIterations() const { return m_flowIterations; }
    void setFlowIterations(int v);
    double flowAlpha() const { return m_flowAlpha; }
    void setFlowAlpha(double v);
    QImage flowImage() const { return m_flowImage; }
    double flowEncode() const { return m_flowEncode; }
    bool seamStitch() const { return m_seamStitch; }
    void setSeamStitch(bool v);
    double seamStrength() const { return m_seamStrength; }
    void setSeamStrength(double v);
    QImage seamImage() const { return m_seamImage; }

    int activeLens() const;
    void setActiveLens(int lens);

    int projection() const;
    void setProjection(int projection);

    VideoDecoder* videoDecoder() const;

    CalibrationPresetModel* calibrationPresets() const;
    CalibrationProfile* currentCalibration() const;
    ColorGrade* colorGrade() const { return m_colorGrade; }

    QQuaternion imuOrientation() const;

    KeyframeModel* keyframes() const { return m_keyframes; }

    bool exportRunning() const { return m_exportRunning; }
    double exportProgress() const { return m_exportProgress; }
    QString exportStatus() const { return m_exportStatus; }
    double exportStart() const { return m_exportStart; }
    void setExportStart(double time);
    double exportEnd() const { return m_exportEnd; }
    void setExportEnd(double time);

    int exportWidth() const { return m_keyframes->exportWidth(); }
    void setExportWidth(int width);
    int exportHeight() const { return m_keyframes->exportHeight(); }
    void setExportHeight(int height);
    double exportFps() const { return m_keyframes->exportFps(); }
    void setExportFps(double fps);
    QString exportCodec() const { return m_keyframes->exportCodec(); }
    void setExportCodec(const QString &codec);
    int exportCrf() const { return m_keyframes->exportCrf(); }
    void setExportCrf(int crf);
    int exportBitrate() const { return m_keyframes->exportBitrate(); }
    void setExportBitrate(int bitrate);
    bool exportVidstab() const { return m_keyframes->exportVidstab(); }
    void setExportVidstab(bool vidstab);
    bool exportVidstabInformed() const { return m_keyframes->exportVidstabInformed(); }
    void setExportVidstabInformed(bool informed);

    QString exportFileName() const { return m_keyframes->exportFileName(); }
    void setExportFileName(const QString &name);

    Q_INVOKABLE void exportFrame(const QString &path, int width, int height);
    Q_INVOKABLE void exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime, const QString &codec, int crf, int bitrateMbps, bool vidstab, bool vidstabInformed, bool gpuBackend = true);
    Q_INVOKABLE QString grabStill(int lens);
    Q_INVOKABLE void dragLook(double angleAboutUp, double angleAboutRight);

    Q_INVOKABLE void addKeyframeAtCurrent();
    Q_INVOKABLE void applyKeyframeInterpolation();

    // IMU drift calibration: samples frames across the current video, detects
    // the horizon in each, and finds the drift that minimizes the IMU-vs-horizon
    // roll error. Runs in a background thread; progress/result surface via
    // calibrationRunning/Progress/Status properties.
    Q_INVOKABLE void calibrateImuDrift();

    bool calibrationRunning() const { return m_calibrationRunning; }
    double calibrationProgress() const { return m_calibrationProgress; }
    QString calibrationStatus() const { return m_calibrationStatus; }

private:
    QQuaternion viewQuatFromEuler() const;
    void extractEulerFromQuat(const QQuaternion &q, double &yaw, double &pitch, double &roll) const;
    void setViewFromQuat(const QQuaternion &q);

    void loadSettings();
    void saveSettings() const;
    void saveKeyframes();
    void setExportRunning(bool running);
    void setExportProgress(double progress);
    ExportSnapshot buildExportSnapshot() const;
    void integrateImu();
    // IMU<->video clock drift estimated from the two stream durations. The IMU
    // and video are recorded on independent clocks of slightly different
    // rates, so drift = (imuDuration - videoDuration) / videoDuration.
    double autoImuDrift() const;

    // IMU-stabilized view quaternion at an arbitrary time (thread-safe: only
    // reads fixed integrator data).
    QQuaternion imuOrientationAt(double time) const;

    // Optical-flow preview: request a recompute for the current frame (queued
    // to the flow worker), and consume its packed result.
    void maybeComputeFlow();
    void onFlowReady(const QImage &image, float encodeScale, const QImage &seamImage);

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
    void imuDriftChanged();
    void imuAccelKiChanged();
    void flowStitchChanged();
    void flowStrengthChanged();
    void flowIterationsChanged();
    void flowAlphaChanged();
    void seamStitchChanged();
    void seamStrengthChanged();
    void seamImageChanged();
    void flowImageReady();
    void usePreviewThumbnailChanged();
    void activeLensChanged();
    void projectionChanged();
    void currentCalibrationChanged();
    void imuOrientationChanged();
    void videoLoaded();
    void flowRequested(const DecodedFrame &frame, const FlowCalibration &cal,
                       const FlowSettings &settings);
    void errorOccurred(const QString &message);
    void exportRunningChanged();
    void exportProgressChanged();
    void exportStatusChanged();
    void exportStartChanged();
    void exportEndChanged();
    void exportWidthChanged();
    void exportHeightChanged();
    void exportFpsChanged();
    void exportCodecChanged();
    void exportCrfChanged();
    void exportBitrateChanged();
    void exportVidstabChanged();
    void exportVidstabInformedChanged();
    void exportFileNameChanged();
    void calibrationRunningChanged();
    void calibrationProgressChanged();
    void calibrationStatusChanged();

private:
    VideoDecoder *m_decoder;
    ImuParser *m_imuParser;
    GyroscopeIntegrator *m_gyroIntegrator;
    CalibrationPresetModel *m_calibrationPresets;
    CalibrationProfile *m_currentCalibration;
    ColorGrade *m_colorGrade;
    KeyframeModel *m_keyframes;
    Exporter *m_exporter;
    QTimer *m_trimSaveTimer;  // coalesces sidecar writes while dragging trim

    bool m_exportRunning;
    double m_exportProgress;
    QString m_exportStatus;
    // The trim in/out markers (kept in App) and the last-used export output
    // options (kept in the KeyframeModel) are both persisted per-video in the
    // keyframe sidecar file.
    double m_exportStart;
    double m_exportEnd;
    bool m_restoringSidecar = false;  // set while loading a video's sidecar file

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
    double m_imuDrift;
    double m_imuAccelKi;
    bool m_usePreview;
    int m_activeLens;
    int m_projection;
    QQuaternion m_viewQuat;

    bool m_flowStitch;
    double m_flowStrength;
    int m_flowIterations;
    double m_flowAlpha;
    QImage m_flowImage;
    double m_flowEncode;
    FlowWorker *m_flowWorker;
    QThread *m_flowThread;
    // True while a flow job is queued/in flight on the worker thread. Used to
    // implement latest-wins scheduling: flow for intermediate frames is
    // dropped so the seam never lags unboundedly behind the decode rate.
    bool m_flowPending;
    // Timestamp of the frame whose flow job is in flight; onFlowReady compares
    // it against the decoder's newest frame to decide whether to run a follow-up
    // (so a paused playback doesn't recompute the same frame in a loop).
    double m_flowLastTs;
    bool m_seamStitch;
    double m_seamStrength;
    QImage m_seamImage;

    ImuDriftCalibrator *m_driftCalibrator;
    bool m_calibrationRunning;
    double m_calibrationProgress;
    QString m_calibrationStatus;
};

#endif // APP_H
