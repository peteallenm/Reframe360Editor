// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef APP_H
#define APP_H

#include <QObject>
#include "folderaccess.h"
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
#include "track.h"
#include "objecttracker.h"
#include "exporter.h"
#include "flowrenderer.h"
#include "visualrotation.h"
#include "syncsolver.h"
#include "gyrocalibration.h"
#include "visualfusion.h"
#include <QMatrix3x3>
#include <QVector3D>

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
            // Dual-mode stabilization (matches App::imuOrientationAt):
            //  - imuSmoothingMs <= 270 (smoothing <= 0.9): q_virtual is the
            //    Gaussian-smoothed "virtual tripod" path -> HIGH-PASS. Removes
            //    shake, keeps intentional motion (a 360° pan stays visible).
            //  - imuSmoothingMs > 270 (smoothing > 0.9): q_virtual is pinned to
            //    the first orientation -> LOW-PASS / hold-world-steady. Cancels
            //    ALL rotation (even a full 360°), holding the world fixed.
            // Return q_virtual^{-1} * q_actual.
            // q_actual must be UNSMOOTHED. orientationAt(..., 0.0f) is not:
            // it still applies the 33 ms Gaussian floor. The old code called it
            // and then threw the result away for an inline slerp on the common
            // path, leaving the smoothed value in place only at the clip's
            // first/last sample -- so export and preview disagreed exactly at
            // the edges. Compute the slerp directly and skip the wasted
            // Gaussian pass (it ran once per exported frame).
            QQuaternion qActual = imuOrientations.last();
            if (tImu <= imuTimestamps.first()) {
                qActual = imuOrientations.first();
            } else if (tImu < imuTimestamps.last()) {
                const auto it = std::lower_bound(imuTimestamps.begin(),
                                                 imuTimestamps.end(), tImu);
                int idx = (it == imuTimestamps.end()) ? imuTimestamps.size() - 1
                                                      : (int)(it - imuTimestamps.begin());
                if (idx > 0 && imuTimestamps[idx] > tImu) idx--;
                if (idx + 1 < imuOrientations.size()) {
                    const double dtIdx = imuTimestamps[idx + 1] - imuTimestamps[idx];
                    float frac = (dtIdx > 0.0) ? (float)((tImu - imuTimestamps[idx]) / dtIdx) : 0.0f;
                    frac = qBound(0.0f, frac, 1.0f);
                    qActual = QQuaternion::slerp(imuOrientations[idx],
                                                 imuOrientations[idx + 1], frac);
                } else {
                    qActual = imuOrientations[idx];
                }
            }

            QQuaternion qVirtual;
            if (imuSmoothingMs > 270.0f) {
                qVirtual = imuOrientations.first();
                if (qVirtual.isNull())
                    qVirtual = qActual;          // guard the preview path has
            } else {
                qVirtual = GyroscopeIntegrator::orientationAt(
                    imuOrientations, imuTimestamps, tImu, imuSmoothingMs);
            }
            // Same single definition the preview uses, so the two cannot drift.
            s.imuOrientation = composeStabilisation(qActual, qVirtual);
        } else {
            s.imuOrientation = kFlipRollQ();
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
    Q_PROPERTY(QString loadError READ loadError NOTIFY loadErrorChanged)
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
    // Folder-wide access (Android's Storage Access Framework tree grant): lets
    // a clip's .imu / _thm / .keyframes.json be resolved by name, and the
    // keyframe sidecar written back, neither of which a single-file grant can
    // do. Inert on desktop, where sidecars are found by path.
    Q_PROPERTY(FolderAccess* folder READ folder CONSTANT)
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
    Q_PROPERTY(bool autoSyncRunning READ autoSyncRunning NOTIFY autoSyncRunningChanged)
    Q_PROPERTY(double autoSyncProgress READ autoSyncProgress NOTIFY autoSyncProgressChanged)
    Q_PROPERTY(QString autoSyncStatus READ autoSyncStatus NOTIFY autoSyncStatusChanged)

    // Object tracking (same shape as the autoSync trio above).
    Q_PROPERTY(bool trackRunning READ trackRunning NOTIFY trackRunningChanged)
    Q_PROPERTY(double trackProgress READ trackProgress NOTIFY trackProgressChanged)
    Q_PROPERTY(QString trackStatus READ trackStatus NOTIFY trackStatusChanged)
    Q_PROPERTY(int trackCount READ trackCount NOTIFY tracksChanged)
    // True while the viewer is waiting for the user to point at something.
    Q_PROPERTY(bool trackArmed READ trackArmed WRITE setTrackArmed NOTIFY trackArmedChanged)
    Q_PROPERTY(int trackPointCount READ trackPointCount NOTIFY trackPointsChanged)
    // Smoothing applied to the tracked direction, in seconds. Applies to every
    // track at once and to new ones: it is a taste setting, not a per-subject
    // one, and a track re-resolves from its stored samples so it can be
    // changed long after the pass was run.
    Q_PROPERTY(double trackSmoothing READ trackSmoothing WRITE setTrackSmoothing
               NOTIFY trackSmoothingChanged)

    // Opening a clip is slow enough to look like a hang -- a 2 GB file over
    // SAF on the phone especially -- so it reports where it has got to.
    Q_PROPERTY(bool loading READ loading NOTIFY loadingChanged)
    Q_PROPERTY(double loadProgress READ loadProgress NOTIFY loadProgressChanged)
    Q_PROPERTY(QString loadStatus READ loadStatus NOTIFY loadProgressChanged)

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
    FolderAccess* folder() const { return m_folder; }

    // Open a clip from the granted folder, resolving its sidecars by name
    // within that folder.
    Q_INVOKABLE void openClipFromFolder(const QString &displayName);

    // A writable destination for an export. On Android nothing can be written
    // by path, so this creates a document in the granted folder and returns
    // its content:// URI; elsewhere it just returns the suggested path.
    Q_INVOKABLE QString exportDestination(const QString &suggestedName);

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
    Q_INVOKABLE void exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime, const QString &codec, int crf, int bitrateMbps, bool vidstab, bool vidstabInformed, bool gpuBackend = true, bool spherical360 = false, bool copyAudio = true);
    Q_INVOKABLE QString grabStill(int lens);

    // Open a clip together with explicitly supplied sidecars. On Android the
    // picker returns opaque content:// URIs: you cannot append ".imu" to one,
    // and a single-file grant does not cover siblings, so the sidecars have to
    // be selected alongside the video and passed in here. Empty arguments fall
    // back to deriving the path from the video, which is what the desktop does.
    Q_INVOKABLE void openClip(const QString &video, const QString &imu,
                              const QString &proxy, const QString &keyframes);

    // Last error from the decoder, shown over the viewer. Cleared whenever a
    // load succeeds. Without this the error signal went nowhere and a failed
    // open just left the previous clip's frame on screen, looking like a hang.
    QString loadError() const { return m_loadError; }

    // 256-bin histogram of the current SOURCE frame for the curves editor:
    // { "r": [...], "g": [...], "b": [...], "luma": [...], "samples": n }.
    // Counts are raw. Measured before the colour grade, so it shows what you
    // are grading rather than the result of the grade. Returns an empty map
    // when no frame is decoded yet.
    Q_INVOKABLE QVariantMap frameHistogram(int maxSamples = 512) const;
    Q_INVOKABLE void dragLook(double angleAboutUp, double angleAboutRight);
    // Stabilisation quaternion for a given video time. Public so LensViewer can
    // pair the orientation with the exact frame it paints (render thread, GUI
    // thread blocked -- see LensViewer::updatePaintNode).
    QQuaternion imuOrientationAt(double time) const;

    Q_INVOKABLE void addKeyframeAtCurrent();
    Q_INVOKABLE void applyKeyframeInterpolation();

    bool trackRunning() const { return m_trackRunning; }
    double trackProgress() const { return m_trackProgress; }
    QString trackStatus() const { return m_trackStatus; }
    int trackCount() const { return m_tracks.size(); }
    bool trackArmed() const { return m_trackArmed; }
    int trackPointCount() const { return m_pendingDirs.size(); }
    double trackSmoothing() const { return m_trackSmoothing; }
    void setTrackSmoothing(double seconds);
    bool loading() const { return m_loading; }
    double loadProgress() const { return m_loadProgress; }
    QString loadStatus() const { return m_loadStatus; }
    void setTrackArmed(bool armed);

    // Arm/pick: the viewer hands back where the user pointed, in NDC, plus the
    // aspect of the surface they pointed at (the framing must be resolved
    // against that, never against the live pane or the export size).
    // Mark a patch on the subject. Several may be marked before tracking
    // starts -- a head and a shirt, say -- and the pass follows whichever is
    // matching best, retrying the others from where that one says they are.
    Q_INVOKABLE void addTrackPoint(double ndcX, double ndcY, double aspect,
                                   double sizeFraction = 0.10);
    Q_INVOKABLE void beginTracking();
    Q_INVOKABLE void clearTrackPoints();
    Q_INVOKABLE QVariantList trackPointNdc(double aspect) const;
    Q_INVOKABLE void cancelTracking();
    Q_INVOKABLE void removeTrack(int index);
    Q_INVOKABLE void clearTracks();
    // Drop the beginning or the end of a track without discarding what was
    // measured: the samples stay, only the part that drives the view narrows,
    // so a handle dragged back out restores the track exactly.
    Q_INVOKABLE void setTrackTrim(int index, double tIn, double tOut);
    Q_INVOKABLE void resetTrackTrim(int index);
    // The track's reference zoom: the fov the framing was picked at. Changing
    // it re-frames the whole track, because the subject is held at a fraction
    // of the frame rather than at a fixed angle -- so this is "how close do you
    // want to be", and the track keeps the subject in the same spot either way.
    Q_INVOKABLE void setTrackZoom(int index, double fovDeg);
    // Whether the view chases the subject's apparent size at all. The size
    // estimate is the least reliable part of a track; being able to switch it
    // off is the difference between a usable track and a deleted one.
    Q_INVOKABLE void setTrackFollowSize(int index, bool on);
    // Time span of a track, for the timeline: [start, end] or an empty list.
    Q_INVOKABLE QVariantList trackSpan(int index) const;
    // Every track as {start, end, lost}, for the timeline to draw. A list of
    // maps rather than a model: the timeline redraws them wholesale whenever
    // tracksChanged fires, and there is nothing to select or edit in place.
    Q_INVOKABLE QVariantList trackSpans() const;
    // Where the tracked subject is on screen right now, as {ndcX, ndcY};
    // empty when no track covers this time. Without this there is no way to
    // see that a track is holding rather than quietly drifting.
    Q_INVOKABLE QVariantList trackMarkerNdc(double aspect) const;

    bool autoSyncRunning() const { return m_autoSyncRunning; }
    double autoSyncProgress() const { return m_autoSyncProgress; }
    QString autoSyncStatus() const { return m_autoSyncStatus; }
    Q_INVOKABLE void autoSyncAndCalibrate();

    // Gyro calibration is applied silently on every video load (sidecar first,
    // camera default second), so there must be a way to undo one that turned
    // out to be wrong — otherwise a bad AutoSync is permanent and invisible.
    // Both of these re-integrate immediately so the effect is visible.
    Q_INVOKABLE void clearGyroCalibration();        // this video's sidecar
    Q_INVOKABLE void clearCameraGyroDefaults();     // camera-wide fallback
    // Promote this video's calibration to the camera-wide default. Explicit
    // only: AutoSync never does this on its own.
    Q_INVOKABLE bool saveGyroCalibrationAsCameraDefault();
    Q_INVOKABLE bool hasGyroCalibration() const;
    Q_INVOKABLE bool hasCameraGyroDefaults() const;

    QMatrix3x3 cameraGyroMatrix() const;
    void setCameraGyroMatrix(const QMatrix3x3 &M);
    QVector3D cameraGyroBias() const;
    void setCameraGyroBias(const QVector3D &b);


private:
    QQuaternion viewQuatFromEuler() const;
    void extractEulerFromQuat(const QQuaternion &q, double &yaw, double &pitch, double &roll) const;

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

    // Optical-flow preview: request a recompute for the current frame (queued
    // to the flow worker), and consume its packed result.
    void maybeComputeFlow();
    void onFlowReady(const QImage &image, float encodeScale, const QImage &seamImage);

signals:
    // A newly decoded frame is available, so any displayed histogram is stale.
    void histogramChanged();
    void loadErrorChanged();
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
    void autoSyncRunningChanged();
    void autoSyncProgressChanged();
    void autoSyncStatusChanged();
    void trackRunningChanged();
    void trackProgressChanged();
    void trackStatusChanged();
    void trackArmedChanged();
    void trackPointsChanged();
    void trackSmoothingChanged();
    void loadingChanged();
    void loadProgressChanged();
    void tracksChanged();

private:
    VideoDecoder *m_decoder;
    ImuParser *m_imuParser;
    GyroscopeIntegrator *m_gyroIntegrator;
    CalibrationPresetModel *m_calibrationPresets;
    CalibrationProfile *m_currentCalibration;
    ColorGrade *m_colorGrade;
    FolderAccess *m_folder = nullptr;
    // Display name of the clip when it was opened from the granted folder;
    // empty for a clip opened any other way. Used to write the sidecar back.
    QString m_folderClipName;
    // Destination document for an export that must be copied there afterwards
    // (a content:// URI cannot be muxed into directly).
    QString m_exportFinalUri;
    KeyframeModel *m_keyframes;
    Exporter *m_exporter;
    QTimer *m_trimSaveTimer;  // coalesces sidecar writes while dragging trim
    // Coalesces re-integration while dragging the Mahony gain slider. A full
    // integrate() + refuse is ~45 ms on a 140 s clip and the slider emits on
    // every mouse-move, so without this the UI stutters through a drag.
    QTimer *m_reintegrateTimer = nullptr;

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
    QString m_loadError;
    // Sidecars supplied explicitly by openClip(); empty means "derive it".
    QString m_imuOverride;
    QString m_proxyOverride;
    QString m_keyframesOverride;
    QString m_pendingImu;
    QString m_pendingProxy;
    QString m_pendingKeyframes;

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

    // Auto-sync pipeline
    VisualRotationComputer *m_visualRotation = nullptr;
    ObjectTracker *m_objectTracker = nullptr;
    QVector<Track> m_tracks;
    // The manual keyframes with every track's resolved keyframes folded in --
    // the array preview and export both interpolate. Rebuilt lazily, and
    // invalidated whenever anything the resolution depends on moves (the IMU
    // chain, the calibration, the keyframes, the trim).
    mutable QVector<Keyframe> m_effectiveKeyframes;
    mutable bool m_effectiveDirty = true;
    bool m_trackRunning = false;
    bool m_trackArmed = false;
    double m_trackProgress = 0.0;
    double m_trackSmoothing = 0.10;   // seconds; see Track::posSmoothSec
    QString m_trackStatus;
    Track m_pendingTrack;          // being filled by the running pass
    QVector<QVector3D> m_pendingDirs;    // marked patches, camera frame at t0
    QVariantList m_pendingNdc;           // ... and where they were tapped
    double m_pendingRadius = 0.05;
    double m_pendingTime = 0.0;
    bool m_loading = false;
    double m_loadProgress = 0.0;
    QString m_loadStatus;
    // Report a stage and let the UI repaint. The load runs on the GUI thread,
    // so without pumping events here the bar would only appear once the work
    // it is reporting on had finished. User input stays excluded, so nothing
    // can be clicked mid-load.
    void setLoadStage(double fraction, const QString &text);
    QString m_pendingTrackVideo;   // guards against a result for the old clip

    const QVector<Keyframe> &effectiveKeyframes() const;
    void invalidateTrackCache();
    // Call after anything that changes imuOrientationAt(): the resolved tracks
    // are only valid for the chain they were built against.
    void imuChainChanged();
    void syncTracksToSidecar();
    TrackLenses trackLenses() const;
    SyncSolver *m_syncSolver = nullptr;
    GyroCalibrator *m_gyroCalibrator = nullptr;
    // Rebuild the optical drift correction on the current IMU chain (no-op
    // when this clip has no visual pairs). Called after every re-integration
    // and as AutoSync stage 4.
    void applyVisualFusion();

    VisualFusion *m_visualFusion = nullptr;
    bool m_deferVisualFusion = false;  // AutoSync fuses on its own thread
    QVector<VisualRotationPair> m_visualPairs;
    bool m_autoSyncRunning = false;
    double m_autoSyncProgress = 0.0;
    QString m_autoSyncStatus;

};

#endif // APP_H
