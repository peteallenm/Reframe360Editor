// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include "app.h"
#include <QFileInfo>
#include <QDir>
#include <QStandardPaths>
#include <QImage>
#include <QSettings>
#include <QDateTime>
#include <QTimer>
#include <QThread>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QtMath>
#include <algorithm>
extern "C" {
#include <libswscale/swscale.h>
}

// Pitch is kept away from the vertical poles (~ +/-90 deg) where the yaw/roll
// Euler decomposition becomes degenerate and the sliders would swap yaw/roll.
static const double kMaxPitch = 89.5;


// Keyframes are persisted per-video as a JSON sidecar next to the source file
// (same convention as the .imu files), so re-opening a video restores its
// keyframe set automatically.
static QString keyframesPathFor(const QString &videoPath)
{
    return videoPath + QStringLiteral(".keyframes.json");
}

App::App(QObject *parent)
    : QObject(parent)
    , m_decoder(new VideoDecoder(this))
    , m_imuParser(new ImuParser(this))
    , m_gyroIntegrator(new GyroscopeIntegrator(this))
    , m_calibrationPresets(new CalibrationPresetModel(this))
    , m_currentCalibration(new CalibrationProfile(this))
    , m_colorGrade(new ColorGrade(this))
    , m_keyframes(new KeyframeModel(this))
    , m_exporter(new Exporter(this))
    , m_trimSaveTimer(nullptr)
    , m_isPlaying(false)
    , m_currentTime(0.0)
    , m_yaw(0.0)
    , m_pitch(0.0)
    , m_roll(0.0)
    , m_fov(90.0)
    // Stabilised, holding the world steady, is what this app is FOR: a fresh
    // install should show a stabilised clip, not raw shake that has to be
    // switched on. smoothing 1.0 is "hold world steady" (below 0.9 is the
    // follow-camera high-pass mode).
    , m_imuStabilize(true)
    , m_imuSmoothing(1.0)
    // Linear sync model: offset(t) = m_imuSyncOffset + m_imuDrift * t. The
    // measured IMU<->video clock drift on this camera is ~0.0038 s/s, so the
    // offset that works at t=0 (~0.17 s) grows to ~0.26 s by 24 s.
    , m_imuSyncOffset(0.17)
    // 0 s/s. The old default of 0.0038 was not only unmeasured, it was nearly
    // double autoImuDrift()'s own kMaxPlausibleDrift (0.002) and 7x the sync
    // solver's MAX_DRIFT (5e-4) — a fresh install started the solve from a
    // drift its own sanity checks would reject. Drift is measured per clip or
    // it is zero.
    , m_imuDrift(0.0)
    , m_imuAccelKi(0.005)
    // Parallax stitching on by default too: the plain blend visibly ghosts
    // anything near the seam, and the disparity matcher is what makes the
    // stitch look right out of the box.
    , m_flowStitch(true)
    , m_flowStrength(1.0)
    , m_flowIterations(kDefaultFlowIterations)
    , m_flowAlpha(kDefaultFlowAlpha)
    , m_flowEncode(16.0)
    , m_flowPending(false)
    , m_flowLastTs(-1.0)
    , m_seamStitch(true)
    , m_seamStrength(1.0)
    , m_usePreview(false)
    , m_activeLens(2)
    , m_projection(0)
    , m_exportRunning(false)
    , m_exportProgress(0.0)
    , m_exportStatus()
    , m_exportStart(0.0)
    , m_exportEnd(0.0)
    , m_autoSyncRunning(false)
    , m_autoSyncProgress(0.0)
{
    connect(m_decoder, &VideoDecoder::durationChanged, this, [this]() {
        // Keep the export trim range inside the newly known clip length.
        const double dur = m_decoder->duration();
        if (dur > 0.0) {
            if (m_exportEnd <= 0.0 || m_exportEnd > dur)
                m_exportEnd = dur;
            m_exportStart = qBound(0.0, m_exportStart, m_exportEnd);
            emit exportStartChanged();
            emit exportEndChanged();
        }
    });
    connect(m_decoder, &VideoDecoder::durationChanged, this, &App::durationChanged);
    connect(m_decoder, &VideoDecoder::currentTimeChanged, this, [this]() {
        m_currentTime = m_decoder->currentTime();
        applyKeyframeInterpolation();
        emit currentTimeChanged();
        if (m_imuStabilize)
            emit imuOrientationChanged();
    });
    connect(m_decoder, &VideoDecoder::errorOccurred, this, &App::errorOccurred);
    // Surface it: the signal alone went nowhere, so a failed open silently left
    // the previous clip's last frame on screen.
    connect(m_decoder, &VideoDecoder::errorOccurred, this, [this](const QString &msg) {
        m_isPlaying = false;
        emit isPlayingChanged();
        m_loadError = msg;
        emit loadErrorChanged();
        qWarning().noquote() << "Load failed:" << msg;
    });

    // Optical-flow stitching: a dedicated worker thread owns the FlowRenderer
    // (QOpenGLContext is thread-affine — the 4.4 context is created lazily on
    // the worker). Recompute whenever a new decoded frame arrives or the
    // calibration changes, and hand the packed RGBA8 field to the preview.
    m_flowThread = new QThread(this);
    m_flowWorker = new FlowWorker;
    m_flowWorker->moveToThread(m_flowThread);
    connect(m_flowThread, &QThread::finished, m_flowWorker, &QObject::deleteLater);
    connect(this, &App::flowRequested, m_flowWorker, &FlowWorker::computeFrame);
    connect(m_flowWorker, &FlowWorker::flowReady, this, &App::onFlowReady);
    connect(m_flowWorker, &FlowWorker::flowFailed, this, [this](const QString &message) {
        m_flowPending = false;
        qWarning().noquote() << "Flow stitching preview:" << message;
    });
    m_flowThread->start();

    connect(m_decoder, &VideoDecoder::frameReady, this, [this]() { maybeComputeFlow(); emit histogramChanged(); });
    auto onCalChanged = [this]() { maybeComputeFlow(); };
    connect(m_currentCalibration, &CalibrationProfile::frontCenterXChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontCenterYChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontRadiusChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontK1Changed, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontK2Changed, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontRotationChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::frontHFlipChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearCenterXChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearCenterYChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearRadiusChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearK1Changed, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearK2Changed, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearRotationChanged, this, onCalChanged);
    connect(m_currentCalibration, &CalibrationProfile::rearHFlipChanged, this, onCalChanged);

    // Keep the per-video keyframe sidecar file in sync whenever the user adds,
    // edits or deletes keyframes.
    connect(m_keyframes, &KeyframeModel::keyframesChanged, this, &App::saveKeyframes);

    // The last-used export output options live in the same sidecar file, so
    // surface their changes to QML and persist them when the user picks new
    // encoder settings. During loadFromFile() (switching videos) the model
    // has already been restored, so a write here would just rewrite what we
    // read — guarded against by m_restoringSidecar.
    connect(m_keyframes, &KeyframeModel::exportSettingsChanged, this, [this]() {
        emit exportWidthChanged();
        emit exportHeightChanged();
        emit exportFpsChanged();
        emit exportCodecChanged();
        emit exportCrfChanged();
        emit exportBitrateChanged();
        emit exportVidstabChanged();
        emit exportVidstabInformedChanged();
        emit exportFileNameChanged();
        if (!m_restoringSidecar)
            saveKeyframes();
    });

    // The export in/out markers live in the same sidecar file, so persist them
    // too. The timeline handles drag continuously, so coalesce the writes
    // rather than rewriting the file on every pixel of movement.
    m_trimSaveTimer = new QTimer(this);
    m_trimSaveTimer->setSingleShot(true);
    m_trimSaveTimer->setInterval(300);
    // Only write the sidecar when the trim actually moved away from the state
    // that was loaded from it — otherwise merely opening a video would
    // rewrite (and reformat/version-bump) the file on every open.
    connect(m_trimSaveTimer, &QTimer::timeout, this, [this]() {
        if (!qFuzzyCompare(m_exportStart, m_keyframes->trimIn())
            || !qFuzzyCompare(m_exportEnd, m_keyframes->trimOut()))
            saveKeyframes();
    });
    connect(this, &App::exportStartChanged, this, [this]() { m_trimSaveTimer->start(); });
    connect(this, &App::exportEndChanged, this, [this]() { m_trimSaveTimer->start(); });

    // Surface export progress / completion to the UI.
    connect(m_exporter, &Exporter::exportProgress, this, [this](double p) {
        setExportProgress(p);
        m_exportStatus = tr("Rendering… %1%").arg((int)qRound(p * 100.0));
        emit exportStatusChanged();
    });
    connect(m_exporter, &Exporter::exportFinished, this, [this](const QString &path) {
        QString finalPath = path;
        if (!m_exportFinalUri.isEmpty()) {
            // Copy the rendered file into the granted folder's document, then
            // drop the scratch copy.
            QFile src(path);
            QFile dst(m_exportFinalUri);
            bool ok = src.open(QIODevice::ReadOnly)
                   && dst.open(QIODevice::WriteOnly | QIODevice::Truncate);
            if (ok) {
                const qint64 kChunk = 1 << 20;
                while (!src.atEnd()) {
                    const QByteArray buf = src.read(kChunk);
                    if (buf.isEmpty() || dst.write(buf) != buf.size()) { ok = false; break; }
                }
            }
            src.close();
            dst.close();
            QFile::remove(path);
            if (!ok) {
                qWarning().noquote() << "Export: could not copy the render into" << m_exportFinalUri;
                m_exportStatus = tr("Export failed: could not write to the chosen folder");
                emit exportStatusChanged();
                QTimer::singleShot(2500, this, [this]() { setExportRunning(false); });
                m_exportFinalUri.clear();
                return;
            }
            finalPath = m_exportFinalUri;
            m_exportFinalUri.clear();
        }
        qInfo().noquote() << "Export complete:" << finalPath;
        m_exportStatus = tr("Export complete: %1").arg(QFileInfo(finalPath).fileName());
        emit exportStatusChanged();
        QTimer::singleShot(1500, this, [this]() { setExportRunning(false); });
    });
    connect(m_exporter, &Exporter::exportError, this, [this](const QString &message) {
        // Also to the log: the status text is transient and easy to miss, and
        // a failed export otherwise looks like nothing happening at all.
        qWarning().noquote() << "Export failed:" << message;
        m_exportStatus = tr("Export failed: %1").arg(message);
        emit exportStatusChanged();
        QTimer::singleShot(2500, this, [this]() { setExportRunning(false); });
    });

    m_viewQuat = viewQuatFromEuler();

    int defaultIdx = m_calibrationPresets->defaultPresetIndex();
    if (defaultIdx >= 0)
        m_calibrationPresets->loadPreset(defaultIdx, m_currentCalibration);

    // Restore the saved view/settings, then keep them persisted automatically
    // whenever the user changes them.
    loadSettings();
    connect(this, &App::imuStabilizeChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuSmoothingChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuSyncOffsetChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuDriftChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuAccelKiChanged, this, [this]() { saveSettings(); });
    connect(this, &App::projectionChanged, this, [this]() { saveSettings(); });
    connect(this, &App::flowStitchChanged, this, [this]() { saveSettings(); });
    connect(this, &App::flowStrengthChanged, this, [this]() { saveSettings(); });
    connect(this, &App::flowIterationsChanged, this, [this]() { saveSettings(); });
    connect(this, &App::flowAlphaChanged, this, [this]() { saveSettings(); });
    connect(this, &App::seamStitchChanged, this, [this]() { saveSettings(); });
    connect(this, &App::seamStrengthChanged, this, [this]() { saveSettings(); });

    // Colour grade changes persist automatically (same convention as the
    // projection / IMU settings).
    auto saveGrade = [this]() { saveSettings(); emit histogramChanged(); };
    connect(m_colorGrade, &ColorGrade::brightnessChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::contrastChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::saturationChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::popChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::brightLowsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::brightLowMidsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::brightHighMidsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::brightHighsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::redLowsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::redMidsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::redHighsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::greenLowsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::greenMidsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::greenHighsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::blueLowsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::blueMidsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::blueHighsChanged, this, saveGrade);
    connect(m_colorGrade, &ColorGrade::curvesChanged, this, saveGrade);

    // Auto-sync pipeline objects
    m_folder = new FolderAccess(this);
    {
        // A tree grant is persistable, so remember it: the user picks the
        // camera's folder once, not once per launch.
        QSettings st;
        m_folder->restore(st.value(QStringLiteral("storage/folderUri")).toString());
    }
    connect(m_folder, &FolderAccess::folderChanged, this, [this]() {
        QSettings st;
        st.setValue(QStringLiteral("storage/folderUri"), m_folder->treeUri());
    });

    m_visualRotation = new VisualRotationComputer(this);
    m_syncSolver = new SyncSolver(this);
    m_gyroCalibrator = new GyroCalibrator(this);
    m_visualFusion = new VisualFusion();
}

App::~App()
{
    if (m_flowThread) {
        m_flowThread->quit();
        m_flowThread->wait();
    }
}

QString App::videoPath() const
{
    return m_videoPath;
}

QString App::exportDestination(const QString &suggestedName)
{
    if (!m_folder || !m_folder->hasFolder())
        return suggestedName;

    // Reduce to a BARE FILE NAME. A document in the tree is created by name,
    // so anything path- or URI-shaped has to be stripped -- otherwise the
    // whole content:// URI becomes the file name, and because the dialog then
    // remembers it, each export nests the previous URI inside the next one and
    // the name grows without limit.
    QString name = suggestedName;
    const int slash = name.lastIndexOf(QLatin1Char('/'));
    if (slash >= 0)
        name = name.mid(slash + 1);

    const bool looksLikeUriLitter = name.contains(QLatin1String("content___"))
                                 || name.contains(QLatin1Char('%'))
                                 || name.contains(QLatin1Char(':'));
    if (name.isEmpty() || looksLikeUriLitter) {
        const QString base = m_folderClipName.isEmpty()
                ? QStringLiteral("render360")
                : QFileInfo(m_folderClipName).completeBaseName();
        name = base + QStringLiteral("_export.mp4");
    }

    const QString uri = m_folder->writableUriFor(name, QStringLiteral("video/mp4"));
    return uri.isEmpty() ? suggestedName : uri;
}

void App::openClipFromFolder(const QString &displayName)
{
    if (!m_folder)
        return;
    const QString video = m_folder->uriFor(displayName);
    if (video.isEmpty())
        return;
    openClip(video,
             m_folder->imuFor(displayName),
             m_folder->proxyFor(displayName),
             m_folder->keyframesFor(displayName));
    // After openClip(), which clears it: any other way of opening a clip must
    // not inherit this name, or its keyframes would be written into the
    // folder under the wrong file.
    m_folderClipName = displayName;
}

void App::openClip(const QString &video, const QString &imu,
                   const QString &proxy, const QString &keyframes)
{
    m_folderClipName.clear();
    m_pendingImu = imu;
    m_pendingProxy = proxy;
    m_pendingKeyframes = keyframes;
    setVideoPath(video);
}

void App::setVideoPath(const QString &path)
{
    // Overrides only apply to the openClip() call that supplied them; a plain
    // setVideoPath (the CLI, or a second open) must fall back to deriving the
    // sidecars from the video path again.
    m_imuOverride = m_pendingImu;
    m_proxyOverride = m_pendingProxy;
    m_keyframesOverride = m_pendingKeyframes;
    m_pendingImu.clear();
    m_pendingProxy.clear();
    m_pendingKeyframes.clear();

    if (m_videoPath == path)
        return;

    m_videoPath = path;
    m_usePreview = false;
    m_flowPending = false;  // drop any in-flight flow job from the old clip
    m_flowLastTs = -1.0;

    // Drop the previous clip's optical results. The fused chain is NOT
    // persisted and was only ever cleared by AutoSync, so loading a second clip
    // left the integrator serving the first clip's fused orientations — wrong
    // attitude, wrong "which way is up", and dependent on load order.
    m_visualPairs.clear();
    if (m_gyroIntegrator)
        m_gyroIntegrator->clearFusedOrientations();
    emit videoPathChanged();
    emit usePreviewThumbnailChanged();

    // A new clip resets the export trim range; durationChanged will set the
    // end point once the decoder reports the clip length.
    m_exportStart = 0.0;
    m_exportEnd = 0.0;
    emit exportStartChanged();
    emit exportEndChanged();

    // A new clip resets the manual view to the level, straight-ahead default.
    setYaw(0.0);
    setPitch(0.0);
    setRoll(0.0);

    if (!path.isEmpty()) {
        // Restore this video's saved keyframes, export in/out markers and
        // last-used export options from the sidecar (or clear them when the
        // video has no sidecar yet). The guard prevents the export-settings
        // change handler from immediately rewriting the file it just read.
        m_restoringSidecar = true;
        m_keyframes->loadFromFile(m_keyframesOverride.isEmpty()
                                  ? keyframesPathFor(path) : m_keyframesOverride);
        m_restoringSidecar = false;
        m_exportStart = m_keyframes->trimIn();
        m_exportEnd = m_keyframes->trimOut();
        emit exportStartChanged();
        emit exportEndChanged();

        if (!m_loadError.isEmpty()) {
            m_loadError.clear();
            emit loadErrorChanged();
        }
#ifdef Q_OS_ANDROID
        // On a phone the proxy is the only preview source that plays smoothly
        // (the 5.7K original software-decodes at a few fps), so preview
        // defaults to the proxy whenever the clip has one. m_videoPath keeps
        // pointing at the full-resolution video, so export is unaffected, and
        // the toolbar toggle still switches to the original on request.
        const QString androidProxy = previewThumbnailPath();
        if (!androidProxy.isEmpty()) {
            m_usePreview = true;
            emit usePreviewThumbnailChanged();
            m_decoder->loadVideo(androidProxy);
        } else
#endif
        m_decoder->loadVideo(path);

        // loadVideo() resets the decoder clock to 0 without emitting
        // currentTimeChanged, so sync our clock and apply the restored
        // keyframes right away instead of waiting for the next seek/play tick.
        m_currentTime = m_decoder->currentTime();
        applyKeyframeInterpolation();

        // Paint the first frame. Without this the viewer stayed black until
        // Play: loadVideo() leaves the decode loop idle, but a seek decodes
        // to its target even while paused (and starts the loop if needed).
        m_decoder->seekTo(0.0);

        QString imuPath = m_imuOverride.isEmpty() ? (path + ".imu") : m_imuOverride;
        if (m_imuOverride.isEmpty() && !QFileInfo::exists(imuPath)) {
            // Thumbnail videos (e.g. YIVR_0807_360_thm.MP4) rarely have their
            // own .imu sidecar. Try the parent full-resolution video's .imu
            // file by stripping a trailing "_thm" before the extension.
            QFileInfo fi(path);
            QString base = fi.completeBaseName(); // e.g. "YIVR_0807_360_thm"
            if (base.endsWith(QStringLiteral("_thm"), Qt::CaseInsensitive)) {
                base.chop(4); // remove "_thm"
                QString parentImu = fi.absolutePath() + QLatin1Char('/')
                                  + base + QLatin1Char('.') + fi.suffix()
                                  + QStringLiteral(".imu");
                if (QFileInfo::exists(parentImu))
                    imuPath = parentImu;
            }
        }
        if (!m_imuOverride.isEmpty() || QFileInfo::exists(imuPath)) {
            m_imuParser->loadFile(imuPath);

            // IMU<->video drift: a value tuned for this video and stored in
            // its keyframe sidecar wins; otherwise use the auto-calculated
            // estimate from the relative stream durations. Only emit when it
            // actually changed so the QML slider tracks the applied value.
            double drift = autoImuDrift();
            if (m_keyframes->hasImuDrift())
                drift = m_keyframes->imuDrift();
            if (!qFuzzyCompare(m_imuDrift, drift)) {
                m_imuDrift = drift;
                emit imuDriftChanged();
            }

            // Restore stored sync offset from sidecar if available
            if (m_keyframes->hasSyncOffset()) {
                double offset = m_keyframes->syncOffset();
                if (!qFuzzyCompare(m_imuSyncOffset, offset)) {
                    m_imuSyncOffset = offset;
                    emit imuSyncOffsetChanged();
                }
            }

            integrateImu();
        }

        emit videoLoaded();
    }
}

QString App::previewThumbnailPath() const
{
    if (!m_proxyOverride.isEmpty())
        return m_proxyOverride;
    if (m_videoPath.isEmpty())
        return QString();

    QFileInfo fi(m_videoPath);
    const QString thumb = fi.absolutePath() + QLatin1Char('/')
                        + fi.completeBaseName() + QLatin1String("_thm.") + fi.suffix();
    return QFileInfo::exists(thumb) ? thumb : QString();
}

bool App::usePreviewThumbnail() const
{
    return m_usePreview;
}

void App::setUsePreviewThumbnail(bool use)
{
    if (m_usePreview == use)
        return;

    m_usePreview = use;
    emit usePreviewThumbnailChanged();

    if (m_videoPath.isEmpty())
        return;

    // Preview plays the low-res *_thm video; m_videoPath (the full video) is
    // always retained, so export keeps using the full-resolution source.
    const QString thumb = previewThumbnailPath();
    if (m_usePreview && !thumb.isEmpty())
        m_decoder->loadVideo(thumb);
    else
        m_decoder->loadVideo(m_videoPath);

    // loadVideo() stops the decoder; restore the previous position (clamped to
    // the new file's duration) and playback state.
    m_decoder->seekTo(qMin(m_currentTime, duration()));
    if (m_isPlaying)
        m_decoder->startPlayback();
}

bool App::isPlaying() const
{
    return m_isPlaying;
}

void App::setIsPlaying(bool playing)
{
    if (m_isPlaying == playing)
        return;

    m_isPlaying = playing;
    emit isPlayingChanged();

    if (playing) {
        m_decoder->startPlayback();
    } else {
        m_decoder->stopPlayback();
    }
}

double App::currentTime() const
{
    return m_currentTime;
}

void App::setCurrentTime(double time)
{
    if (qFuzzyCompare(m_currentTime, time))
        return;

    m_currentTime = time;
    emit currentTimeChanged();

    if (m_imuStabilize)
        emit imuOrientationChanged();

    m_decoder->seekTo(time);
}

double App::duration() const
{
    return m_decoder->duration();
}

double App::yaw() const
{
    return m_yaw;
}

void App::setYaw(double yaw)
{
    if (qFuzzyCompare(m_yaw, yaw))
        return;

    m_yaw = yaw;
    m_viewQuat = viewQuatFromEuler();
    emit yawChanged();
}

double App::pitch() const
{
    return m_pitch;
}

void App::setPitch(double pitch)
{
    pitch = qBound(-kMaxPitch, pitch, kMaxPitch);
    if (qFuzzyCompare(m_pitch, pitch))
        return;

    m_pitch = pitch;
    m_viewQuat = viewQuatFromEuler();
    emit pitchChanged();
}

double App::roll() const
{
    return m_roll;
}

void App::setRoll(double roll)
{
    if (qFuzzyCompare(m_roll, roll))
        return;

    m_roll = roll;
    m_viewQuat = viewQuatFromEuler();
    emit rollChanged();
}

double App::fov() const
{
    return m_fov;
}

void App::setFov(double fov)
{
    if (qFuzzyCompare(m_fov, fov))
        return;

    m_fov = fov;
    emit fovChanged();
}

bool App::imuStabilize() const
{
    return m_imuStabilize;
}

void App::setImuStabilize(bool stabilize)
{
    if (m_imuStabilize == stabilize)
        return;

    m_imuStabilize = stabilize;
    emit imuStabilizeChanged();
    // imuOrientation switches between the stabilized chain and the bare video
    // un-flip, so the viewer has to re-read it.
    emit imuOrientationChanged();
}

double App::imuSmoothing() const
{
    return m_imuSmoothing;
}

void App::setImuSmoothing(double smoothing)
{
    if (qFuzzyCompare(m_imuSmoothing, smoothing))
        return;

    m_imuSmoothing = smoothing;
    emit imuSmoothingChanged();
}

double App::imuSyncOffset() const
{
    return m_imuSyncOffset;
}

void App::setImuSyncOffset(double offset)
{
    if (qFuzzyCompare(m_imuSyncOffset, offset))
        return;

    m_imuSyncOffset = offset;
    emit imuSyncOffsetChanged();
}

double App::imuDrift() const
{
    return m_imuDrift;
}

void App::setImuDrift(double drift)
{
    if (qFuzzyCompare(m_imuDrift, drift))
        return;

    m_imuDrift = drift;
    emit imuDriftChanged();

    // Persist the tuned drift in this video's keyframe sidecar so re-opening
    // the video restores it (the auto-calculated value is only the fallback
    // when the sidecar has no stored drift). Skipped during sidecar restore.
    if (!m_videoPath.isEmpty() && !m_restoringSidecar) {
        m_keyframes->setImuDrift(drift);
        saveKeyframes();
    }
}

double App::imuAccelKi() const
{
    return m_imuAccelKi;
}

void App::setImuAccelKi(double ki)
{
    // Integral gain is clamped to >= 0 (0 disables the drift correction, e.g.
    // for debugging); the upper bound keeps the filter from overshooting.
    ki = qBound(0.0, ki, 0.10);
    if (qFuzzyCompare(m_imuAccelKi, ki))
        return;

    m_imuAccelKi = ki;
    emit imuAccelKiChanged();

    if (!m_imuParser->isLoaded())
        return;
    // Coalesce: re-integrating the whole 400 Hz chain per mouse-move event made
    // dragging the slider stutter.
    if (!m_reintegrateTimer) {
        m_reintegrateTimer = new QTimer(this);
        m_reintegrateTimer->setSingleShot(true);
        m_reintegrateTimer->setInterval(120);
        connect(m_reintegrateTimer, &QTimer::timeout, this, [this]() {
            if (m_imuParser->isLoaded()) {
                integrateImu();
                if (m_imuStabilize)
                    emit imuOrientationChanged();
            }
        });
    }
    m_reintegrateTimer->start();
}

double App::autoImuDrift() const
{
    // The fractional difference in stream DURATIONS is only a clock drift if
    // both streams cover the same interval. They generally do not — the IMU
    // typically starts a little before and stops a little after the video — so
    // a longer IMU stream mostly means "recorded for longer", not "ticks
    // faster". On YIVR_0845 the IMU runs 140.30 s against 139.04 s of video,
    // which this used to report as 0.91% drift: 1.26 s of skew across the clip.
    // Fed to SyncSolver as its starting drift, that dragged the solved offset
    // to -0.395 s when the truth is 0.150 s (verified against a ground-truth
    // sweep: -0.395 explains -0.79 of the motion, 0.150 explains 0.94), and a
    // half-second sync error looks exactly like no stabilisation at all.
    //
    // Both clocks are crystal-derived, so real relative rate error is parts per
    // thousand at worst. Anything larger is a recording-length difference and
    // the honest answer is that we do not know the drift — return 0 rather than
    // invent one, and let the sync solver measure it from the content.
    // Matched to SyncSolver's MAX_DRIFT. Two crystals at +-50 ppm each cannot
    // disagree by more than ~1e-4 s/s; 5e-4 is already generous, and anything
    // above it is a recording-length difference, not a rate difference.
    constexpr double kMaxPlausibleDrift = 5e-4;
    const double imuDur = m_imuParser->duration();
    const double videoDur = m_decoder->duration();
    if (imuDur > 0.0 && videoDur > 0.0) {
        const double d = (imuDur - videoDur) / videoDur;
        if (std::abs(d) <= kMaxPlausibleDrift)
            return d;
        qInfo() << "autoImuDrift: duration difference implies" << d
                << "-- too large for clock drift (IMU" << imuDur << "s, video"
                << videoDur << "s); assuming 0";
        return 0.0;
    }
    return m_imuDrift;
}

void App::integrateImu()
{
    if (!m_imuParser->isLoaded())
        return;

    // The fused chain is a function of the chain we are about to replace, so it
    // is stale the moment we re-integrate. Without this, changing accelKi after
    // AutoSync re-integrated the raw chain while the queries kept returning the
    // old fused one — the slider appeared to do nothing.
    m_gyroIntegrator->clearFusedOrientations();
    // Determine gyro calibration to apply:
    // 1. Sidecar has stored calibration -> use it
    // 2. Camera default has calibration -> use it
    // 3. Otherwise -> identity matrix, zero bias (no correction)
    QMatrix3x3 gyroMatrix;
    QVector3D gyroBias;
    if (m_keyframes->hasGyroCalibration()) {
        gyroMatrix = m_keyframes->gyroMatrix();
        gyroBias = m_keyframes->gyroBias();
    } else {
        QMatrix3x3 camM = cameraGyroMatrix();
        QVector3D camB = cameraGyroBias();
        if (camM != QMatrix3x3() || camB != QVector3D()) {
            gyroMatrix = camM;
            gyroBias = camB;
        }
    }

    // Item 1: fold the visually-calibrated camera-default gyro scales into the
    // matrix. The parser divided raw counts by its hardcoded scale; if a
    // previous AutoSync stored "camera/gyroScale", the effective deg/s should
    // use that divisor instead. The raw deg/s computed by the parser is
    // count/hardcoded; scaling by (hardcoded/cameraDefault) readjusts it, and
    // the stored matrix M (if any) multiplies on top. Applied as a diagonal
    // multiplier on the left of any stored M.
    {
        QSettings cam;
        const double sxx = cam.value(QStringLiteral("camera/gyroScaleX"),
                                     m_imuParser->gyroScaleX()).toDouble();
        const double syy = cam.value(QStringLiteral("camera/gyroScaleY"),
                                     m_imuParser->gyroScaleY()).toDouble();
        const double szz = cam.value(QStringLiteral("camera/gyroScaleZ"),
                                     m_imuParser->gyroScaleZ()).toDouble();
        const bool haveDefault = cam.contains(QStringLiteral("camera/gyroScaleX"));
        if (haveDefault && sxx > 5.0 && syy > 5.0 && szz > 5.0) {
            // Raw deg/s from parser uses gyroScale*; to use cameraDefault, the
            // factor is gyroScale_parser / cameraDefault.
            QMatrix3x3 scaleDiag;
            scaleDiag(0,0) = (float)(m_imuParser->gyroScaleX() / sxx);
            scaleDiag(1,1) = (float)(m_imuParser->gyroScaleY() / syy);
            scaleDiag(2,2) = (float)(m_imuParser->gyroScaleZ() / szz);
            // Combined = scaleDiag * M (scale applied first, then stored M).
            QMatrix3x3 combined;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) {
                    float acc = 0.0f;
                    for (int k = 0; k < 3; k++)
                        acc += scaleDiag(r,k) * gyroMatrix(k,c);
                    combined(r,c) = acc;
                }
            gyroMatrix = combined;
            qDebug() << "GyroIntegrator: camera-default scale folded"
                     << m_imuParser->gyroScaleX()/sxx << m_imuParser->gyroScaleY()/syy
                     << m_imuParser->gyroScaleZ()/szz;
        }
    }

    // No per-sample accelerometer feedback (Kp = Ki = 0). The gyro alone,
    // with a bias measured only when the camera is genuinely still and scales
    // validated by gravity closure, holds attitude to a few degrees over
    // minutes; the absolute tilt datum comes from the integrator's slow
    // (30 s) world-frame gravity re-level. The Mahony terms that used to run
    // here were the cause of the rolled horizon on fast footage — see
    // GyroscopeIntegrator::integrate(). m_imuAccelKi is retained only so old
    // settings files still load; it no longer affects the chain.
    if (m_imuAccelKi > 0.0)
        qInfo() << "imu/accelKi =" << m_imuAccelKi
                << "is ignored: accelerometer feedback is disabled (see integrateImu)";
    m_gyroIntegrator->integrate(m_imuParser->samples(),
                                m_imuParser->imuSampleRate(),
                                m_imuParser->initialQuaternion(),
                                0.0f, 0.0f,
                                gyroMatrix,
                                gyroBias);

    // Rebuild the optical drift correction on top of the new chain. Cheap now
    // that correctionAt() binary-searches its window, and it keeps the
    // Mahony gains live after AutoSync instead of being shadowed by a stale
    // fused chain.
    applyVisualFusion();
}

void App::applyVisualFusion()
{
    if (!m_visualFusion || m_deferVisualFusion || m_visualPairs.size() < 10)
        return;

    m_visualFusion->fuse(m_visualPairs,
                         m_gyroIntegrator->orientations(),
                         m_gyroIntegrator->timestamps(),
                         m_imuSyncOffset, m_imuDrift, 3.0,
                         m_gyroIntegrator->gravityTrust());
    const auto fusedOris = m_visualFusion->fusedOrientations();
    const auto fusedTs = m_visualFusion->fusedTimestamps();
    if (!fusedOris.isEmpty())
        m_gyroIntegrator->setFusedOrientations(fusedOris, fusedTs);
}

void App::setFlowStitch(bool stitch)
{
    if (m_flowStitch == stitch)
        return;

    m_flowStitch = stitch;
    emit flowStitchChanged();

    if (stitch) {
        maybeComputeFlow();
    } else {
        // Disable the warp immediately; drop any stale field so the next
        // toggle-on starts fresh.
        m_flowPending = false;
        m_flowLastTs = -1.0;
        m_flowImage = QImage();
        m_flowEncode = 16.0;
        emit flowImageReady();
    }
}

void App::setFlowStrength(double strength)
{
    if (qFuzzyCompare(m_flowStrength, strength))
        return;
    m_flowStrength = strength;
    emit flowStrengthChanged();
}

void App::setFlowIterations(int iterations)
{
    iterations = qBound(1, iterations, 200);
    if (m_flowIterations == iterations)
        return;
    m_flowIterations = iterations;
    emit flowIterationsChanged();
    // The flow field itself depends on the iteration count.
    if (m_flowStitch)
        maybeComputeFlow();
}

void App::setFlowAlpha(double alpha)
{
    alpha = qBound(1.0, alpha, 100.0);
    if (qFuzzyCompare(m_flowAlpha, alpha))
        return;
    m_flowAlpha = alpha;
    emit flowAlphaChanged();
    // The flow field itself depends on the smoothness weight.
    if (m_flowStitch)
        maybeComputeFlow();
}

void App::setSeamStitch(bool stitch)
{
    if (m_seamStitch == stitch) return;
    m_seamStitch = stitch;
    emit seamStitchChanged();
    if (stitch && m_flowStitch) maybeComputeFlow();
}

void App::setSeamStrength(double strength)
{
    if (qFuzzyCompare(m_seamStrength, strength)) return;
    m_seamStrength = strength;
    emit seamStrengthChanged();
}

void App::maybeComputeFlow()
{
    if (!m_flowStitch || !m_decoder || !m_decoder->hasFrame())
        return;
    // Latest-wins: if the flow worker is already busy, drop this intermediate
    // frame instead of enqueueing it — otherwise the serial worker falls
    // behind during playback and the seam is warped with an ever-staler flow
    // field ("edges of the image appear static").
    if (m_flowPending)
        return;
    m_flowPending = true;

    DecodedFrame frame = m_decoder->currentFrame();  // copy (ref-counted planes)
    m_flowLastTs = frame.timestamp;

    FlowCalibration cal;
    if (m_currentCalibration) {
        cal.frontCenterX = (float)m_currentCalibration->frontCenterX();
        cal.frontCenterY = (float)m_currentCalibration->frontCenterY();
        cal.frontRadius = (float)m_currentCalibration->frontRadius();
        cal.frontK1 = (float)m_currentCalibration->frontK1();
        cal.frontK2 = (float)m_currentCalibration->frontK2();
        cal.frontRotation = (float)m_currentCalibration->frontRotation();
        cal.frontHFlip = m_currentCalibration->frontHFlip();
        cal.rearCenterX = (float)m_currentCalibration->rearCenterX();
        cal.rearCenterY = (float)m_currentCalibration->rearCenterY();
        cal.rearRadius = (float)m_currentCalibration->rearRadius();
        cal.rearK1 = (float)m_currentCalibration->rearK1();
        cal.rearK2 = (float)m_currentCalibration->rearK2();
        cal.rearRotation = (float)m_currentCalibration->rearRotation();
        cal.rearHFlip = m_currentCalibration->rearHFlip();
    }

    FlowSettings settings;
    settings.iterations = m_flowIterations;
    settings.alpha = (float)m_flowAlpha;

    emit flowRequested(frame, cal, settings);
}

void App::onFlowReady(const QImage &image, float encodeScale, const QImage &seamImage)
{
    m_flowPending = false;
    m_flowEncode = encodeScale;
    m_flowImage = image.copy();  // deep copy: the worker must never touch it again
    m_seamImage = seamImage.copy();
    emit flowImageReady();
    emit seamImageChanged();
    // The worker is idle again. If playback has advanced past the frame we
    // just processed, run one more compute for the newest frame (latest-wins).
    // If the timestamp hasn't changed (paused), don't recompute the same frame.
    const double newest = (m_decoder && m_decoder->hasFrame())
        ? m_decoder->currentFrame().timestamp : -1.0;
    if (newest >= 0.0 && newest != m_flowLastTs)
        QMetaObject::invokeMethod(this, [this]() { maybeComputeFlow(); },
                                  Qt::QueuedConnection);
}

int App::activeLens() const
{
    return m_activeLens;
}

void App::setActiveLens(int lens)
{
    if (m_activeLens == lens)
        return;

    m_activeLens = lens;
    emit activeLensChanged();
}

int App::projection() const
{
    return m_projection;
}

void App::setProjection(int projection)
{
    if (m_projection == projection)
        return;

    m_projection = projection;
    emit projectionChanged();
}

void App::dragLook(double angleAboutUp, double angleAboutRight)
{
    // With IMU stabilization the shader composes imuMatrix * euler, so the
    // displayed (stabilized) view is fully described by the euler angles and
    // dragging behaves exactly like free look -- no IMU correction needed.
    //
    // Yaw turns about the WORLD up axis; pitch tilts about the camera's own
    // right axis; roll is never touched. This is the ordinary turntable /
    // panorama-viewer behaviour, and it is what keeps the horizon level.
    //
    // The previous version rotated about the view's LOCAL axes
    // (q * axisAngle(0,1,0,...)). Once the view is pitched, the local up axis
    // is no longer world up, so every horizontal drag tipped a little roll
    // into the view; drag around for a while and the horizon ends up visibly
    // canted with no obvious way back. It also needed a pole-crossing guard,
    // because the Euler decomposition of the drifting quaternion goes
    // degenerate near +-90 deg pitch and would swap yaw and roll.
    //
    // Because the shader's euler is rotY(yaw) * rotX(pitch) * rotZ(roll),
    // adding to yaw IS a pre-multiply by a rotation about world Y, and adding
    // to pitch inserts a rotation about the yawed X axis -- exactly the two
    // rotations wanted. So the drag reduces to adding to the two angles, roll
    // is structurally untouchable, and clamping pitch (setPitch already bounds
    // it to +-kMaxPitch) removes the degenerate pole case entirely rather than
    // detecting it after the fact.
    double yaw = m_yaw + angleAboutUp;
    while (yaw > 180.0) yaw -= 360.0;
    while (yaw < -180.0) yaw += 360.0;

    setYaw(yaw);
    setPitch(m_pitch + angleAboutRight);
}

QQuaternion App::viewQuatFromEuler() const
{
    return QQuaternion::fromAxisAndAngle(0, 1, 0, m_yaw)
         * QQuaternion::fromAxisAndAngle(1, 0, 0, m_pitch)
         * QQuaternion::fromAxisAndAngle(0, 0, 1, m_roll);
}

void App::extractEulerFromQuat(const QQuaternion &q, double &yaw, double &pitch, double &roll) const
{
    float w = q.scalar(), x = q.x(), y = q.y(), z = q.z();

    double m02 = 2.0 * (x * z + y * w);
    double m12 = 2.0 * (y * z - x * w);
    double m22 = 1.0 - 2.0 * (x * x + y * y);
    double m10 = 2.0 * (x * y + z * w);
    double m11 = 1.0 - 2.0 * (x * x + z * z);

    yaw   = qRadiansToDegrees(atan2(m02, m22));
    pitch = qRadiansToDegrees(-asin(qBound(-1.0, m12, 1.0)));
    roll  = qRadiansToDegrees(atan2(m10, m11));

    if (yaw > 180.0) yaw -= 360.0;
    else if (yaw < -180.0) yaw += 360.0;
}

void App::addKeyframeAtCurrent()
{
    double y, p, r;
    extractEulerFromQuat(m_viewQuat, y, p, r);
    m_keyframes->addKeyframe(m_currentTime, y, p, r, m_fov);
}

void App::applyKeyframeInterpolation()
{
    if (!m_keyframes->hasKeyframes())
        return;

    double y, p, r, f;
    m_keyframes->interpolate(m_currentTime, y, p, r, f);
    p = qBound(-kMaxPitch, p, kMaxPitch);

    m_viewQuat = QQuaternion::fromAxisAndAngle(0, 1, 0, y)
               * QQuaternion::fromAxisAndAngle(1, 0, 0, p)
               * QQuaternion::fromAxisAndAngle(0, 0, 1, r);
    m_viewQuat.normalize();
    m_fov = f;

    m_yaw = y;
    m_pitch = p;
    m_roll = r;

    emit yawChanged();
    emit pitchChanged();
    emit rollChanged();
    emit fovChanged();
}

CalibrationPresetModel* App::calibrationPresets() const
{
    return m_calibrationPresets;
}

CalibrationProfile* App::currentCalibration() const
{
    return m_currentCalibration;
}

VideoDecoder* App::videoDecoder() const
{
    return m_decoder;
}

QQuaternion App::imuOrientation() const
{
    return imuOrientationAt(m_currentTime);
}

QQuaternion App::imuOrientationAt(double time) const
{
    if (m_imuStabilize && m_gyroIntegrator) {
        // The integrator fuses the accelerometer with the gyro, so its output
        // is already gravity-aligned. LensViewer passes the conjugate of this
        // quaternion to the shader, so the displayed (stabilized) world
        // direction lives in a gravity-aligned frame and the default 0/0/0
        // view is level.
        //
        // Two-signal architecture (q_virtual^{-1} * q_actual):
        // - q_actual: full-bandwidth, unsmoothed orientation at the frame's
        //   exposure midpoint. Never smooth this — high-frequency shake must
        //   cancel exactly.
        // - q_virtual: deliberately smooth "virtual tripod" path (Gaussian-
        //   smoothed). Smoothing slider (0..1) maps to a window of up to 300 ms.
        // - The shader applies imuOrientation.conjugated(), so we return
        //   q_actual^{-1} * q_virtual, which makes the shader apply
        //   q_virtual^{-1} * q_actual — cancelling shake exactly while
        //   following the smooth virtual path for intentional motion.
        //
        // Sync is a linear model offset(t) = m_imuSyncOffset + m_imuDrift*t
        // (the IMU and video clocks run at slightly different rates, so the
        // offset that aligns the start of the clip is ~0.07 s too small by 24 s).
        const double tImu = time * (1.0 + m_imuDrift) + m_imuSyncOffset;

        // Dual-mode stabilization:
        //  - Smoothing <= 0.9 (HIGH-PASS, the default): q_virtual is the
        //    Gaussian-smoothed "virtual tripod" path. The correction removes the
        //    high-frequency shake (deviation of the true orientation from the
        //    smooth path) while keeping intentional motion. A full 360° pan
        //    stays visible.
        //  - Smoothing > 0.9 (LOW-PASS / "hold world steady"): q_virtual is
        //    pinned to the clip's very first orientation, so q_virtual^{-1}*
        //    q_actual cancels ALL camera rotation. The world is held fixed and
        //    nothing pans, even a 360° rotation. Reachable at the top of the
        //    smoothing slider.
        const float smoothingMs = (float)(m_imuSmoothing * 300.0);
        const QQuaternion qActual = m_gyroIntegrator->orientationAtTimeUnsmoothed(tImu);
        QQuaternion qVirtual;
        if (m_imuSmoothing > 0.9) {
            const QQuaternion qFirst = m_gyroIntegrator->firstOrientation();
            qVirtual = qFirst.isNull() ? qActual : qFirst;
        } else {
            qVirtual = m_gyroIntegrator->orientationAtTime(tImu, smoothingMs);
        }
        // Return q_virtual^{-1} * q_actual. LensViewer builds
        // imuMat.rotate(conjugate(q)) and the shader applies imuMat * ray, so
        // the effective ray transform is q^{-1}; with q = q_virtual^{-1} q_actual
        // the sampled ray becomes (q_virtual^{-1} q_actual)^{-1} = q_actual^{-1}
        // q_virtual, i.e. the high-frequency deviation of the true camera
        // orientation from the smooth virtual path — applied to the video ray
        // this COUNTER-ROTATES the shake and cancels it, while leaving the
        // smooth intentional path untouched. With q_virtual pinned to the first
        // orientation this becomes q_actual^{-1} * q_first, cancelling ALL
        // rotation (low-pass / hold-world-steady). (The inverse ordering here
        // would apply the shake rotation instead of cancelling it — a
        // no-op-looking toggle, which is the bug this fixes.)
        // Hand the shader the composed correction. composeStabilisation()
        // horizon-locks the virtual camera (yaw only, pitch/roll discarded) and
        // applies the video un-flip once, on the outside where it survives.
        // See gyroscopeintegrator.h for why both steps belong there and not in
        // the chain.
        return composeStabilisation(qActual, qVirtual);
    }
    // Stabilization off: the shader still needs the un-flip, or the raw video
    // renders upside down.
    return kFlipRollQ();
}


void App::exportFrame(const QString &path, int width, int height)
{
    if (m_videoPath.isEmpty()) {
        m_exportStatus = tr("Export failed: no video loaded");
        emit exportStatusChanged();
        return;
    }
    if (m_exporter->isRunning())
        return;

    setExportProgress(0.0);
    setExportRunning(true);
    m_exportStatus = tr("Rendering frame…");
    emit exportStatusChanged();

    ExportSnapshot snap = buildExportSnapshot();
    m_exporter->exportFrame(m_videoPath, path, width, height, m_currentTime,
                            snap.stateAt(m_currentTime));
}

void App::exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime, const QString &codec, int crf, int bitrateMbps, bool vidstab, bool vidstabInformed, bool gpuBackend, bool spherical360)
{
    if (m_videoPath.isEmpty()) {
        m_exportStatus = tr("Export failed: no video loaded");
        emit exportStatusChanged();
        return;
    }
    if (m_exporter->isRunning())
        return;

    // A content:// destination cannot be muxed into directly. Two reasons,
    // both fatal:
    //   * the document could not be rewound at the end, so av_write_trailer()
    //     failed, and
    //   * the fragmented-MP4 workaround writes the header BEFORE MediaCodec
    //     has produced its SPS/PPS, leaving a file with no usable avcC --
    //     20 MB of "No start code is found" that nothing will play.
    // So render to a real file in app-private storage, which is seekable and
    // gets a normal MP4, and copy it into the folder afterwards. 20 MB is a
    // fraction of a second.
    QString writePath = path;
    m_exportFinalUri.clear();
    if (path.contains(QStringLiteral("://"))) {
        m_exportFinalUri = path;
        const QString dir = QStandardPaths::writableLocation(QStandardPaths::TempLocation);
        QDir().mkpath(dir);
        writePath = dir + QStringLiteral("/render360_export.mp4");
        QFile::remove(writePath);
    }

    setExportProgress(0.0);
    setExportRunning(true);
    m_exportStatus = tr("Preparing export…");
    emit exportStatusChanged();
    // Remember the output path so it can be restored the next time the export
    // dialog opens for this video (persisted in the keyframe sidecar).
    m_keyframes->setExportFileName(path);

    ExportSettings settings;
    settings.width = width;
    settings.height = height;
    settings.fps = fps;
    settings.codec = codec;
    settings.crf = crf;
    settings.bitrateMbps = bitrateMbps;
    settings.vidstab = vidstab;
    settings.vidstabInformed = vidstabInformed;
    // Only an equirectangular render is actually a sphere; tagging any other
    // projection would make players warp a flat picture around one.
    settings.spherical = spherical360 && m_projection == 1;

    ExportSnapshot snap = buildExportSnapshot();
    m_exporter->exportVideo(m_videoPath, writePath, settings, startTime, endTime,
                            [snap](double t) { return snap.stateAt(t); },
                            gpuBackend);
}

void App::setExportStart(double time)
{
    const double dur = m_decoder ? m_decoder->duration() : 0.0;
    double hi = (dur > 0.0) ? qMin(m_exportEnd, dur) : m_exportEnd;
    time = qBound(0.0, time, hi);
    time = qRound(time * 100.0) / 100.0;  // store at 1/100 s resolution
    if (qFuzzyCompare(m_exportStart, time))
        return;
    m_exportStart = time;
    emit exportStartChanged();
}

void App::setExportEnd(double time)
{
    const double dur = m_decoder ? m_decoder->duration() : 0.0;
    double hi = (dur > 0.0) ? dur : time;
    time = qBound(m_exportStart, time, hi);
    time = qRound(time * 100.0) / 100.0;  // store at 1/100 s resolution
    if (qFuzzyCompare(m_exportEnd, time))
        return;
    m_exportEnd = time;
    emit exportEndChanged();
}

void App::setExportWidth(int width)
{
    m_keyframes->setExportWidth(width);
}

void App::setExportHeight(int height)
{
    m_keyframes->setExportHeight(height);
}

void App::setExportFps(double fps)
{
    m_keyframes->setExportFps(fps);
}

void App::setExportCodec(const QString &codec)
{
    m_keyframes->setExportCodec(codec);
}

void App::setExportCrf(int crf)
{
    m_keyframes->setExportCrf(crf);
}

void App::setExportBitrate(int bitrate)
{
    m_keyframes->setExportBitrate(bitrate);
}

void App::setExportVidstab(bool vidstab)
{
    m_keyframes->setExportVidstab(vidstab);
}

void App::setExportVidstabInformed(bool informed)
{
    m_keyframes->setExportVidstabInformed(informed);
}

void App::setExportFileName(const QString &name)
{
    m_keyframes->setExportFileName(name);
}

void App::setExportRunning(bool running)
{
    if (m_exportRunning == running)
        return;
    m_exportRunning = running;
    if (!running)
        setExportProgress(0.0);
    emit exportRunningChanged();
}

void App::setExportProgress(double progress)
{
    progress = qBound(0.0, progress, 1.0);
    if (qFuzzyCompare(m_exportProgress, progress))
        return;
    m_exportProgress = progress;
    emit exportProgressChanged();
}

ExportSnapshot App::buildExportSnapshot() const
{
    ExportSnapshot s;
    s.base.yaw = m_yaw;
    s.base.pitch = m_pitch;
    s.base.roll = m_roll;
    s.base.fov = m_fov;
    s.base.activeLens = m_activeLens;
    s.base.projection = m_projection;
    s.base.fullRange = m_decoder ? m_decoder->isFullRange() : true;
    if (m_currentCalibration) {
        s.base.frontCenterX = (float)m_currentCalibration->frontCenterX();
        s.base.frontCenterY = (float)m_currentCalibration->frontCenterY();
        s.base.frontRadius = (float)m_currentCalibration->frontRadius();
        s.base.frontK1 = (float)m_currentCalibration->frontK1();
        s.base.frontK2 = (float)m_currentCalibration->frontK2();
        s.base.frontRotation = (float)m_currentCalibration->frontRotation();
        s.base.frontHFlip = m_currentCalibration->frontHFlip();
        s.base.rearCenterX = (float)m_currentCalibration->rearCenterX();
        s.base.rearCenterY = (float)m_currentCalibration->rearCenterY();
        s.base.rearRadius = (float)m_currentCalibration->rearRadius();
        s.base.rearK1 = (float)m_currentCalibration->rearK1();
        s.base.rearK2 = (float)m_currentCalibration->rearK2();
        s.base.rearRotation = (float)m_currentCalibration->rearRotation();
        s.base.rearHFlip = m_currentCalibration->rearHFlip();
        s.base.blendStart = (float)m_currentCalibration->blendStart();
    }
    s.base.flowStitch = m_flowStitch;
    s.base.flowStrength = (float)m_flowStrength;
    s.base.flowIterations = m_flowIterations;
    s.base.flowAlpha = (float)m_flowAlpha;
    s.base.seamStitch = m_seamStitch;
    s.base.seamStrength = (float)m_seamStrength;
    s.base.bandTheta0 = kDefaultBandTheta0;
    s.base.bandTheta1 = kDefaultBandTheta1;
    if (m_colorGrade) {
        s.base.brightness = (float)m_colorGrade->brightness();
        s.base.contrast = (float)m_colorGrade->contrast();
        s.base.saturation = (float)m_colorGrade->saturation();
        s.base.pop = (float)m_colorGrade->pop();
        s.base.brightLows = (float)m_colorGrade->brightLows();
        s.base.brightLowMids = (float)m_colorGrade->brightLowMids();
        s.base.brightHighMids = (float)m_colorGrade->brightHighMids();
        s.base.brightHighs = (float)m_colorGrade->brightHighs();
        s.base.redLows = (float)m_colorGrade->redLows();
        s.base.redMids = (float)m_colorGrade->redMids();
        s.base.redHighs = (float)m_colorGrade->redHighs();
        s.base.greenLows = (float)m_colorGrade->greenLows();
        s.base.greenMids = (float)m_colorGrade->greenMids();
        s.base.greenHighs = (float)m_colorGrade->greenHighs();
        s.base.blueLows = (float)m_colorGrade->blueLows();
        s.base.blueMids = (float)m_colorGrade->blueMids();
        s.base.blueHighs = (float)m_colorGrade->blueHighs();
        s.base.curves = m_colorGrade->curvesActive();
        s.base.curveLut = m_colorGrade->curveLut();
    }
    s.keyframes = m_keyframes->keyframes();
    s.imuStabilize = m_imuStabilize;
    if (m_gyroIntegrator) {
        s.imuOrientations = m_gyroIntegrator->activeOrientations();
        s.imuTimestamps = m_gyroIntegrator->activeTimestamps();
    }
    s.syncOffset = m_imuSyncOffset;
    s.drift = m_imuDrift;
    s.imuSmoothingMs = (float)(m_imuSmoothing * 300.0);
    return s;
}

void App::loadSettings()
{
    QSettings s;
    setImuStabilize(s.value(QStringLiteral("imu/stabilize"), m_imuStabilize).toBool());
    setImuSmoothing(s.value(QStringLiteral("imu/smoothing"), m_imuSmoothing).toDouble());
    setImuSyncOffset(s.value(QStringLiteral("imu/syncOffset"), m_imuSyncOffset).toDouble());
    setImuDrift(s.value(QStringLiteral("imu/drift"), m_imuDrift).toDouble());
    setImuAccelKi(s.value(QStringLiteral("imu/accelKi"), m_imuAccelKi).toDouble());
    // Clamp to the valid projection ids so a stale/corrupt settings value can
    // never put the QML combo box out of range.
    setProjection(qBound(0, s.value(QStringLiteral("projection"), m_projection).toInt(), 3));
    setFlowStitch(s.value(QStringLiteral("flow/stitch"), m_flowStitch).toBool());
    setFlowStrength(s.value(QStringLiteral("flow/strength"), m_flowStrength).toDouble());
    setFlowIterations(qBound(1, s.value(QStringLiteral("flow/iterations"), m_flowIterations).toInt(), 200));
    setFlowAlpha(qBound(1.0, s.value(QStringLiteral("flow/alpha"), m_flowAlpha).toDouble(), 100.0));
    setSeamStitch(s.value(QStringLiteral("seam/stitch"), m_seamStitch).toBool());
    setSeamStrength(s.value(QStringLiteral("seam/strength"), m_seamStrength).toDouble());

    if (m_colorGrade) {
        auto load = [this, &s](const char *key, double def, void (ColorGrade::*setter)(double)) {
            (m_colorGrade->*setter)(s.value(QLatin1String(key), def).toDouble());
        };
        load("grade/brightness", m_colorGrade->brightness(), &ColorGrade::setBrightness);
        load("grade/contrast", m_colorGrade->contrast(), &ColorGrade::setContrast);
        load("grade/saturation", m_colorGrade->saturation(), &ColorGrade::setSaturation);
        load("grade/pop", m_colorGrade->pop(), &ColorGrade::setPop);
        load("grade/brightLows", m_colorGrade->brightLows(), &ColorGrade::setBrightLows);
        load("grade/brightLowMids", m_colorGrade->brightLowMids(), &ColorGrade::setBrightLowMids);
        load("grade/brightHighMids", m_colorGrade->brightHighMids(), &ColorGrade::setBrightHighMids);
        load("grade/brightHighs", m_colorGrade->brightHighs(), &ColorGrade::setBrightHighs);
        load("grade/redLows", m_colorGrade->redLows(), &ColorGrade::setRedLows);
        load("grade/redMids", m_colorGrade->redMids(), &ColorGrade::setRedMids);
        load("grade/redHighs", m_colorGrade->redHighs(), &ColorGrade::setRedHighs);
        load("grade/greenLows", m_colorGrade->greenLows(), &ColorGrade::setGreenLows);
        load("grade/greenMids", m_colorGrade->greenMids(), &ColorGrade::setGreenMids);
        load("grade/greenHighs", m_colorGrade->greenHighs(), &ColorGrade::setGreenHighs);
        load("grade/blueLows", m_colorGrade->blueLows(), &ColorGrade::setBlueLows);
        load("grade/blueMids", m_colorGrade->blueMids(), &ColorGrade::setBlueMids);
        load("grade/blueHighs", m_colorGrade->blueHighs(), &ColorGrade::setBlueHighs);
        m_colorGrade->curvesFromJson(s.value(QStringLiteral("grade/curves")).toString());
    }
}

void App::saveSettings() const
{
    QSettings s;
    s.setValue(QStringLiteral("imu/stabilize"), m_imuStabilize);
    s.setValue(QStringLiteral("imu/smoothing"), m_imuSmoothing);
    s.setValue(QStringLiteral("imu/syncOffset"), m_imuSyncOffset);
    s.setValue(QStringLiteral("imu/drift"), m_imuDrift);
    s.setValue(QStringLiteral("imu/accelKi"), m_imuAccelKi);
    s.setValue(QStringLiteral("projection"), m_projection);
    s.setValue(QStringLiteral("flow/stitch"), m_flowStitch);
    s.setValue(QStringLiteral("flow/strength"), m_flowStrength);
    s.setValue(QStringLiteral("flow/iterations"), m_flowIterations);
    s.setValue(QStringLiteral("flow/alpha"), m_flowAlpha);
    s.setValue(QStringLiteral("seam/stitch"), m_seamStitch);
    s.setValue(QStringLiteral("seam/strength"), m_seamStrength);

    if (m_colorGrade) {
        s.setValue(QStringLiteral("grade/brightness"), m_colorGrade->brightness());
        s.setValue(QStringLiteral("grade/contrast"), m_colorGrade->contrast());
        s.setValue(QStringLiteral("grade/saturation"), m_colorGrade->saturation());
        s.setValue(QStringLiteral("grade/pop"), m_colorGrade->pop());
        s.setValue(QStringLiteral("grade/brightLows"), m_colorGrade->brightLows());
        s.setValue(QStringLiteral("grade/brightLowMids"), m_colorGrade->brightLowMids());
        s.setValue(QStringLiteral("grade/brightHighMids"), m_colorGrade->brightHighMids());
        s.setValue(QStringLiteral("grade/brightHighs"), m_colorGrade->brightHighs());
        s.setValue(QStringLiteral("grade/redLows"), m_colorGrade->redLows());
        s.setValue(QStringLiteral("grade/redMids"), m_colorGrade->redMids());
        s.setValue(QStringLiteral("grade/redHighs"), m_colorGrade->redHighs());
        s.setValue(QStringLiteral("grade/greenLows"), m_colorGrade->greenLows());
        s.setValue(QStringLiteral("grade/greenMids"), m_colorGrade->greenMids());
        s.setValue(QStringLiteral("grade/greenHighs"), m_colorGrade->greenHighs());
        s.setValue(QStringLiteral("grade/blueLows"), m_colorGrade->blueLows());
        s.setValue(QStringLiteral("grade/blueMids"), m_colorGrade->blueMids());
        s.setValue(QStringLiteral("grade/blueHighs"), m_colorGrade->blueHighs());
        s.setValue(QStringLiteral("grade/curves"), m_colorGrade->curvesToJson());
    }
}

void App::saveKeyframes()
{
    if (m_videoPath.isEmpty())
        return;
    // loadFromFile() now emits keyframesChanged (the count property needs the
    // notify); writing here during that restore would rewrite the sidecar we
    // are reading -- and for a folder clip it would miss m_folderClipName,
    // which is only set after the open completes.
    if (m_restoringSidecar)
        return;
    // Sync the live trim range into the model so the sidecar always records
    // the current in/out markers (the model keeps them only as loaded state).
    m_keyframes->setTrimIn(m_exportStart);
    m_keyframes->setTrimOut(m_exportEnd);
    // In a granted folder the sidecar is a document in that tree, created if
    // it does not exist yet -- a plain path would be meaningless (and a
    // single-file grant is read-only, which is why keyframes could not be
    // saved on Android before the folder grant).
    QString target;
    if (m_folder && !m_folderClipName.isEmpty()) {
        target = m_folder->writableUriFor(m_folderClipName + QStringLiteral(".keyframes.json"),
                                          QStringLiteral("application/json"));
    }
    if (target.isEmpty())
        target = keyframesPathFor(m_videoPath);
    m_keyframes->saveToFile(target);
}

void App::clearGyroCalibration()
{
    if (!m_keyframes->hasGyroCalibration())
        return;
    m_keyframes->clearGyroCalibration();
    saveKeyframes();
    qInfo() << "Cleared this video's gyro calibration";
    if (m_imuParser->isLoaded()) {
        integrateImu();
        if (m_imuStabilize)
            emit imuOrientationChanged();
    }
}

void App::clearCameraGyroDefaults()
{
    QSettings s;
    s.remove(QStringLiteral("camera/gyroMatrix"));
    s.remove(QStringLiteral("camera/gyroBias"));
    s.remove(QStringLiteral("camera/gyroScaleX"));
    s.remove(QStringLiteral("camera/gyroScaleY"));
    s.remove(QStringLiteral("camera/gyroScaleZ"));
    qInfo() << "Cleared camera-wide gyro defaults";
    if (m_imuParser->isLoaded()) {
        integrateImu();
        if (m_imuStabilize)
            emit imuOrientationChanged();
    }
}

bool App::saveGyroCalibrationAsCameraDefault()
{
    if (!m_keyframes->hasGyroCalibration()) {
        qWarning() << "No gyro calibration on this video to promote";
        return false;
    }
    setCameraGyroMatrix(m_keyframes->gyroMatrix());
    setCameraGyroBias(m_keyframes->gyroBias());
    qInfo() << "Promoted this video's gyro calibration to the camera default";
    return true;
}

bool App::hasGyroCalibration() const
{
    return m_keyframes->hasGyroCalibration();
}

bool App::hasCameraGyroDefaults() const
{
    return cameraGyroMatrix() != QMatrix3x3() || cameraGyroBias() != QVector3D();
}

QMatrix3x3 App::cameraGyroMatrix() const
{
    QSettings s;
    QVariantList list = s.value(QStringLiteral("camera/gyroMatrix")).toList();
    if (list.size() == 9) {
        QMatrix3x3 M;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                M(i, j) = list[i * 3 + j].toDouble();
        return M;
    }
    return QMatrix3x3();
}

void App::setCameraGyroMatrix(const QMatrix3x3 &M)
{
    QVariantList list;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            list.append((double)M(i, j));
    QSettings s;
    s.setValue(QStringLiteral("camera/gyroMatrix"), list);
}

QVector3D App::cameraGyroBias() const
{
    QSettings s;
    QVariantList list = s.value(QStringLiteral("camera/gyroBias")).toList();
    if (list.size() == 3)
        return QVector3D(list[0].toFloat(), list[1].toFloat(), list[2].toFloat());
    return QVector3D();
}

void App::setCameraGyroBias(const QVector3D &b)
{
    QVariantList list;
    list.append((double)b.x());
    list.append((double)b.y());
    list.append((double)b.z());
    QSettings s;
    s.setValue(QStringLiteral("camera/gyroBias"), list);
}

void App::autoSyncAndCalibrate()
{
    if (m_autoSyncRunning)
        return;
    if (m_videoPath.isEmpty()) {
        m_autoSyncStatus = tr("Error: no video loaded");
        emit autoSyncStatusChanged();
        return;
    }
    if (!m_imuParser->isLoaded()) {
        m_autoSyncStatus = tr("Error: no IMU data loaded");
        emit autoSyncStatusChanged();
        return;
    }
    if (!m_currentCalibration) {
        m_autoSyncStatus = tr("Error: no calibration profile set");
        emit autoSyncStatusChanged();
        return;
    }

    m_autoSyncRunning = true;
    m_autoSyncProgress = 0.0;
    m_autoSyncStatus = tr("Computing visual rotations…");
    emit autoSyncRunningChanged();
    emit autoSyncProgressChanged();
    emit autoSyncStatusChanged();

    m_gyroIntegrator->clearFusedOrientations();

    // Stage 1: Visual Rotation (progress 0-0.4)
    connect(m_visualRotation, &VisualRotationComputer::progressChanged, this,
            [this](double fraction, const QString &status) {
        m_autoSyncProgress = fraction * 0.4;
        emit autoSyncProgressChanged();
        m_autoSyncStatus = status;
        emit autoSyncStatusChanged();
    }, Qt::QueuedConnection);

    connect(m_visualRotation, &VisualRotationComputer::rotationComputed, this,
            [this](const QVector<VisualRotationPair> &pairs) {
        disconnect(m_visualRotation, nullptr, this, nullptr);
        m_visualPairs = pairs;
        if (pairs.isEmpty()) {
            m_autoSyncRunning = false;
            m_autoSyncStatus = tr("Error: no visual rotations found");
            emit autoSyncRunningChanged();
            emit autoSyncStatusChanged();
            return;
        }

        // Stage 2: Sync Solver (progress 0.4-0.6)
        m_autoSyncStatus = tr("Solving sync…");
        emit autoSyncStatusChanged();

        connect(m_syncSolver, &SyncSolver::progressChanged, this,
                [this](double fraction, const QString &status) {
            m_autoSyncProgress = 0.4 + fraction * 0.2;
            emit autoSyncProgressChanged();
            m_autoSyncStatus = status;
            emit autoSyncStatusChanged();
        }, Qt::QueuedConnection);

        connect(m_syncSolver, &SyncSolver::syncSolved, this,
                [this](const SyncResult &result) {
            disconnect(m_syncSolver, nullptr, this, nullptr);
            setImuSyncOffset(result.syncOffset);
            setImuDrift(result.drift);
            m_keyframes->setSyncOffset(result.syncOffset);
            saveKeyframes();

            m_autoSyncStatus = tr("Calibrating gyro…");
            emit autoSyncStatusChanged();

            // Stage 3: Gyro Calibrator (progress 0.6-0.8)
            connect(m_gyroCalibrator, &GyroCalibrator::progressChanged, this,
                    [this](double fraction, const QString &status) {
                m_autoSyncProgress = 0.6 + fraction * 0.2;
                emit autoSyncProgressChanged();
                m_autoSyncStatus = status;
                emit autoSyncStatusChanged();
            }, Qt::QueuedConnection);

            connect(m_gyroCalibrator, &GyroCalibrator::calibrationComputed, this,
                    [this](const GyroCalibration &cal) {
                disconnect(m_gyroCalibrator, nullptr, this, nullptr);
                // Store PER VIDEO only. Writing the camera-wide default here
                // is what turned one bad AutoSync run into a permanent,
                // silent regression on every other clip: integrateImu() falls
                // back to the camera default for any video without its own
                // sidecar calibration, so a single ill-conditioned solve on
                // one clip re-scaled and cross-mixed the gyro for all of them,
                // and survived restarts. Promoting a calibration to the camera
                // default is now an explicit user action
                // (saveGyroCalibrationAsCameraDefault()).
                m_keyframes->setGyroCalibration(cal.matrix, cal.bias);
                saveKeyframes();
                qInfo() << "Gyro calibration accepted for this video: residual"
                        << cal.residualDeg << "deg/s," << cal.samplesUsed << "samples";

                // Diagnostics only. The per-axis diagonal fit (omega_visual =
                // s * omega_raw + b) implies a scale relative to the parser's
                // LSB/(deg/s) divisor, and it used to be written straight into
                // the camera-default gyroScaleX/Y/Z. That was doubly wrong:
                // it is a global write from a per-clip measurement, and
                // integrateImu() then folds the resulting scaleDiag on top of
                // the stored matrix M whose diagonal already encodes the SAME
                // correction — so the scale was applied twice. Report it and
                // let the operator decide.
                {
                    const double sx = cal.diagScale.x(), sy = cal.diagScale.y(), sz = cal.diagScale.z();
                    qInfo() << "Gyro visual diagScale factors:" << sx << sy << sz
                            << "(bias" << cal.diagBias.x() << cal.diagBias.y() << cal.diagBias.z()
                            << "deg/s, resid" << cal.residualDeg << "deg/s, samples" << cal.samplesUsed << ")";
                }

                m_autoSyncStatus = tr("Re-integrating IMU…");
                emit autoSyncStatusChanged();
                // Stage 4 runs the fusion on its own thread; suppress the one
                // integrateImu() would otherwise do inline on the GUI thread.
                m_deferVisualFusion = true;
                integrateImu();
                m_deferVisualFusion = false;

                // Stage 4: Visual Fusion (progress 0.8-1.0)
                m_autoSyncStatus = tr("Fusing visual + IMU…");
                emit autoSyncStatusChanged();
                m_autoSyncProgress = 0.85;
                emit autoSyncProgressChanged();

                auto *fusionThread = QThread::create([this]() {
                    applyVisualFusion();
                });

                connect(fusionThread, &QThread::finished, this, [this, fusionThread]() {
                    fusionThread->deleteLater();

                    m_autoSyncProgress = 1.0;
                    m_autoSyncRunning = false;
                    m_autoSyncStatus = tr("Sync & calibration complete");
                    emit autoSyncProgressChanged();
                    emit autoSyncRunningChanged();
                    emit autoSyncStatusChanged();
                    if (m_imuStabilize)
                        emit imuOrientationChanged();
                }, Qt::QueuedConnection);

                fusionThread->start();
            }, Qt::QueuedConnection);

            connect(m_syncSolver, &SyncSolver::solveFailed, this,
                    [this](const QString &error) {
                disconnect(m_syncSolver, nullptr, this, nullptr);
                m_autoSyncRunning = false;
                m_autoSyncStatus = tr("Sync failed: %1").arg(error);
                emit autoSyncRunningChanged();
                emit autoSyncStatusChanged();
            }, Qt::QueuedConnection);

            connect(m_gyroCalibrator, &GyroCalibrator::calibrationFailed, this,
                    [this](const QString &error) {
                disconnect(m_gyroCalibrator, nullptr, this, nullptr);
                // Gyro calibration is OPTIONAL — the sync offset solved in the
                // previous stage is already saved and is useful on its own, so
                // a rejected calibration must not present as a failed run. The
                // gyro simply stays uncalibrated, which is the correct and safe
                // outcome: the visual rotation on most clips measures well under
                // the true rate, and accepting that as ground truth is exactly
                // what used to scale the gyro to half and wreck stabilisation.
                // The full reason goes to the log; the status line stays short
                // enough not to stretch the control panel (the label elides and
                // carries the text in its tooltip).
                qInfo() << "Gyro calibration rejected (sync offset kept):" << error;
                m_autoSyncStatus = tr("Sync applied; gyro calibration not accepted");
                emit autoSyncStatusChanged();
                m_autoSyncProgress = 1.0;
                m_autoSyncRunning = false;
                emit autoSyncProgressChanged();
                emit autoSyncRunningChanged();
                if (m_imuStabilize)
                    emit imuOrientationChanged();
            }, Qt::QueuedConnection);

            QMatrix3x3 priorM = m_keyframes->hasGyroCalibration()
                ? m_keyframes->gyroMatrix() : cameraGyroMatrix();
            QVector3D priorB = m_keyframes->hasGyroCalibration()
                ? m_keyframes->gyroBias() : cameraGyroBias();
            m_gyroCalibrator->calibrate(m_visualPairs, m_imuParser->samples(),
                                        m_imuSyncOffset, m_imuDrift, priorM, priorB,
                                        m_imuParser->initialQuaternion());
        }, Qt::QueuedConnection);

        // Kick off the sync solver. Its progressChanged/syncSolved signals
        // are connected above; solve completes synchronously here (it is
        // cheap), emitting syncSolved which advances to the gyro stage.
        const double initDrift = m_imuDrift;
        const double initOffset = m_imuSyncOffset;
        m_syncSolver->solve(m_visualPairs, m_imuParser->samples(),
                            initDrift, initOffset);
    }, Qt::QueuedConnection);

    connect(m_visualRotation, &VisualRotationComputer::computationFailed, this,
            [this](const QString &error) {
        disconnect(m_visualRotation, nullptr, this, nullptr);
        m_autoSyncRunning = false;
        m_autoSyncStatus = tr("Visual rotation failed: %1").arg(error);
        emit autoSyncRunningChanged();
        emit autoSyncStatusChanged();
    }, Qt::QueuedConnection);

    // frameSkip=1 (every 2nd frame) rather than the default 3. At 3 the decoder
    // keeps one frame in four and the hop search then spans up to four of THOSE,
    // so the shortest possible pair is 0.13 s and a 5-7 s calibration clip
    // yields only ~15 pairs in total — below what a 12-parameter fit can use,
    // and long enough per hop that the body axes rotate appreciably within one.
    // Halving the stride doubles the pair count and halves the minimum hop.
    // Decoding stays bounded by MAX_DECODED_FRAMES.
    // Hand AutoSync the proxy the user selected, if any: on Android the clip
    // is a content:// URI whose "_thm" sibling cannot be derived, and decoding
    // 2880x5760 for analysis instead of 720x1440 would take ~30 minutes.
    m_visualRotation->setDecodeSourceOverride(m_proxyOverride);
    m_visualRotation->compute(m_videoPath, m_currentCalibration, /*frameSkip=*/1);
}

QString App::grabStill(int lens)
{
    if (!m_decoder || !m_decoder->hasFrame())
        return QString();

    DecodedFrame f = m_decoder->currentFrame();
    int w = f.width;
    int h = f.height;
    if (w <= 0 || h <= 0)
        return QString();

    AVPixelFormat srcFormat = m_decoder->isFullRange() ? AV_PIX_FMT_YUVJ420P : AV_PIX_FMT_YUV420P;
    SwsContext *sws = sws_getContext(w, h, srcFormat, w, h, AV_PIX_FMT_RGB24, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws)
        return QString();

    int dstStride = w * 3;
    QByteArray rgb(dstStride * h, Qt::Uninitialized);

    uint8_t *srcData[4] = {
        (uint8_t*)f.yData.constData(),
        (uint8_t*)f.uData.constData(),
        (uint8_t*)f.vData.constData(),
        nullptr
    };
    int srcLinesize[4] = { f.yStride, f.uStride, f.vStride, 0 };
    uint8_t *dstData[4] = { (uint8_t*)rgb.data(), nullptr, nullptr, nullptr };
    int dstLinesize[4] = { dstStride, 0, 0, 0 };

    sws_scale(sws, srcData, srcLinesize, 0, h, dstData, dstLinesize);
    sws_freeContext(sws);

    QImage full = QImage((const uchar*)rgb.constData(), w, h, dstStride, QImage::Format_RGB888).copy();
    QImage half = (lens == 1) ? full.copy(0, h / 2, w, h / 2) : full.copy(0, 0, w, h / 2);

    QString path = QDir::temp().filePath(QString("render360_still_%1_%2.png")
                                             .arg(lens)
                                             .arg(QDateTime::currentMSecsSinceEpoch()));
    if (!half.save(path))
        return QString();
    return path;
}


QVariantMap App::frameHistogram(int maxSamples) const
{
    QVariantMap out;
    if (!m_decoder)
        return out;

    // Build the grade exactly as the export snapshot does, so the "graded"
    // histogram is the same transform the viewer and the exporter apply.
    GradeParams grade;
    QImage lutImg;
    if (m_colorGrade) {
        grade.brightness = m_colorGrade->brightness();
        grade.contrast = m_colorGrade->contrast();
        grade.saturation = m_colorGrade->saturation();
        grade.pop = m_colorGrade->pop();
        grade.brightLows = m_colorGrade->brightLows();
        grade.brightLowMids = m_colorGrade->brightLowMids();
        grade.brightHighMids = m_colorGrade->brightHighMids();
        grade.brightHighs = m_colorGrade->brightHighs();
        grade.redLows = m_colorGrade->redLows();
        grade.redMids = m_colorGrade->redMids();
        grade.redHighs = m_colorGrade->redHighs();
        grade.greenLows = m_colorGrade->greenLows();
        grade.greenMids = m_colorGrade->greenMids();
        grade.greenHighs = m_colorGrade->greenHighs();
        grade.blueLows = m_colorGrade->blueLows();
        grade.blueMids = m_colorGrade->blueMids();
        grade.blueHighs = m_colorGrade->blueHighs();
        if (m_colorGrade->curvesActive())
            lutImg = m_colorGrade->curveLut().convertToFormat(QImage::Format_RGBA8888);
    }
    const uchar *lut = lutImg.isNull() ? nullptr : lutImg.constScanLine(0);

    FrameHistogram src, graded;
    if (!m_decoder->sampleHistogram(&src, &graded, &grade, lut, maxSamples))
        return out;

    auto toList = [](const quint32 *bins) {
        QVariantList l;
        l.reserve(256);
        for (int i = 0; i < 256; ++i)
            l.append(QVariant(uint(bins[i])));
        return l;
    };
    auto pack = [&toList](const FrameHistogram &h) {
        QVariantMap m;
        m.insert(QStringLiteral("r"), toList(h.r));
        m.insert(QStringLiteral("g"), toList(h.g));
        m.insert(QStringLiteral("b"), toList(h.b));
        m.insert(QStringLiteral("luma"), toList(h.luma));
        return m;
    };
    out.insert(QStringLiteral("source"), pack(src));
    out.insert(QStringLiteral("graded"), pack(graded));
    out.insert(QStringLiteral("samples"), QVariant(qulonglong(src.samples)));
    return out;
}
