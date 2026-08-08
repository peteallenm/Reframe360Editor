#include "app.h"
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QSettings>
#include <QDateTime>
#include <QTimer>
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
    , m_imuStabilize(false)
    , m_imuSmoothing(0.5)
    , m_imuSyncOffset(0.0)
    , m_usePreview(false)
    , m_activeLens(2)
    , m_projection(0)
    , m_exportRunning(false)
    , m_exportProgress(0.0)
    , m_exportStatus()
    , m_exportStart(0.0)
    , m_exportEnd(0.0)
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

    // Keep the per-video keyframe sidecar file in sync whenever the user adds,
    // edits or deletes keyframes.
    connect(m_keyframes, &KeyframeModel::keyframesChanged, this, &App::saveKeyframes);

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
        m_exportStatus = tr("Export complete: %1").arg(QFileInfo(path).fileName());
        emit exportStatusChanged();
        QTimer::singleShot(1500, this, [this]() { setExportRunning(false); });
    });
    connect(m_exporter, &Exporter::exportError, this, [this](const QString &message) {
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
    connect(this, &App::projectionChanged, this, [this]() { saveSettings(); });

    // Colour grade changes persist automatically (same convention as the
    // projection / IMU settings).
    auto saveGrade = [this]() { saveSettings(); };
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
}

App::~App()
{
}

QString App::videoPath() const
{
    return m_videoPath;
}

void App::setVideoPath(const QString &path)
{
    if (m_videoPath == path)
        return;

    m_videoPath = path;
    m_usePreview = false;
    emit videoPathChanged();
    emit usePreviewThumbnailChanged();

    // A new clip resets the export trim range; durationChanged will set the
    // end point once the decoder reports the clip length.
    m_exportStart = 0.0;
    m_exportEnd = 0.0;
    emit exportStartChanged();
    emit exportEndChanged();

    if (!path.isEmpty()) {
        // Restore this video's saved keyframes and export in/out markers (or
        // clear both when the video has no sidecar yet).
        m_keyframes->loadFromFile(keyframesPathFor(path));
        m_exportStart = m_keyframes->trimIn();
        m_exportEnd = m_keyframes->trimOut();
        emit exportStartChanged();
        emit exportEndChanged();

        m_decoder->loadVideo(path);

        // loadVideo() resets the decoder clock to 0 without emitting
        // currentTimeChanged, so sync our clock and apply the restored
        // keyframes right away instead of waiting for the next seek/play tick.
        m_currentTime = m_decoder->currentTime();
        applyKeyframeInterpolation();

        QString imuPath = path + ".imu";
        if (!QFileInfo::exists(imuPath)) {
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
        if (QFileInfo::exists(imuPath)) {
            m_imuParser->loadFile(imuPath);
            // integrate() seeds the orientation from the accelerometer (so the
            // default view is level, +Y = up) and then tracks the camera with
            // the gyro, so the orientations are already gravity-aligned; no
            // separate static per-clip gravity alignment is needed.
            m_gyroIntegrator->integrate(m_imuParser->samples(),
                                        m_imuParser->imuSampleRate(),
                                        m_imuParser->initialQuaternion(),
                                        (float)m_imuSmoothing);
        }

        emit videoLoaded();
    }
}

QString App::previewThumbnailPath() const
{
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
    // dragging behaves exactly like free look — no IMU correction needed.

    // A drag is a rotation about the view's local axes. Apply the pitch and
    // yaw components separately and reject any component that would push the
    // view past the vertical poles (|pitch| ~ 90 deg), where the yaw/roll
    // Euler decomposition is degenerate and the sliders would swap yaw/roll.
    // A crossing shows up as a ~180 deg jump in the extracted yaw or roll.
    auto angleJump = [](double a, double b) {
        double d = (a > b) ? (a - b) : (b - a);
        return (d > 180.0) ? (360.0 - d) : d;
    };
    // Crossing a pole flips BOTH yaw and roll by ~180 deg (Euler identity
    // R(y,p,r) = R(y+180, 180-p, r+180)); a large but legit single-event drag
    // changes only one of them — hence the AND.
    auto crossesPole = [this, &angleJump](const QQuaternion &q) {
        double y, p, r;
        extractEulerFromQuat(q, y, p, r);
        return angleJump(y, m_yaw) > 90.0 && angleJump(r, m_roll) > 90.0;
    };

    QQuaternion q = m_viewQuat;
    QQuaternion afterPitch = (q * QQuaternion::fromAxisAndAngle(1, 0, 0, angleAboutRight)).normalized();
    if (!crossesPole(afterPitch))
        q = afterPitch;

    QQuaternion afterYaw = (q * QQuaternion::fromAxisAndAngle(0, 1, 0, angleAboutUp)).normalized();
    if (!crossesPole(afterYaw))
        q = afterYaw;

    m_viewQuat = q;
    setViewFromQuat(m_viewQuat);
}

QQuaternion App::viewQuatFromEuler() const
{
    return QQuaternion::fromAxisAndAngle(0, 1, 0, m_yaw)
         * QQuaternion::fromAxisAndAngle(1, 0, 0, m_pitch)
         * QQuaternion::fromAxisAndAngle(0, 0, 1, m_roll);
}

void App::setViewFromQuat(const QQuaternion &q)
{
    float w = q.scalar(), x = q.x(), y = q.y(), z = q.z();

    double m02 = 2.0 * (x * z + y * w);
    double m12 = 2.0 * (y * z - x * w);
    double m22 = 1.0 - 2.0 * (x * x + y * y);
    double m10 = 2.0 * (x * y + z * w);
    double m11 = 1.0 - 2.0 * (x * x + z * z);

    double yaw = qRadiansToDegrees(atan2(m02, m22));
    double pitch = qRadiansToDegrees(-asin(qBound(-1.0, m12, 1.0)));
    double roll = qRadiansToDegrees(atan2(m10, m11));

    if (yaw > 180.0) yaw -= 360.0;
    else if (yaw < -180.0) yaw += 360.0;

    m_yaw = yaw;
    m_pitch = pitch;
    m_roll = roll;
    emit yawChanged();
    emit pitchChanged();
    emit rollChanged();
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
        return m_gyroIntegrator->orientationAtTime(time + m_imuSyncOffset);
    }
    return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
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

void App::exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime, bool gpuBackend)
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
    m_exportStatus = tr("Preparing export…");
    emit exportStatusChanged();

    ExportSnapshot snap = buildExportSnapshot();
    m_exporter->exportVideo(m_videoPath, path, width, height, fps, startTime, endTime,
                            [snap](double t) { return snap.stateAt(t); },
                            gpuBackend);
}

void App::setExportStart(double time)
{
    const double dur = m_decoder ? m_decoder->duration() : 0.0;
    double hi = (dur > 0.0) ? qMin(m_exportEnd, dur) : m_exportEnd;
    time = qBound(0.0, time, hi);
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
    if (qFuzzyCompare(m_exportEnd, time))
        return;
    m_exportEnd = time;
    emit exportEndChanged();
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
    }
    s.keyframes = m_keyframes->keyframes();
    s.imuStabilize = m_imuStabilize;
    if (m_gyroIntegrator) {
        s.imuOrientations = m_gyroIntegrator->orientations();
        s.imuTimestamps = m_gyroIntegrator->timestamps();
    }
    s.syncOffset = m_imuSyncOffset;
    return s;
}

void App::loadSettings()
{
    QSettings s;
    setImuStabilize(s.value(QStringLiteral("imu/stabilize"), m_imuStabilize).toBool());
    setImuSmoothing(s.value(QStringLiteral("imu/smoothing"), m_imuSmoothing).toDouble());
    setImuSyncOffset(s.value(QStringLiteral("imu/syncOffset"), m_imuSyncOffset).toDouble());
    // Clamp to the valid projection ids so a stale/corrupt settings value can
    // never put the QML combo box out of range.
    setProjection(qBound(0, s.value(QStringLiteral("projection"), m_projection).toInt(), 3));

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
    }
}

void App::saveSettings() const
{
    QSettings s;
    s.setValue(QStringLiteral("imu/stabilize"), m_imuStabilize);
    s.setValue(QStringLiteral("imu/smoothing"), m_imuSmoothing);
    s.setValue(QStringLiteral("imu/syncOffset"), m_imuSyncOffset);
    s.setValue(QStringLiteral("projection"), m_projection);

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
    }
}

void App::saveKeyframes()
{
    if (m_videoPath.isEmpty())
        return;
    // Sync the live trim range into the model so the sidecar always records
    // the current in/out markers (the model keeps them only as loaded state).
    m_keyframes->setTrimIn(m_exportStart);
    m_keyframes->setTrimOut(m_exportEnd);
    m_keyframes->saveToFile(keyframesPathFor(m_videoPath));
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
