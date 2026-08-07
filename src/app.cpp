#include "app.h"
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QSettings>
#include <QDateTime>
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

App::App(QObject *parent)
    : QObject(parent)
    , m_decoder(new VideoDecoder(this))
    , m_imuParser(new ImuParser(this))
    , m_gyroIntegrator(new GyroscopeIntegrator(this))
    , m_calibrationPresets(new CalibrationPresetModel(this))
    , m_currentCalibration(new CalibrationProfile(this))
    , m_keyframes(new KeyframeModel(this))
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
    , m_gravityAlign()
{
    connect(m_decoder, &VideoDecoder::durationChanged, this, &App::durationChanged);
    connect(m_decoder, &VideoDecoder::currentTimeChanged, this, [this]() {
        m_currentTime = m_decoder->currentTime();
        applyKeyframeInterpolation();
        emit currentTimeChanged();
        if (m_imuStabilize)
            emit imuOrientationChanged();
    });
    connect(m_decoder, &VideoDecoder::errorOccurred, this, &App::errorOccurred);

    m_viewQuat = viewQuatFromEuler();

    int defaultIdx = m_calibrationPresets->defaultPresetIndex();
    if (defaultIdx >= 0)
        m_calibrationPresets->loadPreset(defaultIdx, m_currentCalibration);

    // Restore the saved stabilization/gyro settings, then keep them persisted
    // automatically whenever the user changes them.
    loadSettings();
    connect(this, &App::imuStabilizeChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuSmoothingChanged, this, [this]() { saveSettings(); });
    connect(this, &App::imuSyncOffsetChanged, this, [this]() { saveSettings(); });
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

    if (!path.isEmpty()) {
        m_decoder->loadVideo(path);

        QString imuPath = path + ".imu";
        if (QFileInfo::exists(imuPath)) {
            m_imuParser->loadFile(imuPath);
            m_gyroIntegrator->integrate(m_imuParser->samples(), m_imuParser->imuSampleRate());
            computeGravityAlignment();
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
    if (m_imuStabilize && m_gyroIntegrator) {
        QQuaternion q = m_gyroIntegrator->orientationAtTime(m_currentTime + m_imuSyncOffset);
        // Bake in the gravity alignment: LensViewer passes the conjugate of
        // this quaternion to the shader, so passing A^-1 * Q makes the shader
        // sample Q^-1 * A * euler * ray and the displayed (stabilized) world
        // direction A * euler * ray lives in a gravity-aligned frame. The
        // yaw/pitch/roll look angles are therefore relative to gravity (not to
        // the camera's start pose) and the default 0/0/0 view is level.
        return m_gravityAlign.conjugated() * q;
    }
    return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
}

void App::computeGravityAlignment()
{
    m_gravityAlign = QQuaternion();  // identity: no correction
    if (!m_imuParser)
        return;

    const QVector<ImuSample> &samples = m_imuParser->samples();
    if (samples.isEmpty())
        return;

    // The accelerometer at rest reads the specific force (~1g, pointing away
    // from gravity). Rotating every sample into camera axes with the inverse
    // of the header's IMU->camera quaternion (config[4..7], exposed as
    // ImuParser::initialQuaternion) and taking the per-axis median over the
    // whole clip gives a robust estimate of gravity-up in camera axes: linear
    // accelerations average out, and it survives a pose change between IMU
    // power-on and recording start. Empirically q^-1 * accel == +Y (up) for a
    // level camera on all test files (JustYaw/Pitch/Roll, YIVR_0807/0771).
    const QQuaternion qInv = m_imuParser->initialQuaternion().conjugated();

    QVector<QVector3D> up;
    up.reserve(samples.size());
    for (const ImuSample &s : samples) {
        QVector3D v = qInv.rotatedVector(s.accel);
        float len = v.length();
        if (len > 0.1f)
            up.append(v / len);
    }
    if (up.isEmpty())
        return;

    auto medianComp = [&up](int axis) {
        QVector<float> vals;
        vals.reserve(up.size());
        for (const QVector3D &v : up)
            vals.append(v[axis]);
        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
        return vals[vals.size() / 2];
    };
    QVector3D gUp(medianComp(0), medianComp(1), medianComp(2));
    float gl = gUp.length();
    if (gl < 0.1f)
        return;
    gUp /= gl;

    // A maps the display up axis (+Y) onto the true gravity-up direction, so
    // the leveled default view has "up" = gravity up and roll = 0.
    m_gravityAlign = QQuaternion::rotationTo(QVector3D(0.0f, 1.0f, 0.0f), gUp);
}

void App::exportFrame(const QString &path, int width, int height)
{
    Q_UNUSED(path);
    Q_UNUSED(width);
    Q_UNUSED(height);
}

void App::exportVideo(const QString &path, int width, int height, double fps, double startTime, double endTime)
{
    Q_UNUSED(path);
    Q_UNUSED(width);
    Q_UNUSED(height);
    Q_UNUSED(fps);
    Q_UNUSED(startTime);
    Q_UNUSED(endTime);
}

void App::loadSettings()
{
    QSettings s;
    setImuStabilize(s.value(QStringLiteral("imu/stabilize"), m_imuStabilize).toBool());
    setImuSmoothing(s.value(QStringLiteral("imu/smoothing"), m_imuSmoothing).toDouble());
    setImuSyncOffset(s.value(QStringLiteral("imu/syncOffset"), m_imuSyncOffset).toDouble());
}

void App::saveSettings() const
{
    QSettings s;
    s.setValue(QStringLiteral("imu/stabilize"), m_imuStabilize);
    s.setValue(QStringLiteral("imu/smoothing"), m_imuSmoothing);
    s.setValue(QStringLiteral("imu/syncOffset"), m_imuSyncOffset);
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
