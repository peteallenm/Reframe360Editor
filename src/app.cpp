#include "app.h"
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QDateTime>
#include <QQuaternion>
#include <QMatrix4x4>
#include <QtMath>
extern "C" {
#include <libswscale/swscale.h>
}

App::App(QObject *parent)
    : QObject(parent)
    , m_decoder(new VideoDecoder(this))
    , m_imuParser(new ImuParser(this))
    , m_gyroIntegrator(new GyroscopeIntegrator(this))
    , m_calibrationPresets(new CalibrationPresetModel(this))
    , m_currentCalibration(new CalibrationProfile(this))
    , m_isPlaying(false)
    , m_currentTime(0.0)
    , m_yaw(0.0)
    , m_pitch(0.0)
    , m_roll(0.0)
    , m_fov(90.0)
    , m_imuStabilize(false)
    , m_imuSmoothing(0.5)
    , m_imuSyncOffset(0.0)
    , m_activeLens(2)
    , m_projection(0)
{
    connect(m_decoder, &VideoDecoder::durationChanged, this, &App::durationChanged);
    connect(m_decoder, &VideoDecoder::currentTimeChanged, this, &App::currentTimeChanged);
    connect(m_decoder, &VideoDecoder::errorOccurred, this, &App::errorOccurred);

    m_viewQuat = viewQuatFromEuler();

    int defaultIdx = m_calibrationPresets->defaultPresetIndex();
    if (defaultIdx >= 0)
        m_calibrationPresets->loadPreset(defaultIdx, m_currentCalibration);
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
    emit videoPathChanged();

    if (!path.isEmpty()) {
        m_decoder->loadVideo(path);

        QString imuPath = path + ".imu";
        if (QFileInfo::exists(imuPath)) {
            m_imuParser->loadFile(imuPath);
            m_gyroIntegrator->integrate(m_imuParser->rawGyroData(), m_imuParser->sampleRate());
        }

        emit videoLoaded();
    }
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
    QQuaternion dq = QQuaternion::fromAxisAndAngle(1, 0, 0, angleAboutRight)
                   * QQuaternion::fromAxisAndAngle(0, 1, 0, angleAboutUp);
    m_viewQuat = (m_viewQuat * dq).normalized();
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
        return m_gyroIntegrator->orientationAtTime(m_currentTime + m_imuSyncOffset);
    }
    return QQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
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
