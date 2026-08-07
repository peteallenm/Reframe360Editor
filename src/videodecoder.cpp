#include "videodecoder.h"
#include <QDebug>
#include <QDateTime>

static double getTimeSeconds() {
    return QDateTime::currentMSecsSinceEpoch() / 1000.0;
}

VideoDecoder::VideoDecoder(QObject *parent)
    : QObject(parent)
    , m_formatCtx(nullptr)
    , m_codecCtx(nullptr)
    , m_videoStream(nullptr)
    , m_videoStreamIndex(-1)
    , m_decodeThread(nullptr)
    , m_running(false)
    , m_playing(false)
    , m_duration(0.0)
    , m_currentTime(0.0)
    , m_startTime(0.0)
    , m_videoWidth(0)
    , m_videoHeight(0)
    , m_isFullRange(false)
    , m_hasFrame(false)
{
}

VideoDecoder::~VideoDecoder()
{
    stopPlayback();
    {
        QMutexLocker lock(&m_mutex);
        m_running = false;
    }
    if (m_decodeThread) {
        m_decodeThread->quit();
        m_decodeThread->wait();
        delete m_decodeThread;
    }

    QMutexLocker lock(&m_ffmpegMutex);
    if (m_codecCtx) avcodec_free_context(&m_codecCtx);
    if (m_formatCtx) avformat_close_input(&m_formatCtx);
}

void VideoDecoder::loadVideo(const QString &path)
{
    stopPlayback();

    QMutexLocker ffmpegLock(&m_ffmpegMutex);
    if (m_codecCtx) avcodec_free_context(&m_codecCtx);
    if (m_formatCtx) avformat_close_input(&m_formatCtx);

    qDebug() << "Opening video file:" << path;
    int ret = avformat_open_input(&m_formatCtx, path.toUtf8().constData(), nullptr, nullptr);
    if (ret < 0) {
        emit errorOccurred(QString("Failed to open video: %1").arg(path));
        return;
    }

    ret = avformat_find_stream_info(m_formatCtx, nullptr);
    if (ret < 0) {
        emit errorOccurred("Failed to find stream info");
        return;
    }

    m_videoStreamIndex = av_find_best_stream(m_formatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (m_videoStreamIndex < 0) {
        emit errorOccurred("No video stream found");
        return;
    }

    m_videoStream = m_formatCtx->streams[m_videoStreamIndex];
    AVCodecParameters *codecpar = m_videoStream->codecpar;

    const AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        emit errorOccurred("Failed to find decoder");
        return;
    }

    m_codecCtx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(m_codecCtx, codecpar);

    if (avcodec_open2(m_codecCtx, codec, nullptr) < 0) {
        emit errorOccurred("Failed to open codec");
        return;
    }

    m_videoWidth = codecpar->width;
    m_videoHeight = codecpar->height;
    m_isFullRange = (codecpar->color_range == AVCOL_RANGE_JPEG) ||
                    (codecpar->format == AV_PIX_FMT_YUVJ420P);

    m_duration = (double)m_formatCtx->duration / AV_TIME_BASE;
    emit durationChanged();

    {
        QMutexLocker lock(&m_mutex);
        m_running = true;
        m_currentTime = 0.0;
        m_hasFrame = false;
    }
    qDebug() << "Video loaded:" << m_videoWidth << "x" << m_videoHeight
             << "duration:" << m_duration << "fullRange:" << m_isFullRange;
}

void VideoDecoder::startPlayback()
{
    {
        QMutexLocker lock(&m_mutex);
        m_playing = true;
        m_startTime = getTimeSeconds() - m_currentTime;
    }
    if (!m_decodeThread) {
        m_decodeThread = new QThread();
        connect(m_decodeThread, &QThread::started, this, [this]() { decodeLoop(); }, Qt::DirectConnection);
        m_decodeThread->start();
    }
}

void VideoDecoder::stopPlayback()
{
    QMutexLocker lock(&m_mutex);
    m_playing = false;
}

void VideoDecoder::seekTo(double time)
{
    QMutexLocker ffmpegLock(&m_ffmpegMutex);
    if (!m_formatCtx || !m_codecCtx) return;

    int64_t targetTs = (int64_t)(time / av_q2d(m_videoStream->time_base));
    av_seek_frame(m_formatCtx, m_videoStreamIndex, targetTs, AVSEEK_FLAG_BACKWARD);
    avcodec_flush_buffers(m_codecCtx);

    {
        QMutexLocker lock(&m_mutex);
        m_currentTime = time;
        m_startTime = getTimeSeconds() - time;
    }
    emit currentTimeChanged();
}

double VideoDecoder::duration() const
{
    return m_duration;
}

double VideoDecoder::currentTime() const
{
    return m_currentTime;
}

DecodedFrame VideoDecoder::currentFrame() const
{
    QMutexLocker lock(&m_mutex);
    return m_currentFrame;
}

bool VideoDecoder::hasFrame() const
{
    QMutexLocker lock(&m_mutex);
    return m_hasFrame;
}

void VideoDecoder::decodeLoop()
{
    while (true) {
        bool playing;
        {
            QMutexLocker lock(&m_mutex);
            if (!m_running) return;
            playing = m_playing;
        }

        if (!playing) {
            QThread::msleep(10);
            continue;
        }

        DecodeResult result = decodeFrame();

        if (result == DecodeResult::Eof) {
            // The stream reached the end and rewound to the start (loop-back).
            // Reset the playback clock here, otherwise the throttle below
            // compares against the original start time and the next lap
            // decodes as fast as possible ("plays really fast").
            {
                QMutexLocker lock(&m_mutex);
                m_currentTime = 0.0;
                m_startTime = getTimeSeconds();
            }
            emit currentTimeChanged();
            continue;
        }

        double frameTimestamp;
        {
            QMutexLocker lock(&m_mutex);
            frameTimestamp = m_currentFrame.timestamp;

            // Use actual frame PTS, not a fixed delta
            if (frameTimestamp >= 0.0) {
                m_currentTime = frameTimestamp;
            }

            if (m_currentTime >= m_duration && m_duration > 0.0) {
                m_playing = false;
                m_currentTime = m_duration;
            }
        }
        emit currentTimeChanged();

        // Throttle to real-time: sleep until the next frame is due
        if (frameTimestamp >= 0.0) {
            double wallElapsed = getTimeSeconds() - m_startTime;
            double due = m_currentTime - wallElapsed;
            if (due > 0.0 && due < 1.0) {
                QThread::msleep((unsigned long)(due * 1000.0));
            }
        }
    }
}

VideoDecoder::DecodeResult VideoDecoder::decodeFrame()
{
    QMutexLocker ffmpegLock(&m_ffmpegMutex);
    if (!m_formatCtx || !m_codecCtx) return DecodeResult::NoData;

    AVPacket *pkt = av_packet_alloc();
    AVFrame *frame = av_frame_alloc();

    int ret = av_read_frame(m_formatCtx, pkt);
    if (ret < 0) {
        av_packet_free(&pkt);
        av_frame_free(&frame);
        // End of stream: rewind so the next read starts at the beginning (loop).
        av_seek_frame(m_formatCtx, m_videoStreamIndex, 0, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(m_codecCtx);
        return DecodeResult::Eof;
    }

    if (pkt->stream_index != m_videoStreamIndex) {
        av_packet_free(&pkt);
        av_frame_free(&frame);
        return DecodeResult::NoData;
    }

    ret = avcodec_send_packet(m_codecCtx, pkt);
    av_packet_free(&pkt);
    if (ret < 0) {
        av_frame_free(&frame);
        return DecodeResult::NoData;
    }

    ret = avcodec_receive_frame(m_codecCtx, frame);
    if (ret == 0) {
        DecodedFrame decoded;
        decoded.width = frame->width;
        decoded.height = frame->height;
        decoded.yStride = frame->linesize[0];
        decoded.uStride = frame->linesize[1];
        decoded.vStride = frame->linesize[2];
        decoded.timestamp = frame->best_effort_timestamp * av_q2d(m_videoStream->time_base);

        int ySize = decoded.yStride * decoded.height;
        int uSize = decoded.uStride * (decoded.height / 2);
        int vSize = decoded.vStride * (decoded.height / 2);

        decoded.yData.resize(ySize);
        decoded.uData.resize(uSize);
        decoded.vData.resize(vSize);

        memcpy(decoded.yData.data(), frame->data[0], ySize);
        memcpy(decoded.uData.data(), frame->data[1], uSize);
        memcpy(decoded.vData.data(), frame->data[2], vSize);

        {
            QMutexLocker frameLock(&m_mutex);
            m_currentFrame = decoded;
            m_hasFrame = true;
        }

        emit frameReady();
        av_frame_free(&frame);
        return DecodeResult::Frame;
    }

    av_frame_free(&frame);
    return DecodeResult::NoData;
}
