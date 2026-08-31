#include "videodecoder.h"
#include <QDebug>
#include <QDateTime>
#include "avinput.h"

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
    , m_targetTime(-1.0)
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
    closeInput();
}

// Free the format context and, if one was used, the QFile-backed AVIOContext.
// Order matters: avformat_close_input() must go first, and with AVFMT_FLAG_
// CUSTOM_IO libavformat does NOT free our AVIOContext or its buffer.
void VideoDecoder::closeInput()
{
    if (m_codecCtx) avcodec_free_context(&m_codecCtx);
    m_input.close();          // owns m_formatCtx
    m_formatCtx = nullptr;
}

void VideoDecoder::loadVideo(const QString &path)
{
    stopPlayback();

    // Tear the decode thread down before touching the contexts it is reading.
    // Clearing m_running is what makes decodeLoop() return; the thread then
    // has to be joined and DELETED, because startPlayback()/seekTo() only
    // create one when m_decodeThread is null. Leaving a finished QThread
    // behind meant the second load of a session -- switching to the thumbnail
    // after playing the full video -- never decoded anything again and the
    // viewer sat there frozen.
    {
        QMutexLocker lock(&m_mutex);
        m_running = false;
    }
    if (m_decodeThread) {
        m_decodeThread->quit();
        m_decodeThread->wait();
        delete m_decodeThread;
        m_decodeThread = nullptr;
    }

    // Only now is it safe to close the old input: nothing is decoding from it.
    QMutexLocker ffmpegLock(&m_ffmpegMutex);
    closeInput();

    // Any decoded frame belongs to the OLD clip. Drop it: on a failed open the
    // viewer must not keep showing the previous video, which is what made a
    // failure look like a hang.
    {
        QMutexLocker lock(&m_mutex);
        m_hasFrame = false;
        m_currentFrame = DecodedFrame();
    }

    qDebug() << "Opening video file:" << path;

    // AvInput (avinput.h) handles both a filesystem path and a URL; Android's
    // picker returns content://, which libavformat cannot open itself.
    int ret = m_input.open(path);
    m_formatCtx = m_input.fmt;
    if (ret < 0) {
        char err[AV_ERROR_MAX_STRING_SIZE] = {0};
        av_strerror(ret, err, sizeof(err));
        closeInput();
        emit errorOccurred(tr("Failed to open video: %1 (%2)").arg(path, QString::fromLatin1(err)));
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

#ifdef Q_OS_ANDROID
    // Hardware first: MediaCodec decodes the proxy for free (the CPU cost of
    // software decode was the largest remaining per-frame CPU consumer on the
    // phone -- battery and thermal headroom). tryOpenMediaCodec() is
    // conservative: it only claims the clip after decoding a real frame from
    // it, so a failure of any kind lands on the software path below.
    if (!tryOpenMediaCodec(codecpar))
        m_codecCtx = nullptr;
    if (!m_codecCtx) {
#endif
    m_codecCtx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(m_codecCtx, codecpar);

    // Decode across all cores. This was never set, so playback ran on a single
    // thread -- barely noticeable on a desktop with the 720x1440 proxy, but the
    // difference between usable and apparently hung on a phone opening the
    // 2880x5760 original (~16 M pixels per frame, no hardware decoder in
    // existence handles 5760 tall). VisualRotationComputer already opens its
    // analysis decoder this way; this is the same tuning for playback.
    m_codecCtx->thread_count = 0;                                   // = one per core
    m_codecCtx->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;

    if (avcodec_open2(m_codecCtx, codec, nullptr) < 0) {
        emit errorOccurred("Failed to open codec");
        return;
    }
#ifdef Q_OS_ANDROID
    m_usingHwDecoder = false;
    m_hwBadStreak = 0;
    }
#endif

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
        m_targetTime = -1.0;
        m_hasFrame = false;
    }
    qDebug() << "Video loaded:" << m_videoWidth << "x" << m_videoHeight
             << "duration:" << m_duration << "fullRange:" << m_isFullRange
             << "threads:" << m_codecCtx->thread_count;
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
        m_targetTime = time;
        m_startTime = getTimeSeconds() - time;
        // The last decoded frame predates the seek (it is AHEAD of a backward
        // target), so don't let its timestamp be read as "reached the target"
        // before a fresh frame is decoded.
        m_hasFrame = false;
    }

    // The decode loop decodes forward from the keyframe and publishes
    // currentTimeChanged only once it reaches the target, so the UI shows the
    // actual frame (not the idealized seek time). Ensure the loop is running.
    if (!m_decodeThread) {
        m_decodeThread = new QThread();
        connect(m_decodeThread, &QThread::started, this, [this]() { decodeLoop(); }, Qt::DirectConnection);
        m_decodeThread->start();
    }
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

bool VideoDecoder::sampleHistogram(FrameHistogram *source, FrameHistogram *graded,
                                   const GradeParams *grade, const unsigned char *curveLut,
                                   int maxSamples, bool circularMask) const
{
    if (!source)
        return false;
    *source = FrameHistogram();
    if (graded)
        *graded = FrameHistogram();

    QMutexLocker lock(&m_mutex);
    if (!m_hasFrame)
        return false;
    const DecodedFrame &f = m_currentFrame;
    if (f.width <= 0 || f.height <= 0 || f.yData.isEmpty() || f.uData.isEmpty() || f.vData.isEmpty())
        return false;

    const uchar *Y = reinterpret_cast<const uchar *>(f.yData.constData());
    const uchar *U = reinterpret_cast<const uchar *>(f.uData.constData());
    const uchar *V = reinterpret_cast<const uchar *>(f.vData.constData());

    // Guard against a short plane (a truncated decode) rather than walking off
    // the end of the buffer.
    const int cw = f.width / 2;
    const int ch = f.height / 2;
    if (f.yData.size() < qsizetype(f.yStride) * f.height
        || f.uData.size() < qsizetype(f.uStride) * ch
        || f.vData.size() < qsizetype(f.vStride) * ch)
        return false;

    const int stepX = qMax(1, f.width / qMax(1, maxSamples));
    const int stepY = qMax(1, f.height / qMax(1, maxSamples));

    // Stacked dual fisheye: two square halves, each with an inscribed circle.
    const bool stacked = circularMask && (f.height == 2 * f.width);
    const double cx = f.width * 0.5;
    const double cyTop = f.height * 0.25;
    const double cyBot = f.height * 0.75;
    // 0.99 of the half-width: the image circle is inscribed, and the last
    // texel ring is vignetted rather than black.
    const double rad = 0.99 * (f.width * 0.5);
    const double rad2 = rad * rad;

    const bool full = m_isFullRange;

    for (int y = 0; y < f.height; y += stepY) {
        const uchar *yrow = Y + qsizetype(f.yStride) * y;
        const uchar *urow = U + qsizetype(f.uStride) * (y / 2);
        const uchar *vrow = V + qsizetype(f.vStride) * (y / 2);
        const double dyTop = y - cyTop;
        const double dyBot = y - cyBot;
        const bool topHalf = (y < f.height / 2);
        const double dy = topHalf ? dyTop : dyBot;
        const double dy2 = dy * dy;
        if (stacked && dy2 > rad2)
            continue;                       // whole row outside both circles
        for (int x = 0; x < f.width; x += stepX) {
            if (stacked) {
                const double dx = x - cx;
                if (dx * dx + dy2 > rad2)
                    continue;
            }
            const int cxi = qMin(x / 2, cw - 1);
            double yy = yrow[x] / 255.0;
            const double uu = urow[cxi] / 255.0 - 0.5;
            const double vv = vrow[cxi] / 255.0 - 0.5;
            if (!full)
                yy = (yy - 16.0 / 255.0) * (255.0 / 219.0);
            // Same coefficients as yuvToRgb() in shaders/project.frag, so the
            // histogram describes the pixels the viewer is showing.
            const double r = yy + 1.402 * vv;
            const double g = yy - 0.344136 * uu - 0.714136 * vv;
            const double b = yy + 1.772 * uu;
            // Clamp to [0,1] HERE, before anything else looks at the value:
            // yuvToRgb() clamps in project.frag and in the CPU exporter, so an
            // out-of-gamut YUV triple (common in highlights) must be brought
            // into range BEFORE grading. Grading the unclamped value and
            // clamping afterwards is a different function and drifted from
            // what the viewer shows.
            const double cr0 = qBound(0.0, r, 1.0);
            const double cg0 = qBound(0.0, g, 1.0);
            const double cb0 = qBound(0.0, b, 1.0);

            auto bin = [](double v) {
                return int(qBound(0.0, v, 1.0) * 255.0 + 0.5);
            };
            ++source->r[bin(cr0)];
            ++source->g[bin(cg0)];
            ++source->b[bin(cb0)];
            // Rec.709 luma of the (already range-corrected) RGB.
            ++source->luma[bin(0.2126 * cr0 + 0.7152 * cg0 + 0.0722 * cb0)];
            ++source->samples;

            if (graded) {
                // The same maths the shader and the CPU exporter run, from
                // grade.h -- so this really is the output, not an approximation
                // of it. The grade works in 0..255 and clamps only at the end.
                double gr = cr0 * 255.0, gg = cg0 * 255.0, gb = cb0 * 255.0;
                if (grade)
                    applyGrade(*grade, gr, gg, gb);
                uchar cr = uchar(qBound(0.0, gr, 255.0) + 0.5);
                uchar cg = uchar(qBound(0.0, gg, 255.0) + 0.5);
                uchar cb = uchar(qBound(0.0, gb, 255.0) + 0.5);
                if (curveLut)
                    applyCurveLut(curveLut, cr, cg, cb);
                ++graded->r[cr];
                ++graded->g[cg];
                ++graded->b[cb];
                ++graded->luma[int(qBound(0.0, 0.2126 * cr + 0.7152 * cg + 0.0722 * cb, 255.0) + 0.5)];
                ++graded->samples;
            }
        }
    }
    return source->samples > 0;
}

void VideoDecoder::decodeLoop()
{
    while (true) {
        bool playing;
        double target;
        {
            QMutexLocker lock(&m_mutex);
            if (!m_running) return;
            playing = m_playing;
            target = m_targetTime;
        }

        // Idle while paused with no pending seek (wait for a seek or play).
        // During a seek the target is set even while paused, so the loop
        // decodes forward from the keyframe as fast as possible to reach it.
        if (!playing && target < 0.0) {
            QThread::msleep(10);
            continue;
        }

        DecodeResult result = decodeFrame();

        if (result == DecodeResult::Eof) {
            // The stream reached the end and rewound to the start (loop-back).
            // Reset the playback clock here, otherwise the throttle below
            // compares against the original start time and the next lap
            // decodes as fast as possible ("plays really fast"). Clear any
            // pending seek target so the loop returns to idle instead of
            // spinning forever past the end.
            {
                QMutexLocker lock(&m_mutex);
                m_currentTime = 0.0;
                m_targetTime = -1.0;
                m_startTime = getTimeSeconds();
            }
            emit currentTimeChanged();
            continue;
        }

        double frameTimestamp;
        bool reachedTarget = false;
        {
            QMutexLocker lock(&m_mutex);
            // Ignore the stale frame left from before a seek (m_hasFrame is
            // cleared in seekTo); only a freshly decoded frame's timestamp can
            // satisfy a pending target or update playback time.
            frameTimestamp = m_hasFrame ? m_currentFrame.timestamp : -1.0;

            if (m_targetTime >= 0.0) {
                // Seeking: decode forward and only publish the current time
                // once a frame reaches/passes the target, so the UI shows the
                // actual frame (not the idealized seek time).
                if (frameTimestamp >= 0.0 && frameTimestamp >= m_targetTime) {
                    m_currentTime = frameTimestamp;
                    m_targetTime = -1.0;
                    reachedTarget = true;
                }
            } else if (frameTimestamp >= 0.0) {
                // Normal playback: use actual frame PTS, not a fixed delta.
                m_currentTime = frameTimestamp;
                reachedTarget = true;  // publish the playback position
            }

            if (m_currentTime >= m_duration && m_duration > 0.0) {
                m_playing = false;
                m_currentTime = m_duration;
                reachedTarget = true;
            }
        }

        if (reachedTarget)
            emit currentTimeChanged();

        // Throttle to real-time only during normal playback (no pending seek);
        // a seek decodes as fast as possible to reach the target, whether the
        // user scrubbed while paused or while playing.
        if (playing && target < 0.0 && frameTimestamp >= 0.0) {
            double wallElapsed = getTimeSeconds() - m_startTime;
            double due = m_currentTime - wallElapsed;
            if (due > 0.0 && due < 1.0) {
                QThread::msleep((unsigned long)(due * 1000.0));
            }
        }
    }
}

#ifdef Q_OS_ANDROID
bool VideoDecoder::tryOpenMediaCodec(const AVCodecParameters *codecpar)
{
    // MediaCodec tops out around 4K; the 2880x5760 original always exceeds it
    // (nothing decodes 5760 tall in hardware), the 720x1440 proxy never does.
    const int w = codecpar->width, h = codecpar->height;
    if (w <= 0 || h <= 0 || qMax(w, h) > 4096 || qint64(w) * h > qint64(4096) * 2304)
        return false;

    const char *name = nullptr;
    if (codecpar->codec_id == AV_CODEC_ID_H264)      name = "h264_mediacodec";
    else if (codecpar->codec_id == AV_CODEC_ID_HEVC) name = "hevc_mediacodec";
    if (!name)
        return false;

    const AVCodec *hw = avcodec_find_decoder_by_name(name);
    if (!hw)
        return false;

    AVCodecContext *ctx = avcodec_alloc_context3(hw);
    if (!ctx)
        return false;
    avcodec_parameters_to_context(ctx, codecpar);
    if (avcodec_open2(ctx, hw, nullptr) < 0) {
        avcodec_free_context(&ctx);
        return false;
    }

    // Some devices accept the open and then fail on real data, so nothing is
    // trusted until several consecutive frames have come out WITH VALID
    // TIMESTAMPS -- a decoder that errors internally (C2MtkVdec onError) can
    // still hand frames back, but without PTS, and a timestampless stream
    // pins the playback clock at 0:00. Bounded: a couple of GOPs' worth of
    // packets at most.
    const int kNeedGoodFrames = 5;
    int goodFrames = 0;
    bool decoderBroken = false;
    AVFrame *frame = av_frame_alloc();
    AVPacket *pkt = av_packet_alloc();
    for (int i = 0; i < 120 && goodFrames < kNeedGoodFrames && !decoderBroken; ++i) {
        if (av_read_frame(m_formatCtx, pkt) < 0)
            break;
        if (pkt->stream_index != m_videoStreamIndex) {
            av_packet_unref(pkt);
            continue;
        }
        const int sendRet = avcodec_send_packet(ctx, pkt);
        av_packet_unref(pkt);
        if (sendRet < 0 && sendRet != AVERROR(EAGAIN)) {
            decoderBroken = true;
            break;
        }
        const int recvRet = avcodec_receive_frame(ctx, frame);
        if (recvRet == 0) {
            if (frame->best_effort_timestamp == AV_NOPTS_VALUE)
                decoderBroken = true;   // frames without time are unusable
            else
                ++goodFrames;
        } else if (recvRet != AVERROR(EAGAIN)) {
            decoderBroken = true;
        }
    }
    av_frame_free(&frame);
    av_packet_free(&pkt);

    // Rewind: the probe consumed packets the playback loop must see again.
    av_seek_frame(m_formatCtx, m_videoStreamIndex, 0, AVSEEK_FLAG_BACKWARD);

    if (goodFrames < kNeedGoodFrames) {
        avcodec_free_context(&ctx);
        qInfo() << "MediaCodec decoder" << name << "failed the probe"
                << "(good frames:" << goodFrames << "); using software decode";
        return false;
    }

    avcodec_flush_buffers(ctx);
    m_codecCtx = ctx;
    m_usingHwDecoder = true;
    m_hwBadStreak = 0;
    qInfo() << "Hardware decode:" << name << "for" << w << "x" << h;
    return true;
}

// Replace a misbehaving hardware decoder with the software one, in place,
// keeping the demuxer and playback position. Called from decodeFrame with
// m_ffmpegMutex already held.
bool VideoDecoder::switchToSoftwareDecoder()
{
    if (!m_codecCtx || !m_videoStream)
        return false;
    const AVCodec *sw = avcodec_find_decoder(m_videoStream->codecpar->codec_id);
    if (!sw)
        return false;
    AVCodecContext *ctx = avcodec_alloc_context3(sw);
    if (!ctx)
        return false;
    avcodec_parameters_to_context(ctx, m_videoStream->codecpar);
    ctx->thread_count = 0;
    ctx->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
    if (avcodec_open2(ctx, sw, nullptr) < 0) {
        avcodec_free_context(&ctx);
        return false;
    }
    avcodec_free_context(&m_codecCtx);
    m_codecCtx = ctx;
    m_usingHwDecoder = false;
    m_hwBadStreak = 0;

    // Resume from a keyframe at (or before) where playback got to, and let
    // the loop's seek logic republish time once a fresh frame lands.
    double t;
    {
        QMutexLocker lock(&m_mutex);
        t = qMax(0.0, m_currentTime);
        m_targetTime = t;
        m_hasFrame = false;
    }
    const int64_t ts = (int64_t)(t / av_q2d(m_videoStream->time_base));
    av_seek_frame(m_formatCtx, m_videoStreamIndex, ts, AVSEEK_FLAG_BACKWARD);
    qInfo() << "Hardware decoder produced unusable output; switched to software decode at t =" << t;
    return true;
}
#endif

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
#ifdef Q_OS_ANDROID
        if (m_usingHwDecoder && ++m_hwBadStreak >= 12)
            switchToSoftwareDecoder();
#endif
        return DecodeResult::NoData;
    }

    ret = avcodec_receive_frame(m_codecCtx, frame);
    if (ret == 0) {
#ifdef Q_OS_ANDROID
        if (m_usingHwDecoder) {
            // A frame without a timestamp cannot advance the clock; a stream
            // of them reads as a hang (time pinned at 0:00 while pictures
            // move). Tolerate a few, then abandon the hardware decoder.
            if (frame->best_effort_timestamp == AV_NOPTS_VALUE) {
                if (++m_hwBadStreak >= 12 && switchToSoftwareDecoder()) {
                    av_frame_free(&frame);
                    return DecodeResult::NoData;
                }
            } else {
                m_hwBadStreak = 0;
            }
        }
#endif
        DecodedFrame decoded;
        decoded.width = frame->width;
        decoded.height = frame->height;
        decoded.yStride = frame->linesize[0];
        decoded.uStride = frame->linesize[1];
        decoded.vStride = frame->linesize[2];
        decoded.timestamp = frame->best_effort_timestamp * av_q2d(m_videoStream->time_base);

        int ySize = decoded.yStride * decoded.height;

        decoded.yData.resize(ySize);
        memcpy(decoded.yData.data(), frame->data[0], ySize);

        if (frame->format == AV_PIX_FMT_NV12) {
            // MediaCodec output: chroma is one interleaved UV plane. The
            // viewer, histogram and flow chain all assume three planes, so
            // deinterleave here -- for the proxy that is ~0.5 MB per frame,
            // vastly cheaper than the software decode this path replaced.
            const int cw = decoded.width / 2;
            const int chh = decoded.height / 2;
            decoded.uStride = cw;
            decoded.vStride = cw;
            decoded.uData.resize(cw * chh);
            decoded.vData.resize(cw * chh);
            uchar *du = reinterpret_cast<uchar *>(decoded.uData.data());
            uchar *dv = reinterpret_cast<uchar *>(decoded.vData.data());
            for (int y = 0; y < chh; ++y) {
                const uint8_t *src = frame->data[1] + qint64(y) * frame->linesize[1];
                uchar *u = du + qint64(y) * cw;
                uchar *v = dv + qint64(y) * cw;
                for (int x = 0; x < cw; ++x) {
                    u[x] = src[2 * x];
                    v[x] = src[2 * x + 1];
                }
            }
        } else {
            int uSize = decoded.uStride * (decoded.height / 2);
            int vSize = decoded.vStride * (decoded.height / 2);
            decoded.uData.resize(uSize);
            decoded.vData.resize(vSize);
            memcpy(decoded.uData.data(), frame->data[1], uSize);
            memcpy(decoded.vData.data(), frame->data[2], vSize);
        }

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
#ifdef Q_OS_ANDROID
    // EAGAIN just means "feed more packets"; anything else from a hardware
    // decoder counts against it.
    if (m_usingHwDecoder && ret != AVERROR(EAGAIN) && ++m_hwBadStreak >= 12)
        switchToSoftwareDecoder();
#endif
    return DecodeResult::NoData;
}
