#include "imudriftcalibrator.h"
#include "horizondetector.h"
#include "gyroscopeintegrator.h"

#include <QThread>
#include <QtMath>
#include <QDebug>
#include <cmath>
#include <algorithm>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// Minimum number of valid horizon detections to produce a calibration result.
static const int kMinValidSamples = 5;

ImuDriftCalibrator::ImuDriftCalibrator(QObject *parent)
    : QObject(parent)
{
}

double ImuDriftCalibrator::extractRoll(const QQuaternion &q)
{
    float w = q.scalar(), x = q.x(), y = q.y(), z = q.z();
    double m10 = 2.0 * (x * y + z * w);
    double m11 = 1.0 - 2.0 * (x * x + z * z);
    return qRadiansToDegrees(std::atan2(m10, m11));
}

void ImuDriftCalibrator::startCalibration(const QString &videoPath,
                                           const QVector<QQuaternion> &imuOrientations,
                                           const QVector<double> &imuTimestamps,
                                           double syncOffset,
                                           double initialDrift,
                                           int numSamples)
{
    if (m_running) {
        emit calibrationFailed(QStringLiteral("Calibration already in progress"));
        return;
    }
    if (imuOrientations.isEmpty() || imuTimestamps.isEmpty()) {
        emit calibrationFailed(QStringLiteral("No IMU data available"));
        return;
    }
    if (videoPath.isEmpty()) {
        emit calibrationFailed(QStringLiteral("No video loaded"));
        return;
    }

    // Snapshot all parameters for the worker thread.
    m_videoPath = videoPath;
    m_imuOrientations = imuOrientations;
    m_imuTimestamps = imuTimestamps;
    m_syncOffset = syncOffset;
    m_initialDrift = initialDrift;
    m_numSamples = numSamples;
    m_running = true;

    // Run in a detached thread. The thread self-deletes on finish.
    QThread *t = QThread::create([this] { runCalibration(); });
    connect(t, &QThread::finished, this, [this] { m_running = false; });
    connect(t, &QThread::finished, t, &QThread::deleteLater);
    t->start();
}

void ImuDriftCalibrator::runCalibration()
{
    // Step 1: Get video duration via a lightweight FFmpeg probe.
    AVFormatContext *probeCtx = nullptr;
    if (avformat_open_input(&probeCtx, m_videoPath.toUtf8().constData(),
                            nullptr, nullptr) < 0) {
        emit calibrationFailed(QStringLiteral("Cannot open video: %1").arg(m_videoPath));
        return;
    }
    if (avformat_find_stream_info(probeCtx, nullptr) < 0) {
        avformat_close_input(&probeCtx);
        emit calibrationFailed(QStringLiteral("Cannot read stream info"));
        return;
    }
    double videoDuration = (double)probeCtx->duration / AV_TIME_BASE;
    avformat_close_input(&probeCtx);

    if (videoDuration <= 0.0) {
        emit calibrationFailed(QStringLiteral("Video duration is zero or negative"));
        return;
    }

    // Step 2: Compute evenly-spaced sample timestamps (avoid exact start/end).
    QVector<double> sampleTimes;
    sampleTimes.reserve(m_numSamples);
    for (int i = 0; i < m_numSamples; i++) {
        double t = videoDuration * (i + 0.5) / m_numSamples;
        sampleTimes.append(t);
    }

    // Step 3: For each sample, decode the frame and detect the horizon.
    QVector<SampleData> samples;
    samples.reserve(m_numSamples);

    for (int i = 0; i < m_numSamples; i++) {
        emit progressChanged((double)i / (m_numSamples * 3.0),
                             QStringLiteral("Sampling frame %1/%2…")
                                 .arg(i + 1).arg(m_numSamples));

        QVector<uint8_t> rgb;
        int w = 0, h = 0;
        if (!decodeFrameAt(m_videoPath, sampleTimes[i], rgb, w, h)) {
            qDebug() << "Horizon cal: failed to decode frame at t=" << sampleTimes[i];
            continue;
        }

        HorizonResult hr = HorizonDetector::detectFromRgb24(rgb.data(), w, h);
        if (!hr.valid) {
            qDebug() << "Horizon cal: detection failed at t=" << sampleTimes[i];
            continue;
        }

        samples.append({sampleTimes[i], hr.rollDeg});
    }

    if (samples.size() < kMinValidSamples) {
        emit calibrationFailed(
            QStringLiteral("Only %1 of %2 frames had a detectable horizon (need ≥%3)")
                .arg(samples.size()).arg(m_numSamples).arg(kMinValidSamples));
        return;
    }

    qDebug() << "Horizon cal:" << samples.size() << "valid samples out of"
             << m_numSamples;

    // Step 4: Grid search for the drift that minimizes total roll error.
    //
    // The IMU orientation at video time t is evaluated at IMU time
    //   t_imu = t * (1 + drift) + syncOffset
    // so drift shifts the IMU timeline. A wrong drift causes the IMU roll to
    // read out at the wrong moment, producing a time-varying roll error.
    //
    // Coarse pass: ±5 ms/s around the initial estimate, 200 steps.
    // Fine pass: ±0.5 ms/s around the coarse best, 200 steps.

    emit progressChanged(0.4, QStringLiteral("Searching for optimal drift…"));

    auto searchRange = [&](double lo, double hi, int steps,
                           double &bestDrift, double &bestErr) {
        for (int s = 0; s <= steps; s++) {
            double d = lo + (hi - lo) * s / steps;
            double err = computeTotalError(samples, d);
            if (err < bestErr) {
                bestErr = err;
                bestDrift = d;
            }
        }
    };

    double bestDrift = m_initialDrift;
    double bestErr = computeTotalError(samples, m_initialDrift);

    // Coarse: ±0.005 s/s
    searchRange(m_initialDrift - 0.005, m_initialDrift + 0.005, 200,
                bestDrift, bestErr);

    emit progressChanged(0.7, QStringLiteral("Refining…"));

    // Fine: ±0.0005 s/s around coarse best
    searchRange(bestDrift - 0.0005, bestDrift + 0.0005, 200,
                bestDrift, bestErr);

    // Step 5: Compute RMS residual at the best drift.
    double totalErr = computeTotalError(samples, bestDrift);
    double rmsDeg = std::sqrt(totalErr / samples.size());

    qDebug() << "Horizon cal: drift =" << bestDrift
             << "s/s, RMS residual =" << rmsDeg << "deg";

    emit progressChanged(1.0, QStringLiteral("Calibration complete"));
    emit calibrationFinished(bestDrift, rmsDeg);
}

double ImuDriftCalibrator::computeTotalError(const QVector<SampleData> &samples,
                                              double drift) const
{
    double totalErr = 0.0;
    for (const auto &s : samples) {
        // Map video time to IMU time using the candidate drift.
        double tImu = s.time * (1.0 + drift) + m_syncOffset;

        QQuaternion q = GyroscopeIntegrator::orientationAt(
            m_imuOrientations, m_imuTimestamps, tImu, 0.0f);

        double imuRoll = extractRoll(q);

        // Wrap the error into [-180, 180].
        double err = s.rollDetected - imuRoll;
        while (err > 180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;

        totalErr += err * err;
    }
    return totalErr;
}

bool ImuDriftCalibrator::decodeFrameAt(const QString &videoPath, double time,
                                        QVector<uint8_t> &rgb, int &width, int &height)
{
    AVFormatContext *fmtCtx = nullptr;
    if (avformat_open_input(&fmtCtx, videoPath.toUtf8().constData(),
                            nullptr, nullptr) < 0)
        return false;

    if (avformat_find_stream_info(fmtCtx, nullptr) < 0) {
        avformat_close_input(&fmtCtx);
        return false;
    }

    int videoIdx = av_find_best_stream(fmtCtx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (videoIdx < 0) {
        avformat_close_input(&fmtCtx);
        return false;
    }

    AVStream *stream = fmtCtx->streams[videoIdx];
    AVCodecParameters *codecpar = stream->codecpar;
    const AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        avformat_close_input(&fmtCtx);
        return false;
    }

    AVCodecContext *codecCtx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codecCtx, codecpar);
    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        avcodec_free_context(&codecCtx);
        avformat_close_input(&fmtCtx);
        return false;
    }

    // Seek to a keyframe before the target time.
    int64_t targetTs = av_rescale_q(
        (int64_t)(time * AV_TIME_BASE), AV_TIME_BASE_Q, stream->time_base);
    av_seek_frame(fmtCtx, videoIdx, targetTs, AVSEEK_FLAG_BACKWARD);
    avcodec_flush_buffers(codecCtx);

    AVPacket *pkt = av_packet_alloc();
    AVFrame *frame = av_frame_alloc();
    bool found = false;

    // Decode forward from the keyframe until we reach a frame near the target.
    // Accept frames within 100 ms of the target (the seek may land early).
    while (av_read_frame(fmtCtx, pkt) >= 0) {
        if (pkt->stream_index != videoIdx) {
            av_packet_unref(pkt);
            continue;
        }

        int sendRet = avcodec_send_packet(codecCtx, pkt);
        av_packet_unref(pkt);
        if (sendRet < 0) continue;

        while (avcodec_receive_frame(codecCtx, frame) >= 0) {
            double frameTime = 0.0;
            if (frame->pts != AV_NOPTS_VALUE) {
                frameTime = (double)frame->pts * av_q2d(stream->time_base);
                if (stream->start_time != AV_NOPTS_VALUE)
                    frameTime -= (double)stream->start_time * av_q2d(stream->time_base);
            }

            if (frameTime >= time - 0.1) {
                // Convert to RGB24.
                width = frame->width;
                height = frame->height;
                rgb.resize(width * height * 3);

                enum AVPixelFormat srcFmt = codecCtx->pix_fmt;
                SwsContext *sws = sws_getContext(
                    width, height, srcFmt,
                    width, height, AV_PIX_FMT_RGB24,
                    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
                if (sws) {
                    uint8_t *dstData[4] = { rgb.data(), nullptr, nullptr, nullptr };
                    int dstLinesize[4] = { width * 3, 0, 0, 0 };
                    sws_scale(sws, frame->data, frame->linesize, 0, height,
                              dstData, dstLinesize);
                    sws_freeContext(sws);
                    found = true;
                }
                break;
            }
        }
        if (found) break;
    }

    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&codecCtx);
    avformat_close_input(&fmtCtx);

    return found;
}
