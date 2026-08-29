#include "visualrotation.h"
#include "calibration.h"

#include <QThread>
#include <QDebug>
#include <QMutexLocker>
#include <QElapsedTimer>
#include <QAtomicInt>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <QFileInfo>
#include <cmath>
#include <algorithm>
#include <cstring>

static constexpr double PI = 3.14159265358979323846;
static constexpr int TARGET_WIDTH = 640;
static constexpr int ORB_FEATURES = 800;
static constexpr double RATIO_THRESHOLD = 0.75;
// Outlier rejection is adaptive (see solveRotation): a generous initial
// threshold that stays above the per-pair angular motion, tightening toward the
// running RMS residual each iteration. A fixed 2° threshold collapsed to zero
// inliers as soon as the true per-pair rotation exceeded ~2° (e.g. a fast 360°
// test clip at frameSkip=1 rotates ~7°/pair), which is exactly when tracking is
// needed most.
static constexpr double OUTLIER_INITIAL_DEG = 12.0;
static constexpr double OUTLIER_FLOOR_DEG = 3.0;
static constexpr double OUTLIER_REL = 3.0;          // threshold = max(floor, REL*rms)
static constexpr int MAX_OUTLIER_ITERATIONS = 6;
// Cap on the number of decoded frames held in memory during visual rotation.
// Keeps a 2-minute clip (~3600 frames) in-budget while allowing dense sampling
// on short clips where fast 360° calibration motion lives.
static constexpr int MAX_DECODED_FRAMES = 1400;

// Instrumentation for the AutoSync cost breakdown.
static QAtomicInt g_detectCalls;
static QAtomicInt g_solveCalls;


// ---------------------------------------------------------------------------
// Proxy selection (see decodeFrames)
// ---------------------------------------------------------------------------
namespace {
struct StreamFacts { bool ok = false; qint64 frames = 0; double duration = 0.0; int w = 0, h = 0; };

StreamFacts probeStream(const QString &path)
{
    StreamFacts f;
    AVFormatContext *ctx = nullptr;
    if (avformat_open_input(&ctx, path.toUtf8().constData(), nullptr, nullptr) < 0) return f;
    if (avformat_find_stream_info(ctx, nullptr) >= 0) {
        const int idx = av_find_best_stream(ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        if (idx >= 0) {
            const AVStream *st = ctx->streams[idx];
            f.frames = st->nb_frames;
            f.duration = (st->duration != AV_NOPTS_VALUE)
                       ? st->duration * av_q2d(st->time_base)
                       : (double)ctx->duration / AV_TIME_BASE;
            f.w = st->codecpar->width; f.h = st->codecpar->height;
            f.ok = true;
        }
    }
    avformat_close_input(&ctx);
    return f;
}
} // namespace

QString VisualRotationComputer::chooseDecodeSource(const QString &videoPath)
{
    if (!qgetenv("RENDER360_NO_PROXY").isEmpty())
        return videoPath;

    const QFileInfo fi(videoPath);
    const QString proxy = fi.absolutePath() + QLatin1Char('/') + fi.completeBaseName()
                        + QLatin1String("_thm.") + fi.suffix();
    if (!QFileInfo::exists(proxy))
        return videoPath;

    const StreamFacts a = probeStream(videoPath);
    const StreamFacts b = probeStream(proxy);
    if (!a.ok || !b.ok) return videoPath;

    // Same frame count and (within one frame period) same duration, and the
    // proxy must still be at least as wide as the analysis resolution.
    const bool sameFrames = (a.frames > 0 && a.frames == b.frames);
    const bool sameDuration = std::abs(a.duration - b.duration) < 0.05;
    const bool bigEnough = b.w >= TARGET_WIDTH;
    if (sameFrames && sameDuration && bigEnough) {
        qInfo() << "VisualRotation: decoding proxy" << QFileInfo(proxy).fileName()
                << "(" << b.w << "x" << b.h << "," << b.frames << "frames) in place of the"
                << a.w << "x" << a.h << "original";
        return proxy;
    }
    qInfo() << "VisualRotation: proxy" << QFileInfo(proxy).fileName()
            << "does not match the original (frames" << b.frames << "vs" << a.frames
            << ", duration" << b.duration << "vs" << a.duration << ") -- decoding the original";
    return videoPath;
}

// ---------------------------------------------------------------------------
// Lens model inversion: pixel (normalized half-frame coords) → bearing vector
// ---------------------------------------------------------------------------
// Forward projection (from project.frag shader):
//   theta = acos(-ray.z)           // angle from -Z axis (forward)
//   phi   = atan2(ray.y, ray.x)    // azimuthal angle
//   r     = theta / (PI * 0.5)     // normalized radius [0,1]
//   r_dist = r * (1 + k1*r^2 + k2*r^4)
//   offset = rotate2D(cos(phi), sin(phi), rotation) * r_dist * radius
//   pixel  = center + offset
//
// For rear lens: theta_rear = PI - theta, r_rear = theta_rear / (PI*0.5)
// ---------------------------------------------------------------------------

QVector3D VisualRotationComputer::pixelToBearing(double px, double py, const LensParams &lens)
{
    // 1. Offset from center
    double dx = px - lens.cx;
    double dy = py - lens.cy;

    // 2. Apply hflip (inverse is the same as forward for a flip)
    if (lens.hflip)
        dx = -dx;

    // 3. Apply inverse rotation
    double rotRad = -lens.rotation * PI / 180.0;
    double c = cos(rotRad), s = sin(rotRad);
    double dx2 = dx * c - dy * s;
    double dy2 = dx * s + dy * c;

    // 4. Compute distorted normalized radius
    double rDist = sqrt(dx2 * dx2 + dy2 * dy2) / lens.radius;

    // 5. Compute azimuthal angle.
    // The rear lens looks along +Z while the front looks along -Z, so the two
    // images see the sphere from opposite sides. Carrying the same azimuth
    // convention across both therefore flips the handedness of the rear
    // bearings: the same physical rotation comes out with the opposite sign.
    // Since front and rear correspondences are pooled into ONE Kabsch solve,
    // the two halves then partially cancel and the solved rotation lands at
    // roughly half the truth — measured front-only 0.83-0.93 of the gyro,
    // rear-only 0.86-0.90, pooled 0.44-0.62, with front/rear correlations of
    // opposite sign (-0.67 vs +0.65 on JustPitch). Mirroring the rear azimuth
    // puts both halves in the same handedness.
    // Verified against the alternative: setting rearHFlip=true in the profile
    // does NOT fix this (it mirrors dx before the inverse rotation, which is a
    // different mapping) — it left the rear correlation at the opposite sign
    // and made YIVR_0830 worse, 0.62 -> 0.41. The azimuth itself is what has
    // to be mirrored.
    double phi = lens.mirrorAzimuth ? atan2(-dy2, dx2) : atan2(dy2, dx2);

    // 6. Undistort iteratively: find r such that r*(1 + k1*r^2 + k2*r^4) = rDist
    double r = rDist;  // initial guess
    for (int iter = 0; iter < 10; ++iter) {
        double r2 = r * r;
        double r4 = r2 * r2;
        double f = r * (1.0 + lens.k1 * r2 + lens.k2 * r4) - rDist;
        double df = 1.0 + 3.0 * lens.k1 * r2 + 5.0 * lens.k2 * r4;
        if (fabs(df) < 1e-12) break;
        r -= f / df;
        r = qBound(0.0, r, 1.0);
    }

    // 7. Compute theta
    double theta;
    if (lens.isRear) {
        double thetaRear = r * PI * 0.5;
        theta = PI - thetaRear;
    } else {
        theta = r * PI * 0.5;
    }

    // 8. Convert to bearing vector
    //    From shader: ray.x = sin(theta)*cos(phi), ray.y = sin(theta)*sin(phi), ray.z = -cos(theta)
    double sinTheta = sin(theta);
    double cosTheta = cos(theta);
    return QVector3D(sinTheta * cos(phi), sinTheta * sin(phi), -cosTheta);
}

// ---------------------------------------------------------------------------
// Sequential frame decoder
// ---------------------------------------------------------------------------

bool VisualRotationComputer::decodeFrames(const QString &videoPath, int frameSkip,
                                          QVector<FrameData> &frames,
                                          std::function<bool(double, const QString&)> progressCb)
{
    const double timeLimit = m_timeLimit;
    AVFormatContext *formatCtx = nullptr;
    AVCodecContext *codecCtx = nullptr;
    AVStream *videoStream = nullptr;
    int videoStreamIndex = -1;

    // Decode the camera's low-resolution PROXY when it exists and matches.
    //
    // The YI writes a *_thm.MP4 beside every clip: same encoder, same frame
    // count, same PTS, same GOP structure, at 720x1440 instead of 2880x5760.
    // Feature extraction here downscales to TARGET_WIDTH (640) regardless, so
    // decoding 16.6 MP per frame only to throw 95 % of it away was the single
    // largest cost of AutoSync -- measured 116 s of a 130 s run on YIVR_0845.
    // The proxy is accepted only if its frame count and duration match the
    // main file exactly; otherwise (or with RENDER360_NO_PROXY set) the main
    // file is decoded as before.
    const QString decodePath = chooseDecodeSource(videoPath);

    // Open video
    int ret = avformat_open_input(&formatCtx, decodePath.toUtf8().constData(), nullptr, nullptr);
    if (ret < 0) {
        qWarning() << "VisualRotation: failed to open video:" << decodePath;
        return false;
    }

    ret = avformat_find_stream_info(formatCtx, nullptr);
    if (ret < 0) {
        avformat_close_input(&formatCtx);
        return false;
    }

    videoStreamIndex = av_find_best_stream(formatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (videoStreamIndex < 0) {
        avformat_close_input(&formatCtx);
        return false;
    }

    videoStream = formatCtx->streams[videoStreamIndex];
    AVCodecParameters *codecpar = videoStream->codecpar;

    const AVCodec *codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        avformat_close_input(&formatCtx);
        return false;
    }

    codecCtx = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(codecCtx, codecpar);

    // Decode on every core. This is by far the largest cost in AutoSync: the
    // camera records 2880x5760 (16.6 MP per frame), and measured on a 13 s clip
    // single-threaded decode was 43.7 s of a 47 s run — 93% of the total.
    // thread_count = 0 lets libavcodec pick one thread per core.
    // (GPU decode is not an option here: both the Intel and NVIDIA H.264
    // decoders cap out at 4096x4096, and VAAPI rejects this resolution outright
    // with "Hardware does not support image size 2880x5760".)
    // Skip the in-loop deblocking filter. This is an ANALYSIS-ONLY decoder
    // (playback and export use videodecoder.cpp and are unaffected), and the
    // frames are downscaled 4.5x with INTER_AREA before ORB ever sees them,
    // which averages the block artifacts away. Measured: decode 12.9 s -> 11.2 s
    // with visual-vs-gyro agreement unchanged (mean ratio 0.914 -> 0.922, mean
    // |r| 0.676 -> 0.682 across JustPitch/JustRoll/YIVR_0830).
    codecCtx->skip_loop_filter = AVDISCARD_ALL;
    codecCtx->thread_count = 0;
    codecCtx->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;

    if (avcodec_open2(codecCtx, codec, nullptr) < 0) {
        avcodec_free_context(&codecCtx);
        avformat_close_input(&formatCtx);
        return false;
    }

    int srcWidth = codecpar->width;
    int srcHeight = codecpar->height;
    double duration = (double)formatCtx->duration / AV_TIME_BASE;

    // Compute target dimensions (downscale to TARGET_WIDTH)
    int dstWidth = TARGET_WIDTH;
    int dstHeight = (srcHeight * dstWidth) / srcWidth;
    // Ensure even dimensions for swscale
    dstWidth &= ~1;
    dstHeight &= ~1;

    int halfHeight = dstHeight / 2;

    // Motion-adaptive density (item 3): cap the number of decoded frames so
    // memory stays bounded on long clips while keeping every frame on short
    // clips (where fast motion concentrates). We still honor the caller's
    // frameSkip (they asked for at least that sparse), but never allow the
    // decoded count to blow past MAX_DECODED_FRAMES.
    int useFrameSkip = frameSkip;
    {
        double fps = 0.0;
        if (videoStream->avg_frame_rate.den > 0)
            fps = av_q2d(videoStream->avg_frame_rate);
        if (fps <= 0.0 && duration > 0.0 && videoStream->nb_frames > 0)
            fps = (double)videoStream->nb_frames / duration;
        if (fps <= 0.0)
            fps = 30.0;   // fallback
        const double window = (timeLimit > 0.0) ? std::min(timeLimit, duration) : duration;
        const double estFrames = fps * window;
        if (estFrames > 0.0) {
            int strideForBudget = (int)std::ceil(estFrames / MAX_DECODED_FRAMES);
            strideForBudget = std::max(0, strideForBudget - 1);
            useFrameSkip = std::max(useFrameSkip, strideForBudget);
        }
    }

    // Setup swscale context for YUV420P → BGR24 conversion + downscale
    SwsContext *swsCtx = sws_getContext(
        srcWidth, srcHeight, (AVPixelFormat)codecpar->format,
        dstWidth, dstHeight, AV_PIX_FMT_BGR24,
        SWS_BILINEAR, nullptr, nullptr, nullptr);

    if (!swsCtx) {
        avcodec_free_context(&codecCtx);
        avformat_close_input(&formatCtx);
        return false;
    }

    AVPacket *pkt = av_packet_alloc();
    AVFrame *frame = av_frame_alloc();
    AVFrame *bgrFrame = av_frame_alloc();

    int bgrSize = av_image_get_buffer_size(AV_PIX_FMT_BGR24, dstWidth, dstHeight, 1);
    uint8_t *bgrBuffer = (uint8_t *)av_malloc(bgrSize);
    av_image_fill_arrays(bgrFrame->data, bgrFrame->linesize, bgrBuffer,
                         AV_PIX_FMT_BGR24, dstWidth, dstHeight, 1);

    int frameCount = 0;
    int processedCount = 0;

    // Returns false if the caller asked to stop.
    auto processFrame = [&](AVFrame *f) -> bool {
        frameCount++;

        // Skip frames according to the adaptive frame skip. Every frame still
        // has to be DECODED (inter-frame prediction), but only the kept ones
        // pay for scaling.
        if (frameCount % (useFrameSkip + 1) != 0)
            return true;

        // Get timestamp
        double timestamp = 0.0;
        if (f->best_effort_timestamp != AV_NOPTS_VALUE)
            timestamp = f->best_effort_timestamp * av_q2d(videoStream->time_base);

        if (timeLimit > 0.0 && timestamp > timeLimit)
            return false;   // past the analysed window: stop decoding

        // For planar YUV — which is what the camera records (yuvj420p) — the
        // luma plane already IS the grayscale image, so the old YUV->BGR24
        // conversion plus a second BGR->GRAY pass was pure overhead: three
        // channels built and two of them immediately thrown away. Resizing the
        // Y plane directly also lets OpenCV's SIMD/threaded resize do the
        // 4.5x downscale, and INTER_AREA area-averages rather than point-
        // sampling, so it aliases less than the previous SWS_BILINEAR.
        cv::Mat grayMat;
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get((AVPixelFormat)f->format);
        const bool planarLuma8 = desc
                              && (desc->flags & AV_PIX_FMT_FLAG_PLANAR)
                              && !(desc->flags & AV_PIX_FMT_FLAG_RGB)
                              && desc->comp[0].depth == 8
                              && f->data[0] != nullptr;
        if (planarLuma8) {
            cv::Mat yPlane(f->height, f->width, CV_8UC1, f->data[0], f->linesize[0]);
            cv::resize(yPlane, grayMat, cv::Size(dstWidth, dstHeight), 0, 0, cv::INTER_AREA);
        } else {
            // Fallback for anything unusual (paletted, RGB, >8-bit).
            sws_scale(swsCtx, f->data, f->linesize, 0, srcHeight,
                      bgrFrame->data, bgrFrame->linesize);
            cv::Mat bgrMat(dstHeight, dstWidth, CV_8UC3, bgrFrame->data[0]);
            cv::cvtColor(bgrMat, grayMat, cv::COLOR_BGR2GRAY);
        }

        // Split into front (top half) and rear (bottom half)
        FrameData fd;
        fd.timestamp = timestamp;
        fd.grayFront = grayMat(cv::Rect(0, 0, dstWidth, halfHeight)).clone();
        fd.grayRear = grayMat(cv::Rect(0, halfHeight, dstWidth, halfHeight)).clone();

        frames.append(fd);
        processedCount++;

        // Report progress (decoding is ~50% of total work)
        double fraction = (duration > 0) ? (timestamp / duration * 0.5) : 0.0;
        if (progressCb && !progressCb(fraction, QStringLiteral("Decoding frames...")))
            return false;
        return true;
    };

    bool aborted = false;
    while (!aborted && av_read_frame(formatCtx, pkt) >= 0) {
        if (pkt->stream_index != videoStreamIndex) {
            av_packet_unref(pkt);
            continue;
        }

        ret = avcodec_send_packet(codecCtx, pkt);
        av_packet_unref(pkt);
        if (ret < 0) continue;

        while (avcodec_receive_frame(codecCtx, frame) == 0) {
            const bool keepGoing = processFrame(frame);
            av_frame_unref(frame);
            if (!keepGoing) { aborted = true; break; }
        }
    }

    // Drain the decoder. REQUIRED now that frame threading is on: with
    // FF_THREAD_FRAME libavcodec holds several frames in flight, so without
    // this flush the tail of every clip would silently go missing.
    if (!aborted) {
        avcodec_send_packet(codecCtx, nullptr);
        while (avcodec_receive_frame(codecCtx, frame) == 0) {
            const bool keepGoing = processFrame(frame);
            av_frame_unref(frame);
            if (!keepGoing) break;
        }
    }

    // Cleanup
    av_free(bgrBuffer);
    av_frame_free(&bgrFrame);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    sws_freeContext(swsCtx);
    avcodec_free_context(&codecCtx);
    avformat_close_input(&formatCtx);

    qDebug() << "VisualRotation: decoded" << processedCount << "frames from" << frameCount << "total";
    return processedCount >= 2;
}

// ---------------------------------------------------------------------------
// Feature detection & matching
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// computeFeatures — ORB over every frame, spread across all cores
//
// Embarrassingly parallel (each frame is independent), so this turns the one
// saturated core the user was seeing into all of them. cv::parallel_for_ uses
// OpenCV's existing thread pool, and each worker owns its own ORB instance
// because cv::ORB is not documented as thread-safe for concurrent detect calls.
// ---------------------------------------------------------------------------
// Valid-image mask for one fisheye half: pixels whose distorted radius is
// inside the lens circle. ORB was detecting on the whole square, so up to 21 %
// of its 800 features landed in the black corners outside the image circle;
// those then went through the undistort's qBound(0, r, 1) clamp, which maps
// every out-of-circle pixel onto the rim, and produced correspondences that
// were geometrically meaningless. The mask uses the same normalisation as
// pixelToBearing so the two agree exactly.
static cv::Mat lensMaskFor(const VisualRotationComputer::LensParams &lens, int w, int h)
{
    cv::Mat m(h, w, CV_8UC1, cv::Scalar(0));
    const double rotRad = -lens.rotation * PI / 180.0;
    const double c = cos(rotRad), sn = sin(rotRad);
    for (int y = 0; y < h; y++) {
        uchar *row = m.ptr<uchar>(y);
        const double py = (y + 0.5) / h;
        for (int x = 0; x < w; x++) {
            const double px = (x + 0.5) / w;
            double dx = px - lens.cx, dy = py - lens.cy;
            if (lens.hflip) dx = -dx;
            const double dx2 = dx * c - dy * sn, dy2 = dx * sn + dy * c;
            const double rDist = sqrt(dx2 * dx2 + dy2 * dy2) / lens.radius;
            row[x] = (rDist <= 1.0) ? 255 : 0;
        }
    }
    return m;
}

void VisualRotationComputer::computeFeatures(const QVector<FrameData> &frames,
                                             QVector<FrameFeatures> &out,
                                             int lensMask,
                                             const LensParams &frontLens,
                                             const LensParams &rearLens,
                                             const std::function<void(int)> &progressCb)
{
    const bool useFront = (lensMask & LensFront) != 0;
    const bool useRear  = (lensMask & LensRear) != 0;
    const int n = frames.size();
    out.resize(n);
    if (n == 0) return;

    const cv::Mat maskFront = lensMaskFor(frontLens, frames[0].grayFront.cols, frames[0].grayFront.rows);
    const cv::Mat maskRear  = lensMaskFor(rearLens,  frames[0].grayRear.cols,  frames[0].grayRear.rows);

    QAtomicInt done(0);
    // Chunked so progress can be reported from THIS thread; emitting Qt signals
    // from inside the OpenCV worker pool is not worth the risk.
    const int chunk = std::max(1, n / 16);
    for (int base = 0; base < n; base += chunk) {
        const int end = std::min(n, base + chunk);
        cv::parallel_for_(cv::Range(base, end), [&](const cv::Range &r) {
            auto orb = cv::ORB::create(ORB_FEATURES);
            for (int i = r.start; i < r.end; i++) {
                if (useFront) {
                    orb->detectAndCompute(frames[i].grayFront, maskFront,
                                          out[i].kpFront, out[i].descFront);
                    g_detectCalls.fetchAndAddRelaxed(1);
                }
                if (useRear) {
                    orb->detectAndCompute(frames[i].grayRear, maskRear,
                                          out[i].kpRear, out[i].descRear);
                    g_detectCalls.fetchAndAddRelaxed(1);
                }
                done.fetchAndAddRelaxed(1);
            }
        });
        if (progressCb)
            progressCb(done.loadRelaxed());
    }
}

bool VisualRotationComputer::matchFeatures(const FrameData &frameA, const FrameData &frameB,
                                           const FrameFeatures &featA, const FrameFeatures &featB,
                                           const LensParams &frontLens, const LensParams &rearLens,
                                           QVector<QVector3D> &bearingsA,
                                           QVector<QVector3D> &bearingsB,
                                           int lensMask)
{
    const bool useFront = (lensMask & LensFront) != 0;
    const bool useRear  = (lensMask & LensRear) != 0;

    // Features arrive precomputed (see computeFeatures): detecting here re-ran
    // ORB on frames that had already been processed as the other half of an
    // adjacent pair, and again on every narrowed-hop retry.
    const std::vector<cv::KeyPoint> &kpFrontA = featA.kpFront, &kpFrontB = featB.kpFront;
    const cv::Mat &descFrontA = featA.descFront, &descFrontB = featB.descFront;
    const std::vector<cv::KeyPoint> &kpRearA = featA.kpRear, &kpRearB = featB.kpRear;
    const cv::Mat &descRearA = featA.descRear, &descRearB = featB.descRear;

    // Match front features
    std::vector<cv::DMatch> frontMatches;
    if (useFront && !descFrontA.empty() && !descFrontB.empty()) {
        // Cross-check must be OFF when using knnMatch with k=2: OpenCV's
        // crossCheck mode builds a batchDistance with K==1, which asserts when
        // given k=2. The ratio test below provides the robustness instead.
        cv::BFMatcher matcher(cv::NORM_HAMMING, /*crossCheck=*/false);
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher.knnMatch(descFrontA, descFrontB, knnMatches, 2);

        // Ratio test
        for (const auto &knn : knnMatches) {
            if (knn.size() == 2 && knn[0].distance < RATIO_THRESHOLD * knn[1].distance) {
                frontMatches.push_back(knn[0]);
            }
        }
    }

    // Match rear features
    std::vector<cv::DMatch> rearMatches;
    if (useRear && !descRearA.empty() && !descRearB.empty()) {
        cv::BFMatcher matcher(cv::NORM_HAMMING, false);
        std::vector<std::vector<cv::DMatch>> knnMatches;
        matcher.knnMatch(descRearA, descRearB, knnMatches, 2);

        for (const auto &knn : knnMatches) {
            if (knn.size() == 2 && knn[0].distance < RATIO_THRESHOLD * knn[1].distance) {
                rearMatches.push_back(knn[0]);
            }
        }
    }

    // Convert matches to bearing vectors
    int halfHeightA = frameA.grayFront.rows;
    int halfWidthA = frameA.grayFront.cols;
    int halfHeightB = frameB.grayFront.rows;
    int halfWidthB = frameB.grayFront.cols;

    // Front matches
    for (const auto &m : frontMatches) {
        const auto &ptA = kpFrontA[m.queryIdx].pt;
        const auto &ptB = kpFrontB[m.trainIdx].pt;

        // Convert pixel coords to normalized half-frame coords
        double npxA = ptA.x / halfWidthA;
        double npyA = ptA.y / halfHeightA;
        double npxB = ptB.x / halfWidthB;
        double npyB = ptB.y / halfHeightB;

        bearingsA.append(pixelToBearing(npxA, npyA, frontLens));
        bearingsB.append(pixelToBearing(npxB, npyB, frontLens));
    }

    // Rear matches
    for (const auto &m : rearMatches) {
        const auto &ptA = kpRearA[m.queryIdx].pt;
        const auto &ptB = kpRearB[m.trainIdx].pt;

        double npxA = ptA.x / halfWidthA;
        double npyA = ptA.y / halfHeightA;
        double npxB = ptB.x / halfWidthB;
        double npyB = ptB.y / halfHeightB;

        bearingsA.append(pixelToBearing(npxA, npyA, rearLens));
        bearingsB.append(pixelToBearing(npxB, npyB, rearLens));
    }

    return (bearingsA.size() >= 6);  // minimum for robust solve
}

// ---------------------------------------------------------------------------
// Rotation solve (Wahba/Kabsch with iterative outlier rejection)
// ---------------------------------------------------------------------------

QQuaternion VisualRotationComputer::solveRotation(const QVector<QVector3D> &bearingsA,
                                                  const QVector<QVector3D> &bearingsB,
                                                  int &inliers, double &rmsDeg,
                                                  const QQuaternion &initialGuess)
{
    int n = bearingsA.size();
    if (n < 3) {
        inliers = 0;
        rmsDeg = 999.0;
        return QQuaternion();
    }

    // Iterative outlier rejection. Seed the first iteration from the previous
    // pair's rotation (or identity) so that sustained fast rotation keeps most
    // features within the outlier threshold — starting from identity collapses
    // to zero inliers as soon as the true per-pair rotation exceeds the
    // threshold.
    std::vector<bool> inlierMask(n, true);
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    {
        QQuaternion seed = initialGuess.isNull() ? QQuaternion(1, 0, 0, 0) : initialGuess;
        seed.normalize();
        // Quaternion -> 3x3 rotation matrix
        double w = seed.scalar(), a = seed.x(), b = seed.y(), c = seed.z();
        R.at<double>(0,0) = 1 - 2*(b*b + c*c);
        R.at<double>(0,1) = 2*(a*b - c*w);
        R.at<double>(0,2) = 2*(a*c + b*w);
        R.at<double>(1,0) = 2*(a*b + c*w);
        R.at<double>(1,1) = 1 - 2*(a*a + c*c);
        R.at<double>(1,2) = 2*(b*c - a*w);
        R.at<double>(2,0) = 2*(a*c - b*w);
        R.at<double>(2,1) = 2*(b*c + a*w);
        R.at<double>(2,2) = 1 - 2*(a*a + b*b);
    }

    // Adaptive outlier rejection. Start with a generous threshold large enough
    // to survive the true per-pair rotation (so fast sustained motion stays
    // trackable even when the motion-continuity seed is imperfect), then
    // tighten toward a floor relative to the running RMS each iteration. Keeps
    // the good features while dropping genuine mismatches without the whole
    // solve collapsing to zero inliers.
    //
    // All per-correspondence arithmetic is scalar. The previous version built
    // two heap cv::Mat_<double>(3,1) per correspondence per iteration and ran a
    // full gemm dispatch for each 3x3 outer product -- with ~800 correspondences
    // x 6 iterations x 3 loops x ~900 solves that was the dominant cost of the
    // match+solve stage. Only the 3x3 SVD still goes through OpenCV.
    double Rm[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) Rm[r][c] = R.at<double>(r, c);

    auto residualAngle = [&](const double M[3][3], const QVector3D &p, const QVector3D &q) {
        const double rx = M[0][0]*p.x() + M[0][1]*p.y() + M[0][2]*p.z();
        const double ry = M[1][0]*p.x() + M[1][1]*p.y() + M[1][2]*p.z();
        const double rz = M[2][0]*p.x() + M[2][1]*p.y() + M[2][2]*p.z();
        const double dot = qBound(-1.0, rx*q.x() + ry*q.y() + rz*q.z(), 1.0);
        return std::acos(dot);
    };

    {
        double outlierThreshRad = OUTLIER_INITIAL_DEG * PI / 180.0;
        std::vector<bool> bestMask;
        int bestInliers = 0;
        double bestRms = 999.0;
        double bestR[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

        cv::Mat H(3, 3, CV_64F), U, S, Vt;
        for (int iter = 0; iter < MAX_OUTLIER_ITERATIONS; ++iter) {
            // H = sum(q_i * p_i^T) over inliers.
            double h[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
            int inlierCount = 0;
            for (int i = 0; i < n; ++i) {
                if (!inlierMask[i]) continue;
                const auto &p = bearingsA[i];
                const auto &q = bearingsB[i];
                const double pv[3] = {p.x(), p.y(), p.z()};
                const double qv[3] = {q.x(), q.y(), q.z()};
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                        h[r][c] += qv[r] * pv[c];
                inlierCount++;
            }
            if (inlierCount < 3) break;

            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) H.at<double>(r, c) = h[r][c];
            cv::SVD::compute(H, S, U, Vt);
            cv::Mat Rcv = Vt.t() * U.t();
            if (cv::determinant(Rcv) < 0) {
                cv::Mat V = Vt.t();
                V.col(2) *= -1;
                Rcv = V * U.t();
            }
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) Rm[r][c] = Rcv.at<double>(r, c);

            // Residuals + candidate keep-set under the current threshold.
            std::vector<bool> newMask(n, false);
            double sumSq = 0.0;
            int keepCount = 0;
            for (int i = 0; i < n; ++i) {
                const double angle = residualAngle(Rm, bearingsA[i], bearingsB[i]);
                if (angle <= outlierThreshRad) {
                    newMask[i] = true;
                    sumSq += angle * angle;
                    keepCount++;
                }
            }

            double curRms = (keepCount > 0) ? sqrt(sumSq / keepCount) * 180.0 / PI : 999.0;

            // Track the best kept subset found so far (more inliers wins;
            // tie-break on lower rms).
            if (keepCount >= 3 && keepCount >= bestInliers) {
                if (keepCount > bestInliers || curRms < bestRms) {
                    bestInliers = keepCount;
                    bestRms = curRms;
                    std::memcpy(bestR, Rm, sizeof(Rm));
                    bestMask = newMask;
                }
            }

            inlierMask = newMask;

            // Tighten toward the residual scale, floored so genuine fast motion
            // (> floor) is never wholly rejected.
            double rmsRad = (keepCount > 0) ? sqrt(sumSq / keepCount) : 0.0;
            double nextThresh = std::max(OUTLIER_FLOOR_DEG * PI / 180.0,
                                         OUTLIER_REL * rmsRad);
            if (nextThresh >= outlierThreshRad * 0.999)
                break;  // no further meaningful tightening
            outlierThreshRad = nextThresh;
        }

        // Switch back to the best subset (a single bad tighten can otherwise
        // drop more features than intended).
        if (bestInliers >= 3) {
            std::memcpy(Rm, bestR, sizeof(Rm));
            inlierMask = bestMask;
        }
    }

    // Final residual over the retained inliers.
    {
        double sumSqFinal = 0.0;
        int finalInliers = 0;
        for (int i = 0; i < n; ++i) {
            if (!inlierMask[i]) continue;
            const double angle = residualAngle(Rm, bearingsA[i], bearingsB[i]);
            sumSqFinal += angle * angle;
            finalInliers++;
        }
        inliers = finalInliers;
        rmsDeg = (finalInliers > 0) ? sqrt(sumSqFinal / finalInliers) * 180.0 / PI : 999.0;
    }
    for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) R.at<double>(r, c) = Rm[r][c];

    // Convert rotation matrix to quaternion
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double w, x, y, z;

    if (trace > 0.0) {
        double s = 0.5 / sqrt(trace + 1.0);
        w = 0.25 / s;
        x = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
        y = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
        z = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
    } else {
        if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
            w = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
            x = 0.25 * s;
            y = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
            z = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
            double s = 2.0 * sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
            w = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
            x = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
            y = 0.25 * s;
            z = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        } else {
            double s = 2.0 * sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
            w = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
            x = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
            y = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
            z = 0.25 * s;
        }
    }

    QQuaternion result(w, x, y, z);
    return result;
}

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------

VisualRotationComputer::VisualRotationComputer(QObject *parent)
    : QObject(parent)
{
}

VisualRotationComputer::~VisualRotationComputer()
{
}

// ---------------------------------------------------------------------------
// Main compute() method — runs in background thread
// ---------------------------------------------------------------------------

void VisualRotationComputer::compute(const QString &videoPath,
                                     const CalibrationProfile *calibration,
                                     int frameSkip)
{
    if (!calibration) {
        emit computationFailed(QStringLiteral("Calibration profile is null"));
        return;
    }

    // Capture calibration parameters by value
    LensParams frontLens;
    frontLens.cx = calibration->frontCenterX();
    frontLens.cy = calibration->frontCenterY();
    frontLens.radius = calibration->frontRadius();
    frontLens.k1 = calibration->frontK1();
    frontLens.k2 = calibration->frontK2();
    frontLens.rotation = calibration->frontRotation();
    frontLens.hflip = calibration->frontHFlip();
    frontLens.isRear = false;

    LensParams rearLens;
    rearLens.cx = calibration->rearCenterX();
    rearLens.cy = calibration->rearCenterY();
    rearLens.radius = calibration->rearRadius();
    rearLens.k1 = calibration->rearK1();
    rearLens.k2 = calibration->rearK2();
    rearLens.rotation = calibration->rearRotation();
    rearLens.hflip = calibration->rearHFlip();
    rearLens.isRear = true;

    // Snapshot the mask so the worker is not reading a member that the GUI
    // thread could change mid-run.
    const int lensMask = m_lensMask;

    // Launch background thread
    QThread *thread = QThread::create([=, this]() {
        try {
            // 1. Decode frames
            QVector<FrameData> frames;
            auto progressCb = [this](double fraction, const QString &status) -> bool {
                emit progressChanged(fraction, status);
                return true;  // continue
            };

            emit progressChanged(0.0, QStringLiteral("Opening video..."));

            QElapsedTimer tDecode; tDecode.start();
            if (!decodeFrames(videoPath, frameSkip, frames, progressCb)) {
                emit computationFailed(QStringLiteral("Failed to decode frames from video"));
                return;
            }

            if (frames.size() < 2) {
                emit computationFailed(QStringLiteral("Not enough frames decoded"));
                return;
            }

            const qint64 msDecode = tDecode.elapsed();
            QElapsedTimer tMatch; tMatch.start();
            // Detect ORB on every frame up front, in parallel, instead of
            // inside the hop search where each frame was re-detected.
            QVector<FrameFeatures> feats;
            computeFeatures(frames, feats, lensMask, frontLens, rearLens, [&](int nDone) {
                emit progressChanged(0.5 + 0.15 * (double)nDone / qMax(1, frames.size()),
                                     QStringLiteral("Detecting features..."));
            });

            const qint64 msFeatures = tMatch.elapsed();
            emit progressChanged(0.65, QStringLiteral("Matching features..."));

            // 2. Match features and solve rotations with motion-adaptive density.
            //    Instead of a fixed consecutive-pair stride (which drops fast
            //    sections when the per-pair rotation exceeds the solver's
            //    trackable range), advance greedily: try a hop k (decoded-frame
            //    index delta, capped by frameSkip+1), and if the solved
            //    rotation is unreliable (too few inliers / high residual, i.e.
            //    the hop spans too much motion), shrink k towards 1 and retry.
            //    Net effect: still regions are sampled coarsely (few, large
            //    hops) and fast regions densely (small hops) — motion-adaptive
            //    density with no memory cost beyond the decoded frames.
            const int maxHop = frameSkip + 1;   // accepted wide hop (DEFAULT_FRAME_STRIDE)
            constexpr int minHop = 1;
            // Quality bar for accepting a hop: enough inliers and not an absurd
            // residual (a reliable rotation).
            constexpr int HOP_MIN_INLIERS = 3;
            constexpr double HOP_MAX_RMS_DEG = 30.0;

            // The greedy walk over one contiguous frame range [lo, hi].
            // Progress from inside the parallel walk: frames advanced across all
            // segments, emitted every few frames (Qt signals are thread-safe;
            // the receiver is queued). Without this the bar sat at the end of
            // feature detection until the whole stage finished.
            QAtomicInt framesDone(0);
            const int nFramesTotal = frames.size();
            auto walk = [&](int lo, int hi, QVector<VisualRotationPair> &outPairs) {
                QQuaternion prevRotation;
                int i = lo;
                while (i < hi) {
                    const int before = i;
                    int k = maxHop;
                    QQuaternion bestD; bool bestValid = false;
                    int bestInl = 0; double bestRms = 999.0;
                    int bestK = 1;

                    while (k >= minHop && i + k <= hi) {
                        QVector<QVector3D> bA, bB;
                        if (!matchFeatures(frames[i], frames[i + k], feats[i], feats[i + k],
                                           frontLens, rearLens, bA, bB, lensMask)) {
                            k = k / 2;
                            if (k < minHop) k = minHop;
                            if (i + k > hi) break;
                            continue;
                        }

                        int inl = 0; double rms = 0.0;
                        g_solveCalls.fetchAndAddRelaxed(1);
                        QQuaternion dR = solveRotation(bA, bB, inl, rms, prevRotation);

                        if (inl >= HOP_MIN_INLIERS && rms <= HOP_MAX_RMS_DEG) {
                            bestValid = true;
                            bestD = dR; bestInl = inl; bestRms = rms; bestK = k;
                            break;  // this hop is reliable — accept it
                        }
                        if (inl > bestInl && inl >= 3) {
                            bestValid = true;
                            bestD = dR; bestInl = inl; bestRms = rms; bestK = k;
                        }
                        if (k <= 1)
                            break;
                        k = (k + 1) / 2;   // halve toward the nearest frame
                    }

                    if (bestValid) {
                        VisualRotationPair pair;
                        pair.t0 = frames[i].timestamp;
                        pair.t1 = frames[i + bestK].timestamp;
                        pair.deltaR = bestD;
                        pair.inliers = bestInl;
                        pair.rmsDeg = bestRms;
                        outPairs.append(pair);
                        prevRotation = bestD;
                    }
                    i += (bestValid && bestK >= 1) ? bestK : 1;  // always advance
                    const int done = framesDone.fetchAndAddRelaxed(i - before) + (i - before);
                    if ((done & 15) < (i - before))   // roughly every 16 frames
                        emit progressChanged(0.65 + 0.35 * (double)done / qMax(1, nFramesTotal),
                                             QStringLiteral("Matching %1/%2").arg(done).arg(nFramesTotal));
                }
            };

            // 2. Match features and solve rotations with motion-adaptive density
            //    (greedy hop walk, see walk()). The walk is sequential within a
            //    range but has no state that matters across a range boundary
            //    (prevRotation is only a solver seed), so split the frames into
            //    one contiguous segment per core and walk them concurrently.
            //    Segments share their boundary frame, so the only cost is that
            //    a hop cannot straddle a boundary -- at most one pair per split
            //    is shorter than it would otherwise have been. Pairs come out
            //    ordered because the segments are concatenated in order.
            QVector<VisualRotationPair> pairs;
            {
                const int nFrames = frames.size();
                const int nSeg = qBound(1, qMin(cv::getNumThreads(), nFrames / 40), 16);
                QVector<QVector<VisualRotationPair>> segPairs(nSeg);
                QAtomicInt segsDone(0);
                cv::parallel_for_(cv::Range(0, nSeg), [&](const cv::Range &r) {
                    for (int sIdx = r.start; sIdx < r.end; sIdx++) {
                        const int lo = (int)((qint64)(nFrames - 1) * sIdx / nSeg);
                        const int hi = (int)((qint64)(nFrames - 1) * (sIdx + 1) / nSeg);
                        walk(lo, hi, segPairs[sIdx]);
                        segsDone.fetchAndAddRelaxed(1);
                    }
                }, nSeg);
                for (const auto &sp : segPairs) pairs += sp;
                emit progressChanged(0.99, QStringLiteral("Processing frame %1/%2")
                                     .arg(nFrames).arg(nFrames));
            }

            qInfo("VisualRotation timing: decode %lld ms (%d frames), features %lld ms, "
                  "match+solve %lld ms, %d detectAndCompute calls, %d solves, %d pairs",
                  msDecode, (int)frames.size(), msFeatures, tMatch.elapsed() - msFeatures,
                  g_detectCalls.loadRelaxed(), g_solveCalls.loadRelaxed(), (int)pairs.size());
            emit progressChanged(1.0, QStringLiteral("Complete"));
            emit rotationComputed(pairs);

        } catch (const std::exception &e) {
            emit computationFailed(QString::fromUtf8(e.what()));
        } catch (...) {
            emit computationFailed(QStringLiteral("Unknown error during computation"));
        }
    });

    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    thread->start();
}
