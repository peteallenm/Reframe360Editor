// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include "objecttracker.h"
#include "avinput.h"
#include "projection.h"
#include "visualrotation.h"

#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QThread>
#include <QtMath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

#include <algorithm>
#include <cmath>

namespace {

// --- tuning ---------------------------------------------------------------
// Provisional: set from first principles and sanity-checked on synthetic
// footage. They want re-fitting from measured distributions on real clips
// (tracking_tests --track-stats) before anyone calls them final.
constexpr int    kTemplate          = 48;    // template edge, px
constexpr double kScaleStep         = 1.09;  // the +/- scale probed each frame
// Score floors, measured rather than guessed: on this camera's footage a
// healthy track sits around 0.5-0.7, so the old 0.45 soft floor was sitting
// right on top of normal operation and ending tracks that were working.
constexpr double kScoreHardFloor    = 0.18;  // this bad for kHardRun frames
constexpr int    kHardRun           = 3;     // ... so a burst of motion blur
                                             // cannot end a good track
constexpr double kScoreSoftFloor    = 0.30;  // this bad for kBadRun frames ends it
constexpr int    kBadRun            = 8;
constexpr int    kRejectRun         = 15;    // gated frames before giving up:
                                             // coast through a bad patch, since
                                             // the prediction is usually right
// How fast the running template adopts the subject's current appearance.
constexpr double kRunningBlend      = 0.08;
constexpr double kRunningUpdateMin  = 0.45;  // only learn from a good match
// Re-acquisition. A subject that turns, or is briefly occluded, stops matching
// the patch that was picked while the tracker is still pointing at it; taking
// a fresh template there is what carries a track through that.
constexpr int    kMaxReseeds        = 12;
constexpr double kReseedMinGap      = 0.30;  // seconds between re-seeds
constexpr double kRevivalScore      = 0.50;  // a retried patch must match this well
constexpr double kMinValidFraction  = 0.80;  // of the tile inside a lens circle
constexpr double kMinSeedStdDev     = 8.0;   // grey levels; below this, refuse
// ... and the same sanity applied every frame, not only at seed time. A patch
// with no structure in it cannot be the subject that was picked -- it is sky,
// or a blown highlight. It has to be caught explicitly because it does not
// look like failure: featureless matches featureless at 0.9+, and once the
// running template has drifted into the sky the tracker reports a rock-solid
// lock while sitting on nothing. Slightly below the seed threshold, so a
// subject that merely loses contrast is not thrown away.
constexpr double kMinTrackStdDev    = 5.0;
// Variance is not enough. A smooth gradient -- sky, a blurred wall, water --
// has plenty of it and still correlates just as well ANYWHERE along the
// gradient, so the score stays high while the track slides (the aperture
// problem). What matters is whether the patch can be LOCALISED: its
// correlation peak must stand clear of every rival in the tile.
constexpr double kMinSeedPsr        = 4.0;   // peak-to-sidelobe, at seed time
constexpr double kMinRunPsr         = 2.0;   // ... and while running
constexpr int    kAmbiguousRun      = 12;    // consecutive ambiguous frames
constexpr double kAnchorStillGood   = 0.45;  // anchor match that vetoes an
                                             // ambiguity loss outright
// A sanity gate against teleports, not a speed limit on the subject. Measured
// world-frame motion on real handheld footage runs to 30 deg/s median with a
// p95 near 110, so 40 was rejecting ordinary frames.
// A sanity ceiling, NOT the teleport guard it was documented as. The gate is
// applied to the whole step, and the step is the predicted motion plus a
// residual the search window already bounds -- so this really only caps how
// fast the tracker is willing to believe a subject moves, and 150 capped it
// near 60 deg/s. That truncated ordinary handheld tracks (0830 died at 3.9 s)
// and made a bullet-time orbit, where the subject sweeps 250-350 deg/s,
// impossible: every frame was rejected and the tracker settled on background.
constexpr double kMaxWorldSpeedDeg  = 400.0;
constexpr int    kMaxTileSize       = 240;   // bounds the per-frame search cost
constexpr double kMaxScaleStepLog   = 0.223; // log(1.25) per frame
constexpr double kSeamThetaDeg      = 90.0;
constexpr double kSeamHysteresisDeg = 4.0;

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// How far the winning match stands above the rest of the score map, in
// standard deviations (the peak-to-sidelobe ratio). A confident lock scores
// well above 4; a patch that matches everywhere equally scores near zero.
//
// This replaces a "best rival peak" test, which was measuring the wrong thing:
// with a 48 px template the excluded neighbourhood was only 16 px, so a
// "rival" 17 px away still shared two thirds of its pixels with the true
// match and scored almost as highly. That test therefore fired on a PERFECT
// lock -- the user's 0.897-mean track was ended by it -- while a genuinely
// ambiguous patch scores badly on this one whatever the peak height.
double peakSidelobeRatio(const cv::Mat &scoreMap, const cv::Point &best, int exclude)
{
    double sum = 0.0, sumSq = 0.0;
    int n = 0;
    for (int y = 0; y < scoreMap.rows; ++y) {
        const float *row = scoreMap.ptr<float>(y);
        for (int x = 0; x < scoreMap.cols; ++x) {
            if (std::abs(x - best.x) <= exclude && std::abs(y - best.y) <= exclude)
                continue;
            sum += row[x];
            sumSq += (double)row[x] * row[x];
            ++n;
        }
    }
    if (n < 16) return 99.0;                    // too small to judge; do not gate
    const double mean = sum / n;
    const double var = qMax(1e-9, sumSq / n - mean * mean);
    return (scoreMap.at<float>(best) - mean) / std::sqrt(var);
}

// Bilinear sample of a luma plane, GL texel centres.
double sampleY(const uchar *Y, int w, int h, int stride, double u, double v)
{
    const double x = clampd(u * w - 0.5, 0.0, w - 1.0);
    const double y = clampd(v * h - 0.5, 0.0, h - 1.0);
    const int x0 = (int)x, y0 = (int)y;
    const int x1 = std::min(x0 + 1, w - 1), y1 = std::min(y0 + 1, h - 1);
    const double fx = x - x0, fy = y - y0;
    const double a = Y[y0 * stride + x0], b = Y[y0 * stride + x1];
    const double c = Y[y1 * stride + x0], d = Y[y1 * stride + x1];
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy;
}

QVector3D toQ(const proj::Vec3 &v) { return QVector3D((float)v.x, (float)v.y, (float)v.z); }
proj::Vec3 toV(const QVector3D &v) { return proj::Vec3{v.x(), v.y(), v.z()}; }

// A gnomonic tile in the INERTIAL frame around `fwd`, sampled out of the
// stacked dual-fisheye luma. World-up anchored, so camera roll never rotates
// the patch.
bool buildTile(const uchar *Y, int w, int h, int stride,
               const TrackLenses &lenses, const QQuaternion &qAct,
               const QVector3D &fwd, double pTan, int S, bool preferRear,
               cv::Mat &out, double *validFraction)
{
    QVector3D up(0.0f, 1.0f, 0.0f);
    if (std::fabs(QVector3D::dotProduct(fwd, up)) > 0.996f)
        up = QVector3D(0.0f, 0.0f, 1.0f);          // near the poles
    const QVector3D right = QVector3D::crossProduct(up, fwd).normalized();
    const QVector3D tileUp = QVector3D::crossProduct(fwd, right).normalized();

    out.create(S, S, CV_8U);
    // Validity needs its own mask. Marking invalid pixels by writing 0 into
    // the tile treats every genuinely BLACK pixel as missing and replaces it
    // with the tile mean -- which dissolves a dark subject (black trousers
    // against a light background) into a flat blob that matches nothing.
    cv::Mat valid8(S, S, CV_8U, cv::Scalar(0));
    const QQuaternion qInv = qAct.conjugated();
    const double half = S * 0.5;
    int valid = 0;

    // Lens chosen PER PIXEL, by which one actually sees that direction -- not
    // per tile. A tile centred on the seam is covered by neither lens alone,
    // and treating that as "left the lens" ended tracks exactly when a subject
    // crossed between the two. Each pixel still comes from exactly one lens,
    // so there is no blending and no parallax ghosting; the patch's appearance
    // does shift as it crosses, which the running template absorbs.
    const proj::LensGeom geomFront = proj::LensGeom::make(
        lenses.front.cx, lenses.front.cy, lenses.front.radius,
        lenses.front.k1, lenses.front.k2, lenses.front.rotation, lenses.front.hflip);
    const proj::LensGeom geomRear = proj::LensGeom::make(
        lenses.rear.cx, lenses.rear.cy, lenses.rear.radius,
        lenses.rear.k1, lenses.rear.k2, lenses.rear.rotation, lenses.rear.hflip);
    Q_UNUSED(preferRear);
    double sum = 0.0;

    for (int j = 0; j < S; ++j) {
        uchar *row = out.ptr<uchar>(j);
        uchar *vrow = valid8.ptr<uchar>(j);
        const double ty = -((j + 0.5) - half) * pTan;      // row 0 is the top
        for (int i = 0; i < S; ++i) {
            const double tx = ((i + 0.5) - half) * pTan;
            const QVector3D rayWorld = (fwd + right * (float)tx + tileUp * (float)ty).normalized();
            const QVector3D rayCam = qInv.rotatedVector(rayWorld);
            double theta, phi;
            proj::dirToThetaPhi(toV(rayCam), theta, phi);
            const bool front = theta <= M_PI * 0.5;
            double u, v;
            proj::lensUv(front, theta, phi, front ? geomFront : geomRear, u, v);
            if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) continue;   // mask stays 0
            const double val = sampleY(Y, w, h, stride, u, proj::stackedV(front, v));
            row[i] = (uchar)clampd(val, 0.0, 255.0);
            vrow[i] = 1;
            sum += val;
            ++valid;
        }
    }

    const double frac = (double)valid / (double)(S * S);
    if (validFraction) *validFraction = frac;
    if (valid == 0) return false;

    // Invalid pixels get the tile mean rather than black: TM_CCOEFF_NORMED
    // takes no mask, and a black wedge would dominate the correlation.
    const uchar mean = (uchar)clampd(sum / valid, 0.0, 255.0);
    for (int j = 0; j < S; ++j) {
        uchar *row = out.ptr<uchar>(j);
        const uchar *vrow = valid8.ptr<uchar>(j);
        for (int i = 0; i < S; ++i)
            if (!vrow[i]) row[i] = mean;
    }
    return frac >= 0.10;
}

// The camera orientation at a time, from the pre-sampled chain.
QQuaternion orientationAt(const TrackRequest &req, double t)
{
    const int n = req.camTimes.size();
    if (n == 0) return QQuaternion();
    if (t <= req.camTimes.first()) return req.camOrientations.first();
    if (t >= req.camTimes.last()) return req.camOrientations.last();
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        const int mid = (lo + hi) / 2;
        if (req.camTimes[mid] <= t) lo = mid; else hi = mid;
    }
    const double span = req.camTimes[hi] - req.camTimes[lo];
    const float f = span > 1e-9 ? (float)((t - req.camTimes[lo]) / span) : 0.0f;
    return QQuaternion::slerp(req.camOrientations[lo], req.camOrientations[hi], f);
}

// A minimal sequential decoder: open, seek once, hand out luma frames.
class LumaReader
{
public:
    bool open(const QString &path, QString *error)
    {
        if (m_input.open(path) < 0) { if (error) *error = QObject::tr("cannot open video"); return false; }
        m_fmt = m_input.fmt;
        if (avformat_find_stream_info(m_fmt, nullptr) < 0) {
            if (error) *error = QObject::tr("no stream info"); return false; }
        m_stream = av_find_best_stream(m_fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
        if (m_stream < 0) { if (error) *error = QObject::tr("no video stream"); return false; }
        const AVCodec *codec = avcodec_find_decoder(m_fmt->streams[m_stream]->codecpar->codec_id);
        if (!codec) { if (error) *error = QObject::tr("no decoder"); return false; }
        m_ctx = avcodec_alloc_context3(codec);
        avcodec_parameters_to_context(m_ctx, m_fmt->streams[m_stream]->codecpar);
        // Two threads: this runs alongside whatever else the app is doing, and
        // handing a decoder every core measurably starves the other stage
        // (see the note in exporter.cpp's DecodeReader::open).
        m_ctx->thread_count = qBound(1, QThread::idealThreadCount() / 2, 2);
        m_ctx->thread_type = FF_THREAD_FRAME | FF_THREAD_SLICE;
        if (avcodec_open2(m_ctx, codec, nullptr) < 0) {
            if (error) *error = QObject::tr("cannot open decoder"); return false; }
        m_pkt = av_packet_alloc();
        m_frame = av_frame_alloc();
        return true;
    }

    void seek(double t)
    {
        const AVStream *st = m_fmt->streams[m_stream];
        const int64_t ts = (int64_t)(t / av_q2d(st->time_base));
        av_seek_frame(m_fmt, m_stream, ts, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(m_ctx);
    }

    // Next frame at or after `minTime`. False at end of stream.
    bool next(double minTime, double *tOut)
    {
        const AVStream *st = m_fmt->streams[m_stream];
        const double tb = av_q2d(st->time_base);
        while (true) {
            int ret = av_read_frame(m_fmt, m_pkt);
            if (ret < 0) return false;
            if (m_pkt->stream_index != m_stream) { av_packet_unref(m_pkt); continue; }
            ret = avcodec_send_packet(m_ctx, m_pkt);
            av_packet_unref(m_pkt);
            if (ret < 0) continue;
            while (true) {
                ret = avcodec_receive_frame(m_ctx, m_frame);
                if (ret < 0) break;
                const double t = m_frame->best_effort_timestamp * tb;
                if (t + 1e-6 >= minTime) { if (tOut) *tOut = t; return true; }
                av_frame_unref(m_frame);
            }
        }
    }

    void release() { av_frame_unref(m_frame); }
    const uchar *y() const { return m_frame->data[0]; }
    int width() const { return m_frame->width; }
    int height() const { return m_frame->height; }
    int stride() const { return m_frame->linesize[0]; }

    ~LumaReader()
    {
        if (m_pkt) av_packet_free(&m_pkt);
        if (m_frame) av_frame_free(&m_frame);
        if (m_ctx) avcodec_free_context(&m_ctx);
        m_input.close();
    }

private:
    AvInput m_input;
    AVFormatContext *m_fmt = nullptr;
    AVCodecContext *m_ctx = nullptr;
    AVPacket *m_pkt = nullptr;
    AVFrame *m_frame = nullptr;
    int m_stream = -1;
};

} // namespace

// --- TileTracker ----------------------------------------------------------

// Two templates. The anchor is what the user pointed at and never changes, so
// it cannot drift; the running one follows the subject's appearance as it
// turns, is lit differently or changes shape, which is the only way to hold a
// person for more than a second or two. Position comes from whichever matches
// better this frame, so a stale anchor cannot veto a good running match and a
// drifting running template cannot outvote the anchor.
struct TileTracker::AnchorHolder { cv::Mat anchor; cv::Mat running; };

double TileTracker::scaleRatio() const { return std::exp(m_scaleLog); }

bool TileTracker::begin(const Config &cfg, const uchar *y, int w, int h, int stride,
                        const QQuaternion &camOrientation, double t, QString *error)
{
    m_cfg = cfg;
    m_anchorHolder = QSharedPointer<AnchorHolder>::create();
    m_dirWorld = camOrientation.rotatedVector(cfg.seedDirCam.normalized());
    m_scaleLog = 0.0;
    m_frames = 0;
    m_badRun = m_rejectRun = m_hardRun = m_ambiguousRun = 0;
    m_reseeds = 0;
    m_lastReseedTime = -1e9;
    m_prevTime = t;
    m_lossReason.clear();
    m_omegaRate = 0.0;

    // Fixed pixel budget, adaptive angular resolution: the cost of a frame
    // does not then depend on how big the subject is.
    const double d0Deg = qMax(0.5, cfg.seedRadiusRad * 2.0 * 180.0 / M_PI);
    m_pDeg = clampd(1.3 * d0Deg / kTemplate, 0.125, 0.5);
    m_searchHalfDeg = clampd(0.65 * d0Deg, 3.0, 12.0);
    m_tileSize = (int)clampd(kTemplate + 2 * std::ceil(m_searchHalfDeg / m_pDeg), 80, 128);

    cv::Mat tile;
    double validFrac = 0.0;
    const double pTan = std::tan(m_pDeg * M_PI / 180.0);
    if (!buildTile(y, w, h, stride, cfg.lenses, camOrientation, m_dirWorld,
                   pTan, m_tileSize, false, tile, &validFrac)) {
        if (error) *error = QObject::tr("that point is not visible in either lens");
        return false;
    }
    const int o = (m_tileSize - kTemplate) / 2;
    m_anchorHolder->anchor = tile(cv::Rect(o, o, kTemplate, kTemplate)).clone();
    m_anchorHolder->running = m_anchorHolder->anchor.clone();

    cv::Scalar mean, stddev;
    cv::meanStdDev(m_anchorHolder->anchor, mean, stddev);
    if (stddev[0] < kMinSeedStdDev) {
        if (error)
            *error = QObject::tr("that patch has too little contrast to track "
                                 "(detail %1, needs %2) -- try a more distinct part of it")
                         .arg(stddev[0], 0, 'f', 1).arg(kMinSeedStdDev);
        return false;
    }

    // Can it be found again? Match the template back into its own tile: a
    // distinctive subject peaks once, sharply. Sky or water peaks nearly as
    // well all along itself, and tracking it slides while reporting a
    // perfectly healthy score.
    {
        cv::Mat selfMap;
        cv::matchTemplate(tile, m_anchorHolder->anchor, selfMap, cv::TM_CCOEFF_NORMED);
        double minV, maxV;
        cv::Point minL, maxL;
        cv::minMaxLoc(selfMap, &minV, &maxV, &minL, &maxL);
        const double psr = peakSidelobeRatio(selfMap, maxL, kTemplate / 2);
        if (psr < kMinSeedPsr) {
            if (error)
                *error = QObject::tr("that area looks much the same all over (distinctness %1, "
                                     "needs %2), so tracking would slide across it -- point at "
                                     "something with a distinct edge or pattern")
                             .arg(psr, 0, 'f', 1).arg(kMinSeedPsr);
            return false;
        }
    }

    m_last = TrackSample{};
    m_last.t = t;
    m_lastScore = 1.0;
    recordSample(camOrientation, t, 1.0);
    ++m_frames;
    return true;
}

void TileTracker::recordSample(const QQuaternion &qAct, double t, double score)
{
    const QVector3D cam = qAct.conjugated().rotatedVector(m_dirWorld);
    double theta, phi;
    proj::dirToThetaPhi(toV(cam), theta, phi);
    const bool front = theta <= M_PI * 0.5;
    m_preferRear = !front;
    const auto &lp = front ? m_cfg.lenses.front : m_cfg.lenses.rear;
    const proj::LensGeom geom = proj::LensGeom::make(lp.cx, lp.cy, lp.radius, lp.k1, lp.k2,
                                                     lp.rotation, lp.hflip);
    TrackSample smp;
    smp.t = t;
    smp.lens = front ? 0 : 1;
    proj::lensUv(front, theta, phi, geom, smp.u, smp.v);

    // The subject's edge, in the same coordinates, so the resolver can turn
    // it back into an angular size.
    QVector3D up(0.0f, 1.0f, 0.0f);
    if (std::fabs(QVector3D::dotProduct(m_dirWorld, up)) > 0.996f)
        up = QVector3D(0.0f, 0.0f, 1.0f);
    const QVector3D right = QVector3D::crossProduct(up, m_dirWorld).normalized();
    const double radRad = m_cfg.seedRadiusRad * std::exp(m_scaleLog);
    const QVector3D edgeWorld = (m_dirWorld + right * (float)std::tan(radRad)).normalized();
    const QVector3D edgeCam = qAct.conjugated().rotatedVector(edgeWorld);
    double eTheta, ePhi;
    proj::dirToThetaPhi(toV(edgeCam), eTheta, ePhi);
    double eu, ev;
    proj::lensUv(front, eTheta, ePhi, geom, eu, ev);
    smp.halfW = std::hypot(eu - smp.u, ev - smp.v);
    smp.halfH = smp.halfW;
    smp.conf = clampd(score, 0.0, 1.0);
    m_last = smp;
}

TileTracker::Step TileTracker::step(const uchar *y, int w, int h, int stride,
                                    const QQuaternion &qAct, double t)
{
    if (!m_anchorHolder || m_anchorHolder->anchor.empty()) {
        m_lossReason = QStringLiteral("not seeded");
        return Step::Lost;
    }
    const double dt = qMax(1e-3, t - m_prevTime);

    // Predict: constant angular velocity in the world frame.
    QVector3D predicted = m_dirWorld;
    if (m_omegaRate > 1e-4)
        predicted = QQuaternion::fromAxisAndAngle(m_omegaAxis, (float)(m_omegaRate * dt))
                        .rotatedVector(m_dirWorld).normalized();

    // Hold the search window at a constant ANGULAR radius. The tile samples at
    // m_pDeg * exp(m_scaleLog) per pixel, so a fixed pixel budget means the
    // window silently shrinks as the subject's apparent size grows -- exactly
    // when the subject is closest and sweeping fastest, which is when the
    // window is needed most. Sized here instead, per frame.
    {
        const double perPx = m_pDeg * std::exp(m_scaleLog);
        m_tileSize = (int)clampd(kTemplate + 2 * std::ceil(m_searchHalfDeg / qMax(1e-3, perPx)),
                                 kTemplate + 16, kMaxTileSize);
    }
    const double pTanBase = std::tan(m_pDeg * M_PI / 180.0) * std::exp(m_scaleLog);
    double bestScore = -2.0, bestDx = 0.0, bestDy = 0.0, bestValid = 0.0, bestPsr = 99.0;
    double anchorScore = -2.0;
    double scores[3] = {-2.0, -2.0, -2.0};
    cv::Point bestLoc;
    cv::Mat bestTile;
    int bestK = 1;

    for (int k = 0; k < 3; ++k) {
        const double pTan = pTanBase * std::pow(kScaleStep, k - 1);
        cv::Mat tile;
        double validFrac = 0.0;
        if (!buildTile(y, w, h, stride, m_cfg.lenses, qAct, predicted, pTan,
                       m_tileSize, m_preferRear, tile, &validFrac))
            continue;
        double minV, maxV;
        cv::Point minL, maxL;
        cv::Mat scoreMap;
        cv::matchTemplate(tile, m_anchorHolder->anchor, scoreMap, cv::TM_CCOEFF_NORMED);
        cv::minMaxLoc(scoreMap, &minV, &maxV, &minL, &maxL);
        double useScore = maxV;
        cv::Point useLoc = maxL;
        const cv::Mat *useMap = &scoreMap;
        anchorScore = qMax(anchorScore, maxV);

        cv::Mat runMap;
        cv::matchTemplate(tile, m_anchorHolder->running, runMap, cv::TM_CCOEFF_NORMED);
        double rMin, rMax;
        cv::Point rMinL, rMaxL;
        cv::minMaxLoc(runMap, &rMin, &rMax, &rMinL, &rMaxL);
        if (rMax > useScore) { useScore = rMax; useLoc = rMaxL; useMap = &runMap; }

        scores[k] = useScore;
        if (useScore > bestScore) {
            bestScore = useScore;
            bestK = k;
            bestValid = validFrac;
            bestPsr = peakSidelobeRatio(*useMap, useLoc, kTemplate / 2);
            bestLoc = useLoc;
            bestTile = tile;
            bestDx = (useLoc.x + kTemplate * 0.5 - m_tileSize * 0.5) * pTan;
            bestDy = -(useLoc.y + kTemplate * 0.5 - m_tileSize * 0.5) * pTan;
        }
    }
    m_lastScore = bestScore;

    if (bestScore < -1.0) { m_lossReason = QStringLiteral("out of field"); return Step::Lost; }
    if (bestValid < kMinValidFraction) { m_lossReason = QStringLiteral("left the lens"); return Step::Lost; }

    // Is there anything actually there? Measured at the matched position, not
    // over the whole tile: a tile that is half subject and half sky has plenty
    // of variance while the match sits on the empty half.
    double patchStd = 999.0;
    const bool patchInTile = !bestTile.empty() && bestLoc.x >= 0 && bestLoc.y >= 0
                          && bestLoc.x + kTemplate <= bestTile.cols
                          && bestLoc.y + kTemplate <= bestTile.rows;
    if (patchInTile) {
        cv::Scalar pMean, pStd;
        cv::meanStdDev(bestTile(cv::Rect(bestLoc.x, bestLoc.y, kTemplate, kTemplate)),
                       pMean, pStd);
        patchStd = pStd[0];
    }
    const bool featureless = patchStd < kMinTrackStdDev;

    m_hardRun = (bestScore < kScoreHardFloor) ? m_hardRun + 1 : 0;
    m_badRun = (bestScore < kScoreSoftFloor || featureless) ? m_badRun + 1 : 0;
    // If the ORIGINAL patch still matches well, we are locked on by definition
    // and no amount of background similarity should end the track.
    const bool anchorHolds = anchorScore > kAnchorStillGood;
    m_ambiguousRun = (!anchorHolds && bestPsr < kMinRunPsr) ? m_ambiguousRun + 1 : 0;

    const bool wouldLose = m_hardRun >= kHardRun || m_badRun >= kBadRun
                        || m_ambiguousRun >= kAmbiguousRun;
    if (wouldLose) {
        // Before giving up: re-seed from where we still believe the subject
        // is. A child turning round stops looking like the patch that was
        // picked, and the tracker is usually still ON it -- what has failed is
        // the template, not the position. Adopting the current appearance
        // there keeps the track alive through the turn.
        //
        // Guarded so this cannot quietly become "track the background": the
        // new patch must be distinctive in its own right, re-seeds are rate
        // limited, and they are counted so the caller can say how often it
        // happened.
        const bool canReseed = m_reseeds < kMaxReseeds
                            && (t - m_lastReseedTime) > kReseedMinGap
                            && !bestTile.empty()
                            && bestLoc.x >= 0 && bestLoc.y >= 0
                            && bestLoc.x + kTemplate <= bestTile.cols
                            && bestLoc.y + kTemplate <= bestTile.rows;
        if (canReseed) {
            cv::Mat candidate = bestTile(cv::Rect(bestLoc.x, bestLoc.y,
                                                  kTemplate, kTemplate)).clone();
            cv::Scalar cMean, cStd;
            cv::meanStdDev(candidate, cMean, cStd);
            bool distinct = cStd[0] >= kMinSeedStdDev;
            if (distinct) {
                cv::Mat selfMap;
                cv::matchTemplate(bestTile, candidate, selfMap, cv::TM_CCOEFF_NORMED);
                double sMin, sMax;
                cv::Point sMinL, sMaxL;
                cv::minMaxLoc(selfMap, &sMin, &sMax, &sMinL, &sMaxL);
                distinct = peakSidelobeRatio(selfMap, sMaxL, kTemplate / 2) >= kMinSeedPsr;
            }
            if (distinct) {
                m_anchorHolder->anchor = candidate;
                m_anchorHolder->running = candidate.clone();
                m_hardRun = m_badRun = m_ambiguousRun = 0;
                m_reseeds++;
                m_lastReseedTime = t;
                m_prevTime = t;
                return Step::Rejected;      // coast this frame on the new template
            }
        }
        if (m_hardRun >= kHardRun) m_lossReason = QStringLiteral("lost the subject");
        else if (m_badRun >= kBadRun)
            m_lossReason = featureless ? QStringLiteral("subject moved out of view")
                                       : QStringLiteral("match too weak");
        else m_lossReason = QStringLiteral("subject blends into its surroundings");
        return Step::Lost;
    }

    QVector3D up(0.0f, 1.0f, 0.0f);
    if (std::fabs(QVector3D::dotProduct(predicted, up)) > 0.996f)
        up = QVector3D(0.0f, 0.0f, 1.0f);
    const QVector3D right = QVector3D::crossProduct(up, predicted).normalized();
    const QVector3D tileUp = QVector3D::crossProduct(predicted, right).normalized();
    const QVector3D measured = (predicted + right * (float)bestDx
                                          + tileUp * (float)bestDy).normalized();

    const double stepDeg = std::acos(clampd(QVector3D::dotProduct(m_dirWorld, measured), -1.0, 1.0))
                         * 180.0 / M_PI;
    if (stepDeg / dt > m_cfg.maxWorldSpeedDeg) {
        // Faster than the subject could plausibly move: skip the frame and
        // coast, and only give up if it keeps happening.
        if (++m_rejectRun >= kRejectRun) {
            m_lossReason = QStringLiteral("moved too fast to follow");
            return Step::Lost;
        }
        m_prevTime = t;
        return Step::Rejected;
    }
    m_rejectRun = 0;

    // Scale, from a parabola through the three scores.
    if (bestK == 1 && scores[0] > -1.0 && scores[2] > -1.0
        && scores[1] > scores[0] && scores[1] > scores[2]) {
        const double denom = scores[0] - 2.0 * scores[1] + scores[2];
        if (std::fabs(denom) > 1e-6) {
            const double delta = 0.5 * (scores[0] - scores[2]) / denom;
            m_scaleLog += clampd(delta * std::log(kScaleStep), -kMaxScaleStepLog, kMaxScaleStepLog);
        }
    } else if (bestK != 1) {
        m_scaleLog += clampd((bestK - 1) * std::log(kScaleStep),
                             -kMaxScaleStepLog, kMaxScaleStepLog);
    }

    if (stepDeg > 1e-4) {
        m_omegaAxis = QVector3D::crossProduct(m_dirWorld, measured).normalized();
        m_omegaRate = 0.5 * m_omegaRate + 0.5 * qMin(stepDeg / dt, m_cfg.maxWorldSpeedDeg);
    }

    // Learn the subject's current appearance, slowly, and only from a match
    // worth learning from. The anchor stays untouched as the ground truth.
    // Never learn from a featureless patch, or the template dissolves into the
    // background it happens to be sitting on and the track becomes a confident
    // lock on nothing.
    if (bestScore > kRunningUpdateMin && !featureless && patchInTile) {
        cv::Mat patch = bestTile(cv::Rect(bestLoc.x, bestLoc.y, kTemplate, kTemplate));
        cv::addWeighted(m_anchorHolder->running, 1.0 - kRunningBlend,
                        patch, kRunningBlend, 0.0, m_anchorHolder->running);
    }

    m_dirWorld = measured;
    m_prevTime = t;
    ++m_frames;
    recordSample(qAct, t, bestScore);

    // RENDER360_TRACK_DUMP=<dir>: write the tile as a PGM every few frames.
    // Statistics cannot tell "the tile is stable and the match is wrong" from
    // "the tile is sliding"; looking at them can.
    static const QByteArray dumpDir = qgetenv("RENDER360_TRACK_DUMP");
    if (!dumpDir.isEmpty() && (m_frames % 5) == 0) {
        cv::Mat tile;
        double vf = 0.0;
        if (buildTile(y, w, h, stride, m_cfg.lenses, qAct, m_dirWorld,
                      std::tan(m_pDeg * M_PI / 180.0) * std::exp(m_scaleLog),
                      m_tileSize, m_preferRear, tile, &vf)) {
            const QString path = QString::fromLatin1(dumpDir) +
                QStringLiteral("/tile_%1.pgm").arg(m_frames, 4, 10, QLatin1Char('0'));
            QFile f(path);
            if (f.open(QIODevice::WriteOnly)) {
                f.write(QStringLiteral("P5\n%1 %2\n255\n").arg(tile.cols).arg(tile.rows).toLatin1());
                for (int r = 0; r < tile.rows; ++r)
                    f.write(reinterpret_cast<const char *>(tile.ptr<uchar>(r)), tile.cols);
            }
        }
    }
    return Step::Ok;
}

// One frame's measurement, in the frame the tracker saw: a half-frame fisheye
// coordinate plus the subject's extent, which is what the resolver
// back-projects. Free-standing because with several patches the subject's
// direction belongs to none of them.
static TrackSample sampleFor(double t, const QVector3D &dirWorld, const QQuaternion &qAct,
                             const TrackLenses &lenses, double radiusRad, double score)
{
    const QVector3D cam = qAct.conjugated().rotatedVector(dirWorld.normalized());
    double theta, phi;
    proj::dirToThetaPhi(proj::Vec3{cam.x(), cam.y(), cam.z()}, theta, phi);
    const bool front = theta <= M_PI * 0.5;
    const auto &lp = front ? lenses.front : lenses.rear;
    const proj::LensGeom geom = proj::LensGeom::make(lp.cx, lp.cy, lp.radius, lp.k1, lp.k2,
                                                     lp.rotation, lp.hflip);
    TrackSample smp;
    smp.t = t;                    // never 0: the track's span, and every time
                                  // the resolver evaluates, comes from this
    smp.lens = front ? 0 : 1;
    proj::lensUv(front, theta, phi, geom, smp.u, smp.v);

    QVector3D up(0.0f, 1.0f, 0.0f);
    if (std::fabs(QVector3D::dotProduct(dirWorld.normalized(), up)) > 0.996f)
        up = QVector3D(0.0f, 0.0f, 1.0f);
    const QVector3D right = QVector3D::crossProduct(up, dirWorld.normalized()).normalized();
    const QVector3D edge = (dirWorld.normalized() + right * (float)std::tan(radiusRad)).normalized();
    const QVector3D edgeCam = qAct.conjugated().rotatedVector(edge);
    double eTheta, ePhi;
    proj::dirToThetaPhi(proj::Vec3{edgeCam.x(), edgeCam.y(), edgeCam.z()}, eTheta, ePhi);
    double eu, ev;
    proj::lensUv(front, eTheta, ePhi, geom, eu, ev);
    smp.halfW = std::hypot(eu - smp.u, ev - smp.v);
    smp.halfH = smp.halfW;
    smp.conf = clampd(score, 0.0, 1.0);
    return smp;
}

ObjectTracker::ObjectTracker(QObject *parent) : QObject(parent) {}

ObjectTracker::~ObjectTracker()
{
    cancel();
    if (m_thread) {
        // Wait without a deadline. The worker calls run() on THIS object, so
        // returning while it is still going leaves it holding a destroyed
        // tracker -- there is no safe timeout here, only a wait. The loop
        // tests the cancel flag once per frame, so this is bounded by one
        // frame's decode in every case that is not already wedged.
        m_thread->wait();
        delete m_thread;               // never detach: a worker outliving the
        m_thread = nullptr;            // app is a shutdown crash
    }
}

void ObjectTracker::track(const TrackRequest &req)
{
    if (m_running.loadRelaxed() != 0) {
        // Refuse, but SAY so. Returning silently left App's m_trackRunning
        // stuck true, so the button read "Stop" over a pass that was never
        // started and the panel never came back.
        emit trackFailed(QObject::tr("A track is already running"));
        return;
    }
    // Own the thread outright. It used to delete itself on finished() while
    // m_thread went on pointing at it, so the SECOND track called wait() on
    // freed memory and took the app down with it.
    if (m_thread) {
        // m_running is cleared by the worker BEFORE run() returns, so the
        // thread can still be winding down here. Ask it to stop and wait
        // properly: deleting a QThread that is still running frees its private
        // data underneath the running code, which is the very use-after-free
        // this ownership was introduced to prevent -- a 5 s timeout followed
        // by an unconditional delete just moved the crash to slow storage.
        m_cancel.storeRelaxed(1);
        if (m_thread->wait(30000)) {
            delete m_thread;
        } else {
            // It will not stop. Hand it to Qt to free once it genuinely
            // finishes and let go of the pointer: a leaked thread is
            // survivable, a freed running one is not.
            connect(m_thread, &QThread::finished, m_thread, &QObject::deleteLater);
            qWarning("ObjectTracker: previous pass would not stop; leaving it to finish");
        }
        m_thread = nullptr;
    }
    m_cancel.storeRelaxed(0);
    m_running.storeRelaxed(1);
    m_thread = QThread::create([this, req]() { run(req); });
    m_thread->start();
}

void ObjectTracker::run(TrackRequest req)
{
    QElapsedTimer clock;
    clock.start();
    TrackResult result;

    const QString source = VisualRotationComputer::chooseDecodeSource(
        req.videoPath, req.proxyOverride, 320);
    const bool usingProxy = (source != req.videoPath);

    LumaReader reader;
    QString err;
    if (!reader.open(source, &err)) {
        m_running.storeRelaxed(0);
        emit trackFailed(QObject::tr("Tracking could not read the video: %1").arg(err));
        return;
    }
    reader.seek(req.t0);

    // One tracker per patch the user marked. They follow the same subject, so
    // each keeps its offset from the subject's direction: whichever patch is
    // matching best drives the view, and a patch that fails is retried from
    // where the survivors say it should be. That is what carries a subject
    // through a turn -- the head may be unrecognisable while the shirt is not.
    struct PointState {
        TileTracker tracker;
        QQuaternion offset;         // subject direction -> this patch
        bool alive = false;
        double score = 0.0;
        bool okThisFrame = false;
    };
    QVector<PointState> points;
    points.resize(qMax(1, req.seedDirsCam.size()));

    TileTracker::Config cfg;
    cfg.lenses = req.lenses;
    cfg.seedRadiusRad = req.seedRadiusRad;
    cfg.maxWorldSpeedDeg = req.maxWorldSpeedDeg;

    QVector3D subjectDir;
    int frames = 0;
    double scoreSum = 0.0;
    double lastTime = req.t0;
    QString endReason = QStringLiteral("?");
    QString seedError;

    while (true) {
        if (m_cancel.loadRelaxed() != 0) {
            result.lossReason = QStringLiteral("cancelled");
            break;
        }
        double t = 0.0;
        const double minTime = (frames == 0) ? req.t0 : lastTime + 0.5 / qMax(1.0, req.fps);
        if (!reader.next(minTime, &t)) { endReason = QStringLiteral("end of stream"); break; }
        if (t > req.tEnd + 1e-6) { reader.release(); endReason = QStringLiteral("reached the end time"); break; }

        const QQuaternion qAct = orientationAt(req, t);

        if (frames == 0) {
            // Seed every patch. One that will not take is dropped with a note
            // rather than failing the whole track -- the others may be fine.
            for (int i = 0; i < points.size(); ++i) {
                cfg.seedDirCam = (i < req.seedDirsCam.size()) ? req.seedDirsCam[i]
                                                              : req.seedDirsCam.value(0);
                QString e;
                points[i].alive = points[i].tracker.begin(cfg, reader.y(), reader.width(),
                                                          reader.height(), reader.stride(),
                                                          qAct, t, &e);
                if (!points[i].alive && seedError.isEmpty())
                    seedError = e;
            }
            int alive = 0;
            for (const PointState &p : points) if (p.alive) ++alive;
            result.pointsSeeded = alive;
            if (alive == 0) {
                reader.release();
                m_running.storeRelaxed(0);
                emit trackFailed(seedError.isEmpty()
                    ? QObject::tr("Could not lock on to that") : seedError);
                return;
            }
            for (const PointState &p : points)
                if (p.alive) { subjectDir = p.tracker.worldDir(); break; }
            for (PointState &p : points)
                if (p.alive)
                    p.offset = QQuaternion::rotationTo(subjectDir, p.tracker.worldDir());

            result.samples.append(sampleFor(t, subjectDir, qAct, req.lenses,
                                            req.seedRadiusRad, 1.0));
            scoreSum += 1.0;
            lastTime = t;
            ++frames;
            reader.release();
            continue;
        }

        // Advance every live patch.
        int aliveNow = 0;
        for (PointState &p : points) {
            p.okThisFrame = false;
            if (!p.alive) continue;
            const TileTracker::Step st = p.tracker.step(reader.y(), reader.width(),
                                                        reader.height(), reader.stride(), qAct, t);
            if (st == TileTracker::Step::Lost) {
                p.alive = false;                 // may come back below
                continue;
            }
            ++aliveNow;
            if (st == TileTracker::Step::Ok) {
                p.okThisFrame = true;
                p.score = p.tracker.lastScore();
            }
        }

        // The best patch this frame defines where the subject is.
        int best = -1;
        for (int i = 0; i < points.size(); ++i)
            if (points[i].okThisFrame && (best < 0 || points[i].score > points[best].score))
                best = i;

        if (best < 0 && aliveNow == 0) {
            result.lost = true;
            result.lossTime = t;
            // Report the reason of whichever patch held on longest.
            for (const PointState &p : points)
                if (!p.tracker.lossReason().isEmpty()) result.lossReason = p.tracker.lossReason();
            if (result.lossReason.isEmpty()) result.lossReason = QStringLiteral("lost the subject");
            reader.release();
            break;
        }

        if (best >= 0) {
            subjectDir = points[best].offset.conjugated().rotatedVector(
                             points[best].tracker.worldDir()).normalized();
            // Let the constellation breathe: the patches move relative to each
            // other as the subject turns, so the offsets follow slowly.
            for (PointState &p : points)
                if (p.okThisFrame)
                    p.offset = QQuaternion::slerp(
                        p.offset, QQuaternion::rotationTo(subjectDir, p.tracker.worldDir()), 0.1f);

            // Retry the failed patches where the survivors say they should be.
            for (PointState &p : points) {
                if (p.alive) continue;
                const QVector3D expected = p.offset.rotatedVector(subjectDir).normalized();
                p.tracker.setWorldDir(expected);
                p.tracker.clearRuns();
                if (p.tracker.step(reader.y(), reader.width(), reader.height(),
                                   reader.stride(), qAct, t) == TileTracker::Step::Ok
                    && p.tracker.lastScore() > kRevivalScore) {
                    p.alive = true;
                    p.score = p.tracker.lastScore();
                    result.revivals++;
                }
            }

            result.samples.append(sampleFor(t, subjectDir, qAct, req.lenses,
                                            req.seedRadiusRad * points[best].tracker.scaleRatio(),
                                            points[best].score));
            scoreSum += points[best].score;
        }

        lastTime = t;
        ++frames;
        reader.release();

        if ((frames % 15) == 0) {
            // Seconds tracked, not a percentage: how far it has got through
            // the subject's movement is what you actually want to know, and a
            // percentage of a span you did not choose says little.
            const double span = qMax(1e-6, req.tEnd - req.t0);
            emit progressChanged(clampd((t - req.t0) / span, 0.0, 1.0),
                                 QObject::tr("Tracking… %1 s of %2 s")
                                     .arg(t - req.t0, 0, 'f', 1).arg(span, 0, 'f', 1));
        }
    }

    for (const PointState &p : points) if (p.alive) result.pointsSurviving++;
    result.meanScore = frames > 0 ? scoreSum / frames : 0.0;
    result.msPerFrame = frames > 0 ? (double)clock.elapsed() / frames : 0.0;
    qInfo().noquote() << "ObjectTracker: stopped because:"
                      << (result.lost ? result.lossReason : endReason) << "at t =" << lastTime;
    qInfo("ObjectTracker: %d frames in %lld ms (%.1f ms/frame, %s), mean score %.3f, "
          "%d/%d patches alive, %d revivals%s",
          frames, clock.elapsed(), result.msPerFrame, usingProxy ? "proxy" : "original",
          result.meanScore, result.pointsSurviving, result.pointsSeeded, result.revivals,
          result.lost ? qPrintable(QStringLiteral(", lost: ") + result.lossReason) : "");

    m_running.storeRelaxed(0);
    emit trackFinished(result);
}
