#include "exporter.h"

#include <QImage>
#include <QThread>
#include <QFile>
#include <QtMath>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <thread>
#include <vector>

#include "videodecoder.h"
#include "gpurenderer.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

// ---------------------------------------------------------------------------
// Math helpers — a faithful C++ port of project.frag
// ---------------------------------------------------------------------------

static constexpr double kPi = 3.14159265358979323846;

static double degToRad(double d) { return d * kPi / 180.0; }

static double clampUnit(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

struct Vec3 { double x, y, z; };

static Vec3 normalize3(Vec3 v)
{
    double l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (l < 1e-12) return Vec3{0.0, 0.0, 0.0};
    return Vec3{v.x / l, v.y / l, v.z / l};
}

// Quaternion rotation (equivalent to QQuaternion::rotatedVector, inline for
// speed since it runs per output pixel).
static Vec3 quatRotate(double qw, double qx, double qy, double qz, Vec3 v)
{
    const double tx = 2.0 * (qy * v.z - qz * v.y);
    const double ty = 2.0 * (qz * v.x - qx * v.z);
    const double tz = 2.0 * (qx * v.y - qy * v.x);
    return Vec3{
        v.x + qw * tx + (qy * tz - qz * ty),
        v.y + qw * ty + (qz * tx - qx * tz),
        v.z + qw * tz + (qx * ty - qy * tx),
    };
}

// Bilinear sample with GL texture semantics (texel i sits at (i+0.5)/size,
// clamp-to-edge), so the output matches the linear-filtered GPU sampling.
static double sampleBilinear(const quint8 *data, int w, int h, int stride,
                             double u, double v)
{
    double x = u * w - 0.5;
    double y = v * h - 0.5;
    int x0 = (int)std::floor(x);
    int y0 = (int)std::floor(y);
    int x1 = x0 + 1;
    int y1 = y0 + 1;
    x0 = qBound(0, x0, w - 1);
    x1 = qBound(0, x1, w - 1);
    y0 = qBound(0, y0, h - 1);
    y1 = qBound(0, y1, h - 1);
    double fx = x - std::floor(x);
    double fy = y - std::floor(y);
    double v00 = data[y0 * stride + x0];
    double v10 = data[y0 * stride + x1];
    double v01 = data[y1 * stride + x0];
    double v11 = data[y1 * stride + x1];
    return (v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy)
         + (v01 * (1.0 - fx) + v11 * fx) * fy;
}

// Matches yuvToRgb() in project.frag; y/u/v are already scaled to [0,1].
static void yuvToRgb(double y, double u, double v, bool fullRange,
                     double &r, double &g, double &b)
{
    if (!fullRange) {
        y = (y - 16.0 / 255.0) * 255.0 / 219.0;
        u -= 0.5;
        v -= 0.5;
        r = y + 1.402 * v;
        g = y - 0.344136 * u - 0.714136 * v;
        b = y + 1.772 * u;
    } else {
        r = y + 1.402 * (v - 0.5);
        g = y - 0.344136 * (u - 0.5) - 0.714136 * (v - 0.5);
        b = y + 1.772 * (u - 0.5);
    }
    r = clampUnit(r) * 255.0;
    g = clampUnit(g) * 255.0;
    b = clampUnit(b) * 255.0;
}

static double smoothstep01(double e0, double e1, double x)
{
    double t = clampUnit((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}

// ---------------------------------------------------------------------------
// Offline renderer: projects a decoded fisheye frame into a view, exactly as
// the viewer's shader does.
// ---------------------------------------------------------------------------

static QImage renderFrame(const DecodedFrame &frame, const ExportFrameState &s,
                          int outW, int outH)
{
    QImage img(outW, outH, QImage::Format_RGB888);

    const quint8 *Y = reinterpret_cast<const quint8 *>(frame.yData.constData());
    const quint8 *U = reinterpret_cast<const quint8 *>(frame.uData.constData());
    const quint8 *V = reinterpret_cast<const quint8 *>(frame.vData.constData());
    const int yw = frame.width;
    const int yh = frame.height;
    const int uw = frame.width / 2;
    const int uh = frame.height / 2;
    const int ys = frame.yStride;
    const int us = frame.uStride;
    const int vs = frame.vStride;

    const double aspect = (double)outW / (double)outH;
    const double fovPersp = std::tan(degToRad(s.fov * 0.5));
    const double fovStereo = 2.0 * std::tan(degToRad(s.fov * 0.25));
    const double verticalStretch = (16.0 / 9.0) / (4.0 / 3.0); // 4/3

    // eulerRotation(yaw, pitch, roll) = rotY * rotX * rotZ (see shader)
    const double cy = std::cos(degToRad(s.yaw)), sy = std::sin(degToRad(s.yaw));
    const double cp = std::cos(degToRad(s.pitch)), sp = std::sin(degToRad(s.pitch));
    const double cr = std::cos(degToRad(s.roll)), sr = std::sin(degToRad(s.roll));

    const QQuaternion imuC = s.imuOrientation.conjugated();
    const double iw = imuC.scalar(), ix = imuC.x(), iy = imuC.y(), iz = imuC.z();

    auto rayFor = [&](double u, double v) {
        switch (s.projection) {
        case 1: {  // Equirectangular: maps the full 360x180 sphere
            double lon = (u - 0.5) * 2.0 * kPi;
            double lat = (0.5 - v) * kPi;
            return Vec3{std::cos(lat) * std::sin(lon), std::sin(lat),
                        -std::cos(lat) * std::cos(lon)};
        }
        case 2: {  // Stereographic
            double ndcX = u * 2.0 - 1.0;
            double ndcY = v * 2.0 - 1.0;
            double px = ndcX * fovStereo * aspect;
            double py = -ndcY * fovStereo;
            double rho = std::sqrt(px * px + py * py);
            double nx = (rho > 1e-6) ? px / rho : 0.0;
            double ny = (rho > 1e-6) ? py / rho : 0.0;
            double ang = 2.0 * std::atan(rho * 0.5);
            return Vec3{nx * std::sin(ang), ny * std::sin(ang), -std::cos(ang)};
        }
        case 3: {  // SportsView
            double ndcX = u * 2.0 - 1.0;
            double ndcY = v * 2.0 - 1.0;
            return normalize3(Vec3{ndcX * fovPersp * aspect,
                                   -ndcY * fovPersp * verticalStretch, -1.0});
        }
        default: {  // Perspective (rectilinear)
            double ndcX = u * 2.0 - 1.0;
            double ndcY = v * 2.0 - 1.0;
            return normalize3(Vec3{ndcX * fovPersp * aspect, -ndcY * fovPersp, -1.0});
        }
        }
    };

    // Apply rotZ, then rotX, then rotY (matrix order rotY * rotX * rotZ).
    auto applyEuler = [&](Vec3 v) {
        double zx = cr * v.x - sr * v.y;
        double zy = sr * v.x + cr * v.y;
        double xz = v.z;
        double xx = zx;
        double xy = cp * zy + sp * xz;
        double xzz = -sp * zy + cp * xz;
        return Vec3{cy * xx + sy * xzz, xy, -sy * xx + cy * xzz};
    };

    const double frontK1 = s.frontK1, frontK2 = s.frontK2;
    const double rearK1 = s.rearK1, rearK2 = s.rearK2;
    const double frontRot = degToRad(s.frontRotation);
    const double rearRot = degToRad(s.rearRotation);
    const double frontCr = std::cos(frontRot), frontSr = std::sin(frontRot);
    const double rearCr = std::cos(rearRot), rearSr = std::sin(rearRot);

    auto sampleLens = [&](bool front, double theta, double phi,
                          double &rOut, double &gOut, double &bOut) {
        double r = front ? theta / (kPi * 0.5) : (kPi - theta) / (kPi * 0.5);
        double r2 = r * r;
        double rd = r * (1.0 + (front ? frontK1 : rearK1) * r2
                             + (front ? frontK2 : rearK2) * r2 * r2);
        double c = std::cos(phi), sn = std::sin(phi);
        double cc = front ? frontCr : rearCr;
        double ss = front ? frontSr : rearSr;
        double offx = (c * cc - sn * ss) * rd * (front ? s.frontRadius : s.rearRadius);
        double offy = (c * ss + sn * cc) * rd * (front ? s.frontRadius : s.rearRadius);
        if (front ? s.frontHFlip : s.rearHFlip)
            offx = -offx;

        double cu = (front ? s.frontCenterX : s.rearCenterX) + offx;
        double cv = (front ? s.frontCenterY : s.rearCenterY) + offy;
        double tv = front ? cv * 0.5 : cv * 0.5 + 0.5;

        double yy = sampleBilinear(Y, yw, yh, ys, cu, tv) / 255.0;
        double uu = sampleBilinear(U, uw, uh, us, cu, tv) / 255.0;
        double vv = sampleBilinear(V, uw, uh, vs, cu, tv) / 255.0;
        yuvToRgb(yy, uu, vv, s.fullRange, rOut, gOut, bOut);
    };

    auto renderRows = [&](int y0, int y1) {
        for (int py = y0; py < y1; ++py) {
            quint8 *line = img.scanLine(py);
            for (int px = 0; px < outW; ++px) {
                double u = (px + 0.5) / outW;
                double v = (py + 0.5) / outH;

                Vec3 ray = rayFor(u, v);
                ray = applyEuler(ray);                              // user look
                ray = quatRotate(iw, ix, iy, iz, ray);              // IMU counter-rotation

                double theta = std::acos(qBound(-1.0, -ray.z, 1.0));
                double phi = std::atan2(ray.y, ray.x);

                double rf, gf, bf, rr, gr, br;
                sampleLens(true, theta, phi, rf, gf, bf);
                sampleLens(false, theta, phi, rr, gr, br);

                double r, g, b;
                if (s.activeLens == 0) {          // front only
                    r = rf; g = gf; b = bf;
                } else if (s.activeLens == 1) {   // rear only
                    r = rr; g = gr; b = br;
                } else {                          // auto stitch blend
                    double blend = smoothstep01(s.blendStart, 1.0,
                                                theta / (kPi * 0.5));
                    r = rf + (rr - rf) * blend;
                    g = gf + (gr - gf) * blend;
                    b = bf + (br - bf) * blend;
                }

                // ---- Colour grading (mirror of project.frag) ----
                {
                    const double lumaN = clampUnit((0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0);
                    const double wL = (1.0 - lumaN) * (1.0 - lumaN);
                    const double wH = lumaN * lumaN;
                    const double wM = 1.0 - wL - wH;
                    // Split the mid band into low-mids / high-mids (mirror of
                    // project.frag): they share the old wM weight and cross
                    // over exactly at luma 0.5.
                    const double tMid = std::max(-1.0, std::min(1.0, (lumaN - 0.5) * 4.0));
                    const double wLM = wM * 0.5 * (1.0 - tMid);
                    const double wHM = wM * 0.5 * (1.0 + tMid);

                    r += 255.0 * (s.brightLows * wL + s.brightLowMids * wLM + s.brightHighMids * wHM + s.brightHighs * wH
                                + s.redLows * wL + s.redMids * wM + s.redHighs * wH);
                    g += 255.0 * (s.brightLows * wL + s.brightLowMids * wLM + s.brightHighMids * wHM + s.brightHighs * wH
                                + s.greenLows * wL + s.greenMids * wM + s.greenHighs * wH);
                    b += 255.0 * (s.brightLows * wL + s.brightLowMids * wLM + s.brightHighMids * wHM + s.brightHighs * wH
                                + s.blueLows * wL + s.blueMids * wM + s.blueHighs * wH);

                    const double lumaN2 = clampUnit((0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0);
                    const double midW = 1.0 - std::abs(lumaN2 * 2.0 - 1.0);
                    r += s.pop * 0.2 * midW * (r - 127.5);
                    g += s.pop * 0.2 * midW * (g - 127.5);
                    b += s.pop * 0.2 * midW * (b - 127.5);

                    const double luma255 = 0.2126 * r + 0.7152 * g + 0.0722 * b;
                    r = luma255 + (r - luma255) * s.saturation;
                    g = luma255 + (g - luma255) * s.saturation;
                    b = luma255 + (b - luma255) * s.saturation;

                    r = (r - 127.5) * s.contrast + 127.5;
                    g = (g - 127.5) * s.contrast + 127.5;
                    b = (b - 127.5) * s.contrast + 127.5;

                    r += s.brightness * 255.0;
                    g += s.brightness * 255.0;
                    b += s.brightness * 255.0;
                }

                quint8 *px3 = line + px * 3;
                px3[0] = (quint8)qRound(qBound(0.0, r, 255.0));
                px3[1] = (quint8)qRound(qBound(0.0, g, 255.0));
                px3[2] = (quint8)qRound(qBound(0.0, b, 255.0));
            }
        }
    };

    unsigned nThreads = std::min<unsigned>(std::thread::hardware_concurrency(), 8);
    if (outH < 128)
        nThreads = 1;
    std::vector<std::thread> pool;
    pool.reserve(nThreads);
    int rowsPer = outH / (int)nThreads;
    for (unsigned t = 0; t < nThreads; ++t) {
        int a = (int)t * rowsPer;
        int b = (t == nThreads - 1) ? outH : a + rowsPer;
        pool.emplace_back([&, a, b]() { renderRows(a, b); });
    }
    for (auto &th : pool)
        th.join();

    return img;
}

// ---------------------------------------------------------------------------
// FFmpeg decode reader — owns its own context so exporting can't disturb the
// GUI's live decoder, and runs entirely on the worker thread.
// ---------------------------------------------------------------------------

class DecodeReader
{
public:
    DecodeReader() = default;
    ~DecodeReader() { close(); }

    bool open(const QString &path, QString *error);
    void close();

    // Decode forward from the last position until a frame with PTS >= targetSec
    // is found (seeking BACKWARD to targetSec on the first call). Returns false
    // on EOF/error.
    bool readFrameAt(double targetSec, DecodedFrame *out);

    double duration() const { return m_duration; }
    bool isFullRange() const { return m_fullRange; }
    bool isOpen() const { return m_fmt != nullptr; }

private:
    AVFormatContext *m_fmt = nullptr;
    AVCodecContext *m_dec = nullptr;
    AVStream *m_stream = nullptr;
    int m_streamIdx = -1;
    AVPacket *m_pkt = nullptr;
    AVFrame *m_frame = nullptr;
    double m_duration = 0.0;
    bool m_fullRange = true;
    bool m_seeked = false;
};

bool DecodeReader::open(const QString &path, QString *error)
{
    if (avformat_open_input(&m_fmt, path.toUtf8().constData(), nullptr, nullptr) < 0) {
        if (error) *error = QObject::tr("Failed to open video file");
        return false;
    }
    if (avformat_find_stream_info(m_fmt, nullptr) < 0) {
        if (error) *error = QObject::tr("Failed to read video stream info");
        return false;
    }

    m_streamIdx = av_find_best_stream(m_fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    if (m_streamIdx < 0) {
        if (error) *error = QObject::tr("No video stream found");
        return false;
    }
    m_stream = m_fmt->streams[m_streamIdx];

    const AVCodec *codec = avcodec_find_decoder(m_stream->codecpar->codec_id);
    if (!codec) {
        if (error) *error = QObject::tr("No decoder available for the video");
        return false;
    }
    m_dec = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(m_dec, m_stream->codecpar);
    if (avcodec_open2(m_dec, codec, nullptr) < 0) {
        if (error) *error = QObject::tr("Failed to open video decoder");
        return false;
    }

    m_fullRange = (m_stream->codecpar->color_range == AVCOL_RANGE_JPEG) ||
                  (m_stream->codecpar->format == AV_PIX_FMT_YUVJ420P);
    m_duration = (double)m_fmt->duration / AV_TIME_BASE;
    m_pkt = av_packet_alloc();
    m_frame = av_frame_alloc();
    return true;
}

void DecodeReader::close()
{
    if (m_pkt) av_packet_free(&m_pkt);
    if (m_frame) av_frame_free(&m_frame);
    if (m_dec) avcodec_free_context(&m_dec);
    if (m_fmt) avformat_close_input(&m_fmt);
    m_stream = nullptr;
    m_streamIdx = -1;
}

bool DecodeReader::readFrameAt(double targetSec, DecodedFrame *out)
{
    if (!isOpen()) return false;

    if (!m_seeked) {
        int64_t targetTs = (int64_t)(targetSec / av_q2d(m_stream->time_base));
        av_seek_frame(m_fmt, m_streamIdx, targetTs, AVSEEK_FLAG_BACKWARD);
        avcodec_flush_buffers(m_dec);
        m_seeked = true;
    }

    const double tb = av_q2d(m_stream->time_base);

    while (true) {
        int ret = av_read_frame(m_fmt, m_pkt);
        if (ret < 0)
            return false;  // EOF
        if (m_pkt->stream_index != m_streamIdx) {
            av_packet_unref(m_pkt);
            continue;
        }
        ret = avcodec_send_packet(m_dec, m_pkt);
        av_packet_unref(m_pkt);
        if (ret < 0)
            continue;

        while (true) {
            ret = avcodec_receive_frame(m_dec, m_frame);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0)
                break;

            double ts = m_frame->best_effort_timestamp * tb;
            if (ts >= 0.0 && ts >= targetSec) {
                out->width = m_frame->width;
                out->height = m_frame->height;
                out->yStride = m_frame->linesize[0];
                out->uStride = m_frame->linesize[1];
                out->vStride = m_frame->linesize[2];
                int ySize = out->yStride * out->height;
                int uSize = out->uStride * (out->height / 2);
                int vSize = out->vStride * (out->height / 2);
                out->yData.resize(ySize);
                out->uData.resize(uSize);
                out->vData.resize(vSize);
                memcpy(out->yData.data(), m_frame->data[0], ySize);
                memcpy(out->uData.data(), m_frame->data[1], uSize);
                memcpy(out->vData.data(), m_frame->data[2], vSize);
                out->timestamp = ts;
                av_frame_unref(m_frame);
                return true;
            }
            av_frame_unref(m_frame);
        }
    }
}

// ---------------------------------------------------------------------------
// H.264 / MPEG-4 encoder + MP4 muxer
// ---------------------------------------------------------------------------

class VideoWriter
{
public:
    VideoWriter() = default;
    ~VideoWriter();

    bool open(const QString &path, int w, int h, double fps, QString *error);
    bool writeFrame(const QImage &rgb, QString *error);
    bool finish(QString *error);

private:
    bool flushPackets(QString *error);

    AVFormatContext *m_fmt = nullptr;
    AVCodecContext *m_enc = nullptr;
    AVStream *m_stream = nullptr;
    SwsContext *m_conv = nullptr;
    AVFrame *m_yuv = nullptr;
    AVRational m_tb{1000, 30000};  // 1/fps in units of 1/1000s
    qint64 m_frameIdx = 0;
};

VideoWriter::~VideoWriter()
{
    if (m_conv) sws_freeContext(m_conv);
    if (m_yuv) av_frame_free(&m_yuv);
    if (m_enc) avcodec_free_context(&m_enc);
    if (m_fmt) {
        avio_closep(&m_fmt->pb);
        avformat_free_context(m_fmt);
    }
}

bool VideoWriter::open(const QString &path, int w, int h, double fps, QString *error)
{
    const int den = qMax(1, qRound(fps * 1000.0));
    m_tb = AVRational{1000, den};

    if (avformat_alloc_output_context2(&m_fmt, nullptr, nullptr, path.toUtf8().constData()) < 0 || !m_fmt) {
        if (error) *error = QObject::tr("Cannot determine output container from filename");
        return false;
    }

    const AVCodec *codec = avcodec_find_encoder_by_name("libx264");
    if (!codec)
        codec = avcodec_find_encoder(AV_CODEC_ID_MPEG4);
    if (!codec) {
        if (error) *error = QObject::tr("No video encoder available (libx264/mpeg4)");
        return false;
    }

    m_stream = avformat_new_stream(m_fmt, codec);
    m_enc = avcodec_alloc_context3(codec);
    m_enc->width = w;
    m_enc->height = h;
    m_enc->time_base = m_tb;
    m_enc->pix_fmt = AV_PIX_FMT_YUV420P;
    m_enc->gop_size = qMax(1, qRound(fps * 2.0));
    m_enc->max_b_frames = 0;  // keep the muxer simple (no reorder delay)

    if (codec->id == AV_CODEC_ID_H264) {
        m_enc->bit_rate = 10000000;
        av_opt_set(m_enc->priv_data, "preset", "veryfast", 0);
        av_opt_set(m_enc->priv_data, "crf", "19", 0);
        av_opt_set(m_enc->priv_data, "bframes", "0", 0);
    } else {
        m_enc->bit_rate = 8000000;
    }

    if (m_fmt->oformat->flags & AVFMT_GLOBALHEADER)
        m_enc->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    if (avcodec_open2(m_enc, codec, nullptr) < 0) {
        if (error) *error = QObject::tr("Failed to open video encoder");
        return false;
    }

    m_stream->time_base = m_tb;
    avcodec_parameters_from_context(m_stream->codecpar, m_enc);
    av_opt_set(m_fmt->priv_data, "movflags", "+faststart", 0);  // ignore if unsupported

    if (avio_open(&m_fmt->pb, path.toUtf8().constData(), AVIO_FLAG_WRITE) < 0) {
        if (error) *error = QObject::tr("Cannot write to output file");
        return false;
    }
    if (avformat_write_header(m_fmt, nullptr) < 0) {
        if (error) *error = QObject::tr("Failed to write output header");
        return false;
    }

    m_yuv = av_frame_alloc();
    m_yuv->format = AV_PIX_FMT_YUV420P;
    m_yuv->width = w;
    m_yuv->height = h;
    av_frame_get_buffer(m_yuv, 32);

    m_conv = sws_getContext(w, h, AV_PIX_FMT_RGB24, w, h, AV_PIX_FMT_YUV420P,
                            SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!m_conv) {
        if (error) *error = QObject::tr("Failed to create pixel converter");
        return false;
    }
    return true;
}

bool VideoWriter::writeFrame(const QImage &rgb, QString *error)
{
    if (rgb.format() != QImage::Format_RGB888)
        return false;

    uint8_t *srcData[4] = { const_cast<uchar *>(rgb.constBits()), nullptr, nullptr, nullptr };
    int srcStride[4] = { (int)rgb.bytesPerLine(), 0, 0, 0 };
    uint8_t *dstData[4] = { m_yuv->data[0], m_yuv->data[1], m_yuv->data[2], nullptr };
    int dstStride[4] = { m_yuv->linesize[0], m_yuv->linesize[1], m_yuv->linesize[2], 0 };
    av_frame_make_writable(m_yuv);
    sws_scale(m_conv, srcData, srcStride, 0, m_yuv->height, dstData, dstStride);

    // pts is in time_base ticks. With m_tb = {1000, round(fps*1000)} exactly
    // one tick elapses per frame, so the i-th frame sits at i/fps seconds.
    m_yuv->pts = m_frameIdx;

    int ret = avcodec_send_frame(m_enc, m_yuv);
    if (ret < 0) {
        if (error) *error = QObject::tr("Video encoder rejected a frame");
        return false;
    }
    m_frameIdx++;
    return flushPackets(error);
}

bool VideoWriter::flushPackets(QString *error)
{
    AVPacket *pkt = av_packet_alloc();
    while (true) {
        int ret = avcodec_receive_packet(m_enc, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            av_packet_free(&pkt);
            return true;
        }
        if (ret < 0) {
            av_packet_free(&pkt);
            if (error) *error = QObject::tr("Video encoder error");
            return false;
        }
        av_packet_rescale_ts(pkt, m_enc->time_base, m_stream->time_base);
        pkt->stream_index = m_stream->index;
        ret = av_interleaved_write_frame(m_fmt, pkt);
        av_packet_unref(pkt);
        if (ret < 0) {
            av_packet_free(&pkt);
            if (error) *error = QObject::tr("Failed to write video data");
            return false;
        }
    }
}

bool VideoWriter::finish(QString *error)
{
    // Drain the encoder.
    avcodec_send_frame(m_enc, nullptr);
    if (!flushPackets(error))
        return false;
    if (av_write_trailer(m_fmt) < 0) {
        if (error) *error = QObject::tr("Failed to finalize output file");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Exporter
// ---------------------------------------------------------------------------

Exporter::Exporter(QObject *parent)
    : QObject(parent)
{
}

Exporter::~Exporter()
{
    if (m_thread) {
        m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
    }
}

bool Exporter::beginExport()
{
    if (m_thread) {
        // The worker emits exportFinished/exportError just before its thread
        // unwinds, so when the GUI receives the signal the old thread may
        // still be finishing. Join it (it has already emitted its result, so
        // this returns almost immediately) before recycling the QThread.
        if (m_thread->isRunning())
            m_thread->wait();
        m_thread->deleteLater();
        m_thread = nullptr;
    }
    return true;
}

void Exporter::exportVideo(const QString &videoPath, const QString &outPath,
                           int width, int height, double fps,
                           double startTime, double endTime,
                           const StateProvider &state, bool useGpu)
{
    if (!beginExport())
        return;
    StateProvider sp = state;
    m_thread = QThread::create([this, videoPath, outPath, width, height, fps,
                                startTime, endTime, sp, useGpu]() {
        runVideo(videoPath, outPath, width, height, fps, startTime, endTime, sp, useGpu);
    });
    connect(m_thread, &QThread::finished, this, [this, th = m_thread]() {
        if (m_thread == th)
            m_thread = nullptr;
        th->deleteLater();
    });
    m_thread->start();
}

void Exporter::exportFrame(const QString &videoPath, const QString &outPath,
                           int width, int height, double time,
                           const ExportFrameState &state, bool useGpu)
{
    if (!beginExport())
        return;
    m_thread = QThread::create([this, videoPath, outPath, width, height, time, state, useGpu]() {
        runFrame(videoPath, outPath, width, height, time, state, useGpu);
    });
    connect(m_thread, &QThread::finished, this, [this, th = m_thread]() {
        if (m_thread == th)
            m_thread = nullptr;
        th->deleteLater();
    });
    m_thread->start();
}

void Exporter::runVideo(const QString &videoPath, const QString &outPath,
                        int width, int height, double fps,
                        double startTime, double endTime, StateProvider state,
                        bool useGpu)
{
    const int W = width & ~1;
    const int H = height & ~1;

    auto fail = [this, outPath](const QString &message) {
        QFile::remove(outPath);  // drop any partial output
        emit exportError(message);
    };

    if (fps <= 0.0 || W < 2 || H < 2) {
        fail(tr("Invalid export settings"));
        return;
    }

    // Try the GPU pipeline first; fall back to the CPU rasterizer when no
    // usable GL context can be made (headless CI, old GPUs, etc.).
    GpuRenderer gpu;
    QString gpuErr;
    const bool gpuReady = useGpu && gpu.initialize(&gpuErr);
    if (useGpu && !gpuReady)
        qWarning().noquote() << "GPU export unavailable, falling back to CPU:" << gpuErr;
    else if (gpuReady)
        qInfo().noquote() << "Export using GPU renderer";

    DecodeReader reader;
    QString err;
    if (!reader.open(videoPath, &err)) {
        fail(err);
        return;
    }

    VideoWriter writer;
    if (!writer.open(outPath, W, H, fps, &err)) {
        fail(err);
        return;
    }

    const double dur = reader.duration();
    double start = qMax(0.0, startTime);
    double end = (dur > 0.0) ? qMin(endTime, dur) : endTime;
    if (end <= start)
        end = start + 1.0 / fps;

    const int totalFrames = qMax(1, (int)qCeil((end - start) * fps));
    bool eof = false;
    int framesWritten = 0;

    for (int i = 0; i < totalFrames && !eof; ++i) {
        const double t = start + (double)i / fps;
        DecodedFrame frame;
        if (!reader.readFrameAt(t, &frame)) {
            eof = true;
            break;
        }
        const ExportFrameState s = state(t);
        QImage rendered;
        if (gpuReady) {
            if (!gpu.render(frame, s, W, H, &rendered, &err)) {
                fail(err);
                return;
            }
        } else {
            rendered = renderFrame(frame, s, W, H);
        }
        if (!writer.writeFrame(rendered, &err)) {
            fail(err);
            return;
        }
        framesWritten++;
        emit exportProgress((double)(i + 1) / (double)totalFrames);
    }

    // A range entirely past the end of the clip would otherwise produce an
    // empty, unplayable file — fail loudly instead.
    if (framesWritten == 0) {
        fail(tr("The requested time range is beyond the end of the video"));
        return;
    }

    if (!writer.finish(&err)) {
        fail(err);
        return;
    }
    emit exportFinished(outPath);
}

void Exporter::runFrame(const QString &videoPath, const QString &outPath,
                        int width, int height, double time, ExportFrameState state,
                        bool useGpu)
{
    const int W = width & ~1;
    const int H = height & ~1;
    if (W < 2 || H < 2) {
        emit exportError(tr("Invalid export size"));
        return;
    }

    GpuRenderer gpu;
    QString gpuErr;
    const bool gpuReady = useGpu && gpu.initialize(&gpuErr);
    if (useGpu && !gpuReady)
        qWarning().noquote() << "GPU frame export unavailable, falling back to CPU:" << gpuErr;

    DecodeReader reader;
    QString err;
    if (!reader.open(videoPath, &err)) {
        emit exportError(err);
        return;
    }
    DecodedFrame frame;
    if (!reader.readFrameAt(time, &frame)) {
        emit exportError(tr("Could not decode a frame at the requested time"));
        return;
    }
    QImage rendered;
    if (gpuReady) {
        if (!gpu.render(frame, state, W, H, &rendered, &err)) {
            emit exportError(err);
            return;
        }
    } else {
        rendered = renderFrame(frame, state, W, H);
    }
    if (!rendered.save(outPath)) {
        emit exportError(tr("Could not write image file"));
        return;
    }
    emit exportFinished(outPath);
}
