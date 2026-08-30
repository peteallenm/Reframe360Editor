#ifndef VIDEODECODER_H
#define VIDEODECODER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QByteArray>
#include "avinput.h"
#include "grade.h"
#include <QMetaType>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
}

struct DecodedFrame {
    QByteArray yData;
    QByteArray uData;
    QByteArray vData;
    int width;
    int height;
    int yStride;
    int uStride;
    int vStride;
    double timestamp;
};

Q_DECLARE_METATYPE(DecodedFrame)   // queued to the optical-flow worker

// 256-bin histogram of the decoded SOURCE frame (before the tone curves and
// the rest of the colour grade -- it is what you grade against, not the
// result). Counts are raw; the consumer decides how to normalise.
struct FrameHistogram {
    quint32 r[256] = {};
    quint32 g[256] = {};
    quint32 b[256] = {};
    quint32 luma[256] = {};
    quint64 samples = 0;
};

class VideoDecoder : public QObject
{
    Q_OBJECT
public:
    explicit VideoDecoder(QObject *parent = nullptr);
    ~VideoDecoder();

    void loadVideo(const QString &path);
    void startPlayback();
    void stopPlayback();
    void seekTo(double time);
    double duration() const;
    double currentTime() const;

    DecodedFrame currentFrame() const;
    bool hasFrame() const;

    // Histogram the current frame in place. Done here rather than in the
    // caller because currentFrame() returns by value: at 2880x5760 that is a
    // ~25 MB copy, and a histogram needs to read the planes, not own them.
    //
    // The frame is subsampled to at most maxSamples on each axis (the result
    // is a distribution, so a regular subsample of a few 100k pixels is
    // indistinguishable from the full 16 M and costs ~1 ms instead of ~50).
    //
    // circularMask skips the corners outside the two inscribed fisheye
    // circles. Without it roughly a fifth of a YI 360 frame is the black
    // surround, which lands as a false spike in bin 0 and squashes everything
    // else. Only applied when the frame really is a 1:2 stacked dual-fisheye.
    // When grade is given, *graded* is filled with the histogram of the same
    // pixels put through the colour grade (and the tone-curve LUT, if given) --
    // i.e. what the viewer is actually showing. Both histograms come out of a
    // single pass over the frame.
    bool sampleHistogram(FrameHistogram *source,
                         FrameHistogram *graded = nullptr,
                         const GradeParams *grade = nullptr,
                         const unsigned char *curveLut = nullptr,
                         int maxSamples = 512,
                         bool circularMask = true) const;
    int videoWidth() const { return m_videoWidth; }
    int videoHeight() const { return m_videoHeight; }
    bool isFullRange() const { return m_isFullRange; }

signals:
    void durationChanged();
    void currentTimeChanged();
    void frameReady();
    void errorOccurred(const QString &message);

private slots:
    void decodeLoop();

private:
    void closeInput();

private:
    enum class DecodeResult { Frame, Eof, NoData };
    DecodeResult decodeFrame();

    AVFormatContext *m_formatCtx;

    AvInput m_input;   // owns m_formatCtx (and the QFile bridge for URLs)

    // Custom I/O, used when the path is a URL rather than a filesystem path

    // (Android's picker returns content://). Owned here so it can be torn down

    // in the same places the format context is.

    class QFile *m_ioFile = nullptr;

    AVIOContext *m_avio = nullptr;
    AVCodecContext *m_codecCtx;
    AVStream *m_videoStream;
    int m_videoStreamIndex;

    QThread *m_decodeThread;
    bool m_running;
    bool m_playing;
    double m_duration;
    double m_currentTime;
    double m_startTime;
    // Pending seek destination (seconds), or < 0 when not seeking. While set,
    // the decode loop runs forward from the keyframe and only publishes
    // currentTimeChanged once a decoded frame reaches/passes this time, so the
    // UI shows the actual frame instead of the idealized target.
    double m_targetTime;

    int m_videoWidth;
    int m_videoHeight;
    bool m_isFullRange;

    mutable QMutex m_mutex;
    DecodedFrame m_currentFrame;
    bool m_hasFrame;

    QMutex m_ffmpegMutex;
};

#endif // VIDEODECODER_H
