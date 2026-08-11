#ifndef VIDEODECODER_H
#define VIDEODECODER_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <QByteArray>
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
    enum class DecodeResult { Frame, Eof, NoData };
    DecodeResult decodeFrame();

    AVFormatContext *m_formatCtx;
    AVCodecContext *m_codecCtx;
    AVStream *m_videoStream;
    int m_videoStreamIndex;

    QThread *m_decodeThread;
    bool m_running;
    bool m_playing;
    double m_duration;
    double m_currentTime;
    double m_startTime;

    int m_videoWidth;
    int m_videoHeight;
    bool m_isFullRange;

    mutable QMutex m_mutex;
    DecodedFrame m_currentFrame;
    bool m_hasFrame;

    QMutex m_ffmpegMutex;
};

#endif // VIDEODECODER_H
