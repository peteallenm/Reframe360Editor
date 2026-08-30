#include "avinput.h"

#include <QFile>

extern "C" {
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/mem.h>
#include <libavutil/error.h>
}

namespace {

int qfileRead(void *opaque, uint8_t *buf, int bufSize)
{
    QFile *f = static_cast<QFile *>(opaque);
    const qint64 n = f->read(reinterpret_cast<char *>(buf), bufSize);
    if (n < 0)
        return AVERROR(EIO);
    if (n == 0)
        return AVERROR_EOF;
    return int(n);
}

int64_t qfileSeek(void *opaque, int64_t offset, int whence)
{
    QFile *f = static_cast<QFile *>(opaque);
    if (whence == AVSEEK_SIZE)
        return f->size();
    whence &= ~AVSEEK_FORCE;
    qint64 target;
    switch (whence) {
    case SEEK_SET: target = offset; break;
    case SEEK_CUR: target = f->pos() + offset; break;
    case SEEK_END: target = f->size() + offset; break;
    default: return AVERROR(EINVAL);
    }
    if (target < 0 || !f->seek(target))
        return AVERROR(EIO);
    return f->pos();
}

} // namespace

int AvInput::open(const QString &path)
{
    close();

    if (!path.contains(QStringLiteral("://")))
        return avformat_open_input(&fmt, path.toUtf8().constData(), nullptr, nullptr);

    m_file = new QFile(path);
    if (!m_file->open(QIODevice::ReadOnly)) {
        delete m_file;
        m_file = nullptr;
        return AVERROR(ENOENT);
    }

    const int kIoBuf = 64 * 1024;
    unsigned char *buf = static_cast<unsigned char *>(av_malloc(kIoBuf));
    if (!buf) {
        close();
        return AVERROR(ENOMEM);
    }
    m_avio = avio_alloc_context(buf, kIoBuf, 0, m_file, qfileRead, nullptr, qfileSeek);
    if (!m_avio) {
        av_free(buf);
        close();
        return AVERROR(ENOMEM);
    }

    fmt = avformat_alloc_context();
    fmt->pb = m_avio;
    fmt->flags |= AVFMT_FLAG_CUSTOM_IO;
    const int ret = avformat_open_input(&fmt, nullptr, nullptr, nullptr);
    if (ret < 0)
        close();
    return ret;
}

void AvInput::close()
{
    if (fmt)
        avformat_close_input(&fmt);
    if (m_avio) {
        // With AVFMT_FLAG_CUSTOM_IO libavformat frees neither the AVIOContext
        // nor the buffer we gave it.
        av_freep(&m_avio->buffer);
        avio_context_free(&m_avio);
    }
    delete m_file;
    m_file = nullptr;
}
