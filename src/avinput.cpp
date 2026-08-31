// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

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

// --- writing ---------------------------------------------------------------

namespace {

// Non-const buf: that is the signature libavformat 60 (FFmpeg 6.x) expects.
int qfileWrite(void *opaque, uint8_t *buf, int bufSize)
{
    QFile *f = static_cast<QFile *>(opaque);
    const qint64 n = f->write(reinterpret_cast<const char *>(buf), bufSize);
    return n < 0 ? AVERROR(EIO) : int(n);
}

} // namespace

int AvOutput::open(const QString &path, const char *shortName)
{
    close();

    int ret = avformat_alloc_output_context2(&fmt, nullptr, shortName,
                                             path.contains(QStringLiteral("://"))
                                                 ? nullptr : path.toUtf8().constData());
    if (ret < 0 || !fmt)
        return ret < 0 ? ret : AVERROR_UNKNOWN;

    if (!path.contains(QStringLiteral("://"))) {
        ret = avio_open(&fmt->pb, path.toUtf8().constData(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            close();
            return ret;
        }
        m_ownsAvioOpen = true;
        m_seekable = true;
        return 0;
    }

    m_file = new QFile(path);
    if (!m_file->open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        delete m_file;
        m_file = nullptr;
        close();
        return AVERROR(EACCES);
    }

    const int kIoBuf = 64 * 1024;
    unsigned char *buf = static_cast<unsigned char *>(av_malloc(kIoBuf));
    if (!buf) {
        close();
        return AVERROR(ENOMEM);
    }
    // write_flag = 1, and a seek callback: the MP4 muxer rewinds to patch the
    // header. QFile over a content:// document is seekable when the provider
    // gave a real file descriptor, which the Storage Access Framework does for
    // documents on local storage.
    m_avio = avio_alloc_context(buf, kIoBuf, 1, m_file, nullptr, qfileWrite, qfileSeek);
    if (!m_avio) {
        av_free(buf);
        close();
        return AVERROR(ENOMEM);
    }
    m_seekable = m_file->seek(0);
    m_avio->seekable = m_seekable ? AVIO_SEEKABLE_NORMAL : 0;
    fmt->pb = m_avio;
    fmt->flags |= AVFMT_FLAG_CUSTOM_IO;
    return 0;
}

void AvOutput::close()
{
    if (fmt) {
        if (m_ownsAvioOpen && fmt->pb)
            avio_closep(&fmt->pb);
        avformat_free_context(fmt);
        fmt = nullptr;
    }
    if (m_avio) {
        av_freep(&m_avio->buffer);
        avio_context_free(&m_avio);
    }
    delete m_file;
    m_file = nullptr;
    m_ownsAvioOpen = false;
    m_seekable = true;
}
