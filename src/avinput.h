// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef AVINPUT_H
#define AVINPUT_H

#include <QString>

struct AVFormatContext;
struct AVIOContext;
class QFile;

// ---------------------------------------------------------------------------
// Opening a video for libavformat, from either a filesystem path or a URL.
//
// Android's file picker returns an opaque content:// URI rather than a path
// (and an app has no permission to read /sdcard by path anyway). libavformat
// has no "content" protocol, so handing it such a string simply fails. Qt does
// understand content:// (QAndroidContentFileEngine), so for a URL we let QFile
// do the reading and give libavformat a custom AVIOContext over it. Seeking is
// preserved, which MP4 needs: the moov atom is usually at the end of the file.
//
// This lives in one place because the project opens video in THREE independent
// decoders -- playback (VideoDecoder), AutoSync analysis
// (VisualRotationComputer) and export (DecodeReader). Fixing only the first is
// what left "Auto sync" failing with
// "VisualRotation: failed to open video: content://..." after playback had
// already started working.
// ---------------------------------------------------------------------------
struct AvInput {
    AVFormatContext *fmt = nullptr;

    // Returns 0 on success, or a negative AVERROR. On failure nothing is left
    // allocated. A plain path takes libavformat's own file protocol, exactly as
    // before; only a string containing "://" uses the QFile bridge.
    int open(const QString &path);

    // Safe to call more than once, and called by the destructor.
    void close();

    ~AvInput() { close(); }

    AvInput() = default;
    AvInput(const AvInput &) = delete;
    AvInput &operator=(const AvInput &) = delete;

private:
    AVIOContext *m_avio = nullptr;
    QFile *m_file = nullptr;
};

// Writing, for the same reason: on Android the only place the app may write is
// a document in the granted folder, addressed by a content:// URI, and
// avio_open() cannot open one. MP4 muxing needs a SEEKABLE sink (it rewrites
// the header once the moov atom size is known), so the QFile bridge provides
// seek as well as write; open() reports whether it got one, and the caller can
// switch to a fragmented layout if not.
struct AvOutput {
    AVFormatContext *fmt = nullptr;

    // shortName is the muxer name, e.g. "mp4". Returns 0 on success or a
    // negative AVERROR.
    int open(const QString &path, const char *shortName);

    // False when the sink could not be seeked (write-only stream). MP4 then
    // needs frag_keyframe+empty_moov to be playable.
    bool seekable() const { return m_seekable; }

    // True when writing goes through the QFile bridge rather than
    // libavformat's own file protocol. Seeking a content:// document can
    // SUCCEED on the probe and still not behave well enough for the MP4 muxer
    // to rewind and patch its header, so callers should treat custom I/O as
    // non-rewindable and mux fragmented.
    bool isCustomIo() const { return m_avio != nullptr; }

    void close();

    ~AvOutput() { close(); }

    AvOutput() = default;
    AvOutput(const AvOutput &) = delete;
    AvOutput &operator=(const AvOutput &) = delete;

private:
    AVIOContext *m_avio = nullptr;
    QFile *m_file = nullptr;
    bool m_seekable = true;
    bool m_ownsAvioOpen = false;   // true when avio_open() was used (plain path)
};

#endif // AVINPUT_H
