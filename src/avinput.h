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

#endif // AVINPUT_H
