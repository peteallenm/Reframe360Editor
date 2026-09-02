// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

// Follows an object the user pointed at, frame by frame, on a background
// thread.
//
// It tracks on a STABILISED, RECTIFIED TILE: a small gnomonic patch resampled
// around where the object is predicted to be, built in the inertial frame
// using the camera's own orientation. That removes camera motion before
// matching, which earns four things at once --
//   * the search window covers what the OBJECT did (a few degrees), not what
//     a handheld camera did (100+ deg/s),
//   * the template stays valid anywhere on the sphere instead of only near
//     where it was taken, because the tile is rectified,
//   * apparent size in the tile is a true angular size; measured in raw
//     fisheye pixels it would be lens distortion masquerading as zoom,
//   * the lens seam becomes an explicit handled event rather than silent
//     template corruption.
//
// Matching is cv::matchTemplate with TM_CCOEFF_NORMED (mean-subtracted, so
// exposure changes do not matter) at three scales, with a parabola fit for
// continuous scale. That needs only OpenCV's imgproc, which both platforms
// already link; TrackerCSRT/KCF are opencv_contrib and absent from the
// Android SDK.

#include "track.h"

#include <QAtomicInt>
#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QVector>
#include <QSharedPointer>
#include <QVector3D>

class QThread;

// The per-frame tracking step, decoupled from decoding so it can be driven
// with synthetic frames in tests (which is the only way to have ground truth:
// a known world direction under a known camera motion).
class TileTracker
{
public:
    struct Config {
        TrackLenses lenses;
        QVector3D seedDirCam;         // object bearing at the seed frame
        double seedRadiusRad = 0.02;
        double maxWorldSpeedDeg = 400.0;
    };
    enum class Step { Ok, Rejected, Lost };

    // Seeds the template from this frame. False (with `error`) when the patch
    // has too little contrast to track at all.
    bool begin(const Config &cfg, const uchar *y, int w, int h, int stride,
               const QQuaternion &camOrientation, double t, QString *error);

    Step step(const uchar *y, int w, int h, int stride,
              const QQuaternion &camOrientation, double t);

    // Put the tracker back on a supplied direction (used to retry a patch that
    // failed, from where the surviving patches say it should be).
    void setWorldDir(const QVector3D &dir) { m_dirWorld = dir.normalized(); m_omegaRate = 0.0; }
    void clearRuns() { m_badRun = m_rejectRun = m_hardRun = m_ambiguousRun = 0; }

    const TrackSample &lastSample() const { return m_last; }
    QVector3D worldDir() const { return m_dirWorld; }
    QString lossReason() const { return m_lossReason; }
    double lastScore() const { return m_lastScore; }
    double scaleRatio() const;
    int reseedCount() const { return m_reseeds; }

private:
    struct Impl;
    Config m_cfg;
    TrackSample m_last;
    QVector3D m_dirWorld;
    QVector3D m_omegaAxis{0.0f, 1.0f, 0.0f};
    double m_omegaRate = 0.0;
    double m_scaleLog = 0.0;
    double m_pDeg = 0.25;
    double m_searchHalfDeg = 3.0;   // angular search radius, held constant
    int m_tileSize = 96;            // ... by resizing the tile each frame
    int m_badRun = 0;
    int m_hardRun = 0;
    int m_ambiguousRun = 0;
    int m_reseeds = 0;
    double m_lastReseedTime = -1e9;
    int m_rejectRun = 0;
    int m_frames = 0;
    double m_prevTime = 0.0;
    double m_lastScore = 0.0;
    bool m_preferRear = false;
    QString m_lossReason;
    // The template, held behind a pointer so OpenCV stays out of this header.
    struct AnchorHolder;
    QSharedPointer<AnchorHolder> m_anchorHolder;
    void recordSample(const QQuaternion &camOrientation, double t, double score);
};

// What to track, and everything needed to do it without touching live state.
struct TrackRequest {
    QString videoPath;
    QString proxyOverride;        // Android: the _thm the user granted
    TrackLenses lenses;

    double t0 = 0.0;              // arm time
    double tEnd = 0.0;            // stop here (next keyframe / trim-out / end)
    // One or more bearings on the SAME subject, camera frame at t0 -- e.g. a
    // head and a shirt. Each is followed independently and the best-matching
    // one drives the view, so a patch that fails does not end the track; a
    // failed one is retried from where the others say it should be.
    QVector<QVector3D> seedDirsCam;
    double seedRadiusRad = 0.02;  // angular radius of each patch

    // The camera's own orientation, pre-sampled on the GUI thread so the
    // worker never reads the integrator. Unsmoothed and full-bandwidth: the
    // point is to remove ALL camera motion from the tile, where
    // imuOrientationAt deliberately leaves intentional pan in.
    QVector<QQuaternion> camOrientations;
    QVector<double> camTimes;

    double fps = 30.0;
    // Object angular-speed ceiling. The earlier 150 was justified by measured
    // world motion "~30 deg/s median, p95 near 110" -- but that measurement was
    // taken from tracks the gate had already truncated, so it only ever
    // confirmed itself. Gating the whole step caps the trackable speed at
    // roughly this minus the search window per frame; 400 leaves room for a
    // bullet-time orbit at 250-350 deg/s.
    double maxWorldSpeedDeg = 400.0;
};

struct TrackResult {
    QVector<TrackSample> samples;
    bool lost = false;
    double lossTime = -1.0;
    QString lossReason;
    // Diagnostics, printed by --track-stats and useful when a track goes wrong.
    double meanScore = 0.0;
    double msPerFrame = 0.0;
    int pointsSeeded = 0;         // how many of the requested patches took
    int pointsSurviving = 0;      // ... and how many were still alive at the end
    int revivals = 0;             // times a failed patch was picked back up
};

class ObjectTracker : public QObject
{
    Q_OBJECT
public:
    explicit ObjectTracker(QObject *parent = nullptr);
    ~ObjectTracker() override;

    // Starts a worker thread. One at a time; a second call while running is
    // ignored.
    void track(const TrackRequest &req);
    bool isRunning() const { return m_running.loadRelaxed() != 0; }

    // Safe from any thread. The pass stops at the next frame and reports what
    // it found so far -- a partial track is still useful.
    void cancel() { m_cancel.storeRelaxed(1); }

signals:
    void progressChanged(double fraction, const QString &status);
    void trackFinished(const TrackResult &result);
    void trackFailed(const QString &message);

private:
    void run(TrackRequest req);

    QAtomicInt m_cancel{0};
    QAtomicInt m_running{0};
    QThread *m_thread = nullptr;   // kept, and waited on, unlike AutoSync's
};

#endif // OBJECTTRACKER_H
