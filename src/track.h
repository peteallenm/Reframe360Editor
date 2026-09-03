// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef TRACK_H
#define TRACK_H

// An object track: what the tracker measured, and how that becomes a view.
//
// A track does not store a view. It stores where the object was ON THE SENSOR,
// frame by frame, and resolves into ordinary keyframes on demand -- the
// "virtual keyframes" a user would otherwise place by hand. Everything
// downstream (preview, export, the timeline) then goes through
// KeyframeModel::interpolate exactly as it always has, so there is only ever
// one definition of the view at a given time and preview and export cannot
// drift apart.
//
// Samples are stored as raw half-frame fisheye coordinates rather than world
// directions on purpose. Auto sync, a smoothing change, a re-calibration or a
// gyro solve all rewrite App::imuOrientationAt() at every time; a stored world
// direction would be silently wrong afterwards, while a stored measurement
// simply resolves through the corrected chain. Better stabilisation improves
// an old track instead of invalidating it.

#include "keyframe.h"
#include "visualrotation.h"

#include <QtGlobal>
#include <QJsonArray>
#include <QQuaternion>
#include <QString>
#include <QVector>
#include <functional>

// One measurement, in the frame the tracker actually saw.
struct TrackSample {
    double t = 0.0;          // frame PTS
    double u = 0.5, v = 0.5; // normalised HALF-frame fisheye coords
    int lens = 0;            // 0 = front (top half), 1 = rear (bottom half)
    // The subject's angular half-extent, in RADIANS, as measured in the camera
    // frame. Deliberately not a distance in uv: the fisheye map is anisotropic
    // and non-uniform, so a magnitude written along one direction and read back
    // along the u and v axes comes out up to 19 % wrong, varying with where on
    // the lens the subject happens to be. That turned a pan into a slow zoom.
    double halfW = 0.0;
    double halfH = 0.0;
    double conf = 1.0;       // 0..1 match confidence
};

struct Track {
    QString id;

    double t0 = 0.0;         // armed here
    bool lost = false;       // samples stop because the tracker lost it
    double lossTime = -1.0;

    // Framing: where in the frame the object sat when picked. Stored as NDC
    // plus the aspect and projection it was picked in, so resolving is a pure
    // function of the track -- never of the live viewer pane or the export
    // size, which would make preview and export disagree.
    double framingNdcX = 0.0, framingNdcY = 0.0;
    double framingAspect = 16.0 / 9.0;
    int framingProjection = 0;

    // The view at arm time. yaw0/pitch0 seed the pitch branch and let the
    // resolver assert that it reproduces the armed framing exactly.
    double yaw0 = 0.0, pitch0 = 0.0, roll0 = 0.0, fov0 = 90.0;
    double sizeRad0 = 0.0;   // apparent angular diameter when picked

    // How hard fov chases the subject's size. Defaults are the filter that
    // stops it pumping; see resolve().
    bool fovFollow = true;
    double fovSmoothSec = 0.75;
    double fovDeadband = 0.06;        // 6 %: below this, fov does not move
    double fovMaxRatePerSec = 0.22;   // ~25 %/s
    // Clamp to fov0 * [1/range, range]. Deliberately tight: the size estimate
    // has a small systematic bias, and the deadband below is hysteresis rather
    // than a bound, so a slow drift walks straight through it. This is what
    // stops that becoming "it just keeps zooming in".
    double fovRange = 1.5;

    QVector<TrackSample> samples;     // sorted by t
    // False for tracks read from a "packed14v1" sidecar, whose halfW/halfH are
    // distances in uv rather than angles. Those resolve through the old path so
    // an existing track keeps behaving exactly as it did.
    bool sizeIsAngular = true;

    // How much of the measured span actually drives the view. Trimming is
    // recorded here and NEVER applied to `samples`: dragging a handle back out
    // has to restore what was there, and those measurements cost a decode pass
    // to make. Negative means untrimmed -- t0 and the last sample stand.
    double trimIn = -1.0;
    double trimOut = -1.0;

    double endTime() const { return samples.isEmpty() ? t0 : samples.constLast().t; }
    // The part of the track in use. Both are clamped to the measured span, so
    // a stale trim from an edited sidecar can never widen a track past what
    // was actually tracked.
    double activeStart() const
    { return trimIn >= 0.0 ? qBound(t0, trimIn, qMax(t0, endTime())) : t0; }
    double activeEnd() const
    { return trimOut >= 0.0 ? qBound(t0, trimOut, qMax(t0, endTime())) : endTime(); }
    // True only while the track still runs to where the tracker gave up. Trim
    // the end back and it no longer "ran out" -- it stops because you said so,
    // and neither the timeline nor the hold-and-ease should claim otherwise.
    bool endsAtLoss() const { return lost && activeEnd() >= endTime() - 1e-6; }
    bool operator==(const Track &o) const;
    bool operator!=(const Track &o) const { return !(*this == o); }
};

// The two lenses' geometry, as the solver's back-projection wants it.
struct TrackLenses {
    VisualRotationComputer::LensParams front;
    VisualRotationComputer::LensParams rear;
    static TrackLenses fromCalibration(double fCx, double fCy, double fR, double fK1,
                                       double fK2, double fRot, bool fFlip,
                                       double rCx, double rCy, double rR, double rK1,
                                       double rK2, double rRot, bool rFlip);
};

namespace track {

// Stabilisation quaternion at a time: App::imuOrientationAt, or a stand-in in
// tests. `world = q * cameraBearing`.
using OrientationFn = std::function<QQuaternion(double)>;

// One track -> the keyframes it implies over [t0, tEnd].
QVector<Keyframe> resolve(const Track &tr, const TrackLenses &lenses,
                          const OrientationFn &imuAt, double tEnd);

// Manual keyframes + every track -> the array the app actually interpolates.
// A track runs from where it was armed until the first manual keyframe after
// it, the next track, its own loss, or trimOut -- whichever comes first. A
// manual keyframe always wins.
QVector<Keyframe> compose(const QVector<Keyframe> &manual, const QVector<Track> &tracks,
                          const TrackLenses &lenses, const OrientationFn &imuAt,
                          double trimOut, double duration);

QJsonArray toJson(const QVector<Track> &tracks);
QVector<Track> fromJson(const QJsonArray &arr);

} // namespace track

#endif // TRACK_H
