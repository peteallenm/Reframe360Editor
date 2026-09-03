// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include "track.h"
#include "projection.h"

#include <QByteArray>
#include <QJsonObject>
#include <QtEndian>
#include <algorithm>
#include <cmath>

namespace {

constexpr double kPi = proj::kPi;

double wrap180(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

double angleBetween(const QVector3D &a, const QVector3D &b)
{
    // atan2(|a x b|, a.b), not acos(a.b): these are float vectors and acos
    // near 1 amplifies float epsilon by a square root (0.028 deg of error
    // that is not there).
    const QVector3D an = a.normalized(), bn = b.normalized();
    const double dot = QVector3D::dotProduct(an, bn);
    const double cross = QVector3D::crossProduct(an, bn).length();
    return std::atan2(cross, dot);
}

// --- the fov filter -------------------------------------------------------
// Five stages over the WHOLE array, in log space so zoom in and out are
// symmetric. Offline, so every stage is zero-phase: a causal filter would lag
// the subject, and would make a backward scrub in the preview disagree with a
// forward export over the same frames.
// How far a single frame may sit off the path its neighbours describe before
// it is treated as a bad match rather than as motion. Measured on real tracks
// the detour is 0.03-0.24 deg typically, with a tail to 9.6 deg -- that tail is
// the flicking.
constexpr double kDespikeDeg = 1.0;

double angleBetweenDeg(const QVector3D &a, const QVector3D &b)
{
    return std::atan2((double)QVector3D::crossProduct(a, b).length(),
                      (double)QVector3D::dotProduct(a, b)) * 180.0 / kPi;
}

// Replace a frame that jumps off the path and straight back with the midpoint
// of its neighbours.
//
// A median, not an average, because this is impulse noise: averaging smears one
// bad frame across its neighbours instead of removing it. And a midpoint rather
// than picking a neighbour outright, because the subject is usually moving --
// on a straight run the midpoint IS the missing sample, so a 250 deg/s sweep
// passes through untouched while the spike on top of it does not.
QVector<QVector3D> despike(const QVector<QVector3D> &in)
{
    if (in.size() < 3) return in;
    QVector<QVector3D> out = in;
    for (int i = 1; i + 1 < in.size(); ++i) {
        // Read neighbours from the INPUT, so a run of bad frames cannot drag
        // the correction along behind it.
        const QVector3D &a = in[i - 1], &b = in[i], &c = in[i + 1];
        const double detour = angleBetweenDeg(a, b) + angleBetweenDeg(b, c)
                            - angleBetweenDeg(a, c);
        if (detour > kDespikeDeg)
            out[i] = (a + c).normalized();
    }
    return out;
}

// Zero-phase Gaussian over the directions themselves. Averaging unit vectors
// and renormalising keeps a straight sweep straight -- for symmetric weights
// the average lands on the bisector, which is the point being smoothed -- so
// this takes the jitter off without dragging the subject behind the motion.
QVector<QVector3D> smoothDirs(const QVector<QVector3D> &in, const QVector<double> &times,
                              double sigmaSec)
{
    const int n = in.size();
    if (n < 3 || sigmaSec <= 1e-4) return in;
    QVector<QVector3D> out(n);
    for (int i = 0; i < n; ++i) {
        // The window has to stay SYMMETRIC about i, so it is narrowed at the
        // ends rather than truncated. A one-sided window on a moving subject
        // averages only what is behind (or ahead) of it and drags the result
        // that way -- worth 8 deg at the start of a 120 deg/s sweep, which is
        // a track that begins visibly off its subject. Narrowing keeps every
        // window balanced, so a straight sweep passes through untouched and
        // smoothing simply eases off over the last few frames.
        const int room = qMin(i, n - 1 - i);
        QVector3D acc(0, 0, 0);
        double wsum = 0.0;
        for (int j = i - room; j <= i + room; ++j) {
            const double dt = times[j] - times[i];
            if (std::fabs(dt) > 3.0 * sigmaSec) continue;
            const double w = std::exp(-0.5 * (dt * dt) / (sigmaSec * sigmaSec));
            acc += in[j] * (float)w;
            wsum += w;
        }
        out[i] = (wsum > 0.0 && acc.lengthSquared() > 1e-12) ? acc.normalized() : in[i];
    }
    return out;
}

QVector<double> filterFov(const QVector<double> &rawFov, const QVector<double> &times,
                          const Track &tr)
{
    const int n = rawFov.size();
    QVector<double> out(n, tr.fov0);
    if (n == 0) return out;

    QVector<double> logFov(n);
    for (int i = 0; i < n; ++i)
        logFov[i] = std::log(qBound(10.0, rawFov[i], 170.0));

    // 2. Zero-phase Gaussian over time.
    const double sigma = qMax(1e-3, tr.fovSmoothSec);
    QVector<double> sm(n);
    for (int i = 0; i < n; ++i) {
        double acc = 0.0, wsum = 0.0;
        for (int j = 0; j < n; ++j) {
            const double dt = times[j] - times[i];
            if (std::fabs(dt) > 3.0 * sigma) continue;
            const double w = std::exp(-0.5 * (dt * dt) / (sigma * sigma));
            acc += w * logFov[j];
            wsum += w;
        }
        sm[i] = wsum > 0.0 ? acc / wsum : logFov[i];
    }

    // 3. Deadband: hold until the subject has really changed size. This is
    //    what stops fov pumping -- inside the band the output does not move
    //    at all, instead of moving a little and being smeared by (4).
    const double band = std::log(1.0 + qMax(0.0, tr.fovDeadband));
    QVector<double> held(n);
    double cur = sm[0];
    for (int i = 0; i < n; ++i) {
        if (std::fabs(sm[i] - cur) > band)
            cur = sm[i] - (sm[i] > cur ? band : -band);
        held[i] = cur;
    }

    // 4. Rate limit, forward then backward so it stays zero-phase.
    const double maxRate = qMax(1e-6, tr.fovMaxRatePerSec);
    for (int i = 1; i < n; ++i) {
        const double dt = qMax(1e-6, times[i] - times[i - 1]);
        held[i] = qBound(held[i - 1] - maxRate * dt, held[i], held[i - 1] + maxRate * dt);
    }
    for (int i = n - 2; i >= 0; --i) {
        const double dt = qMax(1e-6, times[i + 1] - times[i]);
        held[i] = qBound(held[i + 1] - maxRate * dt, held[i], held[i + 1] + maxRate * dt);
    }

    // 5. Clamp to a window around the armed fov, then to the UI's range.
    const double lo = std::log(tr.fov0 / qMax(1.0, tr.fovRange));
    const double hi = std::log(tr.fov0 * qMax(1.0, tr.fovRange));
    for (int i = 0; i < n; ++i)
        out[i] = qBound(10.0, std::exp(qBound(lo, held[i], hi)), 170.0);
    return out;
}

// --- decimation -----------------------------------------------------------
// KeyframeModel::interpolate is a linear scan run once per preview frame AND
// once per exported frame, so handing it one keyframe per video frame would
// cost billions of comparisons a minute. Douglas-Peucker keeps only the
// keyframes the motion actually needs: a smooth pan collapses to a handful.
double devianceFrom(const Keyframe &a, const Keyframe &b, const Keyframe &p)
{
    const double span = b.time - a.time;
    const double f = (span > 1e-9) ? (p.time - a.time) / span : 0.0;
    auto lerpAngle = [f](double x, double y) {
        double d = wrap180(y - x);
        return x + d * f;
    };
    const double dy = std::fabs(wrap180(p.yaw - lerpAngle(a.yaw, b.yaw)));
    const double dp = std::fabs(p.pitch - (a.pitch + (b.pitch - a.pitch) * f));
    const double df = std::fabs(p.fov - (a.fov + (b.fov - a.fov) * f));
    // Angles in degrees; fov weighted looser because it moves slowly and a
    // small fov error is far less visible than a framing error.
    return qMax(qMax(dy, dp), df * 0.25);
}

void douglasPeucker(const QVector<Keyframe> &pts, int first, int last,
                    double tol, QVector<bool> &keep)
{
    if (last <= first + 1) return;
    int worst = -1;
    double worstDev = 0.0;
    for (int i = first + 1; i < last; ++i) {
        const double d = devianceFrom(pts[first], pts[last], pts[i]);
        if (d > worstDev) { worstDev = d; worst = i; }
    }
    if (worst < 0 || worstDev <= tol) return;
    keep[worst] = true;
    douglasPeucker(pts, first, worst, tol, keep);
    douglasPeucker(pts, worst, last, tol, keep);
}

QVector<Keyframe> decimate(const QVector<Keyframe> &in, double tolDeg)
{
    if (in.size() <= 2) return in;
    QVector<bool> keep(in.size(), false);
    keep.first() = keep.last() = true;
    douglasPeucker(in, 0, in.size() - 1, tolDeg, keep);
    QVector<Keyframe> out;
    out.reserve(in.size() / 4 + 2);
    for (int i = 0; i < in.size(); ++i)
        if (keep[i]) out.append(in[i]);
    return out;
}

} // namespace

bool Track::operator==(const Track &o) const
{
    if (id != o.id || lost != o.lost || samples.size() != o.samples.size())
        return false;
    if (!qFuzzyCompare(t0 + 1.0, o.t0 + 1.0)) return false;
    // Trims are part of what a track IS: without them here, keyframe.cpp's
    // no-op-load early return treats a reload as "nothing changed" and quietly
    // keeps the trims from whichever clip was open before.
    if (!qFuzzyCompare(trimIn + 1.0, o.trimIn + 1.0)
        || !qFuzzyCompare(trimOut + 1.0, o.trimOut + 1.0)
        || !qFuzzyCompare(posSmoothSec + 1.0, o.posSmoothSec + 1.0))
        return false;
    if (!qFuzzyCompare(framingNdcX + 1.0, o.framingNdcX + 1.0)
        || !qFuzzyCompare(framingNdcY + 1.0, o.framingNdcY + 1.0)
        || framingProjection != o.framingProjection)
        return false;
    if (!qFuzzyCompare(fov0 + 1.0, o.fov0 + 1.0) || !qFuzzyCompare(roll0 + 1.0, o.roll0 + 1.0))
        return false;
    for (int i = 0; i < samples.size(); ++i) {
        const TrackSample &a = samples[i], &b = o.samples[i];
        if (!qFuzzyCompare(a.t + 1.0, b.t + 1.0) || a.lens != b.lens
            || !qFuzzyCompare(a.u + 1.0, b.u + 1.0) || !qFuzzyCompare(a.v + 1.0, b.v + 1.0))
            return false;
    }
    return true;
}

TrackLenses TrackLenses::fromCalibration(double fCx, double fCy, double fR, double fK1,
                                         double fK2, double fRot, bool fFlip,
                                         double rCx, double rCy, double rR, double rK1,
                                         double rK2, double rRot, bool rFlip)
{
    TrackLenses l;
    l.front.cx = fCx; l.front.cy = fCy; l.front.radius = fR;
    l.front.k1 = fK1; l.front.k2 = fK2; l.front.rotation = fRot;
    l.front.hflip = fFlip; l.front.isRear = false;
    // mirrorAzimuth stays FALSE: it is a solver-only convention for pooling
    // front and rear correspondences into one rotation fit. project.frag uses
    // the same phi for both halves, and pixelToBearing is documented as the
    // exact inverse of the shader. Setting it here mirrors the rear lens.
    l.front.mirrorAzimuth = false;
    l.rear.cx = rCx; l.rear.cy = rCy; l.rear.radius = rR;
    l.rear.k1 = rK1; l.rear.k2 = rK2; l.rear.rotation = rRot;
    l.rear.hflip = rFlip; l.rear.isRear = true;
    l.rear.mirrorAzimuth = false;
    return l;
}

namespace track {

QVector<Keyframe> resolve(const Track &tr, const TrackLenses &lenses,
                          const OrientationFn &imuAt, double tEnd)
{
    QVector<Keyframe> out;
    if (tr.samples.isEmpty()) return out;

    // 1. Back-project every sample, and measure its angular size.
    QVector<QVector3D> world;
    QVector<double> times, rawFov;
    world.reserve(tr.samples.size());
    times.reserve(tr.samples.size());
    rawFov.reserve(tr.samples.size());

    const double tanHalfSize0 = std::tan(qMax(1e-4, tr.sizeRad0) * 0.5);
    const double tanHalfFov0 = std::tan(proj::degToRad(tr.fov0) * 0.5);
    const double k = tanHalfSize0 / qMax(1e-9, tanHalfFov0);   // screen fraction to hold

    const double tStart = tr.activeStart();
    for (const TrackSample &s : tr.samples) {
        if (s.t < tStart - 1e-9 || s.t > tEnd + 1e-9) continue;
        const auto &lp = (s.lens == 1) ? lenses.rear : lenses.front;
        const QVector3D cam = VisualRotationComputer::pixelToBearing(s.u, s.v, lp);
        // The shader samples q_imu^-1 * world, so world = q_imu * camera.
        world.append(imuAt(s.t).rotatedVector(cam));
        times.append(s.t);

        double fov = tr.fov0;
        if (tr.fovFollow && s.halfW > 0.0 && s.halfH > 0.0 && tr.sizeRad0 > 1e-4) {
            // Geometric mean of the two half-axes, so a box that changes
            // aspect does not read as a size change.
            double sigma;
            if (tr.sizeIsAngular) {
                sigma = 2.0 * std::sqrt(qMax(1e-9, s.halfW) * qMax(1e-9, s.halfH));
            } else {
                // A v1 track: the size is a uv distance, so it still has to be
                // turned back into an angle the old way, position error and all.
                const QVector3D br = VisualRotationComputer::pixelToBearing(s.u + s.halfW, s.v, lp);
                const QVector3D bd = VisualRotationComputer::pixelToBearing(s.u, s.v + s.halfH, lp);
                sigma = 2.0 * std::sqrt(qMax(1e-9, angleBetween(cam, br))
                                      * qMax(1e-9, angleBetween(cam, bd)));
            }
            fov = 2.0 * std::atan(std::tan(qMax(1e-5, sigma) * 0.5) / qMax(1e-9, k));
            fov = qBound(10.0, fov * 180.0 / kPi, 170.0);
        }
        rawFov.append(fov);
    }
    if (world.isEmpty()) return out;

    // Take the spikes out before smoothing: a Gaussian applied to impulse noise
    // spreads one bad frame over its neighbours instead of removing it.
    world = despike(world);
    world = smoothDirs(world, times, tr.posSmoothSec);

    const QVector<double> fov = tr.fovFollow ? filterFov(rawFov, times, tr)
                                             : QVector<double>(times.size(), tr.fov0);

    // 2. Turn each (world direction, fov) into the view that puts the object
    //    back at the framing the user picked.
    double prevPitch = tr.pitch0;
    double prevYaw = tr.yaw0;
    QVector<Keyframe> dense;
    dense.reserve(world.size());

    for (int i = 0; i < world.size(); ++i) {
        // The framing ray, re-evaluated at THIS fov so the subject holds the
        // same fraction of the frame as the view widens or narrows.
        const proj::ViewBasis rollOnly = proj::ViewBasis::make(
            0.0, 0.0, tr.roll0, fov[i], tr.framingProjection, tr.framingAspect);
        const proj::Vec3 vf = proj::applyEuler(
            proj::rayForNdc(tr.framingNdcX, tr.framingNdcY, rollOnly), rollOnly);

        const QVector3D W = world[i].normalized();
        const double rho = std::sqrt(vf.y * vf.y + vf.z * vf.z);
        double pitch = prevPitch, yaw = prevYaw;
        if (rho > 1e-6) {
            const double alpha = std::atan2(vf.z, vf.y);
            const double c = qBound(-1.0, (double)W.y() / rho, 1.0);
            const double base = std::acos(c);
            // Two branches solve it; take the one nearest the previous frame
            // so the view does not flip when the object crosses the axis.
            const double p1 = base - alpha, p2 = -base - alpha;
            const double prevRad = proj::degToRad(prevPitch);
            pitch = (std::fabs(wrap180((p1 - prevRad) * 180.0 / kPi))
                     <= std::fabs(wrap180((p2 - prevRad) * 180.0 / kPi))) ? p1 : p2;

            const double sp = std::sin(pitch), cp = std::cos(pitch);
            const double A = vf.x;
            const double B = sp * vf.y + cp * vf.z;
            yaw = std::atan2(W.x(), W.z()) - std::atan2(A, B);
            pitch = pitch * 180.0 / kPi;
            yaw = yaw * 180.0 / kPi;
        }
        // Clamp here, not at the preview call site: applyKeyframeInterpolation
        // clamps pitch but ExportSnapshot::stateAt does not, so a track aimed
        // high would otherwise render differently in preview and export.
        pitch = qBound(-89.5, pitch, 89.5);
        yaw = wrap180(yaw);

        Keyframe kf;
        kf.time = times[i];
        kf.yaw = yaw;
        kf.pitch = pitch;
        kf.roll = tr.roll0;
        kf.fov = fov[i];
        dense.append(kf);
        prevPitch = pitch;
        prevYaw = yaw;
    }

    out = decimate(dense, 0.05);
    return out;
}

QVector<Keyframe> compose(const QVector<Keyframe> &manual, const QVector<Track> &tracks,
                          const TrackLenses &lenses, const OrientationFn &imuAt,
                          double trimOut, double duration)
{
    if (tracks.isEmpty()) return manual;

    QVector<Track> sorted = tracks;
    // Order by where each track actually starts, not where it was armed:
    // trimming one track's start past another's would otherwise leave them
    // bounding each other in the wrong order.
    std::sort(sorted.begin(), sorted.end(),
              [](const Track &a, const Track &b) { return a.activeStart() < b.activeStart(); });

    const double clipEnd = (trimOut > 0.0) ? trimOut
                                           : (duration > 0.0 ? duration : 1e9);

    QVector<Keyframe> generated;
    for (int ti = 0; ti < sorted.size(); ++ti) {
        const Track &tr = sorted[ti];

        // Where this track stops governing the view. A manual keyframe always
        // wins, so the first one after t0 ends the track -- truncation happens
        // HERE and never in storage, so deleting that keyframe revives the
        // rest of the track.
        const double tStart = tr.activeStart();
        double tEnd = clipEnd;
        for (const Keyframe &k : manual)
            if (k.time > tStart + 1e-6) { tEnd = qMin(tEnd, k.time); break; }
        if (ti + 1 < sorted.size())
            tEnd = qMin(tEnd, sorted[ti + 1].activeStart());
        tEnd = qMin(tEnd, tr.activeEnd());
        if (tEnd <= tStart) continue;

        QVector<Keyframe> res = resolve(tr, lenses, imuAt, tEnd);
        if (res.isEmpty()) continue;

        // On loss the view holds the last confident direction, then eases into
        // the next manual keyframe rather than stepping to it. A duplicate of
        // the final keyframe half a second before that keyframe is all the
        // existing linear interpolation needs to do both.
        // Only hold-and-ease when the track still runs to where the tracker
        // gave up. A user-trimmed end is a deliberate stop, and holding the
        // last direction past it would fight the very edit they just made.
        if (tr.endsAtLoss() && tEnd >= tr.activeEnd() - 1e-6) {
            double nextManual = -1.0;
            for (const Keyframe &k : manual)
                if (k.time > res.constLast().time + 1e-6) { nextManual = k.time; break; }
            if (nextManual > 0.0) {
                const double holdUntil = nextManual - 0.5;
                if (holdUntil > res.constLast().time + 1e-3) {
                    Keyframe hold = res.constLast();
                    hold.time = holdUntil;
                    res.append(hold);
                }
            }
        }
        generated += res;
    }

    QVector<Keyframe> merged = manual;
    for (const Keyframe &g : generated) {
        bool clash = false;
        for (const Keyframe &m : manual)
            if (qFuzzyCompare(g.time, m.time)) { clash = true; break; }
        if (!clash) merged.append(g);
    }
    std::sort(merged.begin(), merged.end());
    return merged;
}

// --- sidecar --------------------------------------------------------------
// Samples are quantised and packed into one base64 blob. QJsonDocument writes
// Indented, so a plain array would put every value on its own line: a 60 s
// track becomes ~250 KB of sidecar that no one can read past. Packed it is
// ~25 KB on one line, and everything else in the file stays legible.
namespace {

constexpr int kSampleBytes = 14;

QByteArray packSamples(const QVector<TrackSample> &samples, double t0)
{
    QByteArray blob;
    blob.resize(samples.size() * kSampleBytes);
    uchar *p = reinterpret_cast<uchar *>(blob.data());
    for (const TrackSample &s : samples) {
        auto put16 = [&p](double value01) {
            const quint16 q = (quint16)qRound(qBound(0.0, value01, 1.0) * 65535.0);
            qToLittleEndian(q, p); p += 2;
        };
        const quint32 dt = (quint32)qRound(qMax(0.0, s.t - t0) * 100000.0);  // 10 us
        qToLittleEndian(dt, p); p += 4;
        put16(s.u);
        put16(s.v);
        // Angles, scaled into the 0..1 field: 1/65535 of pi is 0.003 deg, far
        // finer than the tracker can measure.
        put16(s.halfW / kPi);
        put16(s.halfH / kPi);
        *p++ = (uchar)qBound(0, s.lens, 255);
        *p++ = (uchar)qBound(0, (int)qRound(s.conf * 255.0), 255);
    }
    return blob;
}

QVector<TrackSample> unpackSamples(const QByteArray &blob, double t0, bool angularSize)
{
    QVector<TrackSample> out;
    const int n = blob.size() / kSampleBytes;
    out.reserve(n);
    const uchar *p = reinterpret_cast<const uchar *>(blob.constData());
    for (int i = 0; i < n; ++i) {
        TrackSample s;
        s.t = t0 + qFromLittleEndian<quint32>(p) / 100000.0; p += 4;
        auto get16 = [&p]() {
            const double v = qFromLittleEndian<quint16>(p) / 65535.0; p += 2; return v;
        };
        s.u = get16();
        s.v = get16();
        s.halfW = get16();
        s.halfH = get16();
        if (angularSize) { s.halfW *= kPi; s.halfH *= kPi; }
        s.lens = *p++;
        s.conf = *p++ / 255.0;
        out.append(s);
    }
    return out;
}

} // namespace

QJsonArray toJson(const QVector<Track> &tracks)
{
    QJsonArray arr;
    for (const Track &t : tracks) {
        QJsonObject o{
            {"id", t.id},
            {"t0", t.t0},
            {"lost", t.lost},
            {"lossTime", t.lossTime},
            {"framing", QJsonObject{
                {"ndcX", t.framingNdcX}, {"ndcY", t.framingNdcY},
                {"aspect", t.framingAspect}, {"projection", t.framingProjection}}},
            {"ref", QJsonObject{
                {"yaw", t.yaw0}, {"pitch", t.pitch0}, {"roll", t.roll0},
                {"fov", t.fov0}, {"sizeRad", t.sizeRad0}}},
            {"posSmoothSec", t.posSmoothSec},
            {"fovFollow", QJsonObject{
                {"on", t.fovFollow}, {"smoothSec", t.fovSmoothSec},
                {"deadband", t.fovDeadband}, {"maxRatePerSec", t.fovMaxRatePerSec},
                {"range", t.fovRange}}},
            {"samplesFormat", QStringLiteral("packed14v2")},
            {"sampleCount", t.samples.size()},
            {"samples", QString::fromLatin1(packSamples(t.samples, t.t0).toBase64())},
        };
        // Written only when set, so an untrimmed track round-trips to exactly
        // the sidecar it came from and the diff stays readable.
        if (t.trimIn >= 0.0) o.insert(QStringLiteral("trimIn"), t.trimIn);
        if (t.trimOut >= 0.0) o.insert(QStringLiteral("trimOut"), t.trimOut);
        arr.append(o);
    }
    return arr;
}

QVector<Track> fromJson(const QJsonArray &arr)
{
    QVector<Track> out;
    for (const QJsonValue &v : arr) {
        const QJsonObject o = v.toObject();
        const QString fmt = o.value("samplesFormat").toString();
        if (fmt != QStringLiteral("packed14v1") && fmt != QStringLiteral("packed14v2"))
            continue;                        // a format this build cannot read
        Track t;
        t.id = o.value("id").toString();
        t.t0 = o.value("t0").toDouble();
        t.lost = o.value("lost").toBool();
        t.lossTime = o.value("lossTime").toDouble(-1.0);
        const QJsonObject fr = o.value("framing").toObject();
        t.framingNdcX = fr.value("ndcX").toDouble();
        t.framingNdcY = fr.value("ndcY").toDouble();
        t.framingAspect = fr.value("aspect").toDouble(16.0 / 9.0);
        t.framingProjection = fr.value("projection").toInt();
        const QJsonObject rf = o.value("ref").toObject();
        t.yaw0 = rf.value("yaw").toDouble();
        t.pitch0 = rf.value("pitch").toDouble();
        t.roll0 = rf.value("roll").toDouble();
        t.fov0 = rf.value("fov").toDouble(90.0);
        t.sizeRad0 = rf.value("sizeRad").toDouble();
        t.posSmoothSec = o.value("posSmoothSec").toDouble(0.10);
        const QJsonObject ff = o.value("fovFollow").toObject();
        t.fovFollow = ff.value("on").toBool(true);
        t.fovSmoothSec = ff.value("smoothSec").toDouble(0.75);
        t.fovDeadband = ff.value("deadband").toDouble(0.06);
        t.fovMaxRatePerSec = ff.value("maxRatePerSec").toDouble(0.22);
        t.fovRange = ff.value("range").toDouble(1.5);
        // v1 stored the subject's size as a distance in uv; v2 stores the
        // angle directly. Old tracks keep resolving through the old path, so
        // one that was framed to taste stays framed the way it was.
        t.sizeIsAngular = (fmt == QStringLiteral("packed14v2"));
        t.samples = unpackSamples(QByteArray::fromBase64(
            o.value("samples").toString().toLatin1()), t.t0, t.sizeIsAngular);
        // Absent means untrimmed. activeStart/activeEnd clamp these to the
        // measured span, so a hand-edited sidecar cannot widen a track.
        t.trimIn = o.value("trimIn").toDouble(-1.0);
        t.trimOut = o.value("trimOut").toDouble(-1.0);
        out.append(t);
    }
    return out;
}

} // namespace track
