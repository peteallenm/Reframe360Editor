// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef PROJECTION_H
#define PROJECTION_H

// The projection maths of project.frag, in C++.
//
// These expressions used to live as lambdas inside exporter.cpp's CPU
// renderFrame(), which meant nothing else could ask "where on screen is this
// direction" or "which fisheye pixel does this ray hit" without copying them.
// Object tracking needs both, so they moved here VERBATIM: same operations in
// the same order, with the same values hoisted per frame (ViewBasis, LensGeom)
// as renderFrame already hoisted. The port is a move, not a rewrite, and
// tracking_tests holds a pinned copy of the original bodies to prove it.
//
// Header-only and free of Qt GUI types so the headless test harness can use it.

#include <QtGlobal>
#include <cmath>

namespace proj {

constexpr double kPi = 3.14159265358979323846;

inline double degToRad(double d) { return d * kPi / 180.0; }

struct Vec3 { double x, y, z; };

inline Vec3 normalize3(Vec3 v)
{
    double l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (l < 1e-12) return Vec3{0.0, 0.0, 0.0};
    return Vec3{v.x / l, v.y / l, v.z / l};
}

// Quaternion rotation (equivalent to QQuaternion::rotatedVector, inline for
// speed since it runs per output pixel).
inline Vec3 quatRotate(double qw, double qx, double qy, double qz, Vec3 v)
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

// Everything the ray maths needs that is constant for a frame. Built once and
// passed down, exactly as renderFrame hoisted these values.
struct ViewBasis {
    double cy = 1.0, sy = 0.0;   // yaw
    double cp = 1.0, sp = 0.0;   // pitch
    double cr = 1.0, sr = 0.0;   // roll
    double fovPersp = 1.0;       // tan(fov/2)
    double fovStereo = 1.0;      // 2*tan(fov/4)
    double aspect = 1.0;
    double verticalStretch = (16.0 / 9.0) / (4.0 / 3.0);   // SportsView, = 4/3
    int projection = 0;

    static ViewBasis make(double yawDeg, double pitchDeg, double rollDeg,
                          double fovDeg, int projection, double aspect)
    {
        ViewBasis b;
        b.aspect = aspect;
        b.fovPersp = std::tan(degToRad(fovDeg * 0.5));
        b.fovStereo = 2.0 * std::tan(degToRad(fovDeg * 0.25));
        b.cy = std::cos(degToRad(yawDeg));   b.sy = std::sin(degToRad(yawDeg));
        b.cp = std::cos(degToRad(pitchDeg)); b.sp = std::sin(degToRad(pitchDeg));
        b.cr = std::cos(degToRad(rollDeg));  b.sr = std::sin(degToRad(rollDeg));
        b.projection = projection;
        return b;
    }
};

// Output pixel (u,v in 0..1, v=0 at the top) -> view-space ray. Mirror of
// project.frag's step A.
inline Vec3 rayForUv(double u, double v, const ViewBasis &b)
{
    switch (b.projection) {
    case 1: {  // Equirectangular: maps the full 360x180 sphere
        double lon = (u - 0.5) * 2.0 * kPi;
        double lat = (0.5 - v) * kPi;
        return Vec3{std::cos(lat) * std::sin(lon), std::sin(lat),
                    -std::cos(lat) * std::cos(lon)};
    }
    case 2: {  // Stereographic
        double ndcX = u * 2.0 - 1.0;
        double ndcY = v * 2.0 - 1.0;
        double px = ndcX * b.fovStereo * b.aspect;
        double py = -ndcY * b.fovStereo;
        double rho = std::sqrt(px * px + py * py);
        double nx = (rho > 1e-6) ? px / rho : 0.0;
        double ny = (rho > 1e-6) ? py / rho : 0.0;
        double ang = 2.0 * std::atan(rho * 0.5);
        return Vec3{nx * std::sin(ang), ny * std::sin(ang), -std::cos(ang)};
    }
    case 3: {  // SportsView
        double ndcX = u * 2.0 - 1.0;
        double ndcY = v * 2.0 - 1.0;
        return normalize3(Vec3{ndcX * b.fovPersp * b.aspect,
                               -ndcY * b.fovPersp * b.verticalStretch, -1.0});
    }
    default: {  // Perspective (rectilinear)
        double ndcX = u * 2.0 - 1.0;
        double ndcY = v * 2.0 - 1.0;
        return normalize3(Vec3{ndcX * b.fovPersp * b.aspect, -ndcY * b.fovPersp, -1.0});
    }
    }
}

// Same, for callers that already hold NDC (-1..1, y down like the screen).
inline Vec3 rayForNdc(double ndcX, double ndcY, const ViewBasis &b)
{
    return rayForUv(ndcX * 0.5 + 0.5, ndcY * 0.5 + 0.5, b);
}

// Apply rotZ, then rotX, then rotY (matrix order rotY * rotX * rotZ), i.e.
// the shader's eulerRotation(yaw, pitch, roll). Takes a view-space ray to the
// stabilised world frame.
inline Vec3 applyEuler(Vec3 v, const ViewBasis &b)
{
    double zx = b.cr * v.x - b.sr * v.y;
    double zy = b.sr * v.x + b.cr * v.y;
    double xz = v.z;
    double xx = zx;
    // rotX columns in the shader are (1,0,0),(0,cp,sp),(0,-sp,cp):
    // y' = cp*y - sp*z, z' = sp*y + cp*z.
    double xy = b.cp * zy - b.sp * xz;
    double xzz = b.sp * zy + b.cp * xz;
    return Vec3{b.cy * xx + b.sy * xzz, xy, -b.sy * xx + b.cy * xzz};
}

// The inverse of applyEuler: stabilised world frame -> view space.
inline Vec3 unapplyEuler(Vec3 w, const ViewBasis &b)
{
    // Transpose of the composed rotation (it is orthonormal).
    double xx = b.cy * w.x - b.sy * w.z;
    double xzz = b.sy * w.x + b.cy * w.z;
    double zy = b.cp * w.y + b.sp * xzz;
    double xz = -b.sp * w.y + b.cp * xzz;
    return Vec3{b.cr * xx + b.sr * zy, -b.sr * xx + b.cr * zy, xz};
}

inline void dirToThetaPhi(Vec3 ray, double &theta, double &phi)
{
    theta = std::acos(qBound(-1.0, -ray.z, 1.0));
    phi = std::atan2(ray.y, ray.x);
}

// One fisheye circle's geometry, with the lens rotation pre-resolved to
// cos/sin exactly as renderFrame hoisted it.
struct LensGeom {
    double centerX = 0.5, centerY = 0.5, radius = 0.5;
    double k1 = 0.0, k2 = 0.0;
    double cosRot = 1.0, sinRot = 0.0;
    bool hflip = false;

    static LensGeom make(double cx, double cy, double radius,
                         double k1, double k2, double rotationDeg, bool hflip)
    {
        LensGeom g;
        g.centerX = cx; g.centerY = cy; g.radius = radius;
        g.k1 = k1; g.k2 = k2; g.hflip = hflip;
        const double rot = degToRad(rotationDeg);
        g.cosRot = std::cos(rot); g.sinRot = std::sin(rot);
        return g;
    }
};

// Direction (as theta from forward, phi about it) -> normalised HALF-FRAME
// fisheye coordinates. This is the same space VisualRotationComputer::
// pixelToBearing consumes, and the half a caller then stacks with stackedV().
inline void lensUv(bool front, double theta, double phi, const LensGeom &g,
                   double &u, double &v)
{
    double r = front ? theta / (kPi * 0.5) : (kPi - theta) / (kPi * 0.5);
    double r2 = r * r;
    double rd = r * (1.0 + g.k1 * r2 + g.k2 * r2 * r2);
    double c = std::cos(phi), sn = std::sin(phi);
    double offx = (c * g.cosRot - sn * g.sinRot) * rd * g.radius;
    double offy = (c * g.sinRot + sn * g.cosRot) * rd * g.radius;
    if (g.hflip)
        offx = -offx;
    u = g.centerX + offx;
    v = g.centerY + offy;
}

// Half-frame v -> the 1:2 stacked frame the camera writes (front on top).
inline double stackedV(bool front, double v) { return front ? v * 0.5 : v * 0.5 + 0.5; }

// The inverse of rayForUv: a view-space direction -> NDC. False when the
// direction cannot be shown in this projection (behind a rectilinear camera).
inline bool ndcForDir(Vec3 v, const ViewBasis &b, double &ndcX, double &ndcY)
{
    v = normalize3(v);
    switch (b.projection) {
    case 1: {
        double lon = std::atan2(v.x, -v.z);
        double lat = std::asin(qBound(-1.0, v.y, 1.0));
        ndcX = lon / kPi;
        ndcY = -lat / (kPi * 0.5);
        return true;
    }
    case 2: {
        double ang = std::acos(qBound(-1.0, -v.z, 1.0));
        double rho = 2.0 * std::tan(ang * 0.5);
        double n = std::sqrt(v.x * v.x + v.y * v.y);
        double nx = (n > 1e-9) ? v.x / n : 0.0;
        double ny = (n > 1e-9) ? v.y / n : 0.0;
        ndcX = (nx * rho) / (b.fovStereo * b.aspect);
        ndcY = -(ny * rho) / b.fovStereo;
        return std::isfinite(ndcX) && std::isfinite(ndcY);
    }
    case 3:
    default: {
        if (v.z >= -1e-6)
            return false;                       // at or behind the image plane
        const double k = -1.0 / v.z;
        const double stretch = (b.projection == 3) ? b.verticalStretch : 1.0;
        ndcX = (v.x * k) / (b.fovPersp * b.aspect);
        ndcY = -(v.y * k) / (b.fovPersp * stretch);
        return true;
    }
    }
}

} // namespace proj

#endif // PROJECTION_H
