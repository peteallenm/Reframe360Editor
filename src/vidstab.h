// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef VIDSTAB_H
#define VIDSTAB_H

#include <QVector>
#include <QString>
#include <QQuaternion>

// ---------------------------------------------------------------------------
// Hybrid stabilization: derive per-frame residual motion from FFmpeg's
// vidstabdetect, fold it back into the native IMU orientation, and re-render
// in a single pass. Kept self-contained so the Exporter can stay a thin
// orchestrator.
// ---------------------------------------------------------------------------

// One frame of fitted vidstab motion: 2D translation (pixels) plus an in-plane
// rotation (radians), in the analysis render's coordinate space.
struct VidStabTransform {
    double dx = 0.0;
    double dy = 0.0;
    double alpha = 0.0;
};

class VidStabAnalysis
{
public:
    // Parse an FFmpeg vidstab detection .trf file ("Frame N (List M [(LM u v x
    // y ...), ...])" per frame) and fit the global rigid motion per frame.
    // Returns false (with *error) if the file can't be opened/parsed.
    bool parseTrf(const QString &trfPath, QString *error);

    // Replicate vidstabtransform's lowpass: a Gaussian window of
    // 2*smoothingWindow+1 frames, then return the correction signal per frame:
    // correction[i] = smoothed[i] - raw[i] (the high-frequency residual that
    // vidstab would remove; positive when the smoothed path leads). Empty if
    // no transforms were parsed.
    QVector<VidStabTransform> corrections(int smoothingWindow) const;

    // Convert a single 2D correction into a small 3D counter-rotation quaternion
    // for a view with the given vertical FOV (radians), aspect and analysis
    // pixel dimensions. The sign convention counter-rotates the view so the
    // detected shake is cancelled, matching vidstabtransform.
    static QQuaternion correctionToQuaternion(const VidStabTransform &t,
                                              double fovRad, double aspect,
                                              int analW, int analH);

    int frameCount() const { return m_transforms.size(); }

private:
    QVector<VidStabTransform> m_transforms;
};

#endif // VIDSTAB_H