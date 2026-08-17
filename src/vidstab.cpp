#include "vidstab.h"

#include <QFile>
#include <QRegularExpression>
#include <QTextStream>
#include <QtMath>

#include <utility>

// ---------------------------------------------------------------------
// Hybrid stabilization analysis.
//
// vidstabdetect writes a *tracked-feature-point* file (one "Frame N" line per
// input frame listing per-point local-motion vectors), NOT the per-frame
// transform directly. vidstabtransform internally fits a global rigid motion
// to those points, smooths the resulting trajectory and removes the residual.
//
// This module replicates that: for every frame we least-squares fit a rigid
// 2D transform (dx, dy, alpha) to the tracked points, Gaussian-lowpass the
// trajectory like vidstabtransform's smoothing filter, and return the
// residual correction (smoothed - raw) that vidstab would have removed. The
// conversions to QQuaternion are done against the analysis render's geometry.
// ---------------------------------------------------------------------

namespace {

// Fit a rigid transform (tx, ty, alpha) to a set of tracked points whose
// observed motion is (u_i, v_i) at position (x_i, y_i):
//     u_i ~= tx - alpha * (y_i - cy)
//     v_i ~= ty + alpha * (x_i - cx)
// With the coordinate origin moved to the mean point position the unknowns
// decouple and the least-squares solution is closed-form.
VidStabTransform fitRigid(const QVector<std::tuple<int, int, int, int> > &points)
{
    VidStabTransform t;
    const int n = points.size();
    if (n == 0)
        return t;

    double cx = 0.0, cy = 0.0;
    for (const auto &p : points)
        cx += std::get<2>(p);
    for (const auto &p : points)
        cy += std::get<3>(p);
    cx /= n;
    cy /= n;

    double sumU = 0.0, sumV = 0.0, sumS = 0.0, sumCross = 0.0;
    for (const auto &p : points) {
        const double u = std::get<0>(p);
        const double v = std::get<1>(p);
        const double x = std::get<2>(p) - cx;
        const double y = std::get<3>(p) - cy;
        sumU += u;
        sumV += v;
        sumS += x * x + y * y;
        sumCross += v * x - u * y;
    }

    t.dx = sumU / n;
    t.dy = sumV / n;
    if (sumS > 1e-9)
        t.alpha = sumCross / sumS;
    return t;
}

} // namespace

bool VidStabAnalysis::parseTrf(const QString &trfPath, QString *error)
{
    QFile f(trfPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error)
            *error = QStringLiteral("Cannot open %1").arg(trfPath);
        return false;
    }

    QVector<VidStabTransform> parsed;
    static const QRegularExpression frameRe(
        QStringLiteral("^\\s*Frame\\s+(\\d+)\\s+\\(List\\s+\\d+\\s+\\[([^]]*)\\]"));
    static const QRegularExpression pointRe(
        QStringLiteral("\\((?:LM|MM)\\s+(-?\\d+)\\s+(-?\\d+)\\s+(\\d+)\\s+(\\d+)\\s+\\d+\\s+[\\d.eE-]+\\s+[\\d.eE-]+\\)"));

    QTextStream in(&f);
    QVector<std::tuple<int, int, int, int> > points;
    while (!in.atEnd()) {
        const QString line = in.readLine();
        const QRegularExpressionMatch fm = frameRe.match(line);
        if (!fm.hasMatch())
            continue;

        points.clear();
        const QString list = fm.captured(2);
        auto pit = pointRe.globalMatch(list);
        while (pit.hasNext()) {
            const QRegularExpressionMatch m = pit.next();
            points.append(std::make_tuple(m.captured(1).toInt(),   // u
                                          m.captured(2).toInt(),   // v
                                          m.captured(3).toInt(),   // x
                                          m.captured(4).toInt())); // y
        }
        parsed.append(fitRigid(points));
    }

    if (parsed.isEmpty()) {
        if (error)
            *error = QStringLiteral("No motion data found in %1").arg(trfPath);
        return false;
    }
    m_transforms = parsed;
    return true;
}

QVector<VidStabTransform> VidStabAnalysis::corrections(int smoothingWindow) const
{
    const int n = m_transforms.size();
    // A moving-average over 2*smoothingWindow+1 frames, matching
    // vidstabtransform's "number of frames*2 + 1 used for lowpass filtering".
    // (libvidstab uses an averaging lowpass, not Gaussian.)
    const int w = qBound(0, 2 * smoothingWindow + 1, qMax(1, 2 * n - 1)) / 2;
    if (w == 0)
        return m_transforms;

    QVector<VidStabTransform> out(n);
    for (int i = 0; i < n; ++i) {
        double sx = 0.0, sy = 0.0, sa = 0.0;
        const int lo = qMax(0, i - w);
        const int hi = qMin(n - 1, i + w);
        const int cnt = hi - lo + 1;
        for (int j = lo; j <= hi; ++j) {
            sx += m_transforms[j].dx;
            sy += m_transforms[j].dy;
            sa += m_transforms[j].alpha;
        }
        out[i].dx = sx / cnt;
        out[i].dy = sy / cnt;
        out[i].alpha = sa / cnt;
    }

    // correction = smoothed - raw: the high-frequency residual that would be
    // removed. A moving-average keeps the trajectory's low frequencies, so the
    // difference is exactly the shake.
    for (int i = 0; i < n; ++i) {
        out[i].dx = out[i].dx - m_transforms[i].dx;
        out[i].dy = out[i].dy - m_transforms[i].dy;
        out[i].alpha = out[i].alpha - m_transforms[i].alpha;
    }
    return out;
}

QQuaternion VidStabAnalysis::correctionToQuaternion(const VidStabTransform &t,
                                                    double fovRad, double aspect,
                                                    int analW, int analH)
{
    // Perspective ray at analysis resolution: a dx pixel shift corresponds to
    // an angular step of 2*tan(fov/2)*aspect/W in yaw and -2*tan(fov/2)/H in
    // pitch. The raw detection is the camera trajectory; the shake we remove
    // is its deviation from the smoothed path, so we counter-rotate by it. The
    // correction is tiny (residual jitter), so the multiplication order used
    // by the caller does not matter to first order.
    const double scaleX = 2.0 * std::tan(fovRad * 0.5) * aspect / qMax(1, analW);
    const double scaleY = -2.0 * std::tan(fovRad * 0.5) / qMax(1, analH);
    const double radToDeg = 180.0 / M_PI;
    const double yawDeg = t.dx * scaleX * radToDeg;
    const double pitchDeg = t.dy * scaleY * radToDeg;
    const double rollDeg = t.alpha * radToDeg;
    return QQuaternion::fromAxisAndAngle(0.0f, 1.0f, 0.0f, (float)yawDeg)
         * QQuaternion::fromAxisAndAngle(1.0f, 0.0f, 0.0f, (float)pitchDeg)
         * QQuaternion::fromAxisAndAngle(0.0f, 0.0f, 1.0f, (float)rollDeg);
}