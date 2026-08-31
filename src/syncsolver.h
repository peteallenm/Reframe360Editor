// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef SYNCSOLVER_H
#define SYNCSOLVER_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QQuaternion>
#include <QString>
#include "visualrotation.h"
#include "imuparser.h"

struct SyncResult {
    double syncOffset;      // new sync offset (s)
    double drift;           // new drift (s/s)
    double residualMs;      // RMS residual of the fit (ms)
    int windowsUsed;        // number of windows with valid correlation
};

Q_DECLARE_METATYPE(SyncResult)

// Running integral of the gyro signal on a uniform grid, so the MEAN gyro over
// any interval is an O(1) lookup instead of a walk over 400 Hz samples.
//
// This exists for two reasons, one of them correctness:
//
//  - Errors-in-variables. A visual pair measures the NET rotation over
//    [t0, t1] divided by dt — a window average. The solver used to compare it
//    against the gyro sampled INSTANTANEOUSLY at the midpoint, which carries
//    all the high-frequency content the visual side has had averaged out. That
//    attenuates and broadens the correlation peak. GyroCalibrator was fixed
//    for exactly this (meanGyroOverWindow); the sync solver was not.
//
//  - Speed. The coarse scan evaluates ~500 lags, each needing the gyro over
//    every visual window. Walking the raw samples made that O(lags x pairs x
//    window), all on the GUI thread.
class GyroIntegral {
public:
    // step: grid spacing in seconds (use the correlation step).
    void build(const QVector<ImuSample> &samples, double step);
    bool isValid() const { return m_prefix.size() > 1; }
    double tBegin() const { return m_t0; }
    double tEnd() const { return m_t0 + (m_prefix.size() - 1) * m_step; }

    // Mean gyro vector over [a, b]. Returns false if the interval is not fully
    // covered by the IMU stream.
    bool meanOver(double a, double b, QVector3D &out) const;

private:
    QVector3D integralAt(double t) const;

    double m_t0 = 0.0;
    double m_step = 0.0;
    QVector<QVector3D> m_prefix;   // running integral from m_t0, deg
};

class SyncSolver : public QObject {
    Q_OBJECT
public:
    explicit SyncSolver(QObject *parent = nullptr);

    void solve(const QVector<VisualRotationPair> &visualPairs,
               const QVector<ImuSample> &imuSamples,
               double initialDrift,
               double initialOffset);

signals:
    void progressChanged(double fraction, const QString &status);
    void syncSolved(const SyncResult &result);
    void solveFailed(const QString &error);

private:
    // Visual rotation rate sample. t0/t1 bound the interval the rotation was
    // measured over, so the gyro can be averaged over the SAME interval.
    struct VisualRateSample {
        double t0, t1;         // interval the pair spans (video time)
        double tMid;           // midpoint (video time)
        QVector3D omegaVisual; // angular velocity (deg/s), 3-axis
        double mag;            // |omegaVisual|, the frame-independent signal
    };

    // Local cross-correlation result for one window
    struct LocalOffset {
        double tWindow;        // window center time (video time)
        double offsetLocal;    // local offset at this window (s)
        double weight;         // weight (rate variance in the window)
    };

    // Step 2: Compute visual rotation rates from pairs
    static QVector<VisualRateSample> computeVisualRates(
        const QVector<VisualRotationPair> &pairs);

    // THE correlation primitive. Pearson correlation of rotation-rate
    // MAGNITUDES between visual samples [lo, hi) and the gyro averaged over the
    // same intervals, mapped by tImu = tVideo * (1 + drift) + lag.
    //
    // Magnitude, not components: omegaVisual is in the camera/bearing frame
    // while the gyro is in raw sensor axes, and those differ by an axis
    // permutation. A component-wise dot product between two different frames
    // does not peak at the true lag. Magnitude is frame-independent.
    //
    // Pearson, not a bare inner product: an unnormalised score is maximised
    // wherever the gyro is biggest, not where the signals line up.
    //
    // Both means and variances are computed over the SAMPLES THAT SURVIVE the
    // range check at this lag, so a lag with less overlap is not penalised by
    // a shrinking sum — the old code divided every lag by a normalisation
    // computed once at zero lag, biasing the peak toward tau = 0.
    //
    // Returns the correlation in [-1, 1], or -2.0 if degenerate.
    static double correlate(const QVector<VisualRateSample> &vis, int lo, int hi,
                            const GyroIntegral &gyro, double drift, double lag);

    // Scan `lag` over [centre - range, centre + range] and return the best,
    // parabolically refined from the cached scores.
    static double bestLag(const QVector<VisualRateSample> &vis, int lo, int hi,
                          const GyroIntegral &gyro, double drift,
                          double centre, double range, double step,
                          double *scoreOut);

    // Per-window local offsets for the joint drift/offset fit.
    static QVector<LocalOffset> crossCorrelateWindows(
        const QVector<VisualRateSample> &visualRates,
        const GyroIntegral &gyro, double drift, double baseLag);

    // Joint weighted line fit of the per-window offsets. driftStdErr is the
    // standard error of the fitted slope, so the caller can tell a measured
    // drift from a slope fitted to window scatter.
    static bool fitLine(const QVector<LocalOffset> &offsets,
                        double &o0, double &drift, double &residualMs,
                        double &driftStdErr);

    // Helper: quaternion to axis-angle (returns axis * angle_in_degrees)
    static QVector3D quaternionToAxisAngle(const QQuaternion &q);
};

#endif // SYNCSOLVER_H
