// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef GYROCALIBRATION_H
#define GYROCALIBRATION_H

#include <QObject>
#include <QVector>
#include <QMatrix3x3>
#include <QVector3D>
#include <QQuaternion>
#include <QString>

#include "visualrotation.h"
#include "imuparser.h"

struct GyroCalibration {
    QMatrix3x3 matrix;    // 3x3 calibration matrix M
    QVector3D bias;       // bias vector b (deg/s)
    QVector3D diagScale;  // per-axis scale corrections (x,y,z) from the
                          // robust diagonal fit (scale factor around 1.0)
    QVector3D diagBias;   // per-axis bias from the diagonal fit (deg/s)
    double residualDeg;   // RMS residual (deg/s)
    int samplesUsed;      // number of samples after outlier rejection
};

Q_DECLARE_METATYPE(GyroCalibration)

class GyroCalibrator : public QObject {
    Q_OBJECT
public:
    explicit GyroCalibrator(QObject *parent = nullptr);

    // imuToCamera: the header's IMU->camera quaternion, as used by
    // GyroscopeIntegrator. The visual rotations are measured in the camera
    // (bearing) frame while the IMU samples are in raw sensor axes, so the fit
    // is done in the camera frame and the result converted back to sensor axes
    // for storage — that is the frame integrate() applies it in. Fitting the
    // two frames against each other directly forces the matrix to absorb the
    // whole IMU->camera rotation, making it a signed permutation rather than a
    // near-identity correction.
    void calibrate(const QVector<VisualRotationPair> &visualPairs,
                   const QVector<ImuSample> &imuSamples,
                   double syncOffset,
                   double drift,
                   const QMatrix3x3 &priorM,
                   const QVector3D &priorB,
                   const QQuaternion &imuToCamera = QQuaternion());

signals:
    void progressChanged(double fraction, const QString &status);
    void calibrationComputed(const GyroCalibration &calibration);
    void calibrationFailed(const QString &error);

private:
    // Internal paired sample for the least-squares solve
    struct PairedSample {
        QVector3D omegaVisual;  // visual rotation rate (deg/s)
        QVector3D omegaRaw;     // raw gyro reading at that time (deg/s)
        double weight;          // measurement confidence, inliers/(1+rmsDeg),
                                // normalised so the best pair in the clip is 1.
    };

    // Helper: quaternion to axis-angle (returns axis * angle_in_degrees)
    static QVector3D quaternionToAxisAngle(const QQuaternion &q);

    // Helper: interpolate gyro at arbitrary time from IMU samples
    static QVector3D interpolateGyro(const QVector<ImuSample> &samples, double t);

    // Helper: mean gyro across [t0, t1]. The visual measurement is an average
    // over the hop, so its gyro counterpart must be too — see buildPairedSamples.
    static QVector3D meanGyroOverWindow(const QVector<ImuSample> &samples,
                                        double t0, double t1);

    // Build paired visual/gyro samples
    QVector<PairedSample> buildPairedSamples(
        const QVector<VisualRotationPair> &visualPairs,
        const QVector<ImuSample> &imuSamples,
        double syncOffset, double drift,
        const QQuaternion &imuToCamera);

    // Solve one row of M and corresponding bias element using weighted least squares
    // component: 0=x, 1=y, 2=z (which visual omega component to fit)
    // weights: per-sample weights (N x 1)
    // priorRow: prior values [m_i1, m_i2, m_i3, b_i] for regularization
    // lambda: regularization weight
    // Returns [m_i1, m_i2, m_i3, b_i]
    static std::array<double, 4> solveRowRegularized(
        const QVector<PairedSample> &samples,
        int component,
        const QVector<double> &weights,
        const std::array<double, 4> &priorRow,
        double lambda);

    // Compute Huber weights from residuals
    static QVector<double> computeHuberWeights(
        const QVector<PairedSample> &samples,
        const QMatrix3x3 &M, const QVector3D &b,
        double delta);

    // Robust per-axis diagonal fit: omega_visual = s * omega_raw + b per axis.
    // Returns scale factors around 1.0 and the per-axis bias. Fewer parameters
    // than the full 3x3 solve, so it stays well-conditioned on short clips
    // (the Just* rotation clips). This is what feeds ImuParser::setGyroScale*.
    QVector<QVector3D> fitDiagonalScales(const QVector<PairedSample> &samples,
                                         const QVector3D &priorScale,
                                         const QVector3D &priorBias) const;
};

#endif // GYROCALIBRATION_H
