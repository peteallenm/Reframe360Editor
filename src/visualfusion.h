#ifndef VISUALFUSION_H
#define VISUALFUSION_H

#include <QVector>
#include <QQuaternion>
#include "visualrotation.h"

// VisualFusion corrects yaw drift (and refines pitch/roll) by fusing visual
// rotation measurements with IMU orientations. The visual rotation chain
// observes all three axes, so fusing it with the IMU orientations fixes yaw
// drift (unobservable from the IMU alone without a magnetometer).
//
// Pipeline:
//  1. Chain visual rotation pairs into a sparse visual orientation trajectory
//  2. Sample IMU orientations at the visual pair times
//  3. Compute correction quaternions C(t) = Q_vis(t) * Q_gyro(t)^-1
//  4. Smooth C(t) with Gaussian-weighted quaternion averaging
//  5. Apply: Q_fused(t) = C_smooth(t) * Q_gyro(t)
class VisualFusion {
public:
    VisualFusion() = default;

    // Fuse visual rotations with IMU orientations to correct yaw drift.
    // visualPairs: rotation deltas between video frames (from VisualRotationComputer)
    // imuOrientations/imuTimestamps: integrated IMU orientation chain (from GyroscopeIntegrator)
    // syncOffset/drift: time mapping tImu = tVideo * (1 + drift) + syncOffset
    // sigmaSeconds: Gaussian smoothing sigma for the correction spline
    void fuse(const QVector<VisualRotationPair> &visualPairs,
              const QVector<QQuaternion> &imuOrientations,
              const QVector<double> &imuTimestamps,
              double syncOffset,
              double drift,
              double sigmaSeconds = 1.5);

    // Query the fused orientation at arbitrary time (IMU time domain).
    // Uses Gaussian-weighted quaternion averaging (same as GyroscopeIntegrator::orientationAt)
    // with a 33ms minimum window for exposure-time averaging.
    QQuaternion orientationAt(double time) const;

    // Access the full fused orientation chain (at IMU timestamps).
    QVector<QQuaternion> fusedOrientations() const;
    QVector<double> fusedTimestamps() const;

private:
    struct CorrectionKnot {
        double time;       // video time
        QQuaternion correction;
        double quality;    // inliers / (1 + rmsDeg)
    };

    // Slerp interpolation of IMU orientations at arbitrary IMU time
    static QQuaternion sampleImuOrientation(const QVector<QQuaternion> &orientations,
                                            const QVector<double> &timestamps,
                                            double time);

    // Gaussian-weighted correction quaternion at arbitrary video time
    QQuaternion correctionAt(double videoTime) const;

    QVector<CorrectionKnot> m_knots;
    double m_sigma = 1.5;

    // Fused output (at IMU timestamps for drop-in replacement)
    QVector<QQuaternion> m_fusedOrientations;
    QVector<double> m_fusedTimestamps;

    // Sync parameters (needed for time mapping during output generation)
    double m_syncOffset = 0.0;
    double m_drift = 0.0;
};

#endif // VISUALFUSION_H
