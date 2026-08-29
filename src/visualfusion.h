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
    // sigmaSeconds: Gaussian smoothing sigma for the correction spline.
    //   Swept against hand-authored reference keyframes on YIVR_0845
    //   (`tracking_tests --groundtruth --sigma=N`), mean horizon error:
    //       sigma  1.50  0.80  0.50  0.30  0.15
    //       error  37.5  31.1  29.7  29.0  27.7   deg   (gyro alone: 96.5)
    //   Angular jerk at 30 fps was 1.730 deg rms for the gyro chain and
    //   1.731-1.733 for the fused chain at EVERY sigma, i.e. fusion adds no
    //   measurable shake -- the correction is smooth, so the old 1.5 s was
    //   simply over-smoothing a drift that moves at ~20 deg/s on an orbit clip.
    //   That sweep optimised TILT accuracy. Fusion is now yaw-only, and a yaw
    //   drift correction must above all be smooth: at 0.5 s the applied
    //   correction moved at 6.7 deg/s mean / 40 deg/s p99 and read as a ~1 Hz
    //   jitter. 3 s, plus the 4 deg/s slew limit in fuse(), keeps it invisible.
    // imuTrust (optional, per IMU sample, [0,1]): where the IMU's absolute
    // attitude is gravity-verified. When given, the visual chain is anchored
    // to the IMU over those samples rather than at its first pair -- see the
    // note in fuse(). Empty = anchor at the first pair (legacy behaviour).
    void fuse(const QVector<VisualRotationPair> &visualPairs,
              const QVector<QQuaternion> &imuOrientations,
              const QVector<double> &imuTimestamps,
              double syncOffset,
              double drift,
              double sigmaSeconds = 0.5,
              const QVector<float> &imuTrust = QVector<float>());

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
