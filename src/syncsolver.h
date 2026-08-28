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
    // Visual rotation rate sample: angular velocity at a time midpoint
    struct VisualRateSample {
        double tMid;           // time midpoint (video time)
        QVector3D omegaVisual; // angular velocity (deg/s), 3-axis
    };

    // Gyro rate sample paired with the visual time it was sampled at
    struct GyroRateSample {
        double tVideo;         // original video time
        QVector3D omegaGyro;   // interpolated gyro rate (deg/s)
    };

    // Local cross-correlation result for one window
    struct LocalOffset {
        double tWindow;        // window center time (video time)
        double offsetLocal;    // local offset at this window (s)
        double weight;         // weight (gyro variance)
    };

    // Step 2: Compute visual rotation rates from pairs
    QVector<VisualRateSample> computeVisualRates(
        const QVector<VisualRotationPair> &pairs);

    // Step 3: Sample gyro rates at visual times
    QVector<GyroRateSample> sampleGyroRates(
        const QVector<VisualRateSample> &visualRates,
        const QVector<ImuSample> &imuSamples,
        double drift, double offset);

    // Step 4a: Global cross-correlation of the full visual-rate and gyro-rate
    // signals over a lag range. Gyro is resampled at (tMid + tau) for each
    // candidate lag tau, giving a true temporal shift of the IMU stream.
    // Returns the best (lagOffset, correlationScore), with optional per-window
    // refinements for the joint offset+drift fit.
    double globalCrossCorrelate(
        const QVector<VisualRateSample> &visualRates,
        const QVector<ImuSample> &imuSamples,
        double drift, double *bestLagOut) const;

    // Step 4b: Per-window local offsets for the joint drift/offset fit, using
    // the global offset as the coarse alignment.
    QVector<LocalOffset> crossCorrelateWindows(
        const QVector<VisualRateSample> &visualRates,
        const QVector<GyroRateSample> &gyroRates);

    // Step 5: Joint line fit
    bool fitLine(const QVector<LocalOffset> &offsets,
                 double &o0, double &drift, double &residualMs);

    // Helper: interpolate gyro at arbitrary time from IMU samples
    static QVector3D interpolateGyro(const QVector<ImuSample> &samples, double t);

    // Helper: quaternion to axis-angle (returns axis * angle_in_degrees)
    static QVector3D quaternionToAxisAngle(const QQuaternion &q);
};

#endif // SYNCSOLVER_H
