#ifndef IMUDRIFTCALIBRATOR_H
#define IMUDRIFTCALIBRATOR_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QQuaternion>

class ImuDriftCalibrator : public QObject
{
    Q_OBJECT
public:
    explicit ImuDriftCalibrator(QObject *parent = nullptr);

    // Start calibration in a background thread. Copies all needed data so the
    // worker never touches live GUI state.
    //   videoPath:       path to the video file
    //   imuOrientations: copied from GyroscopeIntegrator (thread-safe snapshot)
    //   imuTimestamps:   corresponding timestamps
    //   syncOffset:      current IMU-video sync offset (s)
    //   initialDrift:    starting drift estimate to search around (s/s)
    //   numSamples:      number of frames to sample across the video
    void startCalibration(const QString &videoPath,
                          const QVector<QQuaternion> &imuOrientations,
                          const QVector<double> &imuTimestamps,
                          double syncOffset,
                          double initialDrift,
                          int numSamples = 20);

    bool isRunning() const { return m_running; }

signals:
    void progressChanged(double fraction, const QString &status);
    void calibrationFinished(double drift, double residualDeg);
    void calibrationFailed(const QString &error);

private:
    void runCalibration();

    // Decode a single frame near the given timestamp. Returns true on success.
    bool decodeFrameAt(const QString &videoPath, double time,
                       QVector<uint8_t> &rgb, int &width, int &height);

    // Sum of squared roll errors across all valid samples for a given drift.
    struct SampleData {
        double time;
        double rollDetected;
    };
    double computeTotalError(const QVector<SampleData> &samples, double drift) const;

    // Extract roll angle (degrees) from a quaternion, matching App convention.
    static double extractRoll(const QQuaternion &q);

    // Stored calibration parameters (set before thread start, read in worker).
    QString m_videoPath;
    QVector<QQuaternion> m_imuOrientations;
    QVector<double> m_imuTimestamps;
    double m_syncOffset = 0.0;
    double m_initialDrift = 0.0;
    int m_numSamples = 20;
    bool m_running = false;
};

#endif // IMUDRIFTCALIBRATOR_H
