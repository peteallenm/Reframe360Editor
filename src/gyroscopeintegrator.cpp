#include "gyroscopeintegrator.h"
#include <QtMath>
#include <cmath>
#include <QDebug>
#include <iostream>

// Mahony-style complementary filter gains.
static const float kAccelKp    = 0.35f;   // proportional gain, rad/s per rad of tilt
static const float kGyroGate   = 60.0f;   // deg/s: below this the accel is trusted
static const float kMagLow     = 0.9f;    // |accel| window (g) for "camera still"
static const float kMagHigh    = 1.1f;
// World frame convention: +Y is the world "up" vector used to seed and correct
// the orientation (the Mahony filter drives current^-1 * kWorldUp toward the
// measured acceleration). For a level camera the accelerometer reads
// specific-force ≈ +Y in camera axes (after the IMU→camera quaternion), so the
// seed rotationTo(accel, +Y) is near-identity and the default view is level.
//
// kFlipRoll is a constant 180° roll about the forward (Z) axis that un-flips
// the YI camera's inherently 180°-flipped fisheye video (with stabilization
// OFF the video is upside down). It is applied ONLY when storing each
// orientation (current * kFlipRoll), never inside the seed or the gyro
// integration: baking it into the seed would rotate the body frame the gyro
// integrates in, which flips the pitch/yaw stabilization direction and makes
// the accelerometer correction fight the gyro.
static const QVector3D kWorldUp(0.0f, 1.0f, 0.0f);
static const QQuaternion kFlipRoll(0.0f, 0.0f, 0.0f, 1.0f);  // 180° about Z (w,x,y,z)

GyroscopeIntegrator::GyroscopeIntegrator(QObject *parent)
    : QObject(parent)
    , m_sampleRate(400.0)
{
}

QVector3D GyroscopeIntegrator::computeGyroBias(const QVector<ImuSample> &samples) const
{
    // Compute mean gyro during the first samples with low total rate
    // (camera should be still at the start).
    double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    int count = 0;
    const int maxBiasSamples = 400;  // 1 second at 400 Hz

    for (int i = 0; i < qMin(maxBiasSamples, samples.size()); i++) {
        float rate = std::abs(samples[i].gyro.x())
                   + std::abs(samples[i].gyro.y())
                   + std::abs(samples[i].gyro.z());
        if (rate < 30.0f) {  // low total rate -> stationary
            sumX += samples[i].gyro.x();
            sumY += samples[i].gyro.y();
            sumZ += samples[i].gyro.z();
            count++;
        }
    }

    if (count > 0)
        return QVector3D((float)(sumX / count), (float)(sumY / count), (float)(sumZ / count));
    return QVector3D(0.0f, 0.0f, 0.0f);
}

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate,
                                    const QQuaternion &imuToCamera)
{
    m_sampleRate = sampleRate;
    m_orientations.clear();
    m_timestamps.clear();

    if (samples.isEmpty()) return;

    m_orientations.reserve(samples.size());
    m_timestamps.reserve(samples.size());

    QVector3D bias = computeGyroBias(samples);
    qDebug() << "Gyro bias:" << bias.x() << bias.y() << bias.z() << "deg/s";

    // The parser emits s.gyro = (roll, pitch, yaw) = (gx, gy, gz), where
    // gy = pitch, gz = yaw, gx = roll. Map these to camera axes:
    // camera X <- pitch, camera Y <- yaw, camera Z <- roll.
    //
    // Accelerometer fusion: the accelerometer at rest reads the specific force
    // (~1 g, pointing away from gravity), which gives an absolute "which way is
    // up" reference that the gyro integration (which drifts) lacks. The raw
    // sensor accel is rotated into camera axes with the header's IMU->camera
    // quaternion: q^-1 * accel ≈ +Y (up) for a level camera. The world frame
    // is fixed so that +Y is up in the integration frame. The first
    // accelerometer reading seeds the orientation and a Mahony-style
    // proportional correction continuously nudges the predicted up direction
    // toward the measured one (gated to only trust the accel while the camera
    // is nearly still, so linear/centripetal acceleration during fast motion
    // does not corrupt the estimate). Each stored orientation is then
    // post-multiplied by kFlipRoll (180° roll) to un-flip the YI camera's
    // inherently-upside-down video for display.
    const QQuaternion qInv = imuToCamera.conjugated();

    // Seed the orientation from the first camera-still sample (|accel| near 1g
    // and |gyro| low). Scan up to 1 second of samples so that recordings which
    // start while the camera is being picked up / moved don't corrupt the seed.
    // The gate mirrors the Mahony correction gate: 0.9-1.1 g + gyro < 60 deg/s.
    QQuaternion current;
    {
        bool seeded = false;
        const int seedWindow = qMin((int)m_sampleRate, samples.size()); // 1 s
        for (int k = 0; k < seedWindow && !seeded; k++) {
            QVector3D a0 = qInv.rotatedVector(samples[k].accel);
            float m0 = a0.length();
            if (m0 < kMagLow || m0 > kMagHigh)
                continue;
            // Also require the gyro to be low so we don't seed mid-motion.
            float gRaw = samples[k].gyro.length();
            if (gRaw >= kGyroGate)
                continue;
            // Seed the orientation by leveling the camera (accel -> +Y) only.
            // The 180° video un-flip (kFlipRoll) is applied at storage time,
            // NOT here — see the kWorldUp/kFlipRoll comment above.
            current = QQuaternion::rotationTo(a0 / m0,
                                              QVector3D(0.0f, 1.0f, 0.0f));
            qDebug() << "IMU seed: sample" << k << "accel_cam"
                     << a0.x() << a0.y() << a0.z() << "mag" << m0;
            seeded = true;
        }
        if (!seeded) {
            // Fallback: no still moment found in first second; use the first
            // sample with any reasonable accel magnitude (avoids all-zeros).
            for (int k = 0; k < qMin(10, samples.size()); k++) {
                QVector3D a0 = qInv.rotatedVector(samples[k].accel);
                float m0 = a0.length();
                if (m0 > 0.3f) {
                    current = QQuaternion::rotationTo(a0 / m0,
                                                      QVector3D(0.0f, 1.0f, 0.0f));
                    qDebug() << "IMU seed (fallback): sample" << k
                             << "accel_cam" << a0.x() << a0.y() << a0.z();
                    break;
                }
            }
        }
    }

    for (int i = 0; i < samples.size(); i++) {
        m_timestamps.append(samples[i].timestamp);
        // Store the video-un-flip (180° roll) as part of the output orientation.
        // The shader applies the conjugate, so output = current * kFlipRoll
        // makes the shader sample kFlipRoll * current^-1 * ray (first the
        // camera counter-rotation, then the rolled-video pixel correction).
        m_orientations.append(current * kFlipRoll);

        if (i + 1 >= samples.size()) break;

        float dt = (float)(samples[i + 1].timestamp - samples[i].timestamp);
        if (dt <= 0.0f) dt = (float)(1.0 / m_sampleRate);

        // Rotate the bias-corrected gyro from sensor axes into camera axes with
        // the same qInv used for the accelerometer, so the gyro and the Mahony
        // correction below share one frame (level accel -> +Y). No axis flip:
        // flipping X/Y here would rotate the body frame the gyro integrates in
        // and fight the accelerometer correction (doubling pitch/yaw motion).
        QVector3D gyroCorr = samples[i].gyro - bias;                 // sensor axes
        QVector3D gyroCam = qInv.rotatedVector(gyroCorr);            // camera axes
        float cx = gyroCam.x();  // camera X <- pitch
        float cy = gyroCam.y();  // camera Y <- -yaw
        float cz = gyroCam.z();  // camera Z <- -roll

        // Accelerometer gravity correction (body-frame angular velocity).
        QVector3D accel = qInv.rotatedVector(samples[i].accel);  // camera axes
        float accelMag = accel.length();
        float spinRate = std::sqrt(cx * cx + cy * cy + cz * cz);
        bool still = (accelMag > kMagLow && accelMag < kMagHigh
                      && spinRate < kGyroGate);
        if (still) {
            // Predicted gravity-up in the body frame vs measured. The accel
            // reads specific force (up), so up is +accel in camera axes.
            QVector3D upPred = current.conjugated().rotatedVector(kWorldUp);
            QVector3D upMeas = accel / accelMag;

            // cross(upPred, upMeas) is the body-frame axis to rotate around to
            // align them. Its magnitude ≈ sin(angle) which is zero for both 0°
            // (already aligned) and 180° (anti-parallel, singularity) errors.
            // Handle the 180° case: pick any perpendicular axis and apply a
            // finite correction kick to escape the singularity.
            float dotPU = QVector3D::dotProduct(upPred, upMeas);
            QVector3D err;
            if (dotPU < -0.999f) {
                // Nearly anti-parallel: kick toward alignment using a
                // perpendicular axis (choose the axis least aligned with upPred
                // to avoid a near-zero cross product here too).
                QVector3D perp = (std::abs(upPred.x()) < 0.9f)
                                 ? QVector3D(1, 0, 0) : QVector3D(0, 1, 0);
                err = QVector3D::crossProduct(upPred, perp).normalized();
            } else {
                err = QVector3D::crossProduct(upPred, upMeas); // rad (≈ sin θ)
            }

            // Add the correction to the body-frame rate (rad/s -> deg/s).
            const float radToDeg = 180.0f / M_PI;
            cx += err.x() * kAccelKp * radToDeg;
            cy += err.y() * kAccelKp * radToDeg;
            cz += err.z() * kAccelKp * radToDeg;
        }

        float gxRad = qDegreesToRadians(cx);
        float gyRad = qDegreesToRadians(cy);
        float gzRad = qDegreesToRadians(cz);

        float mag = std::sqrt(gxRad * gxRad + gyRad * gyRad + gzRad * gzRad);
        if (mag < 0.0001f) continue;

        QVector3D axis(gxRad / mag, gyRad / mag, gzRad / mag);
        float angle = mag * dt;
        QQuaternion delta = QQuaternion::fromAxisAndAngle(axis, qRadiansToDegrees(angle));
        current = (current * delta).normalized();
    }

    qDebug() << "Gyro integrator:" << m_orientations.size() << "keyframes";
}

QQuaternion GyroscopeIntegrator::orientationAtTime(double time) const
{
    return orientationAt(m_orientations, m_timestamps, time);
}

QQuaternion GyroscopeIntegrator::orientationAt(const QVector<QQuaternion> &orientations,
                                               const QVector<double> &timestamps,
                                               double time)
{
    if (orientations.isEmpty()) return QQuaternion();

    if (time <= timestamps.first())
        return orientations.first();
    if (time >= timestamps.last())
        return orientations.last();

    for (int i = 0; i < timestamps.size() - 1; i++) {
        if (time >= timestamps[i] && time < timestamps[i + 1]) {
            float t = (float)((time - timestamps[i]) /
                              (timestamps[i + 1] - timestamps[i]));
            return QQuaternion::slerp(orientations[i], orientations[i + 1], t);
        }
    }

    return orientations.last();
}