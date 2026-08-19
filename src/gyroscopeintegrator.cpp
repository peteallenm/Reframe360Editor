#include "gyroscopeintegrator.h"
#include <QtMath>
#include <algorithm>
#include <cmath>
#include <QDebug>
#include <iostream>

// Mahony-style complementary filter gains. accelKp is the proportional gain
// (deg/s per rad of tilt error); accelKi is the integral gain, both passed in
// via integrate() so the user can tune them. The integral term accumulates the
// gravity-alignment error while the camera is still and feeds it back into the
// body rate, estimating and cancelling the gyro bias that would otherwise
// integrate into slow, unbounded orientation drift (most visible in roll).
// accelKi too high causes oscillation/overshoot during fast motion.
static const float kGyroGate   = 60.0f;   // deg/s: below this the accel is trusted
static const float kMagLow     = 0.9f;    // |accel| window (g) for "camera still"
static const float kMagHigh    = 1.1f;
// IIR low-pass filter removed: quaternion integration is itself an integrator;
// sensor noise at 400 Hz integrates to almost nothing. The one-pole IIR
// (alpha=0.55, ~50 Hz cutoff) added ~2-3 ms of group delay which at 300°/s
// shake rate caused ~0.9° of instantaneous error — visible in 360 renders.
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
    // Find the quietest 0.5 s window anywhere in the recording and use its
    // mean gyro as the bias. Scanning the whole recording (not just the first
    // second) handles clips that start mid-motion. The Mahony integral term
    // then refines this estimate continuously during still moments.
    const int winSamples = qMax(1, (int)(0.5 * m_sampleRate));  // 0.5 s window
    const int maxScan = qMin(samples.size(), (int)(20 * m_sampleRate)); // cap at 20 s
    if (samples.size() < winSamples)
        return QVector3D(0.0f, 0.0f, 0.0f);

    // Sliding window of total |gyro| rate; keep the window with the lowest sum.
    double bestSum = 0.0, bestX = 0.0, bestY = 0.0, bestZ = 0.0;
    bool haveBest = false;
    double sum = 0.0, sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    const int n = qMin(samples.size(), maxScan);
    for (int i = 0; i < n; i++) {
        sum  += std::abs(samples[i].gyro.x()) + std::abs(samples[i].gyro.y())
              + std::abs(samples[i].gyro.z());
        sumX += samples[i].gyro.x();
        sumY += samples[i].gyro.y();
        sumZ += samples[i].gyro.z();
        if (i >= winSamples) {
            const int j = i - winSamples;
            sum  -= std::abs(samples[j].gyro.x()) + std::abs(samples[j].gyro.y())
                  + std::abs(samples[j].gyro.z());
            sumX -= samples[j].gyro.x();
            sumY -= samples[j].gyro.y();
            sumZ -= samples[j].gyro.z();
        }
        if (i + 1 >= winSamples && (!haveBest || sum < bestSum)) {
            bestSum = sum;
            bestX = sumX; bestY = sumY; bestZ = sumZ;
            haveBest = true;
        }
    }

    if (haveBest)
        return QVector3D((float)(bestX / winSamples), (float)(bestY / winSamples),
                         (float)(bestZ / winSamples));
    return QVector3D(0.0f, 0.0f, 0.0f);
}

void GyroscopeIntegrator::integrate(const QVector<ImuSample> &samples, double sampleRate,
                                    const QQuaternion &imuToCamera,
                                    float accelKp, float accelKi)
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

    // Mahony integral term: accumulates the gravity-alignment error while the
    // accel is trusted, estimating and cancelling the gyro bias. It keeps
    // running across fast-motion sections (the accumulator is not reset when
    // the proportional gate closes), so roll/pitch drift does not re-accumulate
    // once the correction converges.
    QVector3D integralFB(0.0f, 0.0f, 0.0f);

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

        // Use the camera-frame gyro directly without IIR filtering.
        // Quaternion integration is itself an integrator; sensor noise at
        // 400 Hz integrates to almost nothing. The removed IIR (alpha=0.55)
        // added ~2-3 ms lag which caused ~0.9° error at 300°/s shake rate.
        float cx = gyroCam.x();  // camera X <- pitch
        float cy = gyroCam.y();  // camera Y <- -yaw
        float cz = gyroCam.z();  // camera Z <- -roll

        // Accelerometer gravity correction (body-frame angular velocity).
        QVector3D accel = qInv.rotatedVector(samples[i].accel);  // camera axes
        float accelMag = accel.length();
        float spinRate = std::sqrt(cx * cx + cy * cy + cz * cz);
        // Soft/adaptive gating instead of a hard on/off cutoff: the accel is
        // trusted fully below kGyroGate, fades to zero across the next
        // kGyroGate degrees/s, and is disabled above that. This avoids the
        // discontinuity a hard threshold causes when a quick roll decelerates
        // through the gate (the correction popping in/out every sample).
        // The magnitude window (0.9-1.1 g) still rejects linear/centripetal
        // acceleration.
        float gateFactor = 0.0f;
        if (accelMag > kMagLow && accelMag < kMagHigh) {
            if (spinRate < kGyroGate)
                gateFactor = 1.0f;
            else if (spinRate < 2.0f * kGyroGate)
                gateFactor = 1.0f - (spinRate - kGyroGate) / kGyroGate;
        }
        if (gateFactor > 0.0f) {
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

            // Accumulate the integral term (deg/s bias estimate) only while the
            // accel is trusted — that is the only time err carries real signal.
            // err is in rad (≈ sin θ); convert to degrees for consistency with
            // the proportional term's gain.
            const float radToDeg = 180.0f / M_PI;
            integralFB += err * (accelKi * radToDeg) * dt;

            // Add the proportional correction to the body-frame rate (rad->deg).
            const float gain = accelKp * gateFactor * radToDeg;
            cx += err.x() * gain;
            cy += err.y() * gain;
            cz += err.z() * gain;
        }

        // The integral term (estimated gyro bias) is applied to the body rate
        // unconditionally: its accumulation is gated above, but the feedback
        // must persist even while the proportional gate is closed, otherwise
        // the bias correction vanishes during exactly the fast-motion sections
        // where drift would otherwise re-accumulate.
        cx += integralFB.x();
        cy += integralFB.y();
        cz += integralFB.z();

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

QQuaternion GyroscopeIntegrator::orientationAtTime(double time, float smoothingMs) const
{
    return orientationAt(m_orientations, m_timestamps, time, smoothingMs);
}

QQuaternion GyroscopeIntegrator::orientationAtTimeUnsmoothed(double time) const
{
    if (m_orientations.isEmpty()) return QQuaternion();

    if (time <= m_timestamps.first())
        return m_orientations.first();
    if (time >= m_timestamps.last())
        return m_orientations.last();

    // Binary search for the bracketing index
    const auto it = std::lower_bound(m_timestamps.begin(), m_timestamps.end(), time);
    int i = (it == m_timestamps.end()) ? m_timestamps.size() - 1
                                       : (int)(it - m_timestamps.begin());
    if (i > 0 && m_timestamps[i] > time) i--;

    // Slerp between the two bracketing quaternions (no Gaussian smoothing)
    if (i + 1 >= m_orientations.size())
        return m_orientations.last();

    double dt = m_timestamps[i + 1] - m_timestamps[i];
    float t = (dt > 0.0) ? (float)((time - m_timestamps[i]) / dt) : 0.0f;
    t = qBound(0.0f, t, 1.0f);

    return QQuaternion::slerp(m_orientations[i], m_orientations[i + 1], t);
}

QQuaternion GyroscopeIntegrator::orientationAt(const QVector<QQuaternion> &orientations,
                                               const QVector<double> &timestamps,
                                               double time, float smoothingMs)
{
    if (orientations.isEmpty()) return QQuaternion();

    if (time <= timestamps.first())
        return orientations.first();
    if (time >= timestamps.last())
        return orientations.last();

    // Find the bracketing index with a binary search. This runs on every
    // preview/export frame and the IMU chain can be tens of thousands of
    // samples long, so a linear scan (O(n) per call) stalls playback even
    // though it never maxes a core.
    int i;
    {
        const auto it = std::lower_bound(timestamps.begin(), timestamps.end(), time);
        i = (it == timestamps.end()) ? timestamps.size() - 1
                                     : (int)(it - timestamps.begin());
        if (i > 0 && timestamps[i] > time) i--;
    }

    // Minimum window = one 30 fps video frame (~33 ms, ~13 IMU samples at 400 Hz)
    // so even at smoothingMs = 0 we average over the frame exposure time, giving
    // ~11 dB jitter reduction for free. Larger windows use a Gaussian profile
    // (sigma = window/4) so the frequency response rolls off monotonically with
    // no box-filter side lobes. All orientations are pre-computed, so this
    // centered window looks ahead in the array without any real-time latency.
    //
    // The user's smoothingMs sets the window and the 33 ms exposure average is
    // the floor. (The old rate-adaptive scaling between them was removed: it
    // starved slow pans of smoothing while offering no benefit on fast shakes.)
    const float minWindowMs = 33.0f;
    const float windowMs = qMax(minWindowMs, smoothingMs);
    const float halfMs = windowMs / 2.0f;
    const float sigmaMs = windowMs / 4.0f;

    const double t0 = time - halfMs;
    const double t1 = time + halfMs;
    int lo = i, hi = i;
    while (lo > 0 && timestamps[lo] > t0) lo--;
    while (hi < timestamps.size() - 1 && timestamps[hi] < t1) hi++;

    // Markley-style Gaussian-weighted quaternion average: accumulate the
    // quaternion vectors (sign-flipped so each is within 180° of the reference)
    // weighted by a Gaussian of the time offset, then normalize.
    const QQuaternion &ref = orientations[i];
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    float wsum = 0.0f;
    const float invSigma2 = 1.0f / (sigmaMs * sigmaMs);
    for (int j = lo; j <= hi; j++) {
        double dt = (timestamps[j] - time) * 1000.0; // ms
        float weight = std::exp(-0.5f * (float)(dt * dt) * invSigma2);
        QQuaternion q = orientations[j];
        if (QQuaternion::dotProduct(q, ref) < 0.0f)
            q = QQuaternion(-q.scalar(), -q.x(), -q.y(), -q.z());
        x += q.x() * weight; y += q.y() * weight;
        z += q.z() * weight; w += q.scalar() * weight;
        wsum += weight;
    }
    if (wsum <= 0.0f)
        return orientations[i];

    QQuaternion avg(w, x, y, z);
    avg /= wsum;
    avg.normalize();
    return avg;
}