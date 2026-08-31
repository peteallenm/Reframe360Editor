// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef VISUALROTATION_H
#define VISUALROTATION_H

#include <QObject>
#include <QVector>
#include <QQuaternion>
#include <QVector3D>
#include <QString>
#include <QThread>
#include <functional>

#include <opencv2/core.hpp>

class CalibrationProfile;

struct VisualRotationPair {
    double t0, t1;           // timestamps of the two frames
    QQuaternion deltaR;      // rotation from t0 to t1
    int inliers;             // number of matches after outlier rejection
    double rmsDeg;           // RMS residual in degrees

    // Single definition of "this pair is trustworthy enough to fit against".
    // The producer accepts a much looser bar (HOP_MIN_INLIERS / HOP_MAX_RMS_DEG)
    // because it is choosing a hop length; consumers apply their own threshold
    // through this predicate so the bar lives in one place per consumer rather
    // than as three independently-drifting inline tests.
    bool isReliable(int minInliers, double maxRmsDeg) const {
        return inliers >= minInliers && rmsDeg <= maxRmsDeg;
    }
};

Q_DECLARE_METATYPE(VisualRotationPair)

class VisualRotationComputer : public QObject {
    Q_OBJECT
public:
    explicit VisualRotationComputer(QObject *parent = nullptr);
    ~VisualRotationComputer();

    // Analysis decode source supplied by the caller, used instead of deriving

    // the "_thm" sibling from the video path. On Android the video is an

    // opaque content:// URI, so the sibling cannot be derived -- but the user

    // selected the proxy explicitly, so pass it through. Still validated

    // against the original (frame count and duration) before it is trusted.

    void setDecodeSourceOverride(const QString &path) { m_decodeOverride = path; }


    void compute(const QString &videoPath,
                 const CalibrationProfile *calibration,
                 int frameSkip = 3);  // process every (frameSkip+1)th frame

    // Which fisheye halves contribute correspondences to the rotation solve.
    // Bit 0 = front, bit 1 = rear; default = both. Diagnostic hook: the two
    // halves are normally pooled into a single Kabsch solve, so if one half's
    // bearing convention were wrong its correspondences would pull the solved
    // rotation toward identity and the pooled result would under-measure.
    // Solving each half alone isolates that.
    // Analyse only the first N seconds (0 = whole clip). Decoding stops at the
    // limit, so a short window is cheap AND is sampled densely: the
    // MAX_DECODED_FRAMES budget is spread over the limit rather than the whole
    // clip, which matters when looking at high-frequency motion.
    void setTimeLimit(double seconds) { m_timeLimit = seconds; }

    enum LensMask { LensFront = 1, LensRear = 2, LensBoth = 3 };
    void setLensMask(int mask) { m_lensMask = mask; }
    int lensMask() const { return m_lensMask; }

    struct LensParams {
        double cx, cy;       // center in normalized half-frame coords
        double radius;        // fisheye radius in normalized coords
        double k1, k2;        // distortion coefficients
        double rotation;      // lens rotation in degrees
        bool hflip;           // horizontal flip flag
        bool isRear;          // false for front, true for rear
        // Mirror the azimuth when back-projecting. The rear lens looks along
        // +Z while the front looks along -Z, so the two images see the sphere
        // from opposite sides and the shared azimuth convention gives the rear
        // bearings the opposite handedness. Explicit rather than implied by
        // isRear so the convention can be A/B tested.
        bool mirrorAzimuth = false;
    };

    // Bearing back-projection: pixel in half-frame normalized coords → unit vector
    static QVector3D pixelToBearing(double px, double py, const LensParams &lens);

signals:
    void progressChanged(double fraction, const QString &status);
    void rotationComputed(const QVector<VisualRotationPair> &pairs);
    void computationFailed(const QString &error);

private:
    QString m_decodeOverride;
    struct FrameData {
        double timestamp;
        cv::Mat grayFront;    // grayscale front fisheye half
        cv::Mat grayRear;     // grayscale rear fisheye half
        // Dimensions of those halves, kept separately because the pixel data
        // is RELEASED once ORB has run: after feature extraction nothing reads
        // the images again, only their size. Holding 1400 frame pairs at
        // 640x720 was ~645 MB, which a phone cannot spare.
        int halfWidth = 0;
        int halfHeight = 0;
    };

    // ORB output for one frame, computed ONCE up front. The adaptive hop search
    // revisits the same frame as both the B of one pair and the A of the next
    // (and again on every narrowed retry), so detecting inside matchFeatures
    // re-ran ORB about twice per frame for nothing.
    struct FrameFeatures {
        std::vector<cv::KeyPoint> kpFront, kpRear;
        cv::Mat descFront, descRear;
    };

    // Pick the file to decode for analysis: the camera's *_thm proxy when it
    // matches the original frame-for-frame, else the original.
    QString chooseDecodeSource(const QString &videoPath) const;

    // Decode frames from video, returning timestamps and grayscale half-images
    bool decodeFrames(const QString &videoPath, int frameSkip,
                      QVector<FrameData> &frames,
                      std::function<bool(double, const QString&)> progressCb);

    // Detect ORB features and match between consecutive frame pairs
    bool matchFeatures(const FrameData &frameA, const FrameData &frameB,
                       const FrameFeatures &featA, const FrameFeatures &featB,
                       const LensParams &frontLens, const LensParams &rearLens,
                       QVector<QVector3D> &bearingsA,
                       QVector<QVector3D> &bearingsB,
                       int lensMask = LensBoth);

    // Detect ORB on every frame, spread across all cores.
    static void computeFeatures(const QVector<FrameData> &frames,
                                QVector<FrameFeatures> &out,
                                int lensMask,
                                const LensParams &frontLens,
                                const LensParams &rearLens,
                                const std::function<void(int)> &progressCb);

    // Solve rotation from matched bearing pairs using SVD (Kabsch/Wahba).
    // initialGuess seeds the first iteration (use the previous pair's rotation
    // so fast, sustained rotation keeps features within the inlier threshold
    // instead of collapsing to zero inliers).
    QQuaternion solveRotation(const QVector<QVector3D> &bearingsA,
                              const QVector<QVector3D> &bearingsB,
                              int &inliers, double &rmsDeg,
                              const QQuaternion &initialGuess = QQuaternion());

    int m_lensMask = LensBoth;
    double m_timeLimit = 0.0;
};

#endif // VISUALROTATION_H
