#include "horizondetector.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

// Maximum width for internal processing (speed vs accuracy tradeoff).
static const int kMaxProcessWidth = 640;

// Lines within ±30° of horizontal are considered horizon candidates.
// Hough theta is measured from the x-axis, so horizontal = 90°.
static const double kHorizAngleMin = 60.0;   // degrees
static const double kHorizAngleMax = 120.0;

// Cluster lines whose angles differ by less than this.
static const double kClusterAngleTol = 5.0;  // degrees

// Minimum number of horizontal lines to accept a detection.
static const int kMinLines = 3;

HorizonResult HorizonDetector::detect(const QImage &image)
{
    QImage rgb = image.convertToFormat(QImage::Format_RGB888);
    return detectFromRgb24(rgb.constBits(), rgb.width(), rgb.height());
}

HorizonResult HorizonDetector::detectFromRgb24(const uint8_t *rgb, int width, int height)
{
    HorizonResult result;
    if (!rgb || width <= 0 || height <= 0)
        return result;

    // Wrap the raw RGB data in a cv::Mat (no copy).
    cv::Mat rgbMat(height, width, CV_8UC3, const_cast<uint8_t *>(rgb));

    // Convert to grayscale.
    cv::Mat gray;
    cv::cvtColor(rgbMat, gray, cv::COLOR_RGB2GRAY);

    // Downscale for speed if the image is wide.
    double scale = 1.0;
    if (width > kMaxProcessWidth) {
        scale = (double)kMaxProcessWidth / width;
        int newW = kMaxProcessWidth;
        int newH = (int)(height * scale);
        cv::resize(gray, gray, cv::Size(newW, newH));
    }

    // Gaussian blur to suppress noise.
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    // Canny edge detection.
    cv::Mat edges;
    cv::Canny(gray, edges, 50, 150);

    // Standard Hough line transform (better angle resolution than probabilistic).
    std::vector<cv::Vec2f> houghLines;
    int houghThreshold = 80;
    cv::HoughLines(edges, houghLines, 1, CV_PI / 180.0, houghThreshold);

    if (houghLines.empty())
        return result;

    // Filter for near-horizontal lines (theta in [60°, 120°]).
    struct HLine {
        double rho;
        double thetaDeg;
        float votes;
    };
    std::vector<HLine> horizLines;
    horizLines.reserve(houghLines.size());

    for (const auto &hl : houghLines) {
        double thetaDeg = hl[1] * 180.0 / CV_PI;
        if (thetaDeg >= kHorizAngleMin && thetaDeg <= kHorizAngleMax) {
            horizLines.push_back({(double)hl[0], thetaDeg, 0.0f});
        }
    }

    if ((int)horizLines.size() < kMinLines)
        return result;

    // Cluster lines by angle. Find the largest cluster.
    // Sort by angle first.
    std::sort(horizLines.begin(), horizLines.end(),
              [](const HLine &a, const HLine &b) { return a.thetaDeg < b.thetaDeg; });

    // Greedy clustering: start a new cluster when the angle gap exceeds the tolerance.
    std::vector<std::vector<int>> clusters;
    std::vector<int> currentCluster;
    currentCluster.push_back(0);

    for (int i = 1; i < (int)horizLines.size(); i++) {
        if (horizLines[i].thetaDeg - horizLines[i - 1].thetaDeg > kClusterAngleTol) {
            clusters.push_back(std::move(currentCluster));
            currentCluster.clear();
        }
        currentCluster.push_back(i);
    }
    clusters.push_back(std::move(currentCluster));

    // Find the largest cluster.
    int bestClusterIdx = 0;
    for (int c = 1; c < (int)clusters.size(); c++) {
        if (clusters[c].size() > clusters[bestClusterIdx].size())
            bestClusterIdx = c;
    }

    const auto &bestCluster = clusters[bestClusterIdx];
    if ((int)bestCluster.size() < kMinLines)
        return result;

    // Compute weighted average angle and rho from the best cluster.
    // Weight by the Hough accumulator value (votes). Since cv::HoughLines
    // doesn't directly return votes, we weight uniformly (all lines in the
    // cluster are equally valid horizon candidates).
    double sumTheta = 0.0;
    double sumRho = 0.0;
    double weightSum = 0.0;

    for (int idx : bestCluster) {
        // Weight by a proxy for line strength: lines closer to 90° (perfectly
        // horizontal) get slightly higher weight. This is a soft prior.
        double angleDist = std::abs(horizLines[idx].thetaDeg - 90.0);
        double w = 1.0 / (1.0 + angleDist * 0.1);

        sumTheta += horizLines[idx].thetaDeg * w;
        sumRho += horizLines[idx].rho * w;
        weightSum += w;
    }

    double avgThetaDeg = sumTheta / weightSum;
    double avgRho = sumRho / weightSum;

    // Roll angle: the horizon tilt. Theta=90° means perfectly horizontal (0 roll).
    // Positive roll = horizon tilts clockwise (left side up).
    double rollDeg = 90.0 - avgThetaDeg;

    // Pitch: the y-position of the horizon line relative to image center.
    // For a line rho = x*cos(theta) + y*sin(theta), at x = width/2:
    //   y = (rho - (width/2)*cos(theta)) / sin(theta)
    double thetaRad = avgThetaDeg * CV_PI / 180.0;
    double sinTheta = std::sin(thetaRad);
    double cosTheta = std::cos(thetaRad);

    int procW = gray.cols;
    int procH = gray.rows;

    double horizonY = 0.0;
    if (std::abs(sinTheta) > 1e-6)
        horizonY = (avgRho - (procW / 2.0) * cosTheta) / sinTheta;

    // Normalize: 0 = center, negative = above center, positive = below.
    double pitchNorm = (horizonY - procH / 2.0) / (procH / 2.0);

    // Clamp pitch to reasonable range.
    pitchNorm = std::max(-1.0, std::min(1.0, pitchNorm));

    result.valid = true;
    result.rollDeg = rollDeg;
    result.pitchNorm = pitchNorm;
    result.linesUsed = (int)bestCluster.size();

    return result;
}
