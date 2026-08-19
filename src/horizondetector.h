#ifndef HORIZONDETECTOR_H
#define HORIZONDETECTOR_H

#include <QImage>
#include <cstdint>

struct HorizonResult {
    bool valid = false;       // detection succeeded
    double rollDeg = 0.0;    // horizon tilt angle in degrees (positive = clockwise)
    double pitchNorm = 0.0;  // horizon vertical position, normalized: 0 = center, negative = above, positive = below
    int linesUsed = 0;       // number of horizontal lines used in detection
};

class HorizonDetector {
public:
    // Detect the horizon in a QImage (any format, converted internally).
    static HorizonResult detect(const QImage &image);

    // Detect the horizon from raw RGB24 data (width*height*3 bytes, row-major).
    // Avoids the QImage copy when the caller already has raw pixels.
    static HorizonResult detectFromRgb24(const uint8_t *rgb, int width, int height);
};

#endif // HORIZONDETECTOR_H
