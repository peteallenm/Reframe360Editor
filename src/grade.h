#ifndef GRADE_H
#define GRADE_H

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// The colour grade, in one place.
//
// This is the C++ mirror of the grading block in shaders/project.frag. It used
// to exist only inside the CPU exporter's rasterizer loop; the curve editor's
// histogram needs exactly the same maths to show what the grade does, and a
// second hand-written copy of shader maths is precisely how the export once
// ended up rendering a different pitch from the preview. One definition, three
// callers (shader, CPU exporter, histogram).
//
// Deliberately free of Qt types and QObject so both the decoder and the
// exporter can include it without pulling in ColorGrade.
//
// Channel values are 0..255 and are NOT clamped here -- the shader works in
// unclamped float too, and clamping early would change the result of the
// later stages. Clamp once at the end, where you quantise.
//
// Tone curves are NOT applied here. They are a 256-entry LUT the caller
// indexes last, after clamping; see applyCurveLut().
// ---------------------------------------------------------------------------

struct GradeParams {
    double brightness = 0.0;   // additive offset (-1..1)
    double contrast = 1.0;     // scale about 0.5 (0..2)
    double saturation = 1.0;   // mix with luma (0..2)
    double pop = 0.0;          // midtone contrast / clarity (-1..1)
    double brightLows = 0.0, brightLowMids = 0.0, brightHighMids = 0.0, brightHighs = 0.0;
    double redLows = 0.0, redMids = 0.0, redHighs = 0.0;
    double greenLows = 0.0, greenMids = 0.0, greenHighs = 0.0;
    double blueLows = 0.0, blueMids = 0.0, blueHighs = 0.0;

    // True when every control is at its neutral value, so callers can skip the
    // work entirely.
    bool isNeutral() const
    {
        return brightness == 0.0 && contrast == 1.0 && saturation == 1.0 && pop == 0.0
            && brightLows == 0.0 && brightLowMids == 0.0 && brightHighMids == 0.0 && brightHighs == 0.0
            && redLows == 0.0 && redMids == 0.0 && redHighs == 0.0
            && greenLows == 0.0 && greenMids == 0.0 && greenHighs == 0.0
            && blueLows == 0.0 && blueMids == 0.0 && blueHighs == 0.0;
    }
};

// Apply the grade in project.frag's order, in place, on 0..255 channels.
inline void applyGrade(const GradeParams &p, double &r, double &g, double &b)
{
    auto clampUnit = [](double v) { return std::max(0.0, std::min(1.0, v)); };

    const double lumaN = clampUnit((0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0);
    const double wL = (1.0 - lumaN) * (1.0 - lumaN);
    const double wH = lumaN * lumaN;
    const double wM = 1.0 - wL - wH;
    // The mid band is split into low-mids / high-mids: they share the old wM
    // weight and cross over exactly at luma 0.5.
    const double tMid = std::max(-1.0, std::min(1.0, (lumaN - 0.5) * 4.0));
    const double wLM = wM * 0.5 * (1.0 - tMid);
    const double wHM = wM * 0.5 * (1.0 + tMid);

    const double lumaBand = p.brightLows * wL + p.brightLowMids * wLM
                          + p.brightHighMids * wHM + p.brightHighs * wH;
    r += 255.0 * (lumaBand + p.redLows * wL + p.redMids * wM + p.redHighs * wH);
    g += 255.0 * (lumaBand + p.greenLows * wL + p.greenMids * wM + p.greenHighs * wH);
    b += 255.0 * (lumaBand + p.blueLows * wL + p.blueMids * wM + p.blueHighs * wH);

    const double lumaN2 = clampUnit((0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0);
    const double midW = 1.0 - std::abs(lumaN2 * 2.0 - 1.0);
    r += p.pop * 0.2 * midW * (r - 127.5);
    g += p.pop * 0.2 * midW * (g - 127.5);
    b += p.pop * 0.2 * midW * (b - 127.5);

    const double luma255 = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    r = luma255 + (r - luma255) * p.saturation;
    g = luma255 + (g - luma255) * p.saturation;
    b = luma255 + (b - luma255) * p.saturation;

    r = (r - 127.5) * p.contrast + 127.5;
    g = (g - 127.5) * p.contrast + 127.5;
    b = (b - 127.5) * p.contrast + 127.5;

    r += p.brightness * 255.0;
    g += p.brightness * 255.0;
    b += p.brightness * 255.0;
}

// Tone curves, applied last: lut is a 256x1 RGBA8888 scanline (R/G/B are the
// three per-channel LUTs). Indices must already be clamped to 0..255.
inline void applyCurveLut(const unsigned char *lut, unsigned char &r, unsigned char &g, unsigned char &b)
{
    r = lut[int(r) * 4 + 0];
    g = lut[int(g) * 4 + 1];
    b = lut[int(b) * 4 + 2];
}

#endif // GRADE_H
