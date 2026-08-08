#include "colorgrade.h"

ColorGrade::ColorGrade(QObject *parent)
    : QObject(parent)
    , m_brightness(0.0)
    , m_contrast(1.0)
    , m_saturation(1.0)
    , m_pop(0.0)
    , m_brightLows(0.0), m_brightLowMids(0.0), m_brightHighMids(0.0), m_brightHighs(0.0)
    , m_redLows(0.0), m_redMids(0.0), m_redHighs(0.0)
    , m_greenLows(0.0), m_greenMids(0.0), m_greenHighs(0.0)
    , m_blueLows(0.0), m_blueMids(0.0), m_blueHighs(0.0)
{
}

void ColorGrade::setBrightness(double v) { if (qFuzzyCompare(m_brightness, v)) return; m_brightness = v; emit brightnessChanged(); }
void ColorGrade::setContrast(double v) { if (qFuzzyCompare(m_contrast, v)) return; m_contrast = v; emit contrastChanged(); }
void ColorGrade::setSaturation(double v) { if (qFuzzyCompare(m_saturation, v)) return; m_saturation = v; emit saturationChanged(); }
void ColorGrade::setPop(double v) { if (qFuzzyCompare(m_pop, v)) return; m_pop = v; emit popChanged(); }

void ColorGrade::setBrightLows(double v) { if (qFuzzyCompare(m_brightLows, v)) return; m_brightLows = v; emit brightLowsChanged(); }
void ColorGrade::setBrightLowMids(double v) { if (qFuzzyCompare(m_brightLowMids, v)) return; m_brightLowMids = v; emit brightLowMidsChanged(); }
void ColorGrade::setBrightHighMids(double v) { if (qFuzzyCompare(m_brightHighMids, v)) return; m_brightHighMids = v; emit brightHighMidsChanged(); }
void ColorGrade::setBrightHighs(double v) { if (qFuzzyCompare(m_brightHighs, v)) return; m_brightHighs = v; emit brightHighsChanged(); }

void ColorGrade::setRedLows(double v) { if (qFuzzyCompare(m_redLows, v)) return; m_redLows = v; emit redLowsChanged(); }
void ColorGrade::setRedMids(double v) { if (qFuzzyCompare(m_redMids, v)) return; m_redMids = v; emit redMidsChanged(); }
void ColorGrade::setRedHighs(double v) { if (qFuzzyCompare(m_redHighs, v)) return; m_redHighs = v; emit redHighsChanged(); }

void ColorGrade::setGreenLows(double v) { if (qFuzzyCompare(m_greenLows, v)) return; m_greenLows = v; emit greenLowsChanged(); }
void ColorGrade::setGreenMids(double v) { if (qFuzzyCompare(m_greenMids, v)) return; m_greenMids = v; emit greenMidsChanged(); }
void ColorGrade::setGreenHighs(double v) { if (qFuzzyCompare(m_greenHighs, v)) return; m_greenHighs = v; emit greenHighsChanged(); }

void ColorGrade::setBlueLows(double v) { if (qFuzzyCompare(m_blueLows, v)) return; m_blueLows = v; emit blueLowsChanged(); }
void ColorGrade::setBlueMids(double v) { if (qFuzzyCompare(m_blueMids, v)) return; m_blueMids = v; emit blueMidsChanged(); }
void ColorGrade::setBlueHighs(double v) { if (qFuzzyCompare(m_blueHighs, v)) return; m_blueHighs = v; emit blueHighsChanged(); }

void ColorGrade::reset()
{
    setBrightness(0.0);
    setContrast(1.0);
    setSaturation(1.0);
    setPop(0.0);
    setBrightLows(0.0);
    setBrightLowMids(0.0);
    setBrightHighMids(0.0);
    setBrightHighs(0.0);
    setRedLows(0.0);
    setRedMids(0.0);
    setRedHighs(0.0);
    setGreenLows(0.0);
    setGreenMids(0.0);
    setGreenHighs(0.0);
    setBlueLows(0.0);
    setBlueMids(0.0);
    setBlueHighs(0.0);
}
