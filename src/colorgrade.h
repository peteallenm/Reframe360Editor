#ifndef COLORGRADE_H
#define COLORGRADE_H

#include <QObject>

// Colour grading parameters applied in the viewer shader (project.frag) and by
// the exporter's software renderer, so previews and exports always match.
//
// "Image" adjustments act on the whole image:
//   brightness : additive offset  (-1..1, 0 = neutral)
//   contrast   : scale about 0.5  (0..2, 1 = neutral)
//   saturation : mix with luma    (0..2, 1 = neutral)
//   pop        : midtone contrast ("clarity") (-1..1, 0 = neutral)
//
// "Colours" is a 3-way (lows/mids/highs) corrector, once on overall
// brightness and once per channel. Each slider shifts that range of the
// luma (for bright*) or channel (for red*/green*/blue*) by the slider value
// (-1..1, 0 = neutral), weighted by how strongly the pixel falls in the
// shadows / midtones / highlights.
class ColorGrade : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double brightness READ brightness WRITE setBrightness NOTIFY brightnessChanged)
    Q_PROPERTY(double contrast READ contrast WRITE setContrast NOTIFY contrastChanged)
    Q_PROPERTY(double saturation READ saturation WRITE setSaturation NOTIFY saturationChanged)
    Q_PROPERTY(double pop READ pop WRITE setPop NOTIFY popChanged)
    Q_PROPERTY(double brightLows READ brightLows WRITE setBrightLows NOTIFY brightLowsChanged)
    Q_PROPERTY(double brightLowMids READ brightLowMids WRITE setBrightLowMids NOTIFY brightLowMidsChanged)
    Q_PROPERTY(double brightHighMids READ brightHighMids WRITE setBrightHighMids NOTIFY brightHighMidsChanged)
    Q_PROPERTY(double brightHighs READ brightHighs WRITE setBrightHighs NOTIFY brightHighsChanged)
    Q_PROPERTY(double redLows READ redLows WRITE setRedLows NOTIFY redLowsChanged)
    Q_PROPERTY(double redMids READ redMids WRITE setRedMids NOTIFY redMidsChanged)
    Q_PROPERTY(double redHighs READ redHighs WRITE setRedHighs NOTIFY redHighsChanged)
    Q_PROPERTY(double greenLows READ greenLows WRITE setGreenLows NOTIFY greenLowsChanged)
    Q_PROPERTY(double greenMids READ greenMids WRITE setGreenMids NOTIFY greenMidsChanged)
    Q_PROPERTY(double greenHighs READ greenHighs WRITE setGreenHighs NOTIFY greenHighsChanged)
    Q_PROPERTY(double blueLows READ blueLows WRITE setBlueLows NOTIFY blueLowsChanged)
    Q_PROPERTY(double blueMids READ blueMids WRITE setBlueMids NOTIFY blueMidsChanged)
    Q_PROPERTY(double blueHighs READ blueHighs WRITE setBlueHighs NOTIFY blueHighsChanged)

public:
    explicit ColorGrade(QObject *parent = nullptr);

    double brightness() const { return m_brightness; }
    void setBrightness(double v);
    double contrast() const { return m_contrast; }
    void setContrast(double v);
    double saturation() const { return m_saturation; }
    void setSaturation(double v);
    double pop() const { return m_pop; }
    void setPop(double v);

    double brightLows() const { return m_brightLows; }
    void setBrightLows(double v);
    double brightLowMids() const { return m_brightLowMids; }
    void setBrightLowMids(double v);
    double brightHighMids() const { return m_brightHighMids; }
    void setBrightHighMids(double v);
    double brightHighs() const { return m_brightHighs; }
    void setBrightHighs(double v);

    double redLows() const { return m_redLows; }
    void setRedLows(double v);
    double redMids() const { return m_redMids; }
    void setRedMids(double v);
    double redHighs() const { return m_redHighs; }
    void setRedHighs(double v);

    double greenLows() const { return m_greenLows; }
    void setGreenLows(double v);
    double greenMids() const { return m_greenMids; }
    void setGreenMids(double v);
    double greenHighs() const { return m_greenHighs; }
    void setGreenHighs(double v);

    double blueLows() const { return m_blueLows; }
    void setBlueLows(double v);
    double blueMids() const { return m_blueMids; }
    void setBlueMids(double v);
    double blueHighs() const { return m_blueHighs; }
    void setBlueHighs(double v);

    // Reset every parameter to its neutral value.
    Q_INVOKABLE void reset();

signals:
    void brightnessChanged();
    void contrastChanged();
    void saturationChanged();
    void popChanged();
    void brightLowsChanged();
    void brightLowMidsChanged();
    void brightHighMidsChanged();
    void brightHighsChanged();
    void redLowsChanged();
    void redMidsChanged();
    void redHighsChanged();
    void greenLowsChanged();
    void greenMidsChanged();
    void greenHighsChanged();
    void blueLowsChanged();
    void blueMidsChanged();
    void blueHighsChanged();

private:
    double m_brightness;
    double m_contrast;
    double m_saturation;
    double m_pop;
    double m_brightLows, m_brightLowMids, m_brightHighMids, m_brightHighs;
    double m_redLows, m_redMids, m_redHighs;
    double m_greenLows, m_greenMids, m_greenHighs;
    double m_blueLows, m_blueMids, m_blueHighs;
};

#endif // COLORGRADE_H
