#include "colorgrade.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <algorithm>
#include <cmath>

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
    for (auto &c : m_curves) c = { QPointF(0, 0), QPointF(1, 1) };
    rebuildLut();
}

// Monotone cubic Hermite interpolation (Fritsch-Carlson). Monotone so a curve
// through monotone points never overshoots -- a tone curve that dips between
// two rising points produces solarised bands, which is never what a user
// dragging a point intends. Flat extension outside the first/last point.
double ColorGrade::evalCurve(const QVector<QPointF> &pts, double x)
{
    const int n = pts.size();
    if (n == 0) return x;
    if (n == 1) return pts[0].y();
    if (x <= pts.first().x()) return pts.first().y();
    if (x >= pts.last().x()) return pts.last().y();

    QVector<double> h(n - 1), delta(n - 1), m(n);
    for (int i = 0; i < n - 1; ++i) {
        h[i] = std::max(pts[i + 1].x() - pts[i].x(), 1e-6);
        delta[i] = (pts[i + 1].y() - pts[i].y()) / h[i];
    }
    m[0] = delta[0]; m[n - 1] = delta[n - 2];
    for (int i = 1; i < n - 1; ++i) {
        if (delta[i - 1] * delta[i] <= 0.0) m[i] = 0.0;
        else {
            const double w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1];
            m[i] = (w1 + w2) / (w1 / delta[i - 1] + w2 / delta[i]);
        }
    }
    int i = 0;
    while (i < n - 2 && x > pts[i + 1].x()) ++i;
    const double t = (x - pts[i].x()) / h[i];
    const double t2 = t * t, t3 = t2 * t;
    const double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
    const double h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
    return h00 * pts[i].y() + h10 * h[i] * m[i] + h01 * pts[i + 1].y() + h11 * h[i] * m[i + 1];
}

static bool isIdentityCurve(const QVector<QPointF> &pts)
{
    if (pts.size() != 2) return false;
    return qFuzzyIsNull(pts[0].x()) && qFuzzyIsNull(pts[0].y())
        && qFuzzyCompare(pts[1].x(), 1.0) && qFuzzyCompare(pts[1].y(), 1.0);
}

void ColorGrade::rebuildLut()
{
    m_curvesActive = !(isIdentityCurve(m_curves[0]) && isIdentityCurve(m_curves[1])
                       && isIdentityCurve(m_curves[2]) && isIdentityCurve(m_curves[3]));
    QImage lut(256, 1, QImage::Format_RGBA8888);
    uchar *line = lut.scanLine(0);
    for (int i = 0; i < 256; ++i) {
        const double x = i / 255.0;
        const double mx = std::clamp(evalCurve(m_curves[0], x), 0.0, 1.0);
        for (int c = 0; c < 3; ++c) {
            const double y = std::clamp(evalCurve(m_curves[1 + c], mx), 0.0, 1.0);
            line[i * 4 + c] = (uchar)std::lround(y * 255.0);
        }
        line[i * 4 + 3] = 255;
    }
    m_curveLut = lut;
}

QVariantList ColorGrade::curvePoints(int channel) const
{
    QVariantList out;
    if (channel < 0 || channel > 3) return out;
    for (const QPointF &p : m_curves[channel]) out.append(QVariant::fromValue(p));
    return out;
}

void ColorGrade::setCurvePoints(int channel, const QVariantList &points)
{
    if (channel < 0 || channel > 3) return;
    QVector<QPointF> pts;
    for (const QVariant &v : points) {
        const QPointF p = v.toPointF();
        pts.append(QPointF(std::clamp(p.x(), 0.0, 1.0), std::clamp(p.y(), 0.0, 1.0)));
    }
    std::sort(pts.begin(), pts.end(), [](const QPointF &a, const QPointF &b) { return a.x() < b.x(); });
    if (pts.size() < 2) pts = { QPointF(0, 0), QPointF(1, 1) };
    if (pts == m_curves[channel]) return;
    m_curves[channel] = pts;
    rebuildLut();
    emit curvesChanged();
}

void ColorGrade::resetCurve(int channel)
{
    setCurvePoints(channel, { QVariant::fromValue(QPointF(0, 0)), QVariant::fromValue(QPointF(1, 1)) });
}

QVariantList ColorGrade::curveSamples(int channel, int n) const
{
    QVariantList out;
    if (channel < 0 || channel > 3 || n < 2) return out;
    for (int i = 0; i < n; ++i)
        out.append(std::clamp(evalCurve(m_curves[channel], i / double(n - 1)), 0.0, 1.0));
    return out;
}

QString ColorGrade::curvesToJson() const
{
    QJsonObject o;
    const char *keys[4] = { "m", "r", "g", "b" };
    for (int c = 0; c < 4; ++c) {
        QJsonArray arr;
        for (const QPointF &p : m_curves[c]) arr.append(QJsonArray{ p.x(), p.y() });
        o[QLatin1String(keys[c])] = arr;
    }
    return QString::fromUtf8(QJsonDocument(o).toJson(QJsonDocument::Compact));
}

void ColorGrade::curvesFromJson(const QString &json)
{
    const QJsonObject o = QJsonDocument::fromJson(json.toUtf8()).object();
    if (o.isEmpty()) return;
    const char *keys[4] = { "m", "r", "g", "b" };
    bool changed = false;
    for (int c = 0; c < 4; ++c) {
        QVector<QPointF> pts;
        for (const QJsonValue &v : o.value(QLatin1String(keys[c])).toArray()) {
            const QJsonArray xy = v.toArray();
            if (xy.size() == 2) pts.append(QPointF(xy[0].toDouble(), xy[1].toDouble()));
        }
        if (pts.size() >= 2 && pts != m_curves[c]) { m_curves[c] = pts; changed = true; }
    }
    if (changed) { rebuildLut(); emit curvesChanged(); }
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
    for (int c = 0; c < 4; ++c) resetCurve(c);
}
