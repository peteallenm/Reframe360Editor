#ifndef FLOWRENDERER_H
#define FLOWRENDERER_H

#include <QObject>
#include <QImage>
#include <QVector>
#include <QString>
#include <QByteArray>
#include <QtGlobal>

#include "videodecoder.h"

class QOffscreenSurface;
class QOpenGLContext;
class QOpenGLExtraFunctions;
class QOpenGLShaderProgram;

// ---------------------------------------------------------------------------
// Shared optical-flow constants (see OpticalFlow.md)
// ---------------------------------------------------------------------------

// Band texture resolution (phi x theta over [bandTheta0, bandTheta1]).
// 512x128 rather than 1024x256: each texel averages a 2x2 source region, which
// inherently smooths the gradient noise that otherwise shows up as high-
// frequency jitter in the seam, reduces aliasing from the band extraction, and
// is ~4x faster. The encode/decode and project.frag sampling are resolution-
// independent (normalized band coordinates), so no other code changes.
inline constexpr int kFlowBandWidth = 512;
inline constexpr int kFlowBandHeight = 128;

// Default camera-native seam band: the equator straddling 90 deg.
inline constexpr float kDefaultBandTheta0 = 75.0f * 3.14159265358979323846f / 180.0f;
inline constexpr float kDefaultBandTheta1 = 105.0f * 3.14159265358979323846f / 180.0f;

// Parallax matcher defaults. The stage is a 1-D disparity search along theta
// (see disp_match.frag), so the two tunables are:
//   flowIterations -> maximum parallax searched, in band texels (4.3/deg at
//                     512x128 over a 30 deg band; 30 texels = 7 deg, enough for
//                     objects ~0.3 m from the camera)
//   flowAlpha      -> smoothing radius in band texels (Gaussian, confidence-
//                     weighted; see disp_smooth.frag)
// The old Horn-Schunck meanings (iteration count / smoothness weight) are gone;
// the property names are kept so existing settings files still load.
inline constexpr float kDefaultFlowAlpha = 6.0f;       // smoothing radius, texels
inline constexpr int kDefaultFlowIterations = 30;      // max parallax, texels
inline constexpr int kDispSmoothPasses = 3;
inline constexpr int kMatchStep = 2;                 // match at band/2 resolution
inline constexpr float kDispBilateralSigma = 3.0f;   // texels; depth edges sharper than this survive
inline constexpr float kDispTemporalAlpha = 0.5f;      // weight of the NEW frame

// The flow field is packed into an RG/RGBA texture as (ndu*k + 0.5, ndv*k + 0.5)
// where ndu = du/bandWidth (normalized band units, so project.frag needs no
// per-texture size uniforms). k is chosen per frame so the field fits [0,1]
// with headroom, and the consumer decodes with the same k via the u_flowEncode
// uniform.
inline float flowEncodeScaleFor(const QVector<float> &flow, int w, int h)
{
    float maxAbs = 0.0f;
    const int n = flow.size() / 2;
    if (w > 0 && h > 0 && n > 0) {
        const float invW = 1.0f / w;
        const float invH = 1.0f / h;
        for (int i = 0; i < n; ++i) {
            maxAbs = qMax(maxAbs, qAbs(flow[i * 2 + 0] * invW));
            maxAbs = qMax(maxAbs, qAbs(flow[i * 2 + 1] * invH));
        }
    }
    if (maxAbs <= 0.0f)
        return 16.0f;
    return qBound(4.0f, 0.5f / maxAbs, 64.0f);
}

// Everything the band extraction needs from the calibration. Flow depends only
// on (frame, calibration) — never on yaw/pitch/roll/fov/projection/IMU.
struct FlowCalibration {
    float frontCenterX = 0.5f, frontCenterY = 0.5f;
    float frontRadius = 0.5f, frontK1 = 0.0f, frontK2 = 0.0f;
    float frontRotation = 0.0f;
    bool frontHFlip = false;
    float rearCenterX = 0.5f, rearCenterY = 0.5f;
    float rearRadius = 0.5f, rearK1 = 0.0f, rearK2 = 0.0f;
    float rearRotation = 180.0f;
    bool rearHFlip = false;
};

struct FlowSettings {
    float alpha = kDefaultFlowAlpha;
    int iterations = kDefaultFlowIterations;
    float bandTheta0 = kDefaultBandTheta0;
    float bandTheta1 = kDefaultBandTheta1;
    bool seamStitch = false;
    float seamStrength = 1.0f;
};

// ---------------------------------------------------------------------------
// FlowRenderer — offscreen GL engine for the Horn-Schunck displacement field.
//
//   pass 1 (band_extract.frag, MRT): band_front / band_rear luma
//   pass 2 (disp_match.frag):         1-D ZNCC disparity along theta + confidence
//   pass 3 (disp_smooth.frag, x2N):   confidence-weighted separable smoothing
//
// The result is returned in the flow format the rest of the pipeline already
// speaks: a QVector<float> of (du, dv) pairs in texel units, with du always 0
// (parallax between back-to-back lenses is purely along theta). QOpenGLContext is thread-affine: the context +
// surface must be created on the thread that calls compute(). The preview
// path uses FlowWorker; the export path instantiates FlowRenderer directly on
// the export worker thread.
// ---------------------------------------------------------------------------
class FlowRenderer
{
public:
    FlowRenderer() = default;
    ~FlowRenderer();

    FlowRenderer(const FlowRenderer &) = delete;
    FlowRenderer &operator=(const FlowRenderer &) = delete;

    bool isReady() const { return m_ready; }
    int bandWidth() const { return kFlowBandWidth; }
    int bandHeight() const { return kFlowBandHeight; }

    // Create the core-4.4 context + surface, compile the three programs and
    // build the textures/FBOs. Must be called on the thread that will call
    // compute().
    bool initialize(QString *error);

    // Estimate the flow field for one decoded frame. On success *outFlow is
    // filled with kFlowBandWidth*kFlowBandHeight (du, dv) pairs in texels.
    bool compute(const DecodedFrame &frame, const FlowCalibration &cal,
                 const FlowSettings &settings, QVector<float> *outFlow,
                 QVector<float> *outSeam = nullptr,
                 QString *error = nullptr);

    // Pack the raw flow into an RGBA8 image for the RHI preview path
    // (QQuickWindow::createTextureFromImage). *encodeOut receives the k used
    // so the consumer can set the u_flowEncode uniform.
    static QImage packFlowToImage(const QVector<float> &flow, int w, int h,
                                   float *encodeOut);
    static QImage packSeamToImage(const QVector<float> &seam, int w);

private:
    static QByteArray loadResource(const char *path);
    // Pull-push hole filling of the gated matches, then bilateral smoothing.
    static void fillAndSmooth(const QVector<float> &dIn, const QVector<float> &cIn,
                              int W, int H, int radius,
                              QVector<float> *dOut, QVector<float> *cOut);
    bool createContext(QString *error);
    bool compilePrograms(QString *error);
    bool createBandTargets(QString *error);
    void drawQuad();
    void destroy();

    QOffscreenSurface *m_surface = nullptr;
    QOpenGLContext *m_context = nullptr;
    QOpenGLExtraFunctions *m_functions = nullptr;
    // True when the context is OpenGL ES (Android): selects the shader dialect.
    bool m_gles = false;
    QOpenGLShaderProgram *m_bandProgram = nullptr;
    QOpenGLShaderProgram *m_matchProgram = nullptr;
    QOpenGLShaderProgram *m_seamCostProgram = nullptr;

    quint32 m_vao = 0;
    quint32 m_vbo = 0;
    quint32 m_ebo = 0;
    quint32 m_bandFbo = 0;
    quint32 m_gradFbo = 0;
    quint32 m_flowFbo = 0;
    quint32 m_bandFront = 0;
    quint32 m_bandRear = 0;
    quint32 m_gradients = 0;
    quint32 m_flowA = 0;
    quint32 m_flowB = 0;
    quint32 m_costTex = 0;
    quint32 m_seamFbo = 0;
    quint32 m_yTex = 0;
    int m_yW = 0;
    int m_yH = 0;
    QVector<float> m_prevSeam;
    QVector<float> m_prevFlow;     // temporal smoothing of the disparity field
    int m_overlapV0 = 0, m_overlapV1 = 0;   // last measured overlap rows (gl bottom-up)
    bool m_ready = false;
};

// ---------------------------------------------------------------------------
// FlowWorker — thread wrapper for the preview path. Owns the FlowRenderer,
// which is created lazily on the worker thread that this object is moved to.
// ---------------------------------------------------------------------------
class FlowWorker : public QObject
{
    Q_OBJECT
public:
    explicit FlowWorker(QObject *parent = nullptr);

public slots:
    void computeFrame(const DecodedFrame &frame, const FlowCalibration &cal,
                      const FlowSettings &settings);

signals:
    void flowReady(const QImage &image, float encodeScale, const QImage &seamImage);
    void flowFailed(const QString &message);

private:
    FlowRenderer m_renderer;
    bool m_failed = false;
};

Q_DECLARE_METATYPE(FlowCalibration)
Q_DECLARE_METATYPE(FlowSettings)

#endif // FLOWRENDERER_H
