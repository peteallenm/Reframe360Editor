#ifndef GPURENDERER_H
#define GPURENDERER_H

#include <QByteArray>
#include <QImage>
#include <QString>
#include <QMatrix4x4>
#include <QVector>

class QOffscreenSurface;
class QOpenGLContext;
class QOpenGLFunctions_3_3_Core;
class QOpenGLShaderProgram;

struct DecodedFrame;
struct ExportFrameState;

// Standalone OpenGL renderer used by the video exporter. It uploads decoded
// YUV planes as textures, draws a fullscreen quad with the very same
// project.frag that the live viewer uses (compiled directly from the embedded
// GLSL source), and reads the result back into a QImage. Because the export
// shader and the viewer shader are one and the same file, preview/export
// parity is guaranteed by construction instead of by maintaining a C++ mirror.
//
// The class owns its own offscreen GL context and is designed to be created
// and used entirely on the exporter worker thread, so it never touches the
// GUI's scene graph. If no suitable GL (>= 3.3) is available, initialize()
// fails and the caller falls back to the CPU rasterizer.
class GpuRenderer
{
public:
    GpuRenderer() = default;
    ~GpuRenderer();

    GpuRenderer(const GpuRenderer &) = delete;
    GpuRenderer &operator=(const GpuRenderer &) = delete;

    // Create the context, compile the shaders and build the GL objects.
    // Returns false (with *error set) if no usable GL is available.
    bool initialize(QString *error);

    bool isReady() const { return m_ready; }

    // Optional: use these GLSL sources instead of the embedded resources
    // (used by tests that don't have the app's resource bundle).
    void setShaderSourcesForTest(const QByteArray &vertSrc, const QByteArray &fragSrc)
    {
        m_testVertSrc = vertSrc;
        m_testFragSrc = fragSrc;
    }

    // Render one decoded frame at the given state. On success fills *out
    // with a W x H RGB888 image whose top row corresponds to the top of the
    // source video — the same orientation the live viewer shows.
    bool render(const DecodedFrame &frame, const ExportFrameState &state,
                int width, int height, QImage *out, QString *error);

    // Install the optical-flow field produced by FlowRenderer for the frame
    // about to be rendered. The flow is (du, dv) in band-texel units; it is
    // packed as RG16F (same encoding as the preview path: (ndu*k + 0.5, ...))
    // and sampled at sampler unit 4 by project.frag when state.flowStitch is
    // set. The matching decode scale is stored and uploaded as u_flowEncode.
    void setFlowData(const QVector<float> &flow, int w, int h);
    void setSeamData(const QVector<float> &seam, int w);

private:
    static QByteArray loadResource(const char *path);
    bool createContext(QString *error);
    bool compileProgram(const QByteArray &vertSrc, const QByteArray &fragSrc,
                        QOpenGLShaderProgram *&program, QString *error);
    bool ensureFramebuffer(int width, int height, QString *error);
    void uploadPlane(quint32 tex, int w, int h, int stride, const void *data);
    void destroy();

    QOffscreenSurface *m_surface = nullptr;
    QOpenGLContext *m_context = nullptr;
    QOpenGLFunctions_3_3_Core *m_functions = nullptr;
    QOpenGLShaderProgram *m_program = nullptr;
    QByteArray m_testVertSrc;
    QByteArray m_testFragSrc;

    quint32 m_vao = 0;
    quint32 m_vbo = 0;
    quint32 m_ebo = 0;
    quint32 m_fbo = 0;
    quint32 m_colorTex = 0;
    quint32 m_yTex = 0;
    quint32 m_uTex = 0;
    quint32 m_vTex = 0;
    quint32 m_flowTex = 0;
    quint32 m_seamTex = 0;
    quint32 m_curveTex = 0;
    qint64 m_curveKey = -1;
    bool m_seamValid = false;
    int m_fbW = 0;
    int m_fbH = 0;
    bool m_flowValid = false;
    float m_flowEncode = 16.0f;
    bool m_ready = false;
};

#endif // GPURENDERER_H
