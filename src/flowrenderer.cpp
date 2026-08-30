#include "flowrenderer.h"
#include "glsladapt.h"

#include <QFile>
#include <QElapsedTimer>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include "glesext.h"   // GLES 3.0 enums on Android
#include <QPoint>
#include <cmath>
#include <cstring>
#include <QSurfaceFormat>
#include <QVector2D>
#include <QtCore/qfloat16.h>
#include <algorithm>

// Fullscreen-quad vertex source shared by the three flow passes. The quad
// data (x, y, u, v) matches GpuRenderer: v = 0 at the top, so band texture
// row 0 = theta0, consistent with project.frag's v_band mapping.
static const char *kFlowVertSrc = R"(
#version 440
layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_texCoord;
out vec2 v_texCoord;
void main() {
    v_texCoord = a_texCoord;
    gl_Position = vec4(a_position, 0.0, 1.0);
}
)";

FlowRenderer::~FlowRenderer()
{
    destroy();
}

void FlowRenderer::destroy()
{
    if (!m_context)
        return;

    m_context->makeCurrent(m_surface);
    QOpenGLExtraFunctions *f = m_functions;

    delete m_bandProgram; m_bandProgram = nullptr;
    delete m_matchProgram; m_matchProgram = nullptr;
    delete m_seamCostProgram; m_seamCostProgram = nullptr;

    if (m_yTex) { f->glDeleteTextures(1, &m_yTex); m_yTex = 0; }
    if (m_bandFront) { f->glDeleteTextures(1, &m_bandFront); m_bandFront = 0; }
    if (m_bandRear) { f->glDeleteTextures(1, &m_bandRear); m_bandRear = 0; }
    if (m_gradients) { f->glDeleteTextures(1, &m_gradients); m_gradients = 0; }
    if (m_flowA) { f->glDeleteTextures(1, &m_flowA); m_flowA = 0; }
    if (m_flowB) { f->glDeleteTextures(1, &m_flowB); m_flowB = 0; }
    if (m_costTex) { f->glDeleteTextures(1, &m_costTex); m_costTex = 0; }
    if (m_seamFbo) { f->glDeleteFramebuffers(1, &m_seamFbo); m_seamFbo = 0; }
    if (m_vbo) { f->glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_ebo) { f->glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
    if (m_vao) { f->glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    if (m_bandFbo) { f->glDeleteFramebuffers(1, &m_bandFbo); m_bandFbo = 0; }
    if (m_gradFbo) { f->glDeleteFramebuffers(1, &m_gradFbo); m_gradFbo = 0; }
    if (m_flowFbo) { f->glDeleteFramebuffers(1, &m_flowFbo); m_flowFbo = 0; }

    m_context->doneCurrent();
    delete m_context;
    m_context = nullptr;
    delete m_surface;
    m_surface = nullptr;
    m_ready = false;
}

QByteArray FlowRenderer::loadResource(const char *path)
{
    QFile f(QString::fromLatin1(path));
    if (!f.open(QIODevice::ReadOnly))
        return QByteArray();
    return f.readAll();
}

bool FlowRenderer::createContext(QString *error)
{
    // Same story as GpuRenderer: a GLES build of Qt (Android) cannot produce a
    // desktop context, so ask for whatever this Qt has. The stitching shaders
    // used to be compiled verbatim at #version 440, which forced a GL 4.4
    // context; they now go through adaptGlsl() like the export shader, so the
    // real requirement is only GL 3.3 / GLES 3.0 -- multiple render targets,
    // texelFetchOffset and half-float colour buffers are all available there.
    const bool wantGles = (QOpenGLContext::openGLModuleType() == QOpenGLContext::LibGLES);

    QSurfaceFormat fmt;
    fmt.setDepthBufferSize(0);
    fmt.setStencilBufferSize(0);
    if (wantGles) {
        fmt.setRenderableType(QSurfaceFormat::OpenGLES);
        fmt.setVersion(3, 0);
    } else {
        fmt.setRenderableType(QSurfaceFormat::OpenGL);
        fmt.setVersion(3, 3);
        fmt.setProfile(QSurfaceFormat::CoreProfile);
    }

    m_surface = new QOffscreenSurface;
    m_surface->setFormat(fmt);
    m_surface->create();
    if (!m_surface->isValid()) {
        if (error) *error = QStringLiteral("Flow: could not create an offscreen GL surface");
        return false;
    }

    m_context = new QOpenGLContext;
    m_context->setFormat(fmt);
    if (!m_context->create()) {
        if (error) *error = QStringLiteral("Flow: could not create an OpenGL context");
        return false;
    }
    if (!m_context->makeCurrent(m_surface)) {
        if (error) *error = QStringLiteral("Flow: could not activate the offscreen GL context");
        return false;
    }

    m_gles = m_context->isOpenGLES();
    m_functions = m_context->extraFunctions();
    if (!m_functions) {
        if (error) *error = QStringLiteral("Flow: OpenGL function set unavailable");
        return false;
    }
    m_functions->initializeOpenGLFunctions();

    const int maj = m_context->format().majorVersion();
    const int min = m_context->format().minorVersion();
    if (m_gles ? (maj < 3) : (maj < 3 || (maj == 3 && min < 3))) {
        if (error) *error = m_gles
                ? QStringLiteral("Flow stitching requires OpenGL ES 3.0+ (have %1.%2)").arg(maj).arg(min)
                : QStringLiteral("Flow stitching requires OpenGL 3.3+ (have %1.%2)").arg(maj).arg(min);
        return false;
    }

    // The disparity field lives in a half-float colour buffer. On desktop GL
    // that is core; on GLES 3.0 rendering to RGBA16F needs an extension. Mali
    // (the Motorola Edge 40's G77) and every other Android GPU we target ship
    // it, but check rather than silently read back a field of zeros -- a
    // failure mode this renderer has hit before on desktop with RG32F.
    if (m_gles
        && !m_context->hasExtension(QByteArrayLiteral("GL_EXT_color_buffer_half_float"))
        && !m_context->hasExtension(QByteArrayLiteral("GL_EXT_color_buffer_float"))) {
        if (error) *error = QStringLiteral("Flow stitching needs GL_EXT_color_buffer_half_float, which this GPU does not report");
        return false;
    }
    return true;
}

bool FlowRenderer::compilePrograms(QString *error)
{
    // Lower every stage to this context's dialect. band_extract.frag draws to
    // two attachments, so its fragment-output layout(location = N) qualifiers
    // must survive -- adaptGlsl keeps those and strips the rest.
    const GlslTarget target = m_gles ? GlslTarget::EmbeddedEs300 : GlslTarget::DesktopCore330;
    const QByteArray vertSrc = adaptGlsl(QByteArray(kFlowVertSrc), target, GlslStage::Vertex);

    auto compile = [this, &vertSrc, target](const QByteArray &rawFragSrc, QOpenGLShaderProgram *&program, const char *what, QString *err) {
        const QByteArray fragSrc = adaptGlsl(rawFragSrc, target, GlslStage::Fragment);
        program = new QOpenGLShaderProgram;
        program->bindAttributeLocation("a_position", 0);
        program->bindAttributeLocation("a_texCoord", 1);
        if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertSrc)
            || !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragSrc)
            || !program->link()) {
            if (err) *err = QStringLiteral("Flow: %1 shader compile failed: %2")
                                .arg(QLatin1String(what), program->log().trimmed());
            delete program;
            program = nullptr;
            return false;
        }
        return true;
    };

    if (!compile(loadResource(":/shaders_raw/shaders/band_extract.frag"), m_bandProgram, "band_extract", error))
        return false;
    if (!compile(loadResource(":/shaders_raw/shaders/disp_match.frag"), m_matchProgram, "disp_match", error))
        return false;
    if (!compile(loadResource(":/shaders_raw/shaders/seam_cost.frag"), m_seamCostProgram, "seam_cost", error))
        return false;
    return true;
}

bool FlowRenderer::createBandTargets(QString *error)
{
    QOpenGLExtraFunctions *f = m_functions;
    const int W = kFlowBandWidth;
    const int H = kFlowBandHeight;

    if (!m_bandFront) {
        f->glGenTextures(1, &m_bandFront);
        f->glGenTextures(1, &m_bandRear);
        f->glGenTextures(1, &m_gradients);
        f->glGenTextures(1, &m_flowA);
        f->glGenTextures(1, &m_flowB);

        auto tex2d = [f](quint32 tex, GLenum internal, GLenum format, GLenum type) {
            f->glBindTexture(GL_TEXTURE_2D, tex);
            f->glTexImage2D(GL_TEXTURE_2D, 0, internal, W, H, 0, format, type, nullptr);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            // u is periodic (phi wraps), v is not (theta clamps).
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        };
        tex2d(m_bandFront, GL_R8, GL_RED, GL_UNSIGNED_BYTE);
        tex2d(m_bandRear,  GL_R8, GL_RED, GL_UNSIGNED_BYTE);
        tex2d(m_gradients, GL_RGBA16F, GL_RGBA, GL_FLOAT);
        // GL_RG32F is NOT universally color-renderable (some drivers reject it
        // as a render target), which would silently fail every iteration and
        // the readback, leaving the flow field all zeros. GL_RGBA16F is
        // color-renderable since GL 3.0 and has ample precision for the flow
        // (texel units). The shader's out vec2 writes R,G; B,A are don't-care.
        tex2d(m_flowA,     GL_RGBA16F, GL_RGBA, GL_FLOAT);
        tex2d(m_flowB,     GL_RGBA16F, GL_RGBA, GL_FLOAT);

        f->glGenTextures(1, &m_costTex);
        f->glBindTexture(GL_TEXTURE_2D, m_costTex);
        f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, W, H, 0, GL_RGBA, GL_FLOAT, nullptr);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    if (!m_vao) {
        static const float kQuad[] = {
            // x      y      u      v
            -1.f,  -1.f,  0.f,   1.f,
             1.f,  -1.f,  1.f,   1.f,
             1.f,   1.f,  1.f,   0.f,
            -1.f,   1.f,  0.f,   0.f,
        };
        static const quint16 kIndices[] = { 0, 1, 2, 0, 2, 3 };
        f->glGenVertexArrays(1, &m_vao);
        f->glGenBuffers(1, &m_vbo);
        f->glGenBuffers(1, &m_ebo);
        f->glBindVertexArray(m_vao);
        f->glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        f->glBufferData(GL_ARRAY_BUFFER, sizeof(kQuad), kQuad, GL_STATIC_DRAW);
        f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
        f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kIndices), kIndices, GL_STATIC_DRAW);
        f->glEnableVertexAttribArray(0);
        f->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
        f->glEnableVertexAttribArray(1);
        f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                                 reinterpret_cast<void *>(2 * sizeof(float)));
        f->glBindVertexArray(0);
    }

    if (!m_bandFbo) {
        f->glGenFramebuffers(1, &m_bandFbo);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_bandFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_bandFront, 0);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_bandRear, 0);
        // glDrawBuffers must be set BEFORE the completeness check: the check
        // only validates the attachments named in the draw buffers, so doing
        // it before would ignore COLOR_ATTACHMENT1 and silently allow a broken
        // MRT draw (GL_INVALID_FRAMEBUFFER_OPERATION = no-op) on drivers that
        // reject it.
        const GLenum bufs[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
        f->glDrawBuffers(2, bufs);
        if (f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            if (error) *error = QStringLiteral("Flow: band framebuffer is incomplete");
            return false;
        }
    }
    if (!m_gradFbo) {
        f->glGenFramebuffers(1, &m_gradFbo);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_gradFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_gradients, 0);
        if (f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            if (error) *error = QStringLiteral("Flow: gradient framebuffer is incomplete");
            return false;
        }
        { const GLenum one[1] = { GL_COLOR_ATTACHMENT0 }; f->glDrawBuffers(1, one); }
    }
    if (!m_flowFbo) {
        f->glGenFramebuffers(1, &m_flowFbo);
        // The color attachment is (re)attached to flowA/flowB per iteration.
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
        { const GLenum one[1] = { GL_COLOR_ATTACHMENT0 }; f->glDrawBuffers(1, one); }
    }
    if (!m_seamFbo) {
        f->glGenFramebuffers(1, &m_seamFbo);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
        { const GLenum one[1] = { GL_COLOR_ATTACHMENT0 }; f->glDrawBuffers(1, one); }
    }
    return true;
}

void FlowRenderer::drawQuad()
{
    QOpenGLExtraFunctions *f = m_functions;
    f->glBindVertexArray(m_vao);
    f->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
    f->glBindVertexArray(0);
}

bool FlowRenderer::initialize(QString *error)
{
    if (m_ready)
        return true;
    if (!createContext(error))
        return false;
    if (!compilePrograms(error))
        return false;
    if (!createBandTargets(error))
        return false;
    m_ready = true;
    return true;
}

bool FlowRenderer::compute(const DecodedFrame &frame, const FlowCalibration &cal,
                           const FlowSettings &settings, QVector<float> *outFlow,
                           QVector<float> *outSeam,
                           QString *error)
{
    if (!m_ready) {
        if (error) *error = QStringLiteral("Flow renderer is not initialized");
        return false;
    }
    if (!m_context->makeCurrent(m_surface)) {
        if (error) *error = QStringLiteral("Flow: lost the offscreen GL context");
        return false;
    }
    QOpenGLExtraFunctions *f = m_functions;

    const int W = kFlowBandWidth;
    const int H = kFlowBandHeight;
    QElapsedTimer tAll; tAll.start();
    static const bool kTiming = !qgetenv("RENDER360_FLOW_TIMING").isEmpty();
    double tBand = 0, tSeam = 0, tMatch = 0, tRead = 0, tCpu = 0;

    // ---- Upload the stacked Y plane (front top half, rear bottom half). ----
    if (!m_yTex)
        f->glGenTextures(1, &m_yTex);
    f->glBindTexture(GL_TEXTURE_2D, m_yTex);
    f->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    f->glPixelStorei(GL_UNPACK_ROW_LENGTH, frame.yStride == frame.width ? 0 : frame.yStride);
    if (m_yW == frame.width && m_yH == frame.height) {
        f->glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, frame.width, frame.height,
                           GL_RED, GL_UNSIGNED_BYTE, frame.yData.constData());
    } else {
        f->glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, frame.width, frame.height, 0,
                        GL_RED, GL_UNSIGNED_BYTE, frame.yData.constData());
        m_yW = frame.width;
        m_yH = frame.height;
    }
    f->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (!createBandTargets(error))
        return false;

    // ---- Pass 1: extract the band luma for both lenses (MRT). ----
    f->glViewport(0, 0, W, H);
    f->glDisable(GL_DEPTH_TEST);
    f->glDisable(GL_BLEND);
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_bandFbo);
    m_bandProgram->bind();
    m_bandProgram->setUniformValue("u_frontCenter", QVector2D(cal.frontCenterX, cal.frontCenterY));
    m_bandProgram->setUniformValue("u_frontRadius", cal.frontRadius);
    m_bandProgram->setUniformValue("u_frontK1", cal.frontK1);
    m_bandProgram->setUniformValue("u_frontK2", cal.frontK2);
    m_bandProgram->setUniformValue("u_frontHFlip", cal.frontHFlip ? 1 : 0);
    m_bandProgram->setUniformValue("u_rearCenter", QVector2D(cal.rearCenterX, cal.rearCenterY));
    m_bandProgram->setUniformValue("u_rearRadius", cal.rearRadius);
    m_bandProgram->setUniformValue("u_rearK1", cal.rearK1);
    m_bandProgram->setUniformValue("u_rearK2", cal.rearK2);
    m_bandProgram->setUniformValue("u_rearHFlip", cal.rearHFlip ? 1 : 0);
    m_bandProgram->setUniformValue("u_frontRotation", cal.frontRotation);
    m_bandProgram->setUniformValue("u_rearRotation", cal.rearRotation);
    m_bandProgram->setUniformValue("u_bandTheta0", settings.bandTheta0);
    m_bandProgram->setUniformValue("u_bandTheta1", settings.bandTheta1);
    // Pin the sampler unit explicitly (belt-and-braces on top of
    // layout(binding=...)): some drivers don't honour the qualifier for
    // multi-sampler programs, silently sampling default (zero) textures.
    m_bandProgram->setUniformValue("u_texY", 0);
    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, m_yTex);
    drawQuad();

    if (kTiming) { f->glFinish(); tBand = tAll.nsecsElapsed() / 1e6; }

    // ---- Pass 1b: seam cost volume ----
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_costTex, 0);
    m_seamCostProgram->bind();
    m_seamCostProgram->setUniformValue("u_bandFront", 0);
    m_seamCostProgram->setUniformValue("u_bandRear", 1);
    m_seamCostProgram->setUniformValue("u_harrisK", 0.04f);
    m_seamCostProgram->setUniformValue("u_blockMatchWeight", 1.0f);
    m_seamCostProgram->setUniformValue("u_blockMatchRadius", 16);
    m_seamCostProgram->setUniformValue("u_equatorBiasWeight", 0.1f);
    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, m_bandFront);
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, m_bandRear);
    drawQuad();

    // ---- Pass 1c/1d: DP over the cost volume, on the CPU. ----
    // This used to be W separate scissored GL draws (one per column) plus a
    // readback; at 512 columns the draw overhead alone was tens of ms. The cost
    // volume is 512x128 floats -- the DP is trivial on the CPU.
    if (outSeam) {
        outSeam->resize(W);
        QVector<qfloat16> costPacked(W * H * 4);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_costTex, 0);
        f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
        f->glReadPixels(0, 0, W, H, GL_RGBA, GL_HALF_FLOAT, costPacked.data());
        QVector<float> cum(W * H);
        for (int v = 0; v < H; ++v) cum[v * W + 0] = float(costPacked[(v * W) * 4]);
        for (int u = 1; u < W; ++u)
            for (int v = 0; v < H; ++v) {
                const float c = float(costPacked[(v * W + u) * 4]);
                const float pL = cum[qBound(0, v - 1, H - 1) * W + u - 1];
                const float pC = cum[v * W + u - 1];
                const float pR = cum[qBound(0, v + 1, H - 1) * W + u - 1];
                cum[v * W + u] = c + qMin(pL + 0.1f, qMin(pC, pR + 0.1f));
            }
        auto dpAt = [&](int u, int v) { return cum[v * W + u]; };
        // Clamp traceback to the central 60% of the band so the seam stays in
        // the region where both lenses have valid fisheye coverage.
        int vMinS = (int)(0.2f * H);
        int vMaxS = (int)(0.8f * H);
        int bestV = vMinS;
        float minVal = 1e30f;
        for (int v = vMinS; v <= vMaxS; ++v) {
            float val = dpAt(W - 1, v);
            if (val < minVal) { minVal = val; bestV = v; }
        }
        int curV = bestV;
        for (int u = W - 1; u >= 0; --u) {
            (*outSeam)[u] = (float)curV / (float)(H - 1);
            if (u > 0) {
                int vL = qBound(vMinS, curV - 1, vMaxS);
                int vR = qBound(vMinS, curV + 1, vMaxS);
                float cL = dpAt(u - 1, vL);
                float cC = dpAt(u - 1, curV);
                float cR = dpAt(u - 1, vR);
                if (cL <= cC && cL <= cR) curV = vL;
                else if (cR <= cC && cR <= cL) curV = vR;
            }
        }

        // Temporal smoothing: blend with previous seam to reduce flicker
        if (!m_prevSeam.isEmpty() && m_prevSeam.size() == outSeam->size()) {
            const float alpha = 0.7f;  // smoothing factor (0 = no smoothing, 1 = no change)
            for (int i = 0; i < outSeam->size(); ++i)
                (*outSeam)[i] = alpha * (*outSeam)[i] + (1.0f - alpha) * m_prevSeam[i];
        }
        m_prevSeam = *outSeam;
    }

    if (kTiming) tSeam = tAll.nsecsElapsed() / 1e6;

    // Restore viewport for pass 2
    f->glViewport(0, 0, W, H);

    // ---- Overlap rows: where BOTH lenses carry real image. ----
    // Read the two band lumas back (2 x 64 KB) and find, per row, whether it
    // is textured image or the dark rim. Front is valid from theta0 down to
    // some row, rear from some row up to theta1; the matcher and the morph
    // are confined to the intersection.
    int vMin = 0, vMax = H - 1;
    QVector<uchar> bandF(W * H), bandR(W * H);
    {
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_bandFbo);
        f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
        f->glReadBuffer(GL_COLOR_ATTACHMENT0);
        f->glReadPixels(0, 0, W, H, GL_RED, GL_UNSIGNED_BYTE, bandF.data());
        f->glReadBuffer(GL_COLOR_ATTACHMENT1);
        f->glReadPixels(0, 0, W, H, GL_RED, GL_UNSIGNED_BYTE, bandR.data());
        auto rowOk = [&](const QVector<uchar> &b, int row) {
            double sum = 0.0, sum2 = 0.0;
            const uchar *p = b.constData() + row * W;
            for (int x = 0; x < W; ++x) { sum += p[x]; sum2 += double(p[x]) * p[x]; }
            const double mean = sum / W, var = sum2 / W - mean * mean;
            return mean > 40.0 && std::sqrt(qMax(0.0, var)) > 12.0;
        };
        // glReadPixels rows are bottom-up: row index 0 here is theta1.
        // Front: valid at small theta (high row index here), invalid at large.
        int frontValidLow = H;           // lowest row index (largest theta) still valid
        for (int r = H - 1; r >= 0; --r) { if (rowOk(bandF, r)) frontValidLow = r; else break; }
        int rearValidHigh = -1;          // highest row index (smallest theta) still valid
        for (int r = 0; r < H; ++r) { if (rowOk(bandR, r)) rearValidHigh = r; else break; }
        // Convert to the shader's row convention (row 0 = theta0 at the top of
        // the texture as drawn; gl_FragCoord.y = 0 is the bottom = theta1).
        // In gl_FragCoord terms (bottom-up), valid = [frontValidLow, rearValidHigh].
        const int margin = 2;
        vMin = qBound(0, frontValidLow + margin, H - 1);
        vMax = qBound(0, rearValidHigh - margin, H - 1);
        if (vMax - vMin < 8) {          // degenerate: fall back to the middle half
            vMin = H / 4; vMax = 3 * H / 4;
        }
        m_overlapV0 = vMin; m_overlapV1 = vMax;
    }

    // ---- Pass 2: 1-D disparity along theta (ZNCC), with confidence. ----
    // Slider mapping (see flowrenderer.h): iterations -> search range in
    // texels, alpha -> smoothing radius in texels.
    const int searchTexels = qBound(4, settings.iterations, 60);
    const int smoothRadius = qBound(1, (int)std::lround(settings.alpha), 48);

    f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_flowA, 0);
    if (f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        if (error) *error = QStringLiteral("Flow: disparity framebuffer is incomplete "
                                           "(is RGBA16F renderable on this driver?)");
        return false;
    }
    const int MS = kMatchStep;                 // match every MS-th band texel
    const int MW = W / MS, MH = H / MS;
    f->glViewport(0, 0, MW, MH);
    m_matchProgram->bind();
    m_matchProgram->setUniformValue("u_bandFront", 0);
    m_matchProgram->setUniformValue("u_bandRear", 1);
    m_matchProgram->setUniformValue("u_searchTexels", searchTexels);
    m_matchProgram->setUniformValue("u_vMin", vMin);
    m_matchProgram->setUniformValue("u_vMax", vMax);
    m_matchProgram->setUniformValue("u_step", MS);
    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, m_bandFront);
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, m_bandRear);
    drawQuad();
    f->glViewport(0, 0, W, H);
    if (kTiming) { f->glFinish(); tMatch = tAll.nsecsElapsed() / 1e6; }

    const quint32 finalTex = m_flowA;

    // ---- Read back the small flow field (1024x256 x 2 floats = 2 MB). ----
    if (!outFlow)
        return true;
    // GL_FLOAT readbacks from RGBA16F silently return zeros on some drivers
    // (that produced the all-zero flow_debug.png). Read the native half-float
    // format instead and convert here.
    QVector<qfloat16> packedM(MW * MH * 4);
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, finalTex, 0);
    f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
    f->glReadPixels(0, 0, MW, MH, GL_RGBA, GL_HALF_FLOAT, packedM.data());
    if (kTiming) tRead = tAll.nsecsElapsed() / 1e6;
    // Upsample the match grid to band resolution (nearest) into `packed`, the
    // layout the rest of this function and the debug dump expect.
    QVector<qfloat16> packed(W * H * 4);
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        const int mx = qMin(x / MS, MW - 1), my = qMin(y / MS, MH - 1);
        for (int ch = 0; ch < 4; ++ch) packed[(y * W + x) * 4 + ch] = packedM[(my * MW + mx) * 4 + ch];
    }
    // ---- CPU: hole filling + edge-aware smoothing of the gated matches. ----
    // The matcher leaves most texels with zero confidence (untextured sky and
    // grass, skin, anything ambiguous); typically 10-30 % survive. Those sparse
    // values have to be spread across the whole band without smearing real
    // depth edges. Pull-push does the spreading: average down a pyramid
    // (confidence-weighted), then push back up filling only where the finer
    // level had no support. A bilateral pass then smooths within depth regions.
    QVector<float> dRaw(MW * MH), cRaw(MW * MH);
    for (int i = 0; i < MW * MH; ++i) {
        dRaw[i] = float(packedM[i * 4 + 0]);
        cRaw[i] = float(packedM[i * 4 + 1]);
    }
    QVector<float> dFilled, cFilled;
    fillAndSmooth(dRaw, cRaw, MW, MH, qMax(1, smoothRadius / MS), &dFilled, &cFilled);
    // Bilinear upsample of the smoothed field back to band resolution.
    for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
        const float fx = (x + 0.5f) / MS - 0.5f, fy = (y + 0.5f) / MS - 0.5f;
        const int x0 = qBound(0, (int)std::floor(fx), MW - 1), y0 = qBound(0, (int)std::floor(fy), MH - 1);
        const int x1 = qMin(x0 + 1, MW - 1), y1 = qMin(y0 + 1, MH - 1);
        const float tx = qBound(0.0f, fx - x0, 1.0f), ty = qBound(0.0f, fy - y0, 1.0f);
        auto lerp2 = [&](const QVector<float> &v) {
            const float a = v[y0 * MW + x0] * (1 - tx) + v[y0 * MW + x1] * tx;
            const float b = v[y1 * MW + x0] * (1 - tx) + v[y1 * MW + x1] * tx;
            return a * (1 - ty) + b * ty;
        };
        packed[(y * W + x) * 4 + 0] = qfloat16(lerp2(dFilled));
        packed[(y * W + x) * 4 + 1] = qfloat16(lerp2(cFilled));
    }
    if (kTiming) tCpu = tAll.nsecsElapsed() / 1e6;

    outFlow->resize(W * H * 2);
    for (int i = 0; i < W * H; ++i) {
        (*outFlow)[i * 2 + 0] = 0.0f;                        // du: none (epipolar = theta)
        // The band FBO is drawn with v = 1 at the BOTTOM (quad texcoords), so a
        // step of +1 in gl_FragCoord.y is a step of -1 in v, i.e. towards
        // theta0. The matcher measures its disparity in FragCoord rows; the
        // shader consumes dv in +v (= +theta) units. Negate. (Getting this
        // wrong pushed near objects the wrong way: verified against a forced
        // constant field, where -20 registered the subject and +20 doubled it.)
        (*outFlow)[i * 2 + 1] = -float(packed[i * 4 + 0]);   // dv: disparity, texels, +theta
    }

    // Debug dump (RENDER360_FLOW_DEBUG=<dir>): band luma, disparity, confidence.
    {
        const QByteArray dbg = qgetenv("RENDER360_FLOW_DEBUG");
        if (!dbg.isEmpty()) {
            auto dumpR8 = [&](quint32 tex, const char *name) {
                QVector<uchar> buf(W * H);
                f->glBindFramebuffer(GL_FRAMEBUFFER, m_bandFbo);
                f->glReadBuffer(tex == m_bandFront ? GL_COLOR_ATTACHMENT0 : GL_COLOR_ATTACHMENT1);
                f->glReadPixels(0, 0, W, H, GL_RED, GL_UNSIGNED_BYTE, buf.data());
                QImage img(W, H, QImage::Format_Grayscale8);
                for (int y = 0; y < H; ++y)
                    memcpy(img.scanLine(y), buf.constData() + (H - 1 - y) * W, W);
                img.save(QString::fromUtf8(dbg) + "/" + name);
            };
            dumpR8(m_bandFront, "band_front.png");
            dumpR8(m_bandRear, "band_rear.png");
            // disparity: blue = negative, red = positive, scaled to +-searchTexels;
            // confidence: grey.
            QImage disp(W, H, QImage::Format_RGB888), conf(W, H, QImage::Format_Grayscale8);
            float dmin = 1e9f, dmax = -1e9f, cmean = 0.0f;
            for (int y = 0; y < H; ++y) {
                uchar *dl = disp.scanLine(y), *cl = conf.scanLine(y);
                const int sy = H - 1 - y;
                for (int x = 0; x < W; ++x) {
                    const float d = float(packed[(sy * W + x) * 4 + 0]);
                    const float c = float(packed[(sy * W + x) * 4 + 1]);
                    dmin = qMin(dmin, d); dmax = qMax(dmax, d); cmean += c;
                    const float t = qBound(-1.0f, d / searchTexels, 1.0f);
                    dl[x * 3 + 0] = (uchar)(t > 0 ? 128 + 127 * t : 128 + 127 * t);
                    dl[x * 3 + 1] = (uchar)(128 - 127 * qAbs(t));
                    dl[x * 3 + 2] = (uchar)(t < 0 ? 128 - 127 * t : 128 + 127 * t);
                    cl[x] = (uchar)qBound(0.0f, c * 255.0f, 255.0f);
                }
            }
            disp.save(QString::fromUtf8(dbg) + "/disparity.png");
            {   // raw floats, row 0 = theta0 (top), for offline analysis
                QFile fd(QString::fromUtf8(dbg) + "/disparity.f32"), fc(QString::fromUtf8(dbg) + "/confidence.f32");
                if (fd.open(QIODevice::WriteOnly) && fc.open(QIODevice::WriteOnly)) {
                    for (int y = 0; y < H; ++y) {
                        const int sy = H - 1 - y;
                        for (int x = 0; x < W; ++x) {
                            const float d = float(packed[(sy * W + x) * 4 + 0]);
                            const float c = float(packed[(sy * W + x) * 4 + 1]);
                            fd.write(reinterpret_cast<const char *>(&d), 4);
                            fc.write(reinterpret_cast<const char *>(&c), 4);
                        }
                    }
                }
            }
            conf.save(QString::fromUtf8(dbg) + "/confidence.png");
            // Objective seam metric: mean ZNCC (7x9 patches) between front and
            // rear over the overlap rows, at d = 0 and at the smoothed disparity.
            auto zncc = [&](int u, int v, float d) {
                double fs = 0, fs2 = 0, rs = 0, rs2 = 0, cr = 0; int n = 0;
                for (int j = -4; j <= 4; ++j) for (int i = -3; i <= 3; ++i) {
                    const int uu = ((u + i) % W + W) % W;
                    const int vf = qBound(0, v + j, H - 1);
                    const int vr = qBound(0, v + j + (int)std::lround(d), H - 1);
                    const double a = bandF[vf * W + uu] / 255.0, b = bandR[vr * W + uu] / 255.0;
                    fs += a; fs2 += a * a; rs += b; rs2 += b * b; cr += a * b; n++;
                }
                const double fm = fs / n, rm = rs / n;
                const double den = std::sqrt(qMax(1e-12, (fs2 / n - fm * fm) * (rs2 / n - rm * rm)));
                return (cr / n - fm * rm) / den;
            };
            double z0 = 0, zd = 0; int nz = 0;
            for (int v = vMin; v <= vMax; v += 2)
                for (int u = 0; u < W; u += 4) {
                    const float d = float(packed[(v * W + u) * 4 + 0]);
                    z0 += zncc(u, v, 0.0f); zd += zncc(u, v, d); nz++;
                }
            qInfo("Flow debug: overlap rows [%d,%d] of %d; disparity range [%.2f, %.2f] texels, "
                  "mean confidence %.3f; band alignment ZNCC %.3f at d=0 -> %.3f with disparity "
                  "(search +-%d, smooth r=%d)",
                  vMin, vMax, H, dmin, dmax, cmean / (W * H), z0 / qMax(1, nz), zd / qMax(1, nz),
                  searchTexels, smoothRadius);
        }
    }

    // Measurement hook: RENDER360_FLOW_CONST=<texels> replaces the field with a
    // constant, so the shader path can be verified against a known shift.
    {
        const QByteArray cst = qgetenv("RENDER360_FLOW_CONST");
        if (!cst.isEmpty()) {
            const float d = cst.toFloat();
            for (int i = 0; i < outFlow->size(); i += 2) (*outFlow)[i + 1] = d;
            m_prevFlow.clear();
            qWarning() << "Flow: CONSTANT disparity hook" << d << "texels";
            return true;
        }
    }

    // Temporal smoothing. Scene depth at a given direction changes slowly
    // compared with frame rate, while single-frame matching noise does not.
    if (m_prevFlow.size() == outFlow->size()) {
        for (int i = 0; i < outFlow->size(); i += 2)
            (*outFlow)[i + 1] = kDispTemporalAlpha * (*outFlow)[i + 1]
                              + (1.0f - kDispTemporalAlpha) * m_prevFlow[i + 1];
    }
    m_prevFlow = *outFlow;
    if (kTiming)
        qInfo("Flow timing: total %.1f ms  [upload+band %.1f | seam %.1f | match %.1f | readback %.1f | fill+smooth %.1f]",
              tAll.nsecsElapsed() / 1e6, tBand, tSeam - tBand, tMatch - tSeam, tRead - tMatch, tCpu - tRead);
    return true;
}


// ---------------------------------------------------------------------------
// Pull-push hole filling + bilateral smoothing (CPU, band resolution)
// ---------------------------------------------------------------------------
void FlowRenderer::fillAndSmooth(const QVector<float> &dIn, const QVector<float> &cIn,
                                 int W, int H, int radius,
                                 QVector<float> *dOut, QVector<float> *cOut)
{
    // --- pull: build a pyramid of (sum w*d, sum w) ---
    struct Level { int w, h; QVector<double> sd, sw; };
    QVector<Level> pyr;
    {
        Level l0; l0.w = W; l0.h = H; l0.sd.resize(W * H); l0.sw.resize(W * H);
        for (int i = 0; i < W * H; ++i) { l0.sw[i] = cIn[i]; l0.sd[i] = double(cIn[i]) * dIn[i]; }
        pyr.append(l0);
        while (pyr.last().w > 2 && pyr.last().h > 2) {
            const Level &p = pyr.last();
            Level l; l.w = (p.w + 1) / 2; l.h = (p.h + 1) / 2;
            l.sd.resize(l.w * l.h); l.sw.resize(l.w * l.h);
            for (int y = 0; y < l.h; ++y) for (int x = 0; x < l.w; ++x) {
                double sd = 0, sw = 0;
                for (int dy = 0; dy < 2; ++dy) for (int dx = 0; dx < 2; ++dx) {
                    const int sx = qMin(2 * x + dx, p.w - 1), sy = qMin(2 * y + dy, p.h - 1);
                    sd += p.sd[sy * p.w + sx]; sw += p.sw[sy * p.w + sx];
                }
                l.sd[y * l.w + x] = sd; l.sw[y * l.w + x] = sw;
            }
            pyr.append(l);
        }
    }
    // --- push: from coarse to fine, fill where the finer level lacks support ---
    // Confidence saturates at 1 per texel-equivalent so a well-supported area
    // is not overridden by the coarse average.
    for (int li = pyr.size() - 2; li >= 0; --li) {
        Level &f = pyr[li]; const Level &c = pyr[li + 1];
        for (int y = 0; y < f.h; ++y) for (int x = 0; x < f.w; ++x) {
            const int i = y * f.w + x;
            const double w = f.sw[i];
            if (w >= 1.0) continue;                 // fully supported
            const int cx = qMin(x / 2, c.w - 1), cy = qMin(y / 2, c.h - 1);
            const double cw = c.sw[cy * c.w + cx];
            if (cw <= 1e-9) continue;
            const double cd = c.sd[cy * c.w + cx] / cw;
            const double add = (1.0 - w);           // fill the missing support
            f.sd[i] += add * cd; f.sw[i] += add;
        }
    }
    QVector<float> dF(W * H), cF(W * H);
    for (int i = 0; i < W * H; ++i) {
        const double w = pyr[0].sw[i];
        dF[i] = (w > 1e-9) ? float(pyr[0].sd[i] / w) : 0.0f;
        cF[i] = qBound(0.0f, cIn[i], 1.0f);        // keep the ORIGINAL support as confidence
    }

    // --- bilateral smoothing: Gaussian in space x confidence x disparity similarity ---
    // u wraps (phi), v clamps (theta). Two separable-ish passes (u then v),
    // repeated, are a good enough approximation at this resolution.
    const int R = qBound(1, radius, 48);
    const double sigma = R * 0.5, inv2s2 = 1.0 / (2.0 * sigma * sigma);
    const double sigD = kDispBilateralSigma, invD2 = 1.0 / (2.0 * sigD * sigD);
    QVector<float> gauss(2 * R + 1);
    for (int t = -R; t <= R; ++t) gauss[t + R] = float(std::exp(-double(t * t) * inv2s2));
    QVector<float> a = dF, b(W * H);
    for (int pass = 0; pass < 2; ++pass) {
        for (int dir = 0; dir < 2; ++dir) {
            for (int y = 0; y < H; ++y) for (int x = 0; x < W; ++x) {
                const int i = y * W + x;
                const float self = a[i];
                float sd = 0, sw = 0;
                for (int t = -R; t <= R; ++t) {
                    int xx = x, yy = y;
                    if (dir == 0) xx = ((x + t) % W + W) % W; else yy = qBound(0, y + t, H - 1);
                    const int j = yy * W + xx;
                    const float dd = a[j] - self;
                    // filled texels carry a floor confidence so they still smooth
                    const float w = gauss[t + R] * (0.05f + cF[j]) * std::exp(-dd * dd * (float)invD2);
                    sd += w * a[j]; sw += w;
                }
                b[i] = (sw > 1e-9f) ? sd / sw : self;
            }
            std::swap(a, b);
        }
    }
    *dOut = a;
    *cOut = cF;
}

QImage FlowRenderer::packFlowToImage(const QVector<float> &flow, int w, int h,
                                     float *encodeOut)
{
    const float k = flowEncodeScaleFor(flow, w, h);
    if (encodeOut)
        *encodeOut = k;

    QImage img(w, h, QImage::Format_RGBA8888);
    if (w <= 0 || h <= 0 || flow.size() < w * h * 2)
        return img;

    const float invW = 1.0f / w;
    const float invH = 1.0f / h;
    // glReadPixels returns rows bottom-up (v=1 first), while project.frag looks
    // up the flow at v_band = 0 for theta0 — so flip the rows when packing so
    // the uploaded texture's top row (v=0) holds the theta0 flow.
    for (int y = 0; y < h; ++y) {
        uchar *line = img.scanLine(y);
        const int srcY = h - 1 - y;
        for (int x = 0; x < w; ++x) {
            const int i = (srcY * w + x) * 2;
            const float ndu = flow[i + 0] * invW;
            const float ndv = flow[i + 1] * invH;
            line[x * 4 + 0] = (uchar)qBound(0.0, (ndu * k + 0.5) * 255.0, 255.0);
            line[x * 4 + 1] = (uchar)qBound(0.0, (ndv * k + 0.5) * 255.0, 255.0);
            line[x * 4 + 2] = 0;
            line[x * 4 + 3] = 255;
        }
    }
    return img;
}

QImage FlowRenderer::packSeamToImage(const QVector<float> &seam, int w)
{
    QImage img(w, 1, QImage::Format_Grayscale8);
    if (w <= 0 || seam.size() < w)
        return img;
    for (int x = 0; x < w; ++x) {
        float v = qBound(0.0f, seam[x], 1.0f);
        img.scanLine(0)[x] = (uchar)(v * 255.0f);
    }
    return img;
}

// ---------------------------------------------------------------------------
// FlowWorker
// ---------------------------------------------------------------------------

FlowWorker::FlowWorker(QObject *parent)
    : QObject(parent)
{
}

void FlowWorker::computeFrame(const DecodedFrame &frame, const FlowCalibration &cal,
                              const FlowSettings &settings)
{
    if (!m_renderer.isReady()) {
        QString err;
        if (!m_renderer.initialize(&err)) {
            if (!m_failed) {
                m_failed = true;
                emit flowFailed(err);
            }
            return;
        }
    }
    QVector<float> flow;
    QVector<float> seam;
    QString err;
    if (!m_renderer.compute(frame, cal, settings, &flow, &seam, &err)) {
        if (!m_failed) {
            m_failed = true;
            emit flowFailed(err);
        }
        return;
    }
    m_failed = false;
    float encode = 16.0f;
    QImage img = FlowRenderer::packFlowToImage(flow, m_renderer.bandWidth(),
                                                m_renderer.bandHeight(), &encode);
    QImage seamImg = FlowRenderer::packSeamToImage(seam, m_renderer.bandWidth());
    emit flowReady(img, encode, seamImg);
}
