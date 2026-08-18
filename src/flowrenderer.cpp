#include "flowrenderer.h"

#include <QFile>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLFunctions_4_4_Core>
#include <QOpenGLVersionFunctionsFactory>
#include <QOpenGLShaderProgram>
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
    QOpenGLFunctions_4_4_Core *f = m_functions;

    delete m_bandProgram; m_bandProgram = nullptr;
    delete m_gradProgram; m_gradProgram = nullptr;
    delete m_iterProgram; m_iterProgram = nullptr;
    delete m_seamCostProgram; m_seamCostProgram = nullptr;
    delete m_seamDpProgram; m_seamDpProgram = nullptr;

    if (m_yTex) { f->glDeleteTextures(1, &m_yTex); m_yTex = 0; }
    if (m_bandFront) { f->glDeleteTextures(1, &m_bandFront); m_bandFront = 0; }
    if (m_bandRear) { f->glDeleteTextures(1, &m_bandRear); m_bandRear = 0; }
    if (m_gradients) { f->glDeleteTextures(1, &m_gradients); m_gradients = 0; }
    if (m_flowA) { f->glDeleteTextures(1, &m_flowA); m_flowA = 0; }
    if (m_flowB) { f->glDeleteTextures(1, &m_flowB); m_flowB = 0; }
    if (m_costTex) { f->glDeleteTextures(1, &m_costTex); m_costTex = 0; }
    if (m_dpA) { f->glDeleteTextures(1, &m_dpA); m_dpA = 0; }
    if (m_dpB) { f->glDeleteTextures(1, &m_dpB); m_dpB = 0; }
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
    QSurfaceFormat fmt;
    fmt.setVersion(4, 4);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(0);
    fmt.setStencilBufferSize(0);
    fmt.setRenderableType(QSurfaceFormat::OpenGL);

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
        if (error) *error = QStringLiteral("Flow: could not create an OpenGL 4.4 context");
        return false;
    }
    if (!m_context->makeCurrent(m_surface)) {
        if (error) *error = QStringLiteral("Flow: could not activate the offscreen GL context");
        return false;
    }

    m_functions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_4_4_Core>(m_context);
    if (!m_functions) {
        if (error) *error = QStringLiteral("Flow: OpenGL 4.4 function set unavailable");
        return false;
    }

    // The flow shaders use GLSL 440 features (layout(binding=...), RG32F
    // render targets), so the context must really be 4.4 (or newer).
    const int maj = m_context->format().majorVersion();
    const int min = m_context->format().minorVersion();
    if (maj < 4 || (maj == 4 && min < 4)) {
        if (error) *error = QStringLiteral("Flow stitching requires OpenGL 4.4+ (have %1.%2)")
                                .arg(maj).arg(min);
        return false;
    }
    return true;
}

bool FlowRenderer::compilePrograms(QString *error)
{
    auto compile = [this](const QByteArray &fragSrc, QOpenGLShaderProgram *&program, const char *what, QString *err) {
        program = new QOpenGLShaderProgram;
        program->bindAttributeLocation("a_position", 0);
        program->bindAttributeLocation("a_texCoord", 1);
        if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, kFlowVertSrc)
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
    if (!compile(loadResource(":/shaders_raw/shaders/hs_gradients.frag"), m_gradProgram, "hs_gradients", error))
        return false;
    if (!compile(loadResource(":/shaders_raw/shaders/hs_iteration.frag"), m_iterProgram, "hs_iteration", error))
        return false;
    if (!compile(loadResource(":/shaders_raw/shaders/seam_cost.frag"), m_seamCostProgram, "seam_cost", error))
        return false;
    if (!compile(loadResource(":/shaders_raw/shaders/seam_dp.frag"), m_seamDpProgram, "seam_dp", error))
        return false;
    return true;
}

bool FlowRenderer::createBandTargets(QString *error)
{
    QOpenGLFunctions_4_4_Core *f = m_functions;
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
        f->glGenTextures(1, &m_dpA);
        f->glGenTextures(1, &m_dpB);
        f->glBindTexture(GL_TEXTURE_2D, m_costTex);
        f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, W, H, 0, GL_RGBA, GL_FLOAT, nullptr);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        auto dpTex = [f](quint32 tex) {
            f->glBindTexture(GL_TEXTURE_2D, tex);
            f->glTexImage2D(GL_TEXTURE_2D, 0, GL_R16F, W, H, 0, GL_RED, GL_FLOAT, nullptr);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        };
        dpTex(m_dpA);
        dpTex(m_dpB);
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
        f->glDrawBuffer(GL_COLOR_ATTACHMENT0);
    }
    if (!m_flowFbo) {
        f->glGenFramebuffers(1, &m_flowFbo);
        // The color attachment is (re)attached to flowA/flowB per iteration.
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
        f->glDrawBuffer(GL_COLOR_ATTACHMENT0);
    }
    if (!m_seamFbo) {
        f->glGenFramebuffers(1, &m_seamFbo);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
        f->glDrawBuffer(GL_COLOR_ATTACHMENT0);
    }
    return true;
}

void FlowRenderer::drawQuad()
{
    QOpenGLFunctions_4_4_Core *f = m_functions;
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
    QOpenGLFunctions_4_4_Core *f = m_functions;

    const int W = kFlowBandWidth;
    const int H = kFlowBandHeight;

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

    // ---- Pass 1c: DP accumulation (W draws, one per column) ----
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_dpA, 0);
    f->glClearColor(0.f, 0.f, 0.f, 0.f);
    f->glClear(GL_COLOR_BUFFER_BIT);

    quint32 dpRead = m_dpA;
    quint32 dpWrite = m_dpB;
    for (int col = 0; col < W; ++col) {
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dpWrite, 0);
        f->glEnable(GL_SCISSOR_TEST);
        f->glScissor(col, 0, 1, H);
        m_seamDpProgram->bind();
        m_seamDpProgram->setUniformValue("u_cost", 0);
        m_seamDpProgram->setUniformValue("u_dpIn", 1);
        m_seamDpProgram->setUniformValue("u_col", col);
        m_seamDpProgram->setUniformValue("u_width", W);
        m_seamDpProgram->setUniformValue("u_height", H);
        f->glActiveTexture(GL_TEXTURE0);
        f->glBindTexture(GL_TEXTURE_2D, m_costTex);
        f->glActiveTexture(GL_TEXTURE1);
        f->glBindTexture(GL_TEXTURE_2D, dpRead);
        drawQuad();
        f->glDisable(GL_SCISSOR_TEST);
        quint32 tmp = dpRead; dpRead = dpWrite; dpWrite = tmp;
    }
    // dpRead now holds the final cumulative cost

    // ---- Pass 1d: CPU traceback ----
    if (outSeam) {
        outSeam->resize(W);
        f->glBindFramebuffer(GL_FRAMEBUFFER, m_seamFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dpRead, 0);
        f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
        QVector<qfloat16> dpPacked(W * H);
        f->glReadPixels(0, 0, W, H, GL_RED, GL_HALF_FLOAT, dpPacked.data());
        auto dpAt = [&](int u, int v) -> float {
            return float(dpPacked[v * W + u]);
        };
        // Clamp traceback to the central 60% of the band so the seam stays in
        // the region where both lenses have valid fisheye coverage.
        int vMin = (int)(0.2f * H);
        int vMax = (int)(0.8f * H);
        int bestV = vMin;
        float minVal = 1e30f;
        for (int v = vMin; v <= vMax; ++v) {
            float val = dpAt(W - 1, v);
            if (val < minVal) { minVal = val; bestV = v; }
        }
        int curV = bestV;
        for (int u = W - 1; u >= 0; --u) {
            (*outSeam)[u] = (float)curV / (float)(H - 1);
            if (u > 0) {
                int vL = qBound(vMin, curV - 1, vMax);
                int vR = qBound(vMin, curV + 1, vMax);
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

    // Restore viewport for pass 2
    f->glViewport(0, 0, W, H);

    // ---- Pass 2: spatio-temporal gradients. ----
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_gradFbo);
    m_gradProgram->bind();
    m_gradProgram->setUniformValue("u_bandFront", 0);
    m_gradProgram->setUniformValue("u_bandRear", 1);
    f->glActiveTexture(GL_TEXTURE0);
    f->glBindTexture(GL_TEXTURE_2D, m_bandFront);
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, m_bandRear);
    drawQuad();

    // ---- Pass 3: Horn-Schunck Jacobi relaxation, ping-pong RGBA16F. ----
    // Initial flow is zero: clear flowA before the first iteration reads it.
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_flowA, 0);
    if (f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        if (error) *error = QStringLiteral("Flow: flow framebuffer is incomplete "
                                           "(is RGBA16F renderable on this driver?)");
        return false;
    }
    f->glClearColor(0.f, 0.f, 0.f, 0.f);
    f->glClear(GL_COLOR_BUFFER_BIT);

    const int iterations = qBound(1, settings.iterations, 200);
    for (int i = 0; i < iterations; ++i) {
        const bool readA = (i % 2 == 0);
        const quint32 read = readA ? m_flowA : m_flowB;
        const quint32 write = readA ? m_flowB : m_flowA;

        f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
        f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, write, 0);
        m_iterProgram->bind();
        m_iterProgram->setUniformValue("u_alpha", settings.alpha);
        m_iterProgram->setUniformValue("u_gradients", 0);
        m_iterProgram->setUniformValue("u_flowIn", 1);
        f->glActiveTexture(GL_TEXTURE0);
        f->glBindTexture(GL_TEXTURE_2D, m_gradients);
        f->glActiveTexture(GL_TEXTURE1);
        f->glBindTexture(GL_TEXTURE_2D, read);
        drawQuad();
    }

    // After N iterations the result lives in flowA when N is even, else flowB.
    const quint32 finalTex = (iterations % 2 == 0) ? m_flowA : m_flowB;

    // ---- Read back the small flow field (1024x256 x 2 floats = 2 MB). ----
    if (!outFlow)
        return true;
    // GL_FLOAT readbacks from RGBA16F silently return zeros on some drivers
    // (that produced the all-zero flow_debug.png). Read the native half-float
    // format instead and convert here.
    QVector<qfloat16> packed(W * H * 4);
    f->glBindFramebuffer(GL_FRAMEBUFFER, m_flowFbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, finalTex, 0);
    f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
    f->glReadPixels(0, 0, W, H, GL_RGBA, GL_HALF_FLOAT, packed.data());
    outFlow->resize(W * H * 2);
    for (int i = 0; i < W * H; ++i) {
        (*outFlow)[i * 2 + 0] = float(packed[i * 4 + 0]);  // R = du
        (*outFlow)[i * 2 + 1] = float(packed[i * 4 + 1]);  // G = dv
    }
    return true;
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
