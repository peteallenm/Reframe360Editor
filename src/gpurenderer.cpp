#include "gpurenderer.h"

#include <QFile>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVersionFunctionsFactory>
#include <QOpenGLShaderProgram>
#include <QSurfaceFormat>
#include <QVector2D>
#include <QtGlobal>
#include <QtCore/qfloat16.h>
#include <cstring>

#include "videodecoder.h"
#include "exporter.h"
#include "flowrenderer.h"

// ---------------------------------------------------------------------------
// Shader source adaptation.
//
// The shipped GLSL is written for Qt's RHI (qsb) with #version 440 and
// binding-qualified samplers/uniform blocks. For a standalone GL program we
// need a context that supports 440 (GL 4.4), which excludes a lot of real
// hardware. Instead we reuse the exact same source and adapt it minimally:
//   * lower #version 440 -> 330  (GL 3.3, universally available since ~2012)
//   * drop layout(binding = N) qualifiers (a GLSL 4.20 feature)
//   * flatten the Uniforms block into individual global uniforms so values
//     can be uploaded by name (QOpenGLShaderProgram cannot write into UBOs)
// The members and their names still come from project.frag, so the export
// shader can never drift from the viewer's.
// ---------------------------------------------------------------------------

static QByteArray adaptShader(const QByteArray &src)
{
    QByteArray s = src;
    s.replace("#version 440", "#version 330");
    s.replace("#version 430", "#version 330");
    s.replace("#version 450", "#version 330");
    s.replace("#version 460", "#version 330");
    s.replace("layout(binding = 1) uniform sampler2D", "uniform sampler2D");
    s.replace("layout(binding = 2) uniform sampler2D", "uniform sampler2D");
    s.replace("layout(binding = 3) uniform sampler2D", "uniform sampler2D");
    s.replace("layout(binding = 4) uniform sampler2D", "uniform sampler2D");
    s.replace("layout(binding = 5) uniform sampler2D", "uniform sampler2D");
    // layout(location = N) needs GLSL 410 (or an extension) on NVIDIA's
    // compiler, so strip them and assign attribute locations explicitly via
    // QOpenGLShaderProgram::bindAttributeLocation before linking. With a
    // single fragment output GL assigns it location 0 by default.
    s.replace("layout(location = 0) in", "in");
    s.replace("layout(location = 1) in", "in");
    s.replace("layout(location = 2) in", "in");
    s.replace("layout(location = 3) in", "in");
    s.replace("layout(location = 0) out", "out");
    s.replace("layout(location = 1) out", "out");
    s.replace("ubuf.qt_Matrix", "qt_Matrix");   // quad.vert's block instance
    return s;
}

static QByteArray flattenUniformBlock(QByteArray src)
{
    const QList<QByteArray> lines = src.split('\n');
    QList<QByteArray> out;
    out.reserve(lines.size());
    bool inBlock = false;
    for (const QByteArray &line : lines) {
        const QByteArray t = line.trimmed();
        if (t.startsWith("layout(std140") && t.contains("uniform Uniforms")) {
            inBlock = true;
            continue;
        }
        if (inBlock) {
            if (t == "};" || t == "} ubuf;") {
                inBlock = false;
                continue;
            }
            if (!t.isEmpty())
                out.append(QByteArray("uniform ") + t);
            continue;
        }
        out.append(line);
    }
    return out.join('\n');
}

// ---------------------------------------------------------------------------
// GpuRenderer
// ---------------------------------------------------------------------------

GpuRenderer::~GpuRenderer()
{
    destroy();
}

void GpuRenderer::destroy()
{
    if (!m_context)
        return;

    m_context->makeCurrent(m_surface);
    QOpenGLFunctions_3_3_Core *f = m_functions;

    delete m_program;
    m_program = nullptr;

    if (m_yTex) { f->glDeleteTextures(1, &m_yTex); m_yTex = 0; }
    if (m_uTex) { f->glDeleteTextures(1, &m_uTex); m_uTex = 0; }
    if (m_vTex) { f->glDeleteTextures(1, &m_vTex); m_vTex = 0; }
    if (m_flowTex) { f->glDeleteTextures(1, &m_flowTex); m_flowTex = 0; m_flowValid = false; }
    if (m_seamTex) { f->glDeleteTextures(1, &m_seamTex); m_seamTex = 0; m_seamValid = false; }
    if (m_colorTex) { f->glDeleteTextures(1, &m_colorTex); m_colorTex = 0; }
    if (m_vbo) { f->glDeleteBuffers(1, &m_vbo); m_vbo = 0; }
    if (m_ebo) { f->glDeleteBuffers(1, &m_ebo); m_ebo = 0; }
    if (m_vao) { f->glDeleteVertexArrays(1, &m_vao); m_vao = 0; }
    if (m_fbo) { f->glDeleteFramebuffers(1, &m_fbo); m_fbo = 0; }

    m_context->doneCurrent();
    delete m_context;
    m_context = nullptr;
    delete m_surface;
    m_surface = nullptr;
    m_ready = false;
}

bool GpuRenderer::initialize(QString *error)
{
    if (m_ready)
        return true;
    if (!createContext(error))
        return false;

    // Pull the GLSL out of the embedded resources. The .qsb variants are the
    // baked versions Qt's scene graph uses; here we compile the raw source
    // (also embedded via CMake: PREFIX "/shaders_raw" + path from the
    // project dir) so this renderer and the viewer stay in sync on one file.
    const QByteArray rawVert = m_testVertSrc.isEmpty()
            ? loadResource(":/shaders_raw/shaders/quad.vert") : m_testVertSrc;
    const QByteArray rawFrag = m_testFragSrc.isEmpty()
            ? loadResource(":/shaders_raw/shaders/project.frag") : m_testFragSrc;
    const QByteArray vertSrc = flattenUniformBlock(adaptShader(rawVert));
    const QByteArray fragSrc = flattenUniformBlock(adaptShader(rawFrag));

    // Compile, link, and bind the sampler units explicitly (binding
    // qualifiers were stripped for GLSL 330).
    if (!compileProgram(vertSrc, fragSrc, m_program, error))
        return false;
    m_context->makeCurrent(m_surface);
    m_program->bind();
    m_program->setUniformValue("u_texY", 1);
    m_program->setUniformValue("u_texU", 2);
    m_program->setUniformValue("u_texV", 3);
    m_program->setUniformValue("u_flow", 4);
    m_program->setUniformValue("u_seam", 5);

    QOpenGLFunctions_3_3_Core *f = m_functions;
    f->glGenTextures(1, &m_yTex);
    f->glGenTextures(1, &m_uTex);
    f->glGenTextures(1, &m_vTex);

    m_ready = true;
    return true;
}

QByteArray GpuRenderer::loadResource(const char *path)
{
    QFile f(QString::fromLatin1(path));
    if (!f.open(QIODevice::ReadOnly))
        return QByteArray();
    return f.readAll();
}

bool GpuRenderer::createContext(QString *error)
{
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(0);
    fmt.setStencilBufferSize(0);
    fmt.setRenderableType(QSurfaceFormat::OpenGL);

    m_surface = new QOffscreenSurface;
    m_surface->setFormat(fmt);
    m_surface->create();
    if (!m_surface->isValid()) {
        if (error) *error = QStringLiteral("Could not create an offscreen GL surface (no GPU available?)");
        return false;
    }

    m_context = new QOpenGLContext;
    m_context->setFormat(fmt);
    if (!m_context->create()) {
        if (error) *error = QStringLiteral("Could not create an OpenGL context");
        return false;
    }
    if (!m_context->makeCurrent(m_surface)) {
        if (error) *error = QStringLiteral("Could not activate the offscreen GL context");
        return false;
    }

    m_functions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(m_context);
    if (!m_functions) {
        if (error) *error = QStringLiteral("OpenGL 3.3+ function set unavailable");
        return false;
    }

    // GL 3.3+ is required for the adapted GLSL 330 shaders and core profile.
    const QByteArray version = QByteArray::number(m_context->format().majorVersion())
                             + "." + QByteArray::number(m_context->format().minorVersion());
    if (m_context->format().majorVersion() < 3
        || (m_context->format().majorVersion() == 3 && m_context->format().minorVersion() < 3)) {
        if (error) *error = QStringLiteral("OpenGL %1 is too old for GPU export (need 3.3+); using CPU renderer").arg(version.constData());
        return false;
    }
    return true;
}

bool GpuRenderer::compileProgram(const QByteArray &vertSrc, const QByteArray &fragSrc,
                                 QOpenGLShaderProgram *&program, QString *error)
{
    program = new QOpenGLShaderProgram;
    // Pin the vertex attribute locations to match the interleaved quad VBO
    // (x, y, u, v). Must happen before link().
    program->bindAttributeLocation("a_position", 0);
    program->bindAttributeLocation("a_texCoord", 1);
    if (!program->addShaderFromSourceCode(QOpenGLShader::Vertex, vertSrc)
        || !program->addShaderFromSourceCode(QOpenGLShader::Fragment, fragSrc)
        || !program->link()) {
        if (error) *error = QStringLiteral("Shader compile failed: %1").arg(program->log().trimmed());
        delete program;
        program = nullptr;
        return false;
    }
    return true;
}

bool GpuRenderer::ensureFramebuffer(int width, int height, QString *error)
{
    if (m_fbo && width == m_fbW && height == m_fbH)
        return true;

    QOpenGLFunctions_3_3_Core *f = m_functions;

    if (!m_fbo)
        f->glGenFramebuffers(1, &m_fbo);
    if (!m_colorTex)
        f->glGenTextures(1, &m_colorTex);

    f->glBindTexture(GL_TEXTURE_2D, m_colorTex);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
                    GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    f->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    f->glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_TEXTURE_2D, m_colorTex, 0);
    if (f->glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        if (error) *error = QStringLiteral("Framebuffer is incomplete");
        return false;
    }
    m_fbW = width;
    m_fbH = height;
    return true;
}

void GpuRenderer::uploadPlane(quint32 tex, int w, int h, int stride, const void *data)
{
    QOpenGLFunctions_3_3_Core *f = m_functions;
    f->glBindTexture(GL_TEXTURE_2D, tex);
    f->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (stride != w)
        f->glPixelStorei(GL_UNPACK_ROW_LENGTH, stride);
    else
        f->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, h, 0, GL_RED,
                    GL_UNSIGNED_BYTE, data);
    f->glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void GpuRenderer::setFlowData(const QVector<float> &flow, int w, int h)
{
    if (!m_ready || !m_context->makeCurrent(m_surface) || w <= 0 || h <= 0
        || flow.size() < w * h * 2) {
        m_flowValid = false;
        return;
    }

    QOpenGLFunctions_3_3_Core *f = m_functions;

    // Same packing as the preview path (see FlowRenderer::packFlowToImage):
    // e = ndu*k + 0.5 with ndu = du/w, so the field fits [0,1]. Stored as
    // RG16F (half float) — GL 3.3-safe, with enough precision after packing.
    m_flowEncode = flowEncodeScaleFor(flow, w, h);

    if (!m_flowTex)
        f->glGenTextures(1, &m_flowTex);
    f->glBindTexture(GL_TEXTURE_2D, m_flowTex);

    QVector<qfloat16> packed(w * h * 2);
    const float invW = 1.0f / w;
    const float invH = 1.0f / h;
    // Row-flip like FlowRenderer::packFlowToImage: glReadPixels is bottom-up,
    // while project.frag samples v_band=0 for theta0 (the GL texture's v=0
    // edge is data row 0).
    for (int i = 0; i < w * h; ++i) {
        const int y = i / w;
        const int x = i % w;
        const int j = ((h - 1 - y) * w + x) * 2;
        packed[i * 2 + 0] = qfloat16(qBound(0.0f, flow[j + 0] * invW * m_flowEncode + 0.5f, 1.0f));
        packed[i * 2 + 1] = qfloat16(qBound(0.0f, flow[j + 1] * invH * m_flowEncode + 0.5f, 1.0f));
    }

    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, w, h, 0, GL_RG, GL_HALF_FLOAT,
                    packed.constData());
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    m_flowValid = true;
}

void GpuRenderer::setSeamData(const QVector<float> &seam, int w)
{
    if (!m_ready || !m_context->makeCurrent(m_surface) || w <= 0
        || seam.size() < w) {
        m_seamValid = false;
        return;
    }
    QOpenGLFunctions_3_3_Core *f = m_functions;
    if (!m_seamTex)
        f->glGenTextures(1, &m_seamTex);
    f->glBindTexture(GL_TEXTURE_2D, m_seamTex);
    QVector<uchar> packed(w);
    for (int i = 0; i < w; ++i)
        packed[i] = (uchar)qBound(0.0f, seam[i] * 255.0f, 255.0f);
    f->glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, w, 1, 0, GL_RED,
                    GL_UNSIGNED_BYTE, packed.constData());
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    f->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    m_seamValid = true;
}

bool GpuRenderer::render(const DecodedFrame &frame, const ExportFrameState &s,
                         int width, int height, QImage *out, QString *error)
{
    if (!m_ready) {
        if (error) *error = QStringLiteral("GPU renderer is not initialized");
        return false;
    }
    if (!m_context->makeCurrent(m_surface)) {
        if (error) *error = QStringLiteral("Lost the offscreen GL context");
        return false;
    }

    const int W = width & ~1;
    const int H = height & ~1;
    if (W < 2 || H < 2) {
        if (error) *error = QStringLiteral("Invalid render size");
        return false;
    }

    QOpenGLFunctions_3_3_Core *f = m_functions;

    if (!ensureFramebuffer(W, H, error))
        return false;

    // Upload the decoded YUV planes.
    uploadPlane(m_yTex, frame.width, frame.height, frame.yStride,
                frame.yData.constData());
    uploadPlane(m_uTex, frame.width / 2, frame.height / 2, frame.uStride,
                frame.uData.constData());
    uploadPlane(m_vTex, frame.width / 2, frame.height / 2, frame.vStride,
                frame.vData.constData());

    // Set up the fullscreen quad on first use. Two indexed triangles cover
    // the whole viewport with uv v=0 at the top (NDC y=+1), matching the
    // viewer's texture orientation.
    if (!m_vao) {
        static const float kQuad[] = {
            // x      y      u      v
            -1.f,  -1.f,  0.f,   1.f,   // 0: bottom-left
             1.f,  -1.f,  1.f,   1.f,   // 1: bottom-right
             1.f,   1.f,  1.f,   0.f,   // 2: top-right
            -1.f,   1.f,  0.f,   0.f,   // 3: top-left
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

    // --- Set all viewer-mirrored uniforms by name (same values the live
    // viewer sends, see LensViewer/App). ---
    m_program->bind();
    QMatrix4x4 identity;
    m_program->setUniformValue("qt_Matrix", identity);
    m_program->setUniformValue("qt_Opacity", GLfloat(1.0f));

    m_program->setUniformValue("u_yaw", GLfloat(s.yaw));
    m_program->setUniformValue("u_pitch", GLfloat(s.pitch));
    m_program->setUniformValue("u_roll", GLfloat(s.roll));
    m_program->setUniformValue("u_fov", GLfloat(s.fov));
    m_program->setUniformValue("u_activeLens", GLint(s.activeLens));
    m_program->setUniformValue("u_viewAspect", GLfloat(float(W) / float(H)));
    m_program->setUniformValue("u_fullRange", GLint(s.fullRange ? 1 : 0));
    m_program->setUniformValue("u_projection", GLint(s.projection));

    m_program->setUniformValue("u_frontCenter", QVector2D(s.frontCenterX, s.frontCenterY));
    m_program->setUniformValue("u_frontRadius", GLfloat(s.frontRadius));
    m_program->setUniformValue("u_frontK1", GLfloat(s.frontK1));
    m_program->setUniformValue("u_frontK2", GLfloat(s.frontK2));
    m_program->setUniformValue("u_frontRotation", GLfloat(s.frontRotation));
    m_program->setUniformValue("u_hflipFlags", GLint((s.frontHFlip ? 1 : 0) | (s.rearHFlip ? 2 : 0)));

    m_program->setUniformValue("u_rearCenter", QVector2D(s.rearCenterX, s.rearCenterY));
    m_program->setUniformValue("u_rearRadius", GLfloat(s.rearRadius));
    m_program->setUniformValue("u_rearK1", GLfloat(s.rearK1));
    m_program->setUniformValue("u_rearK2", GLfloat(s.rearK2));
    m_program->setUniformValue("u_rearRotation", GLfloat(s.rearRotation));
    m_program->setUniformValue("u_blendStart", GLfloat(s.blendStart));

    QMatrix4x4 imuMat;
    imuMat.rotate(s.imuOrientation.conjugated());
    m_program->setUniformValue("u_imuMatrix", imuMat);

    m_program->setUniformValue("u_brightness", GLfloat(s.brightness));
    m_program->setUniformValue("u_contrast", GLfloat(s.contrast));
    m_program->setUniformValue("u_saturation", GLfloat(s.saturation));
    m_program->setUniformValue("u_pop", GLfloat(s.pop));
    m_program->setUniformValue("u_brightLows", GLfloat(s.brightLows));
    m_program->setUniformValue("u_brightLowMids", GLfloat(s.brightLowMids));
    m_program->setUniformValue("u_brightHighMids", GLfloat(s.brightHighMids));
    m_program->setUniformValue("u_brightHighs", GLfloat(s.brightHighs));
    m_program->setUniformValue("u_redLows", GLfloat(s.redLows));
    m_program->setUniformValue("u_redMids", GLfloat(s.redMids));
    m_program->setUniformValue("u_redHighs", GLfloat(s.redHighs));
    m_program->setUniformValue("u_greenLows", GLfloat(s.greenLows));
    m_program->setUniformValue("u_greenMids", GLfloat(s.greenMids));
    m_program->setUniformValue("u_greenHighs", GLfloat(s.greenHighs));
    m_program->setUniformValue("u_blueLows", GLfloat(s.blueLows));
    m_program->setUniformValue("u_blueMids", GLfloat(s.blueMids));
    m_program->setUniformValue("u_blueHighs", GLfloat(s.blueHighs));

    // Optical-flow stitching uniforms (only consumed when state.flowStitch is
    // set; otherwise u_flowStitch=0 and the shader never samples u_flow).
    m_program->setUniformValue("u_flowStitch", GLint(s.flowStitch ? 1 : 0));
    m_program->setUniformValue("u_flowStrength", GLfloat(s.flowStrength));
    m_program->setUniformValue("u_bandTheta0", GLfloat(s.bandTheta0));
    m_program->setUniformValue("u_bandTheta1", GLfloat(s.bandTheta1));
    m_program->setUniformValue("u_flowEncode", GLfloat(m_flowEncode));
    m_program->setUniformValue("u_seamStitch", GLint((s.seamStitch && m_seamValid) ? 1 : 0));
    m_program->setUniformValue("u_seamStrength", GLfloat(s.seamStrength));

    // Bind the YUV textures to sampler units 1..3 and the flow field to 4.
    f->glActiveTexture(GL_TEXTURE1);
    f->glBindTexture(GL_TEXTURE_2D, m_yTex);
    f->glActiveTexture(GL_TEXTURE2);
    f->glBindTexture(GL_TEXTURE_2D, m_uTex);
    f->glActiveTexture(GL_TEXTURE3);
    f->glBindTexture(GL_TEXTURE_2D, m_vTex);
    if (s.flowStitch && m_flowValid) {
        f->glActiveTexture(GL_TEXTURE4);
        f->glBindTexture(GL_TEXTURE_2D, m_flowTex);
    }
    if (s.seamStitch && m_seamValid) {
        f->glActiveTexture(GL_TEXTURE5);
        f->glBindTexture(GL_TEXTURE_2D, m_seamTex);
    }

    f->glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    f->glViewport(0, 0, W, H);
    f->glDisable(GL_DEPTH_TEST);
    f->glDisable(GL_BLEND);
    f->glClearColor(0.f, 0.f, 0.f, 1.f);
    f->glClear(GL_COLOR_BUFFER_BIT);

    f->glBindVertexArray(m_vao);
    f->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, nullptr);
    f->glBindVertexArray(0);

    // Read back RGBA (bottom-up) and assemble a top-down RGB888 image.
    QByteArray buf(W * H * 4, Qt::Uninitialized);
    f->glPixelStorei(GL_PACK_ALIGNMENT, 1);
    f->glReadPixels(0, 0, W, H, GL_RGBA, GL_UNSIGNED_BYTE, buf.data());

    QImage img(W, H, QImage::Format_RGB888);
    for (int y = 0; y < H; ++y) {
        const uchar *src = reinterpret_cast<const uchar *>(buf.constData())
                         + (H - 1 - y) * W * 4;
        uchar *dst = img.scanLine(y);
        for (int x = 0; x < W; ++x) {
            dst[x * 3 + 0] = src[x * 4 + 0];
            dst[x * 3 + 1] = src[x * 4 + 1];
            dst[x * 3 + 2] = src[x * 4 + 2];
        }
    }
    *out = img;
    return true;
}
