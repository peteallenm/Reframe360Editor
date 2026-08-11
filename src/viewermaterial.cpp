#include "viewermaterial.h"
#include <QOpenGLFunctions>
#include <QOpenGLContext>
#include <cstring>

struct ViewerUniforms {
    float qt_Matrix[16];  // offset 0   (Qt scene graph standard)
    float qt_Opacity;     // offset 64  (Qt scene graph standard)
    float yaw;            // offset 68
    float pitch;          // offset 72
    float roll;           // offset 76
    float fov;            // offset 80
    int activeLens;       // offset 84
    float viewAspect;     // offset 88
    float _padA;          // offset 92
    int fullRange;        // offset 96
    int projection;       // offset 100
    float frontCenter[2]; // offset 104
    float frontRadius;    // offset 112
    float frontK1;        // offset 116
    float frontK2;        // offset 120
    int hflipFlags;       // offset 124 (bit0=front, bit1=rear)
    float rearCenter[2];  // offset 128
    float rearRadius;     // offset 136
    float rearK1;         // offset 140
    float rearK2;         // offset 144
    float blendStart;     // offset 148
    float frontRotation;  // offset 152
    float rearRotation;   // offset 156
    float imuMatrix[16];  // offset 160 (16-aligned)
    float brightness;     // offset 224
    float contrast;       // offset 228
    float saturation;     // offset 232
    float pop;            // offset 236
    float brightLows;       // offset 240
    float brightLowMids;    // offset 244
    float brightHighMids;   // offset 248
    float brightHighs;      // offset 252
    float redLows;          // offset 256
    float redMids;          // offset 260
    float redHighs;         // offset 264
    float greenLows;        // offset 268
    float greenMids;        // offset 272
    float greenHighs;       // offset 276
    float blueLows;         // offset 280
    float blueMids;         // offset 284
    float blueHighs;        // offset 288
    float _padG[3];         // offset 292 (std140 rounds the struct to 16B -> 304)
    // --- Flow stitching (see OpticalFlow.md; std140-locked to project.frag) ---
    int flowStitch;         // offset 304
    int flowIterations;     // offset 308  (for FlowRenderer, not the shader)
    float flowStrength;     // offset 312
    float bandTheta0;       // offset 316  (radians)
    float bandTheta1;       // offset 320  (radians)
    float flowEncode;       // offset 324  (decode scale of the packed flow)
    float _padH[2];         // offset 328  (std140 rounds the struct to 16B -> 336)
};
static_assert(sizeof(ViewerUniforms) == 336, "ViewerUniforms must match std140 layout");

ViewerMaterial::ViewerMaterial()
    : m_yTex(nullptr), m_uTex(nullptr), m_vTex(nullptr)
    , m_viewAspect(1.0f), m_fullRange(true)
    , m_yaw(0.0f), m_pitch(0.0f), m_roll(0.0f), m_fov(90.0f)
    , m_activeLens(2)
    , m_projection(0)
    , m_frontCenterX(0.5f), m_frontCenterY(0.5f), m_frontRadius(0.5f), m_frontK1(0.0f), m_frontK2(0.0f)
    , m_frontRotation(0.0f), m_frontHFlip(false)
    , m_rearCenterX(0.5f), m_rearCenterY(0.5f), m_rearRadius(0.5f), m_rearK1(0.0f), m_rearK2(0.0f)
    , m_rearRotation(180.0f), m_rearHFlip(false)
    , m_blendStart(0.9f)
    , m_brightness(0.0f), m_contrast(1.0f), m_saturation(1.0f), m_pop(0.0f)
    , m_brightLows(0.0f), m_brightLowMids(0.0f), m_brightHighMids(0.0f), m_brightHighs(0.0f)
    , m_redLows(0.0f), m_redMids(0.0f), m_redHighs(0.0f)
    , m_greenLows(0.0f), m_greenMids(0.0f), m_greenHighs(0.0f)
    , m_blueLows(0.0f), m_blueMids(0.0f), m_blueHighs(0.0f)
    , m_flowTex(nullptr), m_flowStitch(false), m_flowStrength(1.0f)
    , m_bandTheta0(kDefaultBandTheta0), m_bandTheta1(kDefaultBandTheta1)
    , m_flowEncode(16.0f)
{
}

ViewerMaterial::~ViewerMaterial()
{
}

QSGMaterialType *ViewerMaterial::type() const
{
    static QSGMaterialType type;
    return &type;
}

int ViewerMaterial::compare(const QSGMaterial *other) const
{
    const ViewerMaterial *m = static_cast<const ViewerMaterial*>(other);
    if (m_yTex != m->m_yTex) return (intptr_t)m_yTex - (intptr_t)m->m_yTex;
    if (m_flowTex != m->m_flowTex) return (intptr_t)m_flowTex - (intptr_t)m->m_flowTex;
    return 0;
}

QSGMaterialShader *ViewerMaterial::createShader(QSGRendererInterface::RenderMode) const
{
    return new ViewerMaterialShader();
}

void ViewerMaterial::setCalibration(float fcx, float fcy, float fr, float fk1, float fk2, float frot,
                                     float rcx, float rcy, float rr, float rk1, float rk2, float rrot,
                                     float blendStart)
{
    m_frontCenterX = fcx; m_frontCenterY = fcy; m_frontRadius = fr;
    m_frontK1 = fk1; m_frontK2 = fk2; m_frontRotation = frot;
    m_rearCenterX = rcx; m_rearCenterY = rcy; m_rearRadius = rr;
    m_rearK1 = rk1; m_rearK2 = rk2; m_rearRotation = rrot;
    m_blendStart = blendStart;
}

void ViewerMaterial::setTextures(QSGTexture *y, QSGTexture *u, QSGTexture *v)
{
    m_yTex = y;
    m_uTex = u;
    m_vTex = v;
}

void ViewerMaterial::setColorGrade(float brightness, float contrast, float saturation, float pop,
                                   float brightLows, float brightLowMids, float brightHighMids, float brightHighs,
                                   float redLows, float redMids, float redHighs,
                                   float greenLows, float greenMids, float greenHighs,
                                   float blueLows, float blueMids, float blueHighs)
{
    m_brightness = brightness;
    m_contrast = contrast;
    m_saturation = saturation;
    m_pop = pop;
    m_brightLows = brightLows;
    m_brightLowMids = brightLowMids;
    m_brightHighMids = brightHighMids;
    m_brightHighs = brightHighs;
    m_redLows = redLows;
    m_redMids = redMids;
    m_redHighs = redHighs;
    m_greenLows = greenLows;
    m_greenMids = greenMids;
    m_greenHighs = greenHighs;
    m_blueLows = blueLows;
    m_blueMids = blueMids;
    m_blueHighs = blueHighs;
}

QByteArray ViewerMaterial::compileUniformData() const
{
    ViewerUniforms u;
    memset(&u, 0, sizeof(ViewerUniforms));
    u.yaw = m_yaw;
    u.pitch = m_pitch;
    u.roll = m_roll;
    u.fov = m_fov;
    u.activeLens = m_activeLens;
    u.projection = m_projection;
    u.viewAspect = m_viewAspect;
    u.fullRange = m_fullRange ? 1 : 0;
    u.frontCenter[0] = m_frontCenterX;
    u.frontCenter[1] = m_frontCenterY;
    u.frontRadius = m_frontRadius;
    u.frontK1 = m_frontK1;
    u.frontK2 = m_frontK2;
    u.hflipFlags = (m_frontHFlip ? 1 : 0) | (m_rearHFlip ? 2 : 0);
    u.rearCenter[0] = m_rearCenterX;
    u.rearCenter[1] = m_rearCenterY;
    u.rearRadius = m_rearRadius;
    u.rearK1 = m_rearK1;
    u.rearK2 = m_rearK2;
    u.blendStart = m_blendStart;
    u.frontRotation = m_frontRotation;
    u.rearRotation = m_rearRotation;
    u.brightness = m_brightness;
    u.contrast = m_contrast;
    u.saturation = m_saturation;
    u.pop = m_pop;
    u.brightLows = m_brightLows;
    u.brightLowMids = m_brightLowMids;
    u.brightHighMids = m_brightHighMids;
    u.brightHighs = m_brightHighs;
    u.redLows = m_redLows;
    u.redMids = m_redMids;
    u.redHighs = m_redHighs;
    u.greenLows = m_greenLows;
    u.greenMids = m_greenMids;
    u.greenHighs = m_greenHighs;
    u.blueLows = m_blueLows;
    u.blueMids = m_blueMids;
    u.blueHighs = m_blueHighs;
    u.flowStitch = m_flowStitch ? 1 : 0;
    u.flowIterations = 0;   // FlowRenderer-only; unused by the shader
    u.flowStrength = m_flowStrength;
    u.bandTheta0 = m_bandTheta0;
    u.bandTheta1 = m_bandTheta1;
    u.flowEncode = m_flowEncode;

    const float *mat = m_imuMatrix.constData();
    for (int i = 0; i < 16; i++) u.imuMatrix[i] = mat[i];

    QByteArray bytes(sizeof(ViewerUniforms), Qt::Uninitialized);
    memcpy(bytes.data(), &u, sizeof(ViewerUniforms));
    return bytes;
}

ViewerMaterialShader::ViewerMaterialShader()
{
    setShaderFileName(QSGMaterialShader::VertexStage, ":/shaders/quad.vert.qsb");
    setShaderFileName(QSGMaterialShader::FragmentStage, ":/shaders/project.frag.qsb");
}

bool ViewerMaterialShader::updateUniformData(RenderState &state,
                                              QSGMaterial *newMaterial,
                                              QSGMaterial *oldMaterial)
{
    QByteArray *buf = state.uniformData();
    Q_ASSERT(buf->size() >= (int)sizeof(ViewerUniforms));
    bool changed = false;

    if (state.isMatrixDirty()) {
        const QMatrix4x4 m = state.combinedMatrix();
        memcpy(buf->data(), m.constData(), 64);
        changed = true;
    }

    if (state.isOpacityDirty()) {
        const float opacity = state.opacity();
        memcpy(buf->data() + 64, &opacity, 4);
        changed = true;
    }

    // Always re-upload the custom block: the material instance stays the same
    // across frames, so the oldMaterial != newMaterial check would only upload
    // once (at first paint), freezing yaw/pitch/roll/fov/imuMatrix/calibration.
    Q_UNUSED(oldMaterial);
    ViewerMaterial *m = static_cast<ViewerMaterial*>(newMaterial);
    QByteArray data = m->compileUniformData();
    memcpy(buf->data() + 68, data.constData() + 68, sizeof(ViewerUniforms) - 68);
    changed = true;

    return changed;
}

void ViewerMaterialShader::updateSampledImage(RenderState &state, int binding,
                                               QSGTexture **texture,
                                               QSGMaterial *newMaterial,
                                               QSGMaterial *)
{
    ViewerMaterial *m = static_cast<ViewerMaterial*>(newMaterial);
    QSGTexture *tex = nullptr;
    switch (binding) {
    case 1: tex = m->yTexture(); break;
    case 2: tex = m->uTexture(); break;
    case 3: tex = m->vTexture(); break;
    case 4: tex = m->flowTexture(); break;
    }
    if (tex) {
        tex->setFiltering(QSGTexture::Linear);
        tex->setMipmapFiltering(QSGTexture::None);
        // The flow's u axis is the periodic phi (front/back seam lives at the
        // u=0/1 wrap), so horizontal wrapping must not smear the seam texels.
        tex->setHorizontalWrapMode(binding == 4 ? QSGTexture::Repeat : QSGTexture::ClampToEdge);
        tex->setVerticalWrapMode(QSGTexture::ClampToEdge);
        tex->commitTextureOperations(state.rhi(), state.resourceUpdateBatch());
        *texture = tex;
    }
}
