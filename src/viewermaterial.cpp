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
};
static_assert(sizeof(ViewerUniforms) == 224, "ViewerUniforms must match std140 layout");

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
    }
    if (tex) {
        tex->setFiltering(QSGTexture::Linear);
        tex->setMipmapFiltering(QSGTexture::None);
        tex->setHorizontalWrapMode(QSGTexture::ClampToEdge);
        tex->setVerticalWrapMode(QSGTexture::ClampToEdge);
        tex->commitTextureOperations(state.rhi(), state.resourceUpdateBatch());
        *texture = tex;
    }
}
