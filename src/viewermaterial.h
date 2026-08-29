#ifndef VIEWERMATERIAL_H
#define VIEWERMATERIAL_H

#include <QSGMaterial>
#include <QSGMaterialShader>
#include <QSGGeometryNode>
#include <QSGTexture>
#include <QMatrix4x4>
#include "flowrenderer.h"

class ViewerMaterial : public QSGMaterial
{
public:
    ViewerMaterial();
    ~ViewerMaterial();

    QSGMaterialShader *createShader(QSGRendererInterface::RenderMode renderMode = QSGRendererInterface::RenderMode2D) const override;
    QSGMaterialType *type() const override;
    int compare(const QSGMaterial *other) const override;

    void setYaw(float v) { m_yaw = v; }
    void setPitch(float v) { m_pitch = v; }
    void setRoll(float v) { m_roll = v; }
    void setFov(float v) { m_fov = v; }
    void setActiveLens(int v) { m_activeLens = v; }
    void setProjection(int v) { m_projection = v; }
    void setImuMatrix(const QMatrix4x4 &m) { m_imuMatrix = m; }
    void setViewAspect(float aspect) { m_viewAspect = aspect; }
    void setFullRange(bool v) { m_fullRange = v; }
    void setCalibration(float fcx, float fcy, float fr, float fk1, float fk2, float frot,
                        float rcx, float rcy, float rr, float rk1, float rk2, float rrot,
                        float blendStart);
    void setHFlip(bool front, bool rear) { m_frontHFlip = front; m_rearHFlip = rear; }
    void setTextures(QSGTexture *y, QSGTexture *u, QSGTexture *v);

    // Optical-flow stitching (see OpticalFlow.md). setFlowTexture installs
    // the packed flow field; setFlow toggles whether project.frag warps the
    // blend band; setFlowEncode passes the packing scale used by the flow
    // producer so the shader can decode the texture.
    void setFlowTexture(QSGTexture *flow) { m_flowTex = flow; }
    void setFlow(bool stitch) { m_flowStitch = stitch; }
    void setFlowStrength(float strength) { m_flowStrength = strength; }
    void setFlowEncode(float encode) { m_flowEncode = encode; }
    void setBandTheta(float theta0, float theta1) { m_bandTheta0 = theta0; m_bandTheta1 = theta1; }
    void setSeamTexture(QSGTexture *seam) { m_seamTex = seam; }
    void setSeamStitch(bool stitch) { m_seamStitch = stitch; }
    void setSeamStrength(float strength) { m_seamStrength = strength; }
    // Tone curves: 256x1 RGBA8 LUT texture (see ColorGrade::curveLut).
    void setCurveTexture(QSGTexture *lut) { m_curveTex = lut; }
    void setCurves(bool on) { m_curves = on; }

    // Colour grading values (see ColorGrade / project.frag for semantics).
    void setColorGrade(float brightness, float contrast, float saturation, float pop,
                       float brightLows, float brightLowMids, float brightHighMids, float brightHighs,
                       float redLows, float redMids, float redHighs,
                       float greenLows, float greenMids, float greenHighs,
                       float blueLows, float blueMids, float blueHighs);

    QSGTexture *yTexture() const { return m_yTex; }
    QSGTexture *uTexture() const { return m_uTex; }
    QSGTexture *vTexture() const { return m_vTex; }
    // When flow is off the flow texture is null, but updateSampledImage must
    // still return a valid QSGTexture for binding 4 (Qt asserts otherwise).
    // Return the Y texture as a harmless stand-in — project.frag gates the
    // u_flow sample on u_flowStitch, so the bound texture's contents never
    // matter when stitching is disabled.
    QSGTexture *flowTexture() const { return m_flowTex ? m_flowTex : m_yTex; }
    QSGTexture *seamTexture() const { return m_seamTex ? m_seamTex : m_yTex; }
    QSGTexture *curveTexture() const { return m_curveTex ? m_curveTex : m_yTex; }

    QByteArray compileUniformData() const;

    float yaw() const { return m_yaw; }
    float pitch() const { return m_pitch; }
    float roll() const { return m_roll; }
    float fov() const { return m_fov; }
    int activeLens() const { return m_activeLens; }
    QMatrix4x4 imuMatrix() const { return m_imuMatrix; }
    bool fullRange() const { return m_fullRange; }
    float frontCenterX() const { return m_frontCenterX; }
    float frontCenterY() const { return m_frontCenterY; }
    float frontRadius() const { return m_frontRadius; }
    float frontK1() const { return m_frontK1; }
    float frontK2() const { return m_frontK2; }
    float frontRotation() const { return m_frontRotation; }
    float rearCenterX() const { return m_rearCenterX; }
    float rearCenterY() const { return m_rearCenterY; }
    float rearRadius() const { return m_rearRadius; }
    float rearK1() const { return m_rearK1; }
    float rearK2() const { return m_rearK2; }
    float rearRotation() const { return m_rearRotation; }
    float blendStart() const { return m_blendStart; }

private:
    QSGTexture *m_yTex, *m_uTex, *m_vTex, *m_flowTex;
    bool m_flowStitch;
    float m_flowStrength;
    float m_bandTheta0, m_bandTheta1;
    float m_flowEncode;
    QSGTexture *m_seamTex;
    bool m_seamStitch;
    float m_seamStrength;
    QSGTexture *m_curveTex;
    bool m_curves;
    float m_viewAspect;
    bool m_fullRange;
    float m_yaw, m_pitch, m_roll, m_fov;
    int m_activeLens;
    int m_projection;
    QMatrix4x4 m_imuMatrix;
    float m_frontCenterX, m_frontCenterY, m_frontRadius, m_frontK1, m_frontK2;
    float m_frontRotation;
    bool m_frontHFlip;
    float m_rearCenterX, m_rearCenterY, m_rearRadius, m_rearK1, m_rearK2;
    float m_rearRotation;
    bool m_rearHFlip;
    float m_blendStart;
    float m_brightness, m_contrast, m_saturation, m_pop;
    float m_brightLows, m_brightLowMids, m_brightHighMids, m_brightHighs;
    float m_redLows, m_redMids, m_redHighs;
    float m_greenLows, m_greenMids, m_greenHighs;
    float m_blueLows, m_blueMids, m_blueHighs;
};

class ViewerMaterialShader : public QSGMaterialShader
{
public:
    ViewerMaterialShader();

    void updateSampledImage(RenderState &state, int binding, QSGTexture **texture,
                            QSGMaterial *newMaterial, QSGMaterial *oldMaterial) override;
    bool updateUniformData(RenderState &state, QSGMaterial *newMaterial,
                           QSGMaterial *oldMaterial) override;
};

#endif // VIEWERMATERIAL_H
