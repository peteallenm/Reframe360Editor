#include "lensviewer.h"
#include "viewermaterial.h"
#include <QSGGeometryNode>
#include <QSGGeometry>
#include <QSGSimpleTextureNode>
#include <QQuickWindow>
#include <QImage>
#include <QMatrix4x4>

LensViewer::LensViewer(QQuickItem *parent)
    : QQuickItem(parent)
    , m_decoder(nullptr)
    , m_yaw(0.0), m_pitch(0.0), m_roll(0.0), m_fov(90.0)
    , m_activeLens(2)
    , m_projection(0)
    , m_frontHFlip(false), m_rearHFlip(false)
    , m_calibration(nullptr)
    , m_colorGrade(nullptr)
    , m_imuOrientation(1.0f, 0.0f, 0.0f, 0.0f)
    , m_yTexture(nullptr)
    , m_uTexture(nullptr)
    , m_vTexture(nullptr)
    , m_lastFrameTimestamp(-1)
{
    setFlag(ItemHasContents, true);
}

LensViewer::~LensViewer()
{
}

void LensViewer::setDecoder(VideoDecoder *decoder)
{
    if (m_decoder == decoder) return;
    m_decoder = decoder;
    if (m_decoder) {
        connect(m_decoder, &VideoDecoder::frameReady, this, [this]() { update(); });
    }
    emit decoderChanged();
    update();
}

void LensViewer::setYaw(double yaw) { if (qFuzzyCompare(m_yaw, yaw)) return; m_yaw = yaw; emit yawChanged(); update(); }
void LensViewer::setPitch(double pitch) { if (qFuzzyCompare(m_pitch, pitch)) return; m_pitch = pitch; emit pitchChanged(); update(); }
void LensViewer::setRoll(double roll) { if (qFuzzyCompare(m_roll, roll)) return; m_roll = roll; emit rollChanged(); update(); }
void LensViewer::setFov(double fov) { if (qFuzzyCompare(m_fov, fov)) return; m_fov = fov; emit fovChanged(); update(); }
void LensViewer::setActiveLens(int lens) { if (m_activeLens == lens) return; m_activeLens = lens; emit activeLensChanged(); update(); }
void LensViewer::setProjection(int projection) { if (m_projection == projection) return; m_projection = projection; emit projectionChanged(); update(); }
void LensViewer::setFrontHFlip(bool v) { if (m_frontHFlip == v) return; m_frontHFlip = v; emit frontHFlipChanged(); update(); }
void LensViewer::setRearHFlip(bool v) { if (m_rearHFlip == v) return; m_rearHFlip = v; emit rearHFlipChanged(); update(); }
void LensViewer::setCalibration(CalibrationProfile *cal)
{
    if (m_calibration == cal) return;
    m_calibration = cal;
    if (cal) {
        connect(cal, &QObject::destroyed, this, [this]() { m_calibration = nullptr; update(); });
        connect(cal, &CalibrationProfile::frontCenterXChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontCenterYChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontRadiusChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontK1Changed, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontK2Changed, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontRotationChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::frontHFlipChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearCenterXChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearCenterYChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearRadiusChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearK1Changed, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearK2Changed, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearRotationChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::rearHFlipChanged, this, [this]() { update(); });
        connect(cal, &CalibrationProfile::blendStartChanged, this, [this]() { update(); });
    }
    emit calibrationChanged();
    update();
}
void LensViewer::setImuOrientation(const QQuaternion &q) { if (m_imuOrientation == q) return; m_imuOrientation = q; emit imuOrientationChanged(); update(); }

void LensViewer::setColorGrade(ColorGrade *grade)
{
    if (m_colorGrade == grade)
        return;
    m_colorGrade = grade;
    if (grade) {
        connect(grade, &QObject::destroyed, this, [this]() { m_colorGrade = nullptr; update(); });
        connect(grade, &ColorGrade::brightnessChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::contrastChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::saturationChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::popChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::brightLowsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::brightMidsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::brightHighsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::redLowsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::redMidsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::redHighsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::greenLowsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::greenMidsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::greenHighsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::blueLowsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::blueMidsChanged, this, [this]() { update(); });
        connect(grade, &ColorGrade::blueHighsChanged, this, [this]() { update(); });
    }
    emit colorGradeChanged();
    update();
}

QSGNode *LensViewer::updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *)
{
    if (!m_decoder || !m_decoder->hasFrame()) {
        return oldNode;
    }

    ViewerMaterial *material = nullptr;
    QSGGeometryNode *node = static_cast<QSGGeometryNode*>(oldNode);

    if (!node) {
        node = new QSGGeometryNode();

        QSGGeometry *geometry = new QSGGeometry(QSGGeometry::defaultAttributes_TexturedPoint2D(), 4);
        geometry->setDrawingMode(QSGGeometry::DrawTriangleStrip);
        node->setGeometry(geometry);
        node->setFlag(QSGNode::OwnsGeometry);

        material = new ViewerMaterial();
        node->setMaterial(material);
        node->setFlag(QSGNode::OwnsMaterial);
    } else {
        material = static_cast<ViewerMaterial*>(node->material());
    }

    QSGGeometry::TexturedPoint2D *vertices = node->geometry()->vertexDataAsTexturedPoint2D();
    float w = boundingRect().width();
    float h = boundingRect().height();

    vertices[0].x = 0; vertices[0].y = 0; vertices[0].tx = 0; vertices[0].ty = 0;
    vertices[1].x = w; vertices[1].y = 0; vertices[1].tx = 1; vertices[1].ty = 0;
    vertices[2].x = 0; vertices[2].y = h; vertices[2].tx = 0; vertices[2].ty = 1;
    vertices[3].x = w; vertices[3].y = h; vertices[3].tx = 1; vertices[3].ty = 1;

    material->setYaw((float)m_yaw);
    material->setPitch((float)m_pitch);
    material->setRoll((float)m_roll);
    material->setFov((float)m_fov);
    material->setActiveLens(m_activeLens);
    material->setProjection(m_projection);

    QMatrix4x4 imuMat;
    imuMat.rotate(m_imuOrientation.conjugated());
    material->setImuMatrix(imuMat);

    if (m_decoder && m_decoder->hasFrame()) {
        DecodedFrame frame = m_decoder->currentFrame();
        int frameKey = (int)(frame.timestamp * 1000);
        if (frameKey != m_lastFrameTimestamp) {
            m_lastFrameTimestamp = frameKey;

            QImage yImg = QImage(reinterpret_cast<const uchar*>(frame.yData.constData()), frame.width, frame.height, frame.yStride, QImage::Format_Grayscale8).copy();
            QImage uImg = QImage(reinterpret_cast<const uchar*>(frame.uData.constData()), frame.width / 2, frame.height / 2, frame.uStride, QImage::Format_Grayscale8).copy();
            QImage vImg = QImage(reinterpret_cast<const uchar*>(frame.vData.constData()), frame.width / 2, frame.height / 2, frame.vStride, QImage::Format_Grayscale8).copy();

            delete m_yTexture;
            delete m_uTexture;
            delete m_vTexture;

            m_yTexture = window()->createTextureFromImage(yImg);
            m_uTexture = window()->createTextureFromImage(uImg);
            m_vTexture = window()->createTextureFromImage(vImg);
        }

        QRectF r = boundingRect();
        float aspect = (r.height() > 1.0) ? (float)(r.width() / r.height()) : 1.0f;
        material->setViewAspect(aspect);
        material->setFullRange(m_decoder->isFullRange());
        material->setTextures(m_yTexture, m_uTexture, m_vTexture);
    }

    if (m_colorGrade) {
        material->setColorGrade(
            (float)m_colorGrade->brightness(), (float)m_colorGrade->contrast(),
            (float)m_colorGrade->saturation(), (float)m_colorGrade->pop(),
            (float)m_colorGrade->brightLows(), (float)m_colorGrade->brightMids(),
            (float)m_colorGrade->brightHighs(),
            (float)m_colorGrade->redLows(), (float)m_colorGrade->redMids(),
            (float)m_colorGrade->redHighs(),
            (float)m_colorGrade->greenLows(), (float)m_colorGrade->greenMids(),
            (float)m_colorGrade->greenHighs(),
            (float)m_colorGrade->blueLows(), (float)m_colorGrade->blueMids(),
            (float)m_colorGrade->blueHighs());
    }

    if (m_calibration) {
        material->setCalibration(
            (float)m_calibration->frontCenterX(), (float)m_calibration->frontCenterY(),
            (float)m_calibration->frontRadius(), (float)m_calibration->frontK1(), (float)m_calibration->frontK2(),
            (float)m_calibration->frontRotation(),
            (float)m_calibration->rearCenterX(), (float)m_calibration->rearCenterY(),
            (float)m_calibration->rearRadius(), (float)m_calibration->rearK1(), (float)m_calibration->rearK2(),
            (float)m_calibration->rearRotation(),
            (float)m_calibration->blendStart()
        );
        material->setHFlip(m_calibration->frontHFlip(), m_calibration->rearHFlip());
    }

    node->markDirty(QSGNode::DirtyGeometry | QSGNode::DirtyMaterial);
    return node;
}
