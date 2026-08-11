#ifndef LENSVIEWER_H
#define LENSVIEWER_H

#include <QQuickItem>
#include <QQuaternion>
#include <QQmlEngine>
#include <QSGTexture>
#include <QImage>
#include "videodecoder.h"
#include "calibration.h"
#include "colorgrade.h"

class ViewerMaterial;

class LensViewer : public QQuickItem
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(VideoDecoder* decoder READ decoder WRITE setDecoder NOTIFY decoderChanged)
    Q_PROPERTY(double yaw READ yaw WRITE setYaw NOTIFY yawChanged)
    Q_PROPERTY(double pitch READ pitch WRITE setPitch NOTIFY pitchChanged)
    Q_PROPERTY(double roll READ roll WRITE setRoll NOTIFY rollChanged)
    Q_PROPERTY(double fov READ fov WRITE setFov NOTIFY fovChanged)
    Q_PROPERTY(int activeLens READ activeLens WRITE setActiveLens NOTIFY activeLensChanged)
    Q_PROPERTY(int projection READ projection WRITE setProjection NOTIFY projectionChanged)
    Q_PROPERTY(bool frontHFlip READ frontHFlip WRITE setFrontHFlip NOTIFY frontHFlipChanged)
    Q_PROPERTY(bool rearHFlip READ rearHFlip WRITE setRearHFlip NOTIFY rearHFlipChanged)
    Q_PROPERTY(CalibrationProfile* calibration READ calibration WRITE setCalibration NOTIFY calibrationChanged)
    Q_PROPERTY(ColorGrade* colorGrade READ colorGrade WRITE setColorGrade NOTIFY colorGradeChanged)
    Q_PROPERTY(QQuaternion imuOrientation READ imuOrientation WRITE setImuOrientation NOTIFY imuOrientationChanged)
    Q_PROPERTY(bool flowStitch READ flowStitch WRITE setFlowStitch NOTIFY flowStitchChanged)
    Q_PROPERTY(double flowStrength READ flowStrength WRITE setFlowStrength NOTIFY flowStrengthChanged)
    Q_PROPERTY(QImage flowImage READ flowImage WRITE setFlowImage NOTIFY flowImageChanged)
    Q_PROPERTY(double flowEncode READ flowEncode WRITE setFlowEncode NOTIFY flowEncodeChanged)

public:
    explicit LensViewer(QQuickItem *parent = nullptr);
    ~LensViewer();

    VideoDecoder* decoder() const { return m_decoder; }
    void setDecoder(VideoDecoder *decoder);

    double yaw() const { return m_yaw; }
    void setYaw(double yaw);
    double pitch() const { return m_pitch; }
    void setPitch(double pitch);
    double roll() const { return m_roll; }
    void setRoll(double roll);
    double fov() const { return m_fov; }
    void setFov(double fov);
    int activeLens() const { return m_activeLens; }
    void setActiveLens(int lens);
    int projection() const { return m_projection; }
    void setProjection(int projection);
    bool frontHFlip() const { return m_frontHFlip; }
    void setFrontHFlip(bool v);
    bool rearHFlip() const { return m_rearHFlip; }
    void setRearHFlip(bool v);
    CalibrationProfile* calibration() const { return m_calibration; }
    void setCalibration(CalibrationProfile *cal);
    ColorGrade* colorGrade() const { return m_colorGrade; }
    void setColorGrade(ColorGrade *grade);
    QQuaternion imuOrientation() const { return m_imuOrientation; }
    void setImuOrientation(const QQuaternion &q);
    bool flowStitch() const { return m_flowStitch; }
    void setFlowStitch(bool v);
    double flowStrength() const { return m_flowStrength; }
    void setFlowStrength(double v);
    QImage flowImage() const { return m_flowImage; }
    void setFlowImage(const QImage &img);
    double flowEncode() const { return m_flowEncode; }
    void setFlowEncode(double v);

protected:
    QSGNode *updatePaintNode(QSGNode *oldNode, UpdatePaintNodeData *data) override;

signals:
    void decoderChanged();
    void yawChanged();
    void pitchChanged();
    void rollChanged();
    void fovChanged();
    void activeLensChanged();
    void projectionChanged();
    void frontHFlipChanged();
    void rearHFlipChanged();
    void calibrationChanged();
    void colorGradeChanged();
    void imuOrientationChanged();
    void flowStitchChanged();
    void flowStrengthChanged();
    void flowImageChanged();
    void flowEncodeChanged();

private:
    VideoDecoder *m_decoder;
    double m_yaw, m_pitch, m_roll, m_fov;
    int m_activeLens;
    int m_projection;
    bool m_frontHFlip, m_rearHFlip;
    CalibrationProfile *m_calibration;
    ColorGrade *m_colorGrade;
    QQuaternion m_imuOrientation;

    QSGTexture *m_yTexture;
    QSGTexture *m_uTexture;
    QSGTexture *m_vTexture;
    int m_lastFrameTimestamp;

    bool m_flowStitch;
    double m_flowStrength;
    double m_flowEncode;
    QImage m_flowImage;
    QSGTexture *m_flowTexture;
    qint64 m_flowTextureKey;
};

#endif // LENSVIEWER_H
