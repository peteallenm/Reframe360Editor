#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QObject>
#include <QAbstractListModel>
#include <QVector>

class CalibrationProfile : public QObject
{
    Q_OBJECT
    Q_PROPERTY(double frontCenterX READ frontCenterX WRITE setFrontCenterX NOTIFY frontCenterXChanged)
    Q_PROPERTY(double frontCenterY READ frontCenterY WRITE setFrontCenterY NOTIFY frontCenterYChanged)
    Q_PROPERTY(double frontRadius READ frontRadius WRITE setFrontRadius NOTIFY frontRadiusChanged)
    Q_PROPERTY(double frontK1 READ frontK1 WRITE setFrontK1 NOTIFY frontK1Changed)
    Q_PROPERTY(double frontK2 READ frontK2 WRITE setFrontK2 NOTIFY frontK2Changed)
    Q_PROPERTY(double frontRotation READ frontRotation WRITE setFrontRotation NOTIFY frontRotationChanged)
    Q_PROPERTY(bool frontHFlip READ frontHFlip WRITE setFrontHFlip NOTIFY frontHFlipChanged)
    Q_PROPERTY(double rearCenterX READ rearCenterX WRITE setRearCenterX NOTIFY rearCenterXChanged)
    Q_PROPERTY(double rearCenterY READ rearCenterY WRITE setRearCenterY NOTIFY rearCenterYChanged)
    Q_PROPERTY(double rearRadius READ rearRadius WRITE setRearRadius NOTIFY rearRadiusChanged)
    Q_PROPERTY(double rearK1 READ rearK1 WRITE setRearK1 NOTIFY rearK1Changed)
    Q_PROPERTY(double rearK2 READ rearK2 WRITE setRearK2 NOTIFY rearK2Changed)
    Q_PROPERTY(double rearRotation READ rearRotation WRITE setRearRotation NOTIFY rearRotationChanged)
    Q_PROPERTY(bool rearHFlip READ rearHFlip WRITE setRearHFlip NOTIFY rearHFlipChanged)
    Q_PROPERTY(double blendStart READ blendStart WRITE setBlendStart NOTIFY blendStartChanged)

public:
    explicit CalibrationProfile(QObject *parent = nullptr);

    double frontCenterX() const { return m_frontCenterX; }
    void setFrontCenterX(double v);
    double frontCenterY() const { return m_frontCenterY; }
    void setFrontCenterY(double v);
    double frontRadius() const { return m_frontRadius; }
    void setFrontRadius(double v);
    double frontK1() const { return m_frontK1; }
    void setFrontK1(double v);
    double frontK2() const { return m_frontK2; }
    void setFrontK2(double v);
    double frontRotation() const { return m_frontRotation; }
    void setFrontRotation(double v);
    bool frontHFlip() const { return m_frontHFlip; }
    void setFrontHFlip(bool v);

    double rearCenterX() const { return m_rearCenterX; }
    void setRearCenterX(double v);
    double rearCenterY() const { return m_rearCenterY; }
    void setRearCenterY(double v);
    double rearRadius() const { return m_rearRadius; }
    void setRearRadius(double v);
    double rearK1() const { return m_rearK1; }
    void setRearK1(double v);
    double rearK2() const { return m_rearK2; }
    void setRearK2(double v);
    double rearRotation() const { return m_rearRotation; }
    void setRearRotation(double v);
    bool rearHFlip() const { return m_rearHFlip; }
    void setRearHFlip(bool v);

    double blendStart() const { return m_blendStart; }
    void setBlendStart(double v);

signals:
    void frontCenterXChanged();
    void frontCenterYChanged();
    void frontRadiusChanged();
    void frontK1Changed();
    void frontK2Changed();
    void frontRotationChanged();
    void frontHFlipChanged();
    void rearCenterXChanged();
    void rearCenterYChanged();
    void rearRadiusChanged();
    void rearK1Changed();
    void rearK2Changed();
    void rearRotationChanged();
    void rearHFlipChanged();
    void blendStartChanged();

private:
    double m_frontCenterX, m_frontCenterY;
    double m_frontRadius, m_frontK1, m_frontK2;
    double m_frontRotation;
    bool m_frontHFlip;
    double m_rearCenterX, m_rearCenterY;
    double m_rearRadius, m_rearK1, m_rearK2;
    double m_rearRotation;
    bool m_rearHFlip;
    double m_blendStart;
};

struct CalibrationPreset {
    QString name;
    double frontCenterX, frontCenterY, frontRadius, frontK1, frontK2;
    double frontRotation;
    bool frontHFlip;
    double rearCenterX, rearCenterY, rearRadius, rearK1, rearK2;
    double rearRotation;
    bool rearHFlip;
    double blendStart;
    bool isDefault;
};

class CalibrationPresetModel : public QAbstractListModel
{
    Q_OBJECT
public:
    enum Roles {
        NameRole = Qt::UserRole + 1,
        FrontCenterXRole, FrontCenterYRole, FrontRadiusRole, FrontK1Role, FrontK2Role,
        FrontRotationRole, FrontHFlipRole,
        RearCenterXRole, RearCenterYRole, RearRadiusRole, RearK1Role, RearK2Role,
        RearRotationRole, RearHFlipRole,
        BlendStartRole,
        IsDefaultRole
    };

    explicit CalibrationPresetModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    Q_INVOKABLE void loadPreset(int index, CalibrationProfile *profile);
    Q_INVOKABLE void savePreset(const QString &name, CalibrationProfile *profile);
    Q_INVOKABLE void removePreset(int index);
    Q_INVOKABLE void setDefaultPreset(int index);
    Q_INVOKABLE int defaultPresetIndex() const;
    Q_INVOKABLE QString defaultPresetName() const;

private:
    void persist() const;
    void loadFromFile();
    // Presets baked into the binary, applied before the user's own file so a
    // fresh install (especially Android, which cannot import the desktop's
    // config) starts with the measured YI 360 profiles.
    void loadFactoryDefaults();
    void loadFromPath(const QString &path);
    int indexByName(const QString &name) const;

    QVector<CalibrationPreset> m_presets;
};

#endif // CALIBRATION_H
