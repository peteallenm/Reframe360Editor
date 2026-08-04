#include "calibration.h"

#include <QFile>
#include <QDir>
#include <QStandardPaths>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>

CalibrationProfile::CalibrationProfile(QObject *parent)
    : QObject(parent)
    , m_frontCenterX(0.5), m_frontCenterY(0.5)
    , m_frontRadius(0.5), m_frontK1(0.0), m_frontK2(0.0)
    , m_frontRotation(0.0), m_frontHFlip(false)
    , m_rearCenterX(0.5), m_rearCenterY(0.5)
    , m_rearRadius(0.5), m_rearK1(0.0), m_rearK2(0.0)
    , m_rearRotation(180.0), m_rearHFlip(false)
    , m_blendStart(0.9)
{
}

void CalibrationProfile::setFrontCenterX(double v) { if (qFuzzyCompare(m_frontCenterX, v)) return; m_frontCenterX = v; emit frontCenterXChanged(); }
void CalibrationProfile::setFrontCenterY(double v) { if (qFuzzyCompare(m_frontCenterY, v)) return; m_frontCenterY = v; emit frontCenterYChanged(); }
void CalibrationProfile::setFrontRadius(double v) { if (qFuzzyCompare(m_frontRadius, v)) return; m_frontRadius = v; emit frontRadiusChanged(); }
void CalibrationProfile::setFrontK1(double v) { if (qFuzzyCompare(m_frontK1, v)) return; m_frontK1 = v; emit frontK1Changed(); }
void CalibrationProfile::setFrontK2(double v) { if (qFuzzyCompare(m_frontK2, v)) return; m_frontK2 = v; emit frontK2Changed(); }
void CalibrationProfile::setFrontRotation(double v) { if (qFuzzyCompare(m_frontRotation, v)) return; m_frontRotation = v; emit frontRotationChanged(); }
void CalibrationProfile::setFrontHFlip(bool v) { if (m_frontHFlip == v) return; m_frontHFlip = v; emit frontHFlipChanged(); }
void CalibrationProfile::setRearCenterX(double v) { if (qFuzzyCompare(m_rearCenterX, v)) return; m_rearCenterX = v; emit rearCenterXChanged(); }
void CalibrationProfile::setRearCenterY(double v) { if (qFuzzyCompare(m_rearCenterY, v)) return; m_rearCenterY = v; emit rearCenterYChanged(); }
void CalibrationProfile::setRearRadius(double v) { if (qFuzzyCompare(m_rearRadius, v)) return; m_rearRadius = v; emit rearRadiusChanged(); }
void CalibrationProfile::setRearK1(double v) { if (qFuzzyCompare(m_rearK1, v)) return; m_rearK1 = v; emit rearK1Changed(); }
void CalibrationProfile::setRearK2(double v) { if (qFuzzyCompare(m_rearK2, v)) return; m_rearK2 = v; emit rearK2Changed(); }
void CalibrationProfile::setRearRotation(double v) { if (qFuzzyCompare(m_rearRotation, v)) return; m_rearRotation = v; emit rearRotationChanged(); }
void CalibrationProfile::setRearHFlip(bool v) { if (m_rearHFlip == v) return; m_rearHFlip = v; emit rearHFlipChanged(); }
void CalibrationProfile::setBlendStart(double v) { if (qFuzzyCompare(m_blendStart, v)) return; m_blendStart = v; emit blendStartChanged(); }

static CalibrationPreset makeDefaultPreset()
{
    CalibrationPreset p;
    p.name = "YI 360 Default";
    p.frontCenterX = 0.5;
    p.frontCenterY = 0.5;
    p.frontRadius = 0.5;
    p.frontK1 = 0.0;
    p.frontK2 = 0.0;
    p.frontRotation = 0.0;
    p.frontHFlip = false;
    p.rearCenterX = 0.5;
    p.rearCenterY = 0.5;
    p.rearRadius = 0.5;
    p.rearK1 = 0.0;
    p.rearK2 = 0.0;
    p.rearRotation = 180.0;
    p.rearHFlip = false;
    p.blendStart = 0.9;
    p.isDefault = true;
    return p;
}

static QString presetsFilePath()
{
    QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty())
        dir = QDir::home().filePath(".config/360Render");
    QDir().mkpath(dir);
    return dir + QStringLiteral("/calibration_presets.json");
}

CalibrationPresetModel::CalibrationPresetModel(QObject *parent)
    : QAbstractListModel(parent)
{
    m_presets.append(makeDefaultPreset());
    loadFromFile();
}

int CalibrationPresetModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return m_presets.size();
}

QVariant CalibrationPresetModel::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() >= m_presets.size())
        return QVariant();

    const auto &p = m_presets[index.row()];
    switch (role) {
    case NameRole: return p.name;
    case FrontCenterXRole: return p.frontCenterX;
    case FrontCenterYRole: return p.frontCenterY;
    case FrontRadiusRole: return p.frontRadius;
    case FrontK1Role: return p.frontK1;
    case FrontK2Role: return p.frontK2;
    case FrontRotationRole: return p.frontRotation;
    case FrontHFlipRole: return p.frontHFlip;
    case RearCenterXRole: return p.rearCenterX;
    case RearCenterYRole: return p.rearCenterY;
    case RearRadiusRole: return p.rearRadius;
    case RearK1Role: return p.rearK1;
    case RearK2Role: return p.rearK2;
    case RearRotationRole: return p.rearRotation;
    case RearHFlipRole: return p.rearHFlip;
    case BlendStartRole: return p.blendStart;
    case IsDefaultRole: return p.isDefault;
    }
    return QVariant();
}

QHash<int, QByteArray> CalibrationPresetModel::roleNames() const
{
    return {
        {NameRole, "name"},
        {FrontCenterXRole, "frontCenterX"},
        {FrontCenterYRole, "frontCenterY"},
        {FrontRadiusRole, "frontRadius"},
        {FrontK1Role, "frontK1"},
        {FrontK2Role, "frontK2"},
        {FrontRotationRole, "frontRotation"},
        {FrontHFlipRole, "frontHFlip"},
        {RearCenterXRole, "rearCenterX"},
        {RearCenterYRole, "rearCenterY"},
        {RearRadiusRole, "rearRadius"},
        {RearK1Role, "rearK1"},
        {RearK2Role, "rearK2"},
        {RearRotationRole, "rearRotation"},
        {RearHFlipRole, "rearHFlip"},
        {BlendStartRole, "blendStart"},
        {IsDefaultRole, "isDefault"},
    };
}

void CalibrationPresetModel::loadPreset(int index, CalibrationProfile *profile)
{
    if (index < 0 || index >= m_presets.size() || !profile) return;
    const auto &p = m_presets[index];
    profile->setFrontCenterX(p.frontCenterX);
    profile->setFrontCenterY(p.frontCenterY);
    profile->setFrontRadius(p.frontRadius);
    profile->setFrontK1(p.frontK1);
    profile->setFrontK2(p.frontK2);
    profile->setFrontRotation(p.frontRotation);
    profile->setFrontHFlip(p.frontHFlip);
    profile->setRearCenterX(p.rearCenterX);
    profile->setRearCenterY(p.rearCenterY);
    profile->setRearRadius(p.rearRadius);
    profile->setRearK1(p.rearK1);
    profile->setRearK2(p.rearK2);
    profile->setRearRotation(p.rearRotation);
    profile->setRearHFlip(p.rearHFlip);
    profile->setBlendStart(p.blendStart);
}

void CalibrationPresetModel::savePreset(const QString &name, CalibrationProfile *profile)
{
    if (!profile || name.trimmed().isEmpty())
        return;

    CalibrationPreset p;
    p.name = name.trimmed();
    p.frontCenterX = profile->frontCenterX();
    p.frontCenterY = profile->frontCenterY();
    p.frontRadius = profile->frontRadius();
    p.frontK1 = profile->frontK1();
    p.frontK2 = profile->frontK2();
    p.frontRotation = profile->frontRotation();
    p.frontHFlip = profile->frontHFlip();
    p.rearCenterX = profile->rearCenterX();
    p.rearCenterY = profile->rearCenterY();
    p.rearRadius = profile->rearRadius();
    p.rearK1 = profile->rearK1();
    p.rearK2 = profile->rearK2();
    p.rearRotation = profile->rearRotation();
    p.rearHFlip = profile->rearHFlip();
    p.blendStart = profile->blendStart();

    int idx = indexByName(p.name);
    if (idx >= 0) {
        p.isDefault = m_presets[idx].isDefault;
        m_presets[idx] = p;
    } else {
        p.isDefault = false;
        m_presets.append(p);
    }

    persist();
    beginResetModel();
    endResetModel();
}

void CalibrationPresetModel::setDefaultPreset(int index)
{
    if (index < 0 || index >= m_presets.size())
        return;

    for (int i = 0; i < m_presets.size(); ++i)
        m_presets[i].isDefault = (i == index);

    persist();
    beginResetModel();
    endResetModel();
}

int CalibrationPresetModel::defaultPresetIndex() const
{
    for (int i = 0; i < m_presets.size(); ++i) {
        if (m_presets[i].isDefault)
            return i;
    }
    return -1;
}

QString CalibrationPresetModel::defaultPresetName() const
{
    int i = defaultPresetIndex();
    return (i >= 0) ? m_presets[i].name : QString();
}

void CalibrationPresetModel::removePreset(int index)
{
    if (index < 0 || index >= m_presets.size())
        return;

    m_presets.removeAt(index);
    persist();
    beginResetModel();
    endResetModel();
}

int CalibrationPresetModel::indexByName(const QString &name) const
{
    for (int i = 0; i < m_presets.size(); ++i) {
        if (m_presets[i].name == name)
            return i;
    }
    return -1;
}

void CalibrationPresetModel::persist() const
{
    QJsonArray arr;
    for (const auto &p : m_presets) {
        QJsonObject front{
            {"centerX", p.frontCenterX}, {"centerY", p.frontCenterY},
            {"radius", p.frontRadius}, {"k1", p.frontK1}, {"k2", p.frontK2},
            {"rotation", p.frontRotation}, {"hflip", p.frontHFlip},
        };
        QJsonObject rear{
            {"centerX", p.rearCenterX}, {"centerY", p.rearCenterY},
            {"radius", p.rearRadius}, {"k1", p.rearK1}, {"k2", p.rearK2},
            {"rotation", p.rearRotation}, {"hflip", p.rearHFlip},
        };
        QJsonObject obj{
            {"name", p.name},
            {"front", front},
            {"rear", rear},
            {"blendStart", p.blendStart},
            {"default", p.isDefault},
        };
        arr.append(obj);
    }
    QJsonObject root{{"presets", arr}};

    QFile f(presetsFilePath());
    if (f.open(QIODevice::WriteOnly))
        f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void CalibrationPresetModel::loadFromFile()
{
    QFile f(presetsFilePath());
    if (!f.open(QIODevice::ReadOnly))
        return;

    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
        return;

    const QJsonArray arr = doc.object().value("presets").toArray();
    for (const QJsonValue &v : arr) {
        const QJsonObject obj = v.toObject();
        const QJsonObject front = obj.value("front").toObject();
        const QJsonObject rear = obj.value("rear").toObject();

        CalibrationPreset p;
        p.name = obj.value("name").toString();
        if (p.name.isEmpty())
            continue;
        p.frontCenterX = front.value("centerX").toDouble(0.5);
        p.frontCenterY = front.value("centerY").toDouble(0.5);
        p.frontRadius = front.value("radius").toDouble(0.5);
        p.frontK1 = front.value("k1").toDouble(0.0);
        p.frontK2 = front.value("k2").toDouble(0.0);
        p.frontRotation = front.value("rotation").toDouble(0.0);
        p.frontHFlip = front.value("hflip").toBool(false);
        p.rearCenterX = rear.value("centerX").toDouble(0.5);
        p.rearCenterY = rear.value("centerY").toDouble(0.5);
        p.rearRadius = rear.value("radius").toDouble(0.5);
        p.rearK1 = rear.value("k1").toDouble(0.0);
        p.rearK2 = rear.value("k2").toDouble(0.0);
        p.rearRotation = rear.value("rotation").toDouble(180.0);
        p.rearHFlip = rear.value("hflip").toBool(false);
        p.blendStart = obj.value("blendStart").toDouble(0.9);
        p.isDefault = obj.value("default").toBool(false);

        int idx = indexByName(p.name);
        if (idx >= 0)
            m_presets[idx] = p;
        else
            m_presets.append(p);
    }
}
