// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef IMUPARSER_H
#define IMUPARSER_H

#include <QObject>
#include <QString>
#include <QVector>
#include <QQuaternion>
#include <cstdint>

struct ImuSample {
    double timestamp;
    QVector3D gyro;   // deg/s
    QVector3D accel;  // g
    uint32_t counter; // hardware counter from t2 record (~1 MHz, ~2500 counts/sample)
};

class ImuParser : public QObject
{
    Q_OBJECT
public:
    explicit ImuParser(QObject *parent = nullptr);

    bool loadFile(const QString &path);
    // Reproduction hook (tests only): use these LSB/(deg/s) scales instead of
    // the built-in ones on the next loadFile().
    void setGyroScaleOverride(double x, double y, double z)
    { m_scaleOverride = true; m_ovX = x; m_ovY = y; m_ovZ = z; }

    QQuaternion initialQuaternion() const { return m_initialQuaternion; }
    double imuSampleRate() const { return m_imuSampleRate; }
    QVector<ImuSample> samples() const { return m_rawData; }
    bool isLoaded() const { return m_loaded; }
    // Duration spanned by the parsed sample stream (seconds).
    double duration() const { return m_imuSampleRate > 0.0 ? m_rawData.size() / m_imuSampleRate : 0.0; }

    // Per-axis gyro scales in LSB/(deg/s), calibrated so a 360° rotation on
    // each axis integrates to exactly ±360°: X=roll 32.18, Y=pitch 33.51,
    // Z=yaw 33.64 (vs the nominal header 32.8).
    double gyroScaleX() const { return m_gyroScaleX; }
    double gyroScaleY() const { return m_gyroScaleY; }
    double gyroScaleZ() const { return m_gyroScaleZ; }

    // Per-axis scale refinement from the visual-rotation calibration (item 1).
    // Normally the hardcoded per-axis scales make a 360° integration read
    // exactly 360°; a visually-derived correction can refine them per-clip by
    // scaling the raw LSB/(deg/s) divisor. The integrator supplements these
    // with the gyro calibration matrix M anyway, so this is a convenience for
    // feeding corrected scales back into the parser.
    void setGyroScaleX(double v) { m_gyroScaleX = v; }
    void setGyroScaleY(double v) { m_gyroScaleY = v; }
    void setGyroScaleZ(double v) { m_gyroScaleZ = v; }

private:
    QQuaternion m_initialQuaternion;
    double m_imuSampleRate;
    bool m_scaleOverride = false;
    double m_ovX = 0, m_ovY = 0, m_ovZ = 0;
    double m_gyroScaleX;
    double m_gyroScaleY;
    double m_gyroScaleZ;
    QVector<ImuSample> m_rawData;
    bool m_loaded = false;
};

#endif // IMUPARSER_H
