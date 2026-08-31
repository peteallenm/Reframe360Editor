// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Render360 1.0

Dialog {
    id: calibrationDialog
    title: qsTr("Lens Calibration") + " — " + lensName
    modal: false
    dim: false
    standardButtons: Dialog.Ok | Dialog.Cancel

    // Size against the window rather than fixed 780x640. Those are LOGICAL
    // pixels: on a phone (the Edge 40 reports a device pixel ratio of ~2.6) a
    // 640-high dialog does not fit a 1080-high screen, and the Ok/Cancel
    // footer ended up off the bottom edge with no way to reach it. Bounding to
    // the overlay keeps the footer on screen; the StackLayout inside already
    // has Layout.fillHeight, so the content shrinks to match.
    parent: Overlay.overlay
    anchors.centerIn: parent
    readonly property real availW: parent ? parent.width : 780
    readonly property real availH: parent ? parent.height : 640
    width: Math.min(780, availW - 24)
    height: Math.min(640, availH - 24)

    property int lens: 0
    property string stillSource: ""

    readonly property double curCX: lens === 0 ? app.currentCalibration.frontCenterX : app.currentCalibration.rearCenterX
    readonly property double curCY: lens === 0 ? app.currentCalibration.frontCenterY : app.currentCalibration.rearCenterY
    readonly property double curR: lens === 0 ? app.currentCalibration.frontRadius : app.currentCalibration.rearRadius
    readonly property double curRot: lens === 0 ? app.currentCalibration.frontRotation : app.currentCalibration.rearRotation
    readonly property string lensName: lens === 0 ? qsTr("Front Lens") : qsTr("Rear Lens")

    function setCX(v) { if (lens === 0) app.currentCalibration.frontCenterX = v; else app.currentCalibration.rearCenterX = v; }
    function setCY(v) { if (lens === 0) app.currentCalibration.frontCenterY = v; else app.currentCalibration.rearCenterY = v; }
    function setR(v) { if (lens === 0) app.currentCalibration.frontRadius = v; else app.currentCalibration.rearRadius = v; }
    function setRot(v) { if (lens === 0) app.currentCalibration.frontRotation = v; else app.currentCalibration.rearRotation = v; }

    function refreshStill() {
        stillSource = "file://" + app.grabStill(lens)
    }

    function resetLensDefaults() {
        if (lens === 0) {
            app.currentCalibration.frontCenterX = 0.5
            app.currentCalibration.frontCenterY = 0.5
            app.currentCalibration.frontRadius = 0.5
            app.currentCalibration.frontK1 = 0.0
            app.currentCalibration.frontK2 = 0.0
            app.currentCalibration.frontRotation = 0.0
            app.currentCalibration.frontHFlip = false
        } else {
            app.currentCalibration.rearCenterX = 0.5
            app.currentCalibration.rearCenterY = 0.5
            app.currentCalibration.rearRadius = 0.5
            app.currentCalibration.rearK1 = 0.0
            app.currentCalibration.rearK2 = 0.0
            app.currentCalibration.rearRotation = 180.0
            app.currentCalibration.rearHFlip = false
        }
    }

    onOpened: refreshStill()
    onLensChanged: refreshStill()

    contentItem: ColumnLayout {
        spacing: 8
        anchors.fill: parent

        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            Label { text: qsTr("Editing lens:"); font.bold: true }

            ComboBox {
                id: lensCombo
                Layout.preferredWidth: 200
                font.pixelSize: 14
                model: [qsTr("Front Lens"), qsTr("Rear Lens")]
                currentIndex: calibrationDialog.lens
                onCurrentIndexChanged: calibrationDialog.lens = currentIndex
            }

            Rectangle {
                Layout.preferredHeight: 28
                Layout.preferredWidth: 90
                radius: 14
                color: calibrationDialog.lens === 0 ? "#4a6fb5" : "#b57a3a"

                Label {
                    anchors.centerIn: parent
                    text: calibrationDialog.lensName
                    color: "white"
                    font.bold: true
                    font.pixelSize: 12
                }
            }

            Item { Layout.fillWidth: true }

            Button {
                text: qsTr("Refresh Still")
                onClicked: refreshStill()
            }
        }

        TabBar {
            id: tabBar
            Layout.fillWidth: true
            TabButton { text: qsTr("Fisheye Editor") }
            TabButton { text: qsTr("Equirectangular Output") }
        }

        StackLayout {
            id: stack
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabBar.currentIndex

            // ---------------- Fisheye editor tab ----------------
            RowLayout {
                spacing: 16

                Item {
                    id: calibArea
                    Layout.preferredWidth: 460
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignTop

                    property real size: Math.min(width, height)

                    Image {
                        id: stillImage
                        anchors.centerIn: parent
                        width: calibArea.size
                        height: calibArea.size
                        source: calibrationDialog.stillSource
                        fillMode: Image.PreserveAspectFit
                        cache: false
                    }

                    Canvas {
                        id: overlay
                        anchors.fill: stillImage
                        antialiasing: true

                        function drawHandle(ctx, x, y, color) {
                            ctx.beginPath()
                            ctx.arc(x, y, 7, 0, Math.PI * 2)
                            ctx.fillStyle = color
                            ctx.fill()
                            ctx.strokeStyle = "white"
                            ctx.lineWidth = 1.5
                            ctx.stroke()
                        }

                        onPaint: {
                            var ctx = getContext("2d")
                            var s = width
                            ctx.clearRect(0, 0, s, s)

                            var cx = calibrationDialog.curCX * s
                            var cy = calibrationDialog.curCY * s
                            var r = calibrationDialog.curR * s
                            var rot = calibrationDialog.curRot * Math.PI / 180

                            ctx.beginPath()
                            ctx.arc(cx, cy, Math.max(r, 2), 0, Math.PI * 2)
                            ctx.strokeStyle = "#4fc3f7"
                            ctx.lineWidth = 2
                            ctx.stroke()

                            ctx.strokeStyle = "rgba(255,255,255,0.85)"
                            ctx.lineWidth = 1.5
                            ctx.beginPath()
                            ctx.moveTo(cx - 12, cy); ctx.lineTo(cx + 12, cy)
                            ctx.moveTo(cx, cy - 12); ctx.lineTo(cx, cy + 12)
                            ctx.stroke()

                            var hx = cx + (r + 14) * Math.cos(rot)
                            var hy = cy + (r + 14) * Math.sin(rot)
                            ctx.beginPath()
                            ctx.moveTo(cx, cy); ctx.lineTo(hx, hy)
                            ctx.strokeStyle = "#ffb74d"
                            ctx.lineWidth = 2
                            ctx.stroke()

                            drawHandle(ctx, cx + r, cy, "#ffeb3b")
                            drawHandle(ctx, hx, hy, "#4fc3f7")
                        }
                    }

                    MouseArea {
                        id: overlayMouse
                        anchors.fill: parent
                        cursorShape: pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor

                        property string mode: "center"

                        function hitTest(mx, my) {
                            var s = overlay.width
                            var cx = calibrationDialog.curCX * s
                            var cy = calibrationDialog.curCY * s
                            var r = Math.max(calibrationDialog.curR * s, 2)
                            var rot = calibrationDialog.curRot * Math.PI / 180
                            var hx = cx + (r + 14) * Math.cos(rot)
                            var hy = cy + (r + 14) * Math.sin(rot)

                            if (Math.hypot(mx - hx, my - hy) < 16) return "rotation"
                            if (Math.hypot(mx - (cx + r), my - cy) < 16) return "radius"
                            return "center"
                        }

                        onPressed: (mouse) => {
                            mode = hitTest(mouse.x, mouse.y)
                            applyPosition(mouse.x, mouse.y)
                        }
                        onPositionChanged: (mouse) => {
                            if (!pressed) return
                            applyPosition(mouse.x, mouse.y)
                        }

                        function applyPosition(mx, my) {
                            var s = overlay.width
                            var cx = calibrationDialog.curCX
                            var cy = calibrationDialog.curCY
                            if (mode === "center") {
                                calibrationDialog.setCX(Math.max(0, Math.min(1, mx / s)))
                                calibrationDialog.setCY(Math.max(0, Math.min(1, my / s)))
                            } else if (mode === "radius") {
                                var px = mx / s - cx
                                var py = my / s - cy
                                calibrationDialog.setR(Math.max(0, Math.min(1, Math.hypot(px, py))))
                            } else if (mode === "rotation") {
                                var dx = mx / s - cx
                                var dy = my / s - cy
                                calibrationDialog.setRot(Math.atan2(dy, dx) * 180 / Math.PI)
                            }
                        }
                    }

                    Label {
                        anchors.bottom: parent.bottom
                        anchors.horizontalCenter: parent.horizontalCenter
                        anchors.bottomMargin: 6
                        text: qsTr("Drag to move centre · edge handle = radius · outer handle = rotation")
                        font.pixelSize: 11
                        color: "white"
                        style: Text.Outline
                        styleColor: "black"
                    }
                }

                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 8

                    Label {
                        text: calibrationDialog.lensName
                        font.bold: true
                        font.pixelSize: 14
                    }

                    RowLayout {
                        Label { text: qsTr("Centre X:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: cXS
                            Layout.fillWidth: true
                            from: 0.4; to: 0.6; stepSize: 0.001
                            value: calibrationDialog.curCX
                            onValueChanged: if (pressed) calibrationDialog.setCX(value)
                        }
                        Label { text: cXS.value.toFixed(3); Layout.preferredWidth: 50 }
                    }

                    RowLayout {
                        Label { text: qsTr("Centre Y:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: cYS
                            Layout.fillWidth: true
                            from: 0.4; to: 0.6; stepSize: 0.001
                            value: calibrationDialog.curCY
                            onValueChanged: if (pressed) calibrationDialog.setCY(value)
                        }
                        Label { text: cYS.value.toFixed(3); Layout.preferredWidth: 50 }
                    }

                    RowLayout {
                        Label { text: qsTr("Radius:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: rS
                            Layout.fillWidth: true
                            from: 0.4; to: 0.6; stepSize: 0.001
                            value: calibrationDialog.curR
                            onValueChanged: if (pressed) calibrationDialog.setR(value)
                        }
                        Label { text: rS.value.toFixed(3); Layout.preferredWidth: 50 }
                    }

                    RowLayout {
                        Label { text: qsTr("Rotation:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: rotS
                            Layout.fillWidth: true
                            from: -180; to: 180; stepSize: 0.1
                            value: calibrationDialog.curRot
                            onValueChanged: if (pressed) calibrationDialog.setRot(value)
                        }
                        Label { text: rotS.value.toFixed(0) + "\u00B0"; Layout.preferredWidth: 50 }
                    }

                    CheckBox {
                        id: hflipCheck
                        text: qsTr("Horizontal Flip")
                        checked: lens === 0 ? app.currentCalibration.frontHFlip : app.currentCalibration.rearHFlip
                        onCheckedChanged: {
                            if (lens === 0) app.currentCalibration.frontHFlip = checked
                            else app.currentCalibration.rearHFlip = checked
                        }
                    }

                    Button {
                        text: qsTr("Reset " + calibrationDialog.lensName + " to Defaults")
                        Layout.fillWidth: true
                        onClicked: resetLensDefaults()
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

                    RowLayout {
                        Label { text: qsTr("K1:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: k1S
                            Layout.fillWidth: true
                            from: -1; to: 1; stepSize: 0.001
                            value: lens === 0 ? app.currentCalibration.frontK1 : app.currentCalibration.rearK1
                            onValueChanged: if (pressed) { if (lens === 0) app.currentCalibration.frontK1 = value; else app.currentCalibration.rearK1 = value }
                        }
                        Label { text: k1S.value.toFixed(3); Layout.preferredWidth: 50 }
                    }

                    RowLayout {
                        Label { text: qsTr("K2:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: k2S
                            Layout.fillWidth: true
                            from: -1; to: 1; stepSize: 0.001
                            value: lens === 0 ? app.currentCalibration.frontK2 : app.currentCalibration.rearK2
                            onValueChanged: if (pressed) { if (lens === 0) app.currentCalibration.frontK2 = value; else app.currentCalibration.rearK2 = value }
                        }
                        Label { text: k2S.value.toFixed(3); Layout.preferredWidth: 50 }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

                    RowLayout {
                        Label { text: qsTr("Blend Start:"); Layout.preferredWidth: 90 }
                        Slider {
                            id: blendS
                            Layout.fillWidth: true
                            from: 0; to: 1; stepSize: 0.01
                            value: app.currentCalibration.blendStart
                            onValueChanged: if (pressed) app.currentCalibration.blendStart = value
                        }
                        Label { text: blendS.value.toFixed(2); Layout.preferredWidth: 50 }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // ---------------- Equirectangular output tab ----------------
            ColumnLayout {
                spacing: 8

                Label {
                    text: qsTr("Full equirectangular (2:1) output — front lens left half, rear lens right half. Adjust centre/radius/rotation to align the seam.")
                    font.pixelSize: 12
                    wrapMode: Text.WordWrap
                    Layout.fillWidth: true
                }

                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    Rectangle {
                        id: eqFrame
                        anchors.centerIn: parent
                        width: Math.min(parent.width, parent.height * 2)
                        height: width / 2
                        color: "#1a1a1a"
                        border.color: "#444"

                        LensViewer {
                            id: eqViewer
                            anchors.fill: parent
                            decoder: app.videoDecoder
                            projection: 1
                            activeLens: 2
                            yaw: app.yaw
                            pitch: app.pitch
                            roll: app.roll
                            fov: app.fov
                            calibration: app.currentCalibration
                            colorGrade: app.colorGrade
                            // app.imuOrientation already returns the constant 180 deg video un-flip
                            // when stabilization is off; overriding it with identity here put the
                            // render back upside down.
                            imuOrientation: app.imuOrientation
                        }
                    }
                }
            }
        }

        Connections {
            target: app.currentCalibration
            function onFrontCenterXChanged() { overlay.requestPaint() }
            function onFrontCenterYChanged() { overlay.requestPaint() }
            function onFrontRadiusChanged() { overlay.requestPaint() }
            function onFrontRotationChanged() { overlay.requestPaint() }
            function onRearCenterXChanged() { overlay.requestPaint() }
            function onRearCenterYChanged() { overlay.requestPaint() }
            function onRearRadiusChanged() { overlay.requestPaint() }
            function onRearRotationChanged() { overlay.requestPaint() }
        }
    }
}
