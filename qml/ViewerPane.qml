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

Item {
    id: viewerPane

    // Emitted by the empty-state button; main.qml routes it to the platform's
    // open flow (folder on Android, file dialog on desktop).
    signal openRequested()

    Rectangle {
        anchors.fill: parent
        color: "#1a1a1a"

        LensViewer {
            id: lensViewer
            anchors.fill: parent
            decoder: app.videoDecoder
            yaw: app.yaw
            pitch: app.pitch
            roll: app.roll
            fov: app.fov
            activeLens: app.activeLens
            projection: app.projection
            calibration: app.currentCalibration
            colorGrade: app.colorGrade
            // app.imuOrientation already returns the constant 180 deg video un-flip
            // when stabilization is off; overriding it with identity here put the
            // render back upside down.
            imuOrientation: app.imuOrientation
            // named "controller": a property called "app" would shadow the context
            // object of the same name for every other binding in this item.
            controller: app
            flowStitch: app.flowStitch
            flowStrength: app.flowStrength
            flowImage: app.flowImage
            flowEncode: app.flowEncode
            seamStitch: app.seamStitch
            seamStrength: app.seamStrength
            seamImage: app.seamImage
            curveLut: app.colorGrade.curveLut
            curvesActive: app.colorGrade.curvesActive
        }

        MouseArea {
            id: dragArea
            anchors.fill: parent
            enabled: !!app.videoPath
            cursorShape: app.trackArmed ? Qt.CrossCursor
                                        : (pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor)
            acceptedButtons: Qt.LeftButton

            property point dragOrigin
            // A tap has to be told apart from a drag: this one MouseArea owns
            // every press, so "point at the subject" and "look around" are the
            // same gesture until the finger moves.
            property point pressPoint
            property bool moved: false
            readonly property int tapSlop: 8

            onPressed: (mouse) => {
                dragOrigin = Qt.point(mouse.x, mouse.y)
                pressPoint = Qt.point(mouse.x, mouse.y)
                moved = false
            }
            onPositionChanged: (mouse) => {
                if (!pressed) return
                if (Math.abs(mouse.x - pressPoint.x) > tapSlop
                    || Math.abs(mouse.y - pressPoint.y) > tapSlop)
                    moved = true
                var dx = mouse.x - dragOrigin.x
                var dy = mouse.y - dragOrigin.y
                if (dx === 0 && dy === 0) return
                // While armed the drag does not look around -- the gesture is
                // reserved for the pick, so a small wobble cannot re-aim the
                // view under the finger.
                if (!app.trackArmed) {
                    var sens = app.fov / Math.max(height, 1)
                    // Grab semantics: the picture follows the hand. Dragging DOWN
                    // must move the scene down, i.e. tilt the view up. The vertical
                    // sign was inverted (reported by the user); horizontal was right.
                    app.dragLook(dx * sens, dy * sens);
                }
                dragOrigin = Qt.point(mouse.x, mouse.y)
            }
            onReleased: (mouse) => {
                if (!app.trackArmed || moved) return
                app.startTrackAt(mouse.x / Math.max(width, 1) * 2 - 1,
                                 mouse.y / Math.max(height, 1) * 2 - 1,
                                 width / Math.max(height, 1))
            }
        }

        // Where the tracked subject is right now. Without it there is no way to
        // tell a track that is holding from one that has quietly drifted.
        Item {
            id: trackMarker
            visible: app.trackCount > 0 && ndc.length === 2
            property var ndc: []
            width: 34; height: 34
            x: visible ? (ndc[0] * 0.5 + 0.5) * parent.width - width / 2 : 0
            y: visible ? (ndc[1] * 0.5 + 0.5) * parent.height - height / 2 : 0

            function refresh() {
                ndc = app.trackMarkerNdc(lensViewer.width / Math.max(lensViewer.height, 1))
            }
            Connections {
                target: app
                function onCurrentTimeChanged() { trackMarker.refresh() }
                function onTracksChanged() { trackMarker.refresh() }
                function onYawChanged() { trackMarker.refresh() }
            }

            Rectangle {
                anchors.fill: parent
                color: "transparent"
                border.color: "#66d9ff"
                border.width: 2
                radius: width / 2
                opacity: 0.85
            }
            Rectangle {
                anchors.centerIn: parent
                width: 3; height: 3; radius: 1.5
                color: "#66d9ff"
            }
        }

        // Armed: say what to do, because a crosshair alone does not explain it.
        Rectangle {
            visible: app.trackArmed
            anchors.horizontalCenter: parent.horizontalCenter
            anchors.top: parent.top
            anchors.topMargin: 16
            width: hint.implicitWidth + 24
            height: hint.implicitHeight + 14
            radius: 6
            color: "#cc1e88c7"
            Label {
                id: hint
                anchors.centerIn: parent
                text: qsTr("Tap what you want to follow")
                color: "white"
                font.pixelSize: 13
            }
        }

        // Empty state: a first launch used to be a black screen with a
        // passive sentence, and on Android nothing hinted that a FOLDER (not
        // a file) is what gets opened. The action itself is the hint.
        Column {
            anchors.centerIn: parent
            spacing: 16
            visible: !app.videoPath && !app.loadError

            Label {
                anchors.horizontalCenter: parent.horizontalCenter
                text: qsTr("Open a 360 video to begin")
                font.pixelSize: 24
                color: "#888888"
            }
            Label {
                anchors.horizontalCenter: parent.horizontalCenter
                visible: app.folder.available
                width: Math.min(viewerPane.width * 0.8, 460)
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.Wrap
                text: app.folder.hasFolder
                      ? qsTr("Your folder \u201c%1\u201d is connected. The sensor data and preview files next to each clip are found automatically.").arg(app.folder.folderName)
                      : qsTr("Pick the folder your clips are in (the camera's DCIM folder, or wherever you copied them). The sensor data and preview files next to each clip are found automatically.")
                font.pixelSize: 13
                color: "#666666"
            }
            Button {
                anchors.horizontalCenter: parent.horizontalCenter
                highlighted: true
                text: app.folder.available
                      ? (app.folder.hasFolder ? qsTr("Choose a clip…") : qsTr("Open folder…"))
                      : qsTr("Open video…")
                onClicked: viewerPane.openRequested()
            }
        }

        // A failed open used to be invisible: the error signal went nowhere and
        // the previous clip's last frame stayed on screen, which reads as a hang.
        Label {
            anchors.centerIn: parent
            width: parent.width * 0.8
            text: app.loadError
            font.pixelSize: 16
            color: "#ff8080"
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.Wrap
            visible: app.loadError.length > 0
            padding: 12
            background: Rectangle { color: "#cc000000"; radius: 6 }
        }
    }
}
