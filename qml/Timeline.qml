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

Pane {
    id: timeline
    // Targeting SDK 36 means the app draws edge-to-edge, UNDER the system
    // bars. In landscape the navigation bar sits on one SIDE, and it was
    // covering the play button. SafeArea gives the real insets for whichever
    // rotation, so this is right both ways up rather than a fixed nudge.
    //
    // Guarded on the platform because SafeArea is QtQuick 6.9+, and the
    // desktop here builds against distro Qt 6.4: an unresolved attached
    // property makes the arithmetic NaN, which collapsed the whole footer.
    // The conditional short-circuits before SafeArea is ever looked up.
    readonly property real safeLeft: Qt.platform.os === "android" ? SafeArea.margins.left : 0
    readonly property real safeRight: Qt.platform.os === "android" ? SafeArea.margins.right : 0
    readonly property real safeBottom: Qt.platform.os === "android" ? SafeArea.margins.bottom : 0

    height: 60 + safeBottom
    padding: 8
    leftPadding: 8 + safeLeft
    rightPadding: 8 + safeRight
    bottomPadding: 8 + safeBottom

    Dialog {
        id: clearKeyframesDialog
        title: qsTr("Clear keyframes?")
        modal: true
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(380, (parent ? parent.width : 380) - 24)
        standardButtons: Dialog.Yes | Dialog.No
        onAccepted: app.keyframes.clearKeyframes()
        Label {
            // FIXED width: a sole Label becomes the Dialog's contentItem, so
            // any binding from the dialog's own geometry (parent.width,
            // availableWidth) loops through implicitHeight. The dialog is
            // capped at 380 wide, so 330 always fits inside the padding.
            width: 330
            wrapMode: Text.Wrap
            text: qsTr("Remove all %1 keyframes from this clip? This cannot be undone.")
                    .arg(app.keyframes.count)
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 8

        ToolButton {
            id: playButton
            // Drawn, not icon.name and not a glyph:
            //   * icon.name resolves through the freedesktop icon theme, which
            //     exists on desktop Linux and NOT on Android -- the button
            //     rendered completely empty there.
            //   * U+275A (heavy vertical bar) for pause is missing from the
            //     Edge 40's default font and came out as two tofu boxes.
            // Two Rectangles and a Canvas triangle depend on no font or theme.
            implicitWidth: 40
            implicitHeight: 36
            onClicked: app.isPlaying = !app.isPlaying
            enabled: app.videoPath !== ""
            ToolTip.text: app.isPlaying ? qsTr("Pause") : qsTr("Play")
            ToolTip.visible: hovered

            contentItem: Item {
                implicitWidth: 40
                implicitHeight: 36

                Row {
                    anchors.centerIn: parent
                    spacing: 4
                    visible: app.isPlaying
                    Repeater {
                        model: 2
                        Rectangle {
                            width: 5; height: 16; radius: 1
                            color: playButton.enabled ? Material.foreground : Material.hintTextColor
                        }
                    }
                }

                Canvas {
                    anchors.centerIn: parent
                    width: 16; height: 16
                    visible: !app.isPlaying
                    // Repaint when the tint changes (enabled/disabled).
                    property color tint: playButton.enabled ? Material.foreground : Material.hintTextColor
                    onTintChanged: requestPaint()
                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.fillStyle = tint
                        ctx.beginPath()
                        ctx.moveTo(1, 0); ctx.lineTo(width - 1, height / 2); ctx.lineTo(1, height)
                        ctx.closePath(); ctx.fill()
                    }
                }
            }
        }

        ToolButton {
            text: "+ KF"   // no icon.name: no icon theme on Android (see play button)
            ToolTip.text: "Add keyframe at current time"
            ToolTip.visible: hovered
            onClicked: app.addKeyframeAtCurrent()
            enabled: app.videoPath !== ""
        }

        ToolButton {
            text: qsTr("Clear")
            font.pixelSize: 12
            enabled: app.videoPath !== "" && app.keyframes.count > 0
            onClicked: clearKeyframesDialog.open()
            ToolTip.text: qsTr("Remove every keyframe from this clip")
            ToolTip.visible: hovered
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true

            Slider {
                id: timeSlider
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                from: 0
                to: app.duration
                value: app.currentTime
                enabled: app.videoPath !== ""

                onMoved: {
                    app.currentTime = value
                }
            }

            // ---- Export trim range (in/out markers) ----
            // A shaded band with two draggable handles at the top of the
            // timeline marks the section that will be exported. The handles
            // are positioned by Binding so external changes (loading a new
            // clip, restoring the sidecar's in/out markers) always re-sync
            // them; dragging writes back to app.exportStart/exportEnd, which
            // are persisted in the per-video keyframe sidecar.
            Item {
                id: trimStrip
                anchors.top: parent.top
                anchors.topMargin: 2
                height: 12
                anchors.left: parent.left
                anchors.right: parent.right
                visible: app.videoPath !== ""

                Rectangle {
                    id: rangeShade
                    y: 1
                    height: 6
                    radius: 2
                    color: Material.accent
                    opacity: 0.35
                    Binding {
                        target: rangeShade
                        property: "x"
                        value: trimStrip.width * (app.exportStart / Math.max(app.duration, 0.001))
                    }
                    Binding {
                        target: rangeShade
                        property: "width"
                        value: Math.max(0, trimStrip.width * ((app.exportEnd - app.exportStart) / Math.max(app.duration, 0.001)))
                    }
                }

                Rectangle {
                    id: inHandle
                    objectName: "trimInHandle"
                    width: 7
                    height: 12
                    radius: 2
                    color: Material.accent
                    border.color: "white"
                    border.width: 1
                    onXChanged: {
                        if (dragIn.drag.active)
                            app.exportStart = Math.max(0, Math.min(app.exportEnd, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: inHandle
                        property: "x"
                        when: !dragIn.drag.active
                        value: trimStrip.width * (app.exportStart / Math.max(app.duration, 0.001)) - 3
                    }
                    MouseArea {
                        id: dragIn
                        anchors.fill: parent
                        drag.target: parent
                        drag.axis: Drag.XAxis
                        drag.minimumX: -3
                        drag.maximumX: trimStrip.width - 4
                        cursorShape: Qt.SizeHorCursor
                    }
                }

                Rectangle {
                    id: outHandle
                    objectName: "trimOutHandle"
                    width: 7
                    height: 12
                    radius: 2
                    color: Material.accent
                    border.color: "white"
                    border.width: 1
                    onXChanged: {
                        if (dragOut.drag.active)
                            app.exportEnd = Math.max(app.exportStart, Math.min(app.duration, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: outHandle
                        property: "x"
                        when: !dragOut.drag.active
                        value: trimStrip.width * (app.exportEnd / Math.max(app.duration, 0.001)) - 3
                    }
                    MouseArea {
                        id: dragOut
                        anchors.fill: parent
                        drag.target: parent
                        drag.axis: Drag.XAxis
                        drag.minimumX: -3
                        drag.maximumX: trimStrip.width - 4
                        cursorShape: Qt.SizeHorCursor
                    }
                }
            }

            // Keyframe markers
            // Tracks, drawn under the keyframe dots and deliberately unlike
            // them: a bar spanning what the track covers, with a bracket at
            // each end. Keyframes are things you placed; a track is a stretch
            // of time something else is driving, and the two should never be
            // mistaken for each other.
            Repeater {
                id: trackBars
                model: trackSpanList

                Item {
                    required property var modelData
                    required property int index
                    readonly property real x0: (parent.width - 8)
                                               * (modelData.start / Math.max(app.duration, 0.001))
                    readonly property real x1: (parent.width - 8)
                                               * (modelData.end / Math.max(app.duration, 0.001))
                    x: x0
                    y: parent.height - 22
                    width: Math.max(x1 - x0, 2) + 8
                    height: 10

                    // The span itself.
                    Rectangle {
                        x: 4
                        y: 3
                        width: Math.max(parent.width - 8, 2)
                        height: 4
                        radius: 2
                        color: "#f0a030"
                        opacity: 0.75
                    }
                    // Start bracket.
                    Rectangle {
                        x: 2; y: 0; width: 3; height: 10; radius: 1
                        color: "#f0a030"
                    }
                    // End bracket: red when the tracker lost the subject there,
                    // so a track that ran out is visibly different from one
                    // that gave up.
                    Rectangle {
                        x: parent.width - 5; y: 0; width: 3; height: 10; radius: 1
                        color: modelData.lost ? "#e05050" : "#f0a030"
                    }

                    MouseArea {
                        anchors.fill: parent
                        anchors.margins: -6
                        cursorShape: Qt.PointingHandCursor
                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        onClicked: (mouse) => {
                            if (mouse.button === Qt.RightButton) {
                                trackMenu.targetIndex = index
                                trackMenu.popup()
                            } else {
                                app.currentTime = modelData.start
                            }
                        }
                        // Touch has no right button (same reason as the
                        // keyframe markers).
                        onPressAndHold: {
                            trackMenu.targetIndex = index
                            trackMenu.popup()
                        }
                        ToolTip.text: modelData.lost
                            ? qsTr("Track %1: %2 s to %3 s, then lost the subject")
                                  .arg(index + 1).arg(modelData.start.toFixed(1))
                                  .arg(modelData.end.toFixed(1))
                            : qsTr("Track %1: %2 s to %3 s")
                                  .arg(index + 1).arg(modelData.start.toFixed(1))
                                  .arg(modelData.end.toFixed(1))
                        ToolTip.visible: containsMouse
                        hoverEnabled: true
                    }
                }
            }

            Repeater {
                model: app.keyframes
                Rectangle {
                    x: (parent.width - 8) * (kfTime / Math.max(app.duration, 0.001))
                    y: parent.height - 14
                    width: 8
                    height: 8
                    radius: 4
                    color: Material.accent
                    border.color: "white"
                    border.width: 1

                    MouseArea {
                        // An 8 px marker is an impossible touch target; the
                        // hit area extends well past the dot.
                        anchors.fill: parent
                        anchors.margins: -8
                        cursorShape: Qt.PointingHandCursor
                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        onClicked: (mouse) => {
                            if (mouse.button === Qt.RightButton) {
                                kfContextMenu.targetIndex = index
                                kfContextMenu.popup()
                            } else {
                                app.currentTime = kfTime
                            }
                        }
                        onDoubleClicked: {
                            kfEditDialog.openFor(index)
                        }
                        // Touch has no right button, so a long press opens the
                        // same menu: without this there was no way at all to
                        // delete a keyframe on Android.
                        onPressAndHold: {
                            kfContextMenu.targetIndex = index
                            kfContextMenu.popup()
                        }
                    }
                }
            }
        }

        Label {
            text: {
                var mins = Math.floor(app.currentTime / 60);
                var secs = Math.floor(app.currentTime % 60);
                return mins + ":" + (secs < 10 ? "0" : "") + secs;
            }
            Layout.preferredWidth: 50
        }

        Label {
            text: "/"
            Layout.preferredWidth: 10
        }

        Label {
            text: {
                var mins = Math.floor(app.duration / 60);
                var secs = Math.floor(app.duration % 60);
                return mins + ":" + (secs < 10 ? "0" : "") + secs;
            }
            Layout.preferredWidth: 50
        }
    }

    // Context menu for keyframe markers (right-click)
    // Refreshed wholesale on tracksChanged: a track is added or removed as a
    // unit, never edited in place.
    property var trackSpanList: []
    Connections {
        target: app
        function onTracksChanged() { trackSpanList = app.trackSpans() }
    }
    Component.onCompleted: trackSpanList = app.trackSpans()

    Menu {
        id: trackMenu
        property int targetIndex: -1
        MenuItem {
            text: qsTr("Delete this track")
            onTriggered: if (trackMenu.targetIndex >= 0) app.removeTrack(trackMenu.targetIndex)
        }
        MenuItem {
            text: qsTr("Go to start")
            onTriggered: {
                const spans = app.trackSpans()
                if (trackMenu.targetIndex >= 0 && trackMenu.targetIndex < spans.length)
                    app.currentTime = spans[trackMenu.targetIndex].start
            }
        }
    }

    Menu {
        id: kfContextMenu
        property int targetIndex: -1

        MenuItem {
            text: qsTr("Delete Keyframe")
            onClicked: app.keyframes.removeKeyframe(kfContextMenu.targetIndex)
        }
    }

    // Hidden dialog for editing keyframe
    Dialog {
        id: kfEditDialog
        title: "Edit Keyframe"
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel | Dialog.Discard
        width: 300

        property int editIndex: -1

        function openFor(idx) {
            editIndex = idx
            // Fetch each value by role: the role context properties (kfTime,
            // kfYaw, ...) only exist inside the Repeater delegate scope, not
            // here, so read them through the model's data().
            timeField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0101).toFixed(2)   // TimeRole
            yawField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0102).toFixed(1)     // YawRole
            pitchField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0103).toFixed(1)   // PitchRole
            rollField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0104).toFixed(1)    // RollRole
            fovField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0105).toFixed(0)     // FovRole
            open()
        }

        ColumnLayout {
            anchors.fill: parent
            spacing: 8

            RowLayout { Label { text: "Time:"; Layout.preferredWidth: 60 } TextField { id: timeField; Layout.fillWidth: true } }
            RowLayout { Label { text: "Yaw:"; Layout.preferredWidth: 60 } TextField { id: yawField; Layout.fillWidth: true } }
            RowLayout { Label { text: "Pitch:"; Layout.preferredWidth: 60 } TextField { id: pitchField; Layout.fillWidth: true } }
            RowLayout { Label { text: "Roll:"; Layout.preferredWidth: 60 } TextField { id: rollField; Layout.fillWidth: true } }
            RowLayout { Label { text: "FOV:"; Layout.preferredWidth: 60 } TextField { id: fovField; Layout.fillWidth: true } }
        }

        onAccepted: {
            if (editIndex >= 0) {
                app.keyframes.updateKeyframe(editIndex,
                    parseFloat(timeField.text),
                    parseFloat(yawField.text),
                    parseFloat(pitchField.text),
                    parseFloat(rollField.text),
                    parseFloat(fovField.text))
            }
        }
        onDiscarded: {
            if (editIndex >= 0)
                app.keyframes.removeKeyframe(editIndex)
        }
    }
}
