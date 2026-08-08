import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

Pane {
    id: timeline
    height: 60
    padding: 8

    RowLayout {
        anchors.fill: parent
        spacing: 8

        ToolButton {
            id: playButton
            icon.name: app.isPlaying ? "media-playback-pause" : "media-playback-start"
            onClicked: app.isPlaying = !app.isPlaying
            enabled: app.videoPath !== ""
        }

        ToolButton {
            icon.name: "list-add"
            text: "KF"
            ToolTip.text: "Add keyframe at current time"
            ToolTip.visible: hovered
            onClicked: app.addKeyframeAtCurrent()
            enabled: app.videoPath !== ""
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
                        if (dragIn.active)
                            app.exportStart = Math.max(0, Math.min(app.exportEnd, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: inHandle
                        property: "x"
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
                        if (dragOut.active)
                            app.exportEnd = Math.max(app.exportStart, Math.min(app.duration, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: outHandle
                        property: "x"
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
                        anchors.fill: parent
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
