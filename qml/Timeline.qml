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
            var kf = app.keyframes.data(app.keyframes.index(idx, 0), 0x0101)  // TimeRole
            timeField.text = (kfTime !== undefined ? kfTime : 0).toFixed(2)
            yawField.text = (kfYaw !== undefined ? kfYaw : 0).toFixed(1)
            pitchField.text = (kfPitch !== undefined ? kfPitch : 0).toFixed(1)
            rollField.text = (kfRoll !== undefined ? kfRoll : 0).toFixed(1)
            fovField.text = (kfFov !== undefined ? kfFov : 90).toFixed(0)
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
