import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

ScrollView {
    id: controlPanel
    contentWidth: availableWidth

    ColumnLayout {
        width: parent.width
        spacing: 16

        Item { height: 16 }

        GroupBox {
            title: qsTr("View Direction")
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 8

                RowLayout {
                    Label { text: qsTr("Yaw:"); Layout.preferredWidth: 60 }
                    Slider {
                        id: yawSlider
                        Layout.fillWidth: true
                        from: -180; to: 180; stepSize: 1
                        value: app.yaw
                        onValueChanged: if (pressed) app.yaw = value
                    }
                    Label { text: app.yaw.toFixed(0) + "°"; Layout.preferredWidth: 50 }
                }

                RowLayout {
                    Label { text: qsTr("Pitch:"); Layout.preferredWidth: 60 }
                    Slider {
                        id: pitchSlider
                        Layout.fillWidth: true
                        // Pitch is clamped away from the vertical poles to keep
                        // yaw/roll from swapping (see App::setPitch).
                        from: -89.5; to: 89.5; stepSize: 1
                        value: app.pitch
                        onValueChanged: if (pressed) app.pitch = value
                    }
                    Label { text: app.pitch.toFixed(0) + "°"; Layout.preferredWidth: 50 }
                }

                RowLayout {
                    Label { text: qsTr("Roll:"); Layout.preferredWidth: 60 }
                    Slider {
                        id: rollSlider
                        Layout.fillWidth: true
                        from: -180; to: 180; stepSize: 1
                        value: app.roll
                        onValueChanged: if (pressed) app.roll = value
                    }
                    Label { text: app.roll.toFixed(0) + "°"; Layout.preferredWidth: 50 }
                }

                RowLayout {
                    Label { text: qsTr("FOV:"); Layout.preferredWidth: 60 }
                    Slider {
                        id: fovSlider
                        Layout.fillWidth: true
                        from: 10; to: 180; stepSize: 1
                        value: app.fov
                        onValueChanged: if (pressed) app.fov = value
                    }
                    Label { text: app.fov.toFixed(0) + "°"; Layout.preferredWidth: 50 }
                }
            }
        }

        GroupBox {
            title: qsTr("Lens Selection")
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 8

                ComboBox {
                    id: lensCombo
                    Layout.fillWidth: true
                    model: [qsTr("Front Lens"), qsTr("Rear Lens"), qsTr("Auto Stitch")]
                    currentIndex: app.activeLens
                    onCurrentIndexChanged: app.activeLens = currentIndex
                }

                RowLayout {
                    Label { text: qsTr("Projection:"); Layout.preferredWidth: 80 }
                    ComboBox {
                        id: projectionCombo
                        Layout.fillWidth: true
                        model: [qsTr("Perspective"), qsTr("Equirectangular"), qsTr("Stereographic"), qsTr("SportsView")]
                        currentIndex: app.projection
                        onCurrentIndexChanged: app.projection = currentIndex
                    }
                }
            }
        }

        GroupBox {
            title: qsTr("IMU Stabilization")
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 8

                CheckBox {
                    id: imuCheck
                    text: qsTr("Auto-stabilize")
                    checked: app.imuStabilize
                    onCheckedChanged: app.imuStabilize = checked
                }

                RowLayout {
                    enabled: imuCheck.checked
                    Label { text: qsTr("Smoothing:"); Layout.preferredWidth: 80 }
                    Slider {
                        id: smoothingSlider
                        Layout.fillWidth: true
                        from: 0.0; to: 1.0
                        value: app.imuSmoothing
                        onMoved: app.imuSmoothing = value
                    }
                    Label { text: smoothingSlider.value.toFixed(2); Layout.preferredWidth: 40 }
                }

                RowLayout {
                    enabled: imuCheck.checked
                    Label { text: qsTr("Sync offset:"); Layout.preferredWidth: 80 }
                    Slider {
                        id: syncSlider
                        Layout.fillWidth: true
                        from: 0; to: 0.3
                        value: app.imuSyncOffset
                        stepSize: 0.01
                        onMoved: app.imuSyncOffset = value
                    }
                    Label { text: syncSlider.value.toFixed(2) + "s"; Layout.preferredWidth: 50 }
                }
            }
        }

        GroupBox {
            title: qsTr("Calibration")
            Layout.fillWidth: true

            ColumnLayout {
                anchors.fill: parent
                spacing: 8

                ComboBox {
                    id: presetCombo
                    Layout.fillWidth: true
                    model: app.calibrationPresets
                    textRole: "name"
                    onActivated: {
                        app.calibrationPresets.loadPreset(currentIndex, app.currentCalibration)
                        // Keep the loaded profile's name in the field so Save updates it.
                        presetNameField.text = presetCombo.currentText
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 4

                    TextField {
                        id: presetNameField
                        Layout.fillWidth: true
                        placeholderText: qsTr("Profile name")
                    }

                    Button {
                        text: qsTr("Save")
                        enabled: presetNameField.text.trim().length > 0
                        onClicked: {
                            app.calibrationPresets.savePreset(presetNameField.text, app.currentCalibration)
                            presetNameField.clear()
                        }
                    }

                    Button {
                        text: qsTr("Delete")
                        enabled: presetCombo.count > 0
                        onClicked: app.calibrationPresets.removePreset(presetCombo.currentIndex)
                    }
                }

                Button {
                    text: qsTr("Set as Default Profile")
                    Layout.fillWidth: true
                    enabled: presetCombo.count > 0
                    onClicked: {
                        app.calibrationPresets.setDefaultPreset(presetCombo.currentIndex)
                        app.calibrationPresets.loadPreset(presetCombo.currentIndex, app.currentCalibration)
                    }
                }

                Label {
                    text: qsTr("Default profile: ") + (app.calibrationPresets.defaultPresetName() || qsTr("(none)"))
                    font.pixelSize: 11
                    color: "#aaa"
                }

                Button {
                    text: qsTr("Edit Calibration...")
                    Layout.fillWidth: true
                    onClicked: calibrationDialog.open()
                }
            }
        }

        Item { Layout.fillHeight: true }
    }
}
