import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

Item {
    id: controlPanel

    // A labelled slider whose value is bound to a target property and written
    // back through an explicit callback. Each instance binds its value/onUserChange
    // directly (e.g. value: app.colorGrade.brightness), so no dynamic property
    // lookup is involved anywhere. onMoved fires only on user interaction, and
    // the Binding keeps the handle in sync with external changes (Reset All,
    // settings load) even after the user has dragged it.
    component SliderRow: RowLayout {
        id: row
        Layout.fillWidth: true
        property string labelText: ""
        property real value: 0
        property var onUserChange: null   // function(newValue) called on drag
        property real min: 0
        property real max: 1
        property real step: 0.01
        property int decimals: 2
        property string suffixText: ""
        property bool controlEnabled: true
        enabled: row.controlEnabled

        Label {
            text: row.labelText
            Layout.preferredWidth: 88
        }
        Slider {
            id: slider
            Layout.fillWidth: true
            from: row.min
            to: row.max
            stepSize: row.step
            // Write back while the user is dragging (same pattern as the View
            // tab's yaw/pitch sliders), so the preview updates live.
            onValueChanged: { if (pressed && row.onUserChange) row.onUserChange(value) }
            Binding {
                target: slider
                property: "value"
                value: row.value
            }
        }
        Label {
            text: row.value.toFixed(row.decimals) + row.suffixText
            Layout.preferredWidth: 56
        }
    }

    component SectionHeader: Label {
        id: header
        property string sectionTitle: ""
        property color accent: "#ddd"
        text: header.sectionTitle
        font.bold: true
        font.pixelSize: 12
        color: header.accent
        Layout.fillWidth: true
        Layout.topMargin: 6
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TabBar {
            id: panelTabs
            objectName: "panelTabs"
            Layout.fillWidth: true
            Material.elevation: 2

            TabButton { text: qsTr("View") }
            TabButton { text: qsTr("Colours") }
            TabButton { text: qsTr("Lens") }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: panelTabs.currentIndex

            // =========================== View tab ===========================
            ScrollView {
                contentWidth: availableWidth
                clip: true

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

                            RowLayout {
                                enabled: imuCheck.checked
                                Label { text: qsTr("Drift:"); Layout.preferredWidth: 80 }
                                Slider {
                                    id: driftSlider
                                    Layout.fillWidth: true
                                    from: -0.01; to: 0.01
                                    value: app.imuDrift
                                    stepSize: 0.0001
                                    onMoved: app.imuDrift = value
                                }
                                Label { text: (driftSlider.value * 1000).toFixed(1) + " ms/s"; Layout.preferredWidth: 50 }
                            }
                        }
                    }

                    GroupBox {
                        title: qsTr("Flow Stitching")
                        Layout.fillWidth: true
                        ToolTip.text: qsTr("Estimates the parallax displacement between the two lenses in the seam band (Horn-Schunck optical flow) and warps the rear view to align with the front before blending, removing seam ghosting on near objects.")
                        ToolTip.visible: hovered

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            CheckBox {
                                id: flowCheck
                                text: qsTr("Enable")
                                checked: app.flowStitch
                                onCheckedChanged: app.flowStitch = checked
                            }

                            SliderRow {
                                labelText: qsTr("Strength:")
                                value: app.flowStrength
                                onUserChange: (v) => app.flowStrength = v
                                min: 0; max: 2; step: 0.05
                                controlEnabled: flowCheck.checked
                            }

                            SliderRow {
                                labelText: qsTr("Iterations:")
                                value: app.flowIterations
                                onUserChange: (v) => app.flowIterations = v
                                min: 20; max: 100; step: 1; decimals: 0
                                controlEnabled: flowCheck.checked
                            }

                            SliderRow {
                                labelText: qsTr("Smoothness:")
                                value: app.flowAlpha
                                onUserChange: (v) => app.flowAlpha = v
                                min: 1; max: 100; step: 1; decimals: 0
                                controlEnabled: flowCheck.checked
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // ======================= Colours tab =====================
            ScrollView {
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item { height: 16 }

                    GroupBox {
                        title: qsTr("Image Adjustment")
                        Layout.fillWidth: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 10

                            Label {
                                text: qsTr("Image")
                                font.bold: true
                                font.pixelSize: 12
                                Layout.fillWidth: true
                            }

                            SliderRow {
                                labelText: qsTr("Brightness:")
                                value: app.colorGrade.brightness
                                onUserChange: (v) => app.colorGrade.brightness = v
                                min: -1; max: 1
                            }
                            SliderRow {
                                labelText: qsTr("Contrast:")
                                value: app.colorGrade.contrast
                                onUserChange: (v) => app.colorGrade.contrast = v
                                min: 0; max: 2
                            }
                            SliderRow {
                                labelText: qsTr("Saturation:")
                                value: app.colorGrade.saturation
                                onUserChange: (v) => app.colorGrade.saturation = v
                                min: 0; max: 2
                            }
                            SliderRow {
                                labelText: qsTr("Pop:")
                                value: app.colorGrade.pop
                                onUserChange: (v) => app.colorGrade.pop = v
                                min: -1; max: 1
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                height: 1
                                color: "#444"
                                Layout.topMargin: 6
                            }

                            Label {
                                text: qsTr("Colours")
                                font.bold: true
                                font.pixelSize: 12
                                Layout.fillWidth: true
                            }

                            SectionHeader { sectionTitle: qsTr("Brightness (Lows / Low Mids / High Mids / Highs)") }
                            SliderRow { labelText: qsTr("Lows:");      value: app.colorGrade.brightLows;      onUserChange: (v) => app.colorGrade.brightLows = v;      min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Low Mids:");  value: app.colorGrade.brightLowMids;    onUserChange: (v) => app.colorGrade.brightLowMids = v;    min: -1; max: 1 }
                            SliderRow { labelText: qsTr("High Mids:"); value: app.colorGrade.brightHighMids;   onUserChange: (v) => app.colorGrade.brightHighMids = v;   min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Highs:");     value: app.colorGrade.brightHighs;      onUserChange: (v) => app.colorGrade.brightHighs = v;      min: -1; max: 1 }

                            SectionHeader { sectionTitle: qsTr("Red"); accent: "#ef9a9a" }
                            SliderRow { labelText: qsTr("Lows:");  value: app.colorGrade.redLows;  onUserChange: (v) => app.colorGrade.redLows = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Mids:");  value: app.colorGrade.redMids;  onUserChange: (v) => app.colorGrade.redMids = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Highs:"); value: app.colorGrade.redHighs; onUserChange: (v) => app.colorGrade.redHighs = v; min: -1; max: 1 }

                            SectionHeader { sectionTitle: qsTr("Green"); accent: "#a5d6a7" }
                            SliderRow { labelText: qsTr("Lows:");  value: app.colorGrade.greenLows;  onUserChange: (v) => app.colorGrade.greenLows = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Mids:");  value: app.colorGrade.greenMids;  onUserChange: (v) => app.colorGrade.greenMids = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Highs:"); value: app.colorGrade.greenHighs; onUserChange: (v) => app.colorGrade.greenHighs = v; min: -1; max: 1 }

                            SectionHeader { sectionTitle: qsTr("Blue"); accent: "#90caf9" }
                            SliderRow { labelText: qsTr("Lows:");  value: app.colorGrade.blueLows;  onUserChange: (v) => app.colorGrade.blueLows = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Mids:");  value: app.colorGrade.blueMids;  onUserChange: (v) => app.colorGrade.blueMids = v;  min: -1; max: 1 }
                            SliderRow { labelText: qsTr("Highs:"); value: app.colorGrade.blueHighs; onUserChange: (v) => app.colorGrade.blueHighs = v; min: -1; max: 1 }

                            Button {
                                text: qsTr("Reset All")
                                Layout.fillWidth: true
                                Layout.topMargin: 8
                                onClicked: app.colorGrade.reset()
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // ======================== Lens tab =======================
            ScrollView {
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item { height: 16 }

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
        }
    }
}
