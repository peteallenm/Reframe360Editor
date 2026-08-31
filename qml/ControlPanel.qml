import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// Side panel. Tabs are grouped by WHEN you touch them:
//   View      framing you keyframe while watching (yaw/pitch/roll/FOV, projection)
//   Stabilise set once per clip: stabilisation mode + Auto sync (+ Advanced)
//   Stitch    everything about the two lenses: lens mode, parallax, calibration
//   Colour    primaries + tone curves
// A status strip under the tabs shows the per-clip state from any tab.
Item {
    id: controlPanel

    // A labelled slider whose value is bound to a target property and written
    // back through an explicit callback. onValueChanged fires only while the
    // user drags (pressed), and the Binding keeps the handle in sync with
    // external changes (Reset, settings load) even after the user has dragged.
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
        property string tip: ""
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
            hoverEnabled: row.tip.length > 0
            ToolTip.text: row.tip
            ToolTip.visible: hovered && row.tip.length > 0
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

    // A collapsible section for rarely-needed controls.
    component Disclosure: ColumnLayout {
        id: disc
        property string title: ""
        property bool open: false
        default property alias content: discContent.data
        Layout.fillWidth: true
        spacing: 4

        RowLayout {
            Layout.fillWidth: true
            // Drawn, not a glyph: U+25B8/U+25BE are missing from the Edge 40's
            // default font and rendered as tofu boxes, same as the play/pause
            // button did. A rotated triangle depends on no font.
            Canvas {
                id: discArrow
                width: 10; height: 10
                // A RowLayout gives an item zero size unless it is told one.
                Layout.preferredWidth: 10
                Layout.preferredHeight: 10
                Layout.alignment: Qt.AlignVCenter
                property bool open: disc.open
                onOpenChanged: requestPaint()
                onPaint: {
                    var ctx = getContext("2d")
                    ctx.reset()
                    ctx.fillStyle = "#bbb"
                    ctx.beginPath()
                    if (open) { ctx.moveTo(0, 2); ctx.lineTo(10, 2); ctx.lineTo(5, 9) }
                    else      { ctx.moveTo(2, 0); ctx.lineTo(9, 5); ctx.lineTo(2, 10) }
                    ctx.closePath(); ctx.fill()
                }
            }
            Label {
                text: disc.title
                font.pixelSize: 12
                color: "#bbb"
                Layout.fillWidth: true
            }
            // Handlers, not a MouseArea: an anchored MouseArea inside a
            // RowLayout is layout-managed and Qt warns it is undefined
            // behaviour. TapHandler covers the whole row without anchoring.
            TapHandler { onTapped: disc.open = !disc.open }
            HoverHandler { cursorShape: Qt.PointingHandCursor }
        }
        ColumnLayout {
            id: discContent
            visible: disc.open
            Layout.fillWidth: true
            spacing: 8
        }
    }

    // A ScrollView that can be flicked by touch even where a slider sits
    // under the finger: pressDelay holds the press back long enough for a
    // flick to win, while press-and-hold (or any drag after the delay) still
    // goes to the slider. Desktop keeps instant presses.
    component PanelScroll: ScrollView {
        contentWidth: availableWidth
        clip: true
        Component.onCompleted: {
            if (Qt.platform.os === "android" && contentItem instanceof Flickable)
                contentItem.pressDelay = 120
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        TabBar {
            id: panelTabs
            objectName: "panelTabs"
            Layout.fillWidth: true
            Material.elevation: 2

            // Four tabs in a 340 px panel: keep the labels short and the font small
            // enough that none of them elides.
            TabButton { text: qsTr("View"); font.pixelSize: 12 }
            TabButton { text: qsTr("Stabilise"); font.pixelSize: 12 }
            TabButton { text: qsTr("Stitch"); font.pixelSize: 12 }
            TabButton { text: qsTr("Colour"); font.pixelSize: 12 }
        }

        // Per-clip state, visible from any tab.
        Rectangle {
            Layout.fillWidth: true
            height: 22
            color: "#262626"
            Label {
                anchors.fill: parent
                anchors.leftMargin: 10
                anchors.rightMargin: 10
                verticalAlignment: Text.AlignVCenter
                font.pixelSize: 11
                color: "#9a9a9a"
                elide: Text.ElideRight
                text: {
                    var parts = []
                    if (!app.videoPath) return qsTr("No clip loaded")
                    parts.push(app.imuStabilize
                               ? (app.imuSmoothing > 0.9 ? qsTr("Hold world steady") : qsTr("Follow camera"))
                               : qsTr("Not stabilised"))
                    if (app.imuStabilize)
                        parts.push(app.autoSyncRunning ? qsTr("syncing…")
                                                       : qsTr("sync %1 s").arg(app.imuSyncOffset.toFixed(3)))
                    parts.push(app.activeLens === 2
                               ? (app.flowStitch ? qsTr("stitch: parallax") : qsTr("stitch: plain blend"))
                               : (app.activeLens === 0 ? qsTr("front lens only") : qsTr("rear lens only")))
                    if (app.colorGrade.curvesActive) parts.push(qsTr("curves"))
                    return parts.join("  ·  ")
                }
            }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: panelTabs.currentIndex

            // =========================== View ===========================
            PanelScroll {

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
                                    Layout.fillWidth: true
                                    from: 10; to: 180; stepSize: 1
                                    value: app.fov
                                    onValueChanged: if (pressed) app.fov = value
                                }
                                Label { text: app.fov.toFixed(0) + "°"; Layout.preferredWidth: 50 }
                            }

                            Label {
                                text: qsTr("Drag the video to look around. Keyframes (timeline) record these four values.")
                                font.pixelSize: 10
                                color: "#888"
                                wrapMode: Text.Wrap
                                Layout.fillWidth: true
                            }
                        }
                    }

                    GroupBox {
                        title: qsTr("Projection")
                        Layout.fillWidth: true
                        ComboBox {
                            anchors.fill: parent
                            model: [qsTr("Perspective"), qsTr("Equirectangular"), qsTr("Stereographic"), qsTr("SportsView")]
                            currentIndex: app.projection
                            onCurrentIndexChanged: app.projection = currentIndex
                        }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // ========================= Stabilise =========================
            PanelScroll {

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item { height: 16 }

                    GroupBox {
                        title: qsTr("Stabilisation")
                        Layout.fillWidth: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8

                            CheckBox {
                                id: imuCheck
                                text: qsTr("Stabilise (motion sensor)")
                                checked: app.imuStabilize
                                onCheckedChanged: app.imuStabilize = checked
                            }

                            // Auto sync first: it is the one per-clip action
                            // (aligns the sensor clock to the video, then adds
                            // optical yaw correction) and it must be reachable
                            // without scrolling on a phone.
                            RowLayout {
                                enabled: imuCheck.checked
                                Layout.fillWidth: true
                                spacing: 8
                                Button {
                                    text: qsTr("Auto sync")
                                    enabled: !app.autoSyncRunning && imuCheck.checked
                                    onClicked: app.autoSyncAndCalibrate()
                                    ToolTip.text: qsTr("Aligns the motion sensor to the video by tracking the picture, then adds optical yaw-drift correction. Takes ~20 s on a 2-minute clip.")
                                    ToolTip.visible: hovered
                                }
                                Label {
                                    id: autoSyncLabel
                                    text: app.autoSyncRunning
                                          ? qsTr("Syncing… %1%").arg((app.autoSyncProgress * 100).toFixed(0))
                                          : (app.autoSyncStatus.length ? app.autoSyncStatus
                                                                       : qsTr("Sync offset %1 s").arg(app.imuSyncOffset.toFixed(3)))
                                    font.pixelSize: 11
                                    color: app.autoSyncRunning ? "#aaa" : "#8f8"
                                    Layout.fillWidth: true
                                    Layout.preferredWidth: 0
                                    Layout.minimumWidth: 0
                                    elide: Text.ElideRight
                                    HoverHandler { id: autoSyncHover }
                                    ToolTip.text: autoSyncLabel.text
                                    ToolTip.visible: autoSyncHover.hovered && autoSyncLabel.truncated
                                }
                            }

                            Rectangle { Layout.fillWidth: true; height: 1; color: "#444"; Layout.topMargin: 4 }

                            // The two behaviours used to be the ends of one slider with a
                            // hidden threshold at 0.9. They are different things, so they
                            // are a choice; the smoothing amount only exists in Follow mode.
                            Label { text: qsTr("Mode"); font.pixelSize: 11; color: "#aaa"; enabled: imuCheck.checked }
                            RowLayout {
                                enabled: imuCheck.checked
                                Layout.fillWidth: true
                                property real followAmount: app.imuSmoothing > 0.9 ? 0.5 : app.imuSmoothing
                                RadioButton {
                                    id: followMode
                                    text: qsTr("Follow camera")
                                    checked: app.imuSmoothing <= 0.9
                                    onClicked: app.imuSmoothing = parent.followAmount
                                    ToolTip.text: qsTr("Removes shake but keeps your pans and tilts.")
                                    ToolTip.visible: hovered
                                }
                                RadioButton {
                                    id: holdMode
                                    text: qsTr("Hold world steady")
                                    checked: app.imuSmoothing > 0.9
                                    onClicked: app.imuSmoothing = 1.0
                                    ToolTip.text: qsTr("Cancels all camera rotation: the scene stays fixed and you choose where to look with the View controls.")
                                    ToolTip.visible: hovered
                                }
                            }

                            SliderRow {
                                labelText: qsTr("Smoothing:")
                                value: Math.min(app.imuSmoothing, 0.9)
                                onUserChange: (v) => app.imuSmoothing = v
                                min: 0; max: 0.9; step: 0.01
                                controlEnabled: imuCheck.checked && followMode.checked
                                visible: followMode.checked
                                tip: qsTr("How much of the camera's own motion to keep. Low = follows the camera closely, high = smooth, floaty motion.")
                            }

                            Disclosure {
                                title: qsTr("Advanced")
                                enabled: imuCheck.checked

                                SliderRow {
                                    labelText: qsTr("Sync offset:")
                                    value: app.imuSyncOffset
                                    onUserChange: (v) => app.imuSyncOffset = v
                                    min: 0; max: 0.3; step: 0.001; decimals: 3; suffixText: " s"
                                    tip: qsTr("Sensor time minus video time. Auto sync measures this; only touch it if the stabilisation looks a fraction of a second early or late.")
                                }
                                SliderRow {
                                    labelText: qsTr("Clock drift:")
                                    value: app.imuDrift * 1000
                                    onUserChange: (v) => app.imuDrift = v / 1000
                                    min: -1; max: 1; step: 0.01; decimals: 2; suffixText: " ms/s"
                                    tip: qsTr("Relative rate of the two clocks. Two crystal clocks differ by well under 0.5 ms/s; Auto sync leaves this at 0 unless it can measure otherwise.")
                                }

                                Label {
                                    text: qsTr("Gyro calibration is fitted by Auto sync and accepted only when physically plausible. These manage what is stored.")
                                    font.pixelSize: 10
                                    color: "#888"
                                    wrapMode: Text.Wrap
                                    Layout.fillWidth: true
                                }
                                GridLayout {
                                    enabled: !app.autoSyncRunning
                                    Layout.fillWidth: true
                                    columns: 2
                                    columnSpacing: 6
                                    rowSpacing: 6
                                    Button {
                                        text: qsTr("Clear video cal")
                                        Layout.fillWidth: true
                                        ToolTip.text: qsTr("Discard this video's stored gyro calibration and re-integrate from the raw sensor.")
                                        ToolTip.visible: hovered
                                        onClicked: app.clearGyroCalibration()
                                    }
                                    Button {
                                        text: qsTr("Clear camera cal")
                                        Layout.fillWidth: true
                                        ToolTip.text: qsTr("Discard the camera-wide gyro calibration applied to every clip without one of its own.")
                                        ToolTip.visible: hovered
                                        onClicked: app.clearCameraGyroDefaults()
                                    }
                                    Button {
                                        text: qsTr("Set as camera default")
                                        Layout.fillWidth: true
                                        Layout.columnSpan: 2
                                        ToolTip.text: qsTr("Apply this video's calibration to every future clip that has none of its own.")
                                        ToolTip.visible: hovered
                                        onClicked: app.saveGyroCalibrationAsCameraDefault()
                                    }
                                }
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // =========================== Stitch ===========================
            PanelScroll {

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item { height: 16 }

                    GroupBox {
                        title: qsTr("Lenses")
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 8
                            ComboBox {
                                Layout.fillWidth: true
                                model: [qsTr("Front lens only"), qsTr("Rear lens only"), qsTr("Both (stitched)")]
                                currentIndex: app.activeLens
                                onCurrentIndexChanged: app.activeLens = currentIndex
                            }
                        }
                    }

                    GroupBox {
                        title: qsTr("Parallax Correction")
                        Layout.fillWidth: true
                        enabled: app.activeLens === 2
                        ToolTip.text: qsTr("Measures how far apart the two lenses see each part of the seam and morphs both views toward each other, so near and far objects line up at the same time.")
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
                                tip: qsTr("1 = the measured parallax. Lower if the seam looks over-corrected.")
                            }
                            SliderRow {
                                labelText: qsTr("Max parallax:")
                                value: app.flowIterations
                                onUserChange: (v) => app.flowIterations = v
                                min: 4; max: 60; step: 1; decimals: 0
                                controlEnabled: flowCheck.checked
                                tip: qsTr("Search range, in seam-band texels (about 4 per degree). 30 covers objects down to ~30 cm.")
                            }
                            SliderRow {
                                labelText: qsTr("Smoothness:")
                                value: app.flowAlpha
                                onUserChange: (v) => app.flowAlpha = v
                                min: 1; max: 30; step: 1; decimals: 0
                                controlEnabled: flowCheck.checked
                                tip: qsTr("Smoothing radius of the parallax map. Higher hides noise, lower follows object edges more tightly.")
                            }

                            Rectangle { Layout.fillWidth: true; height: 1; color: "#444"; Layout.topMargin: 4 }

                            CheckBox {
                                id: seamCheck
                                text: qsTr("Feature-avoiding seam")
                                checked: app.seamStitch
                                onCheckedChanged: app.seamStitch = checked
                                enabled: flowCheck.checked
                            }
                            SliderRow {
                                labelText: qsTr("Seam strength:")
                                value: app.seamStrength
                                onUserChange: (v) => app.seamStrength = v
                                min: 0; max: 1; step: 0.05
                                controlEnabled: flowCheck.checked && seamCheck.checked
                            }
                        }
                    }

                    GroupBox {
                        title: qsTr("Lens Calibration")
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
                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 6
                                Button {
                                    text: qsTr("Set as default")
                                    Layout.fillWidth: true
                                    enabled: presetCombo.count > 0
                                    onClicked: {
                                        app.calibrationPresets.setDefaultPreset(presetCombo.currentIndex)
                                        app.calibrationPresets.loadPreset(presetCombo.currentIndex, app.currentCalibration)
                                    }
                                }
                                Button {
                                    text: qsTr("Edit calibration…")
                                    Layout.fillWidth: true
                                    onClicked: calibrationDialog.open()
                                }
                            }
                            Label {
                                text: qsTr("Default profile: ") + (app.calibrationPresets.defaultPresetName() || qsTr("(none)"))
                                font.pixelSize: 11
                                color: "#aaa"
                            }
                        }
                    }

                    Item { Layout.fillHeight: true }
                }
            }

            // =========================== Colour ===========================
            PanelScroll {

                ColumnLayout {
                    width: parent.width
                    spacing: 16

                    Item { height: 16 }

                    GroupBox {
                        title: qsTr("Primaries")
                        Layout.fillWidth: true

                        ColumnLayout {
                            anchors.fill: parent
                            spacing: 10

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
                                tip: qsTr("Midtone contrast (clarity).")
                            }
                        }
                    }

                    GroupBox {
                        title: qsTr("Curves")
                        Layout.fillWidth: true
                        ToolTip.text: qsTr("Tone curves: Master shapes brightness for all channels; Red/Green/Blue tint the shadows, midtones or highlights. Applied after the primaries.")
                        ToolTip.visible: hovered

                        CurveEditor {
                            anchors.fill: parent
                        }
                    }

                    Button {
                        text: qsTr("Reset all colour")
                        Layout.fillWidth: true
                        onClicked: app.colorGrade.reset()
                    }

                    Item { Layout.fillHeight: true }
                }
            }
        }
    }
}
