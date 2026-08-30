import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Dialogs
import Render360 1.0

Dialog {
    id: exportDialog
    title: qsTr("Export Video")
    modal: true
    standardButtons: Dialog.NoButton
    closePolicy: Popup.CloseOnEscape

    // Bound to the window, not a fixed 480 wide with a fixed 380 body. Those
    // are LOGICAL pixels: on a phone (device pixel ratio ~2.6) title + body +
    // button row came to more than the 1080-high screen and the Export button
    // sat below the bottom edge, unreachable -- the body scrolls, but the
    // buttons live outside it deliberately, so they have to fit.
    parent: Overlay.overlay
    anchors.centerIn: parent
    readonly property real availW: parent ? parent.width : 480
    readonly property real availH: parent ? parent.height : 720
    width: Math.min(480, availW - 24)

    property string defaultFolder: ""
    property string defaultBase: "export"

    readonly property bool isNvenc: app.exportCodec === "hevc_nvenc"

    function ensureSuffix(path) {
        var p = path.toString()
        return p.toLowerCase().endsWith(".mp4") ? p : p + ".mp4"
    }

    function formatTime(t) {
        var m = Math.floor(t / 60)
        var s = Math.floor(t % 60)
        var cs = Math.round((t % 1) * 100)
        return m + ":" + (s < 10 ? "0" : "") + s + "." + (cs < 10 ? "0" : "") + cs
    }

    onOpened: {
        // Restore the last-used output path for this video (persisted in the
        // keyframe sidecar) if we have one; otherwise fall back to deriving
        // one from the current folder/base.
        if (app.exportFileName !== "")
            outputField.text = app.exportFileName
        else if (outputField.text === "")
            outputField.text = defaultFolder + "/" + defaultBase + "_export.mp4"
        // Keep the resolution / fps / codec / quality combos synced to the
        // persisted settings (they may have been restored at startup).
        syncResolutionCombo()
        syncFpsCombo()
        syncCodecCombo()
    }

    function syncResolutionCombo() {
        for (var i = 0; i < resolutionModel.count; ++i) {
            if (resolutionModel.get(i).w === app.exportWidth &&
                resolutionModel.get(i).h === app.exportHeight) {
                resolutionCombo.currentIndex = i
                return
            }
        }
        resolutionCombo.currentIndex = -1
    }

    function syncFpsCombo() {
        var best = 0
        var bestDiff = 1e9
        for (var i = 0; i < fpsCombo.count; ++i) {
            var val = parseInt(fpsCombo.model[i])
            var d = Math.abs(val - app.exportFps)
            if (d < bestDiff) { bestDiff = d; best = i }
        }
        fpsCombo.currentIndex = best
    }

    function syncCodecCombo() {
        codecCombo.currentIndex = Math.max(0, codecCombo.indexOfValue(app.exportCodec))
    }

    // Body area height (the fields scroll internally if they exceed this).
    // Leave room for the title bar, the button row and padding (~220) so the
    // whole dialog fits; the body scrolls inside whatever is left.
    property int bodyHeight: Math.max(180, Math.min(380, availH - 220))

    contentItem: ColumnLayout {
        spacing: 8
        implicitWidth: 440

        // Scrollable body: fields reachable without pushing the action buttons
        // off the bottom of the window. The body uses a fixed content width so
        // there is no horizontal scrolling.
        ScrollView {
            id: bodyScroll
            Layout.fillWidth: true
            Layout.preferredWidth: 440
            Layout.preferredHeight: exportDialog.bodyHeight
            clip: true
            ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

            ColumnLayout {
                id: bodyLayout
                spacing: 12
                width: 424
                implicitWidth: 424

        // ---- Output file ----
        RowLayout {
            spacing: 8
            Label { text: qsTr("Output:"); Layout.preferredWidth: 60 }
            TextField {
                id: outputField
                Layout.fillWidth: true
                placeholderText: qsTr("Output MP4 path")
            }
            Button {
                text: qsTr("Browse…")
                onClicked: saveDialog.open()
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- Resolution / frame rate ----
        RowLayout {
            spacing: 8
            Label { text: qsTr("Resolution:"); Layout.preferredWidth: 60 }
            ComboBox {
                id: resolutionCombo
                Layout.fillWidth: true
                textRole: "label"
                model: ListModel {
                    id: resolutionModel
                    ListElement { label: "720p (1280×720)";    w: 1280; h: 720 }
                    ListElement { label: "1080p (1920×1080)";  w: 1920; h: 1080 }
                    ListElement { label: "1440p (2560×1440)";  w: 2560; h: 1440 }
                    ListElement { label: "4K (3840×2160)";     w: 3840; h: 2160 }
                }
                onActivated: {
                    app.exportWidth = resolutionModel.get(currentIndex).w
                    app.exportHeight = resolutionModel.get(currentIndex).h
                }
            }
        }

        RowLayout {
            spacing: 8
            Label { text: qsTr("Frame rate:"); Layout.preferredWidth: 60 }
            ComboBox {
                id: fpsCombo
                Layout.fillWidth: true
                model: ["24 fps", "25 fps", "30 fps", "50 fps", "60 fps"]
                onActivated: app.exportFps = parseInt(currentText)
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- Codec ----
        RowLayout {
            spacing: 8
            Label { text: qsTr("Codec:"); Layout.preferredWidth: 60 }
            ComboBox {
                id: codecCombo
                Layout.fillWidth: true
                model: [
                    { text: "H.264 (libx264) – CPU", value: "libx264" },
                    { text: "H.265 (libx265) – CPU", value: "libx265" },
                    { text: "HEVC NVENC – GPU (NVIDIA)", value: "hevc_nvenc" }
                ]
                textRole: "text"
                valueRole: "value"
                onActivated: app.exportCodec = currentValue
            }
        }

        // ---- Quality: CRF for CPU codecs, bitrate for NVENC ----
        RowLayout {
            spacing: 8
            Label { text: isNvenc ? qsTr("Bitrate:") : qsTr("Quality:"); Layout.preferredWidth: 60 }
            Slider {
                id: qualitySlider
                Layout.fillWidth: true
                from: isNvenc ? 1 : 0
                to: isNvenc ? 50 : 51
                stepSize: 1
                value: isNvenc ? app.exportBitrate : app.exportCrf
                onMoved: {
                    if (isNvenc) app.exportBitrate = value
                    else app.exportCrf = value
                }
            }
            Label {
                Layout.preferredWidth: 70
                text: isNvenc ? qualitySlider.value.toFixed(0) + " Mbps" : qsTr("CRF ") + qualitySlider.value.toFixed(0)
            }
        }

        Label {
            text: isNvenc ? qsTr("Lower = smaller file. HEVC NVENC targets this average bitrate.")
                          : qsTr("CRF scale 0–51: lower is higher quality (≈19 recommended).")
            font.pixelSize: 11
            color: Material.secondaryTextColor
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- Extra stabilization (mutually exclusive options) ----
        ColumnLayout {
            spacing: 4
            Label {
                text: qsTr("Extra stabilization:")
                font.pixelSize: 12
                color: Material.primaryTextColor
            }
            RadioButton {
                id: stabNone
                text: qsTr("None")
                checked: !app.exportVidstab && !app.exportVidstabInformed
                onToggled: {
                    if (checked) {
                        app.exportVidstab = false
                        app.exportVidstabInformed = false
                    }
                }
            }
            RadioButton {
                id: stabHybrid
                text: qsTr("Hybrid stabilization (experimental)")
                checked: app.exportVidstabInformed
                onToggled: {
                    if (checked) {
                        app.exportVidstabInformed = true
                        app.exportVidstab = false
                    }
                }
            }
            RadioButton {
                id: stabPost
                text: qsTr("FFmpeg vidstab post-processing")
                checked: app.exportVidstab
                onToggled: {
                    if (checked) {
                        app.exportVidstab = true
                        app.exportVidstabInformed = false
                    }
                }
            }
            Label {
                text: qsTr("Hybrid (experimental): analyzes a low-res pass with vidstabdetect and folds the detected jitter into the native render (single clean export). May not improve 360 content — see classic post-processing for reliable results. Post-processing: runs vidstabdetect + vidstabtransform on the output (two ffmpeg passes, re-encodes).")
                font.pixelSize: 11
                color: Material.secondaryTextColor
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- Trim range (read-only) ----
        RowLayout {
            Label { text: qsTr("Trim:"); Layout.preferredWidth: 60 }
            Label {
                Layout.fillWidth: true
                text: app.exportStart.toFixed(2) + "s – " + app.exportEnd.toFixed(2) + "s  (" +
                      formatTime(app.exportStart) + " – " + formatTime(app.exportEnd) + ")"
            }
        }

            } // bodyLayout
        } // bodyScroll (ScrollView)        // ---- Buttons (always visible, outside the scrollable body) ----
        RowLayout {
            Layout.fillWidth: true
            Layout.topMargin: 4
            Item { Layout.fillWidth: true }
            Button {
                text: qsTr("Cancel")
                onClicked: exportDialog.reject()
            }
            Button {
                text: qsTr("Export")
                highlighted: true
                enabled: app.videoPath !== "" && !app.exportRunning && outputField.text !== ""
                onClicked: exportDialog.accept()
            }
        }
    }

    onAccepted: {
        var path = ensureSuffix(outputField.text)
        app.exportVideo(path,
                        app.exportWidth, app.exportHeight,
                        app.exportFps,
                        app.exportStart, app.exportEnd,
                        app.exportCodec,
                        app.exportCrf,
                        app.exportBitrate,
                        app.exportVidstab,
                        app.exportVidstabInformed,
                        true)
    }

    FileDialog {
        id: saveDialog
        title: qsTr("Export Video")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "mp4"
        nameFilters: ["MP4 video (*.mp4)"]
        onAccepted: {
            var p = selectedFile.toString().replace("file://", "")
            outputField.text = ensureSuffix(p)
        }
    }
}