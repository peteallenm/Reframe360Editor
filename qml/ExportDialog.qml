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
    // Encoders driven by a target bitrate rather than CRF: NVENC and both
    // Android MediaCodec encoders. The slider was showing CRF on Android,
    // which the hardware encoder silently ignores -- the "quality" choice did
    // nothing and only the (hidden) bitrate mattered.
    readonly property bool bitrateMode: isNvenc || Qt.platform.os === "android"

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
        // On Android a remembered value is a bare NAME; anything path- or
        // URI-shaped is stale litter from a content:// URI and must not be
        // reused, or each export nests the last URI inside the next name.
        if (app.exportFileName !== ""
            && !(Qt.platform.os === "android"
                 && (app.exportFileName.indexOf("/") >= 0
                     || app.exportFileName.indexOf("%") >= 0
                     || app.exportFileName.indexOf(":") >= 0)))
            outputField.text = app.exportFileName
        else if (outputField.text === "") {
            // On Android there is no writable path to prefill: the export
            // becomes a document created in the granted folder, so only the
            // NAME is meaningful here (a content:// URI has no usable folder
            // part, and the old prefill produced an unusable string).
            outputField.text = Qt.platform.os === "android"
                ? defaultBase + "_export.mp4"
                : defaultFolder + "/" + defaultBase + "_export.mp4"
        }
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
            Label {
                text: Qt.platform.os === "android" ? qsTr("Name:") : qsTr("Output:")
                Layout.preferredWidth: 60
            }
            TextField {
                id: outputField
                Layout.fillWidth: true
                // Android: with no hint at all Qt asks the keyboard for
                // sentence capitalisation, and the keyboard then re-derives its
                // shift state whenever the field updates -- cancelling a shift
                // the moment it is pressed, so a capital letter cannot be typed
                // at all. A file name is not a sentence: no auto-capitalisation
                // and no autocorrect, and shift then stays where it is put.
                inputMethodHints: Qt.ImhNoAutoUppercase | Qt.ImhNoPredictiveText
                placeholderText: Qt.platform.os === "android" ? qsTr("Output file name")
                                                              : qsTr("Output MP4 path")
            }
            // No Browse on Android: the export is created by NAME inside the
            // granted folder (nothing else is writable by path), so a native
            // save picker would only produce an unusable content:// string.
            Button {
                visible: Qt.platform.os !== "android"
                text: qsTr("Browse…")
                onClicked: saveDialog.open()
            }
        }

        Label {
            visible: Qt.platform.os === "android"
            text: app.folder.hasFolder
                  ? qsTr("Saved into “%1”.").arg(app.folder.folderName)
                  : qsTr("Open a folder first (Open… in the toolbar) so the export has somewhere to be saved.")
            color: app.folder.hasFolder ? Material.secondaryTextColor : "#ffb0b0"
            font.pixelSize: 11
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
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
                    // 2:1 sizes -- the native shape of a full equirectangular
                    // sphere. Up to 3840×1920 the phone's hardware encoder
                    // copes; 5.7K exceeds it and falls back to software
                    // encoding on Android (warned below).
                    ListElement { label: "360 2.5K (2560×1280)"; w: 2560; h: 1280 }
                    ListElement { label: "360 4K (3840×1920)";   w: 3840; h: 1920 }
                    ListElement { label: "360 5.7K (5760×2880)"; w: 5760; h: 2880 }
                }
                onActivated: {
                    app.exportWidth = resolutionModel.get(currentIndex).w
                    app.exportHeight = resolutionModel.get(currentIndex).h
                }
            }
        }

        Label {
            visible: Qt.platform.os === "android"
                     && (app.exportWidth > 4096 || app.exportHeight > 2304)
            text: qsTr("\u26a0 Bigger than the phone's hardware encoder can handle: a software encoder takes over. Expect the export to take many times the clip length, and keep the app in the foreground.")
            color: "#ffcc80"
            font.pixelSize: 11
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
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
                // Android has no libx264/libx265/NVENC; it has the phone's
                // hardware encoders, which are bitrate-driven (no CRF).
                readonly property bool onAndroid: Qt.platform.os === "android"
                model: onAndroid
                    ? [ { text: "H.264 – hardware", value: "h264_mediacodec" },
                        { text: "H.265 / HEVC – hardware", value: "hevc_mediacodec" } ]
                    : [ { text: "H.264 (libx264) – CPU", value: "libx264" },
                        { text: "H.265 (libx265) – CPU", value: "libx265" },
                        { text: "HEVC NVENC – GPU (NVIDIA)", value: "hevc_nvenc" } ]
                textRole: "text"
                valueRole: "value"
                onActivated: app.exportCodec = currentValue
            }
        }

        // ---- Quality: CRF for CPU codecs, bitrate for NVENC ----
        RowLayout {
            spacing: 8
            Label { text: bitrateMode ? qsTr("Bitrate:") : qsTr("Quality:"); Layout.preferredWidth: 60 }
            Slider {
                id: qualitySlider
                Layout.fillWidth: true
                from: bitrateMode ? 1 : 0
                to: bitrateMode ? 50 : 51
                stepSize: 1
                value: bitrateMode ? app.exportBitrate : app.exportCrf
                onMoved: {
                    if (bitrateMode) app.exportBitrate = value
                    else app.exportCrf = value
                }
            }
            Label {
                Layout.preferredWidth: 70
                text: bitrateMode ? qualitySlider.value.toFixed(0) + " Mbps" : qsTr("CRF ") + qualitySlider.value.toFixed(0)
            }
        }

        Label {
            text: bitrateMode ? qsTr("Higher = better quality, bigger file. The hardware encoder targets this average bitrate (12–20 Mbps is good for 1080p).")
                          : qsTr("CRF scale 0–51: lower is higher quality (≈19 recommended).")
            font.pixelSize: 11
            color: Material.secondaryTextColor
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        // ---- audio ----
        ColumnLayout {
            spacing: 2
            CheckBox {
                id: audioCheck
                text: qsTr("Keep the original audio")
                checked: true
            }
            Label {
                text: qsTr("Copies the clip's own sound into the export, cut to the same range. Copied rather than re-encoded, so it costs nothing and loses nothing.")
                font.pixelSize: 11
                color: Material.secondaryTextColor
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
                Layout.leftMargin: 8
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- 360 metadata (equirectangular renders only) ----
        ColumnLayout {
            visible: app.projection === 1
            spacing: 2
            CheckBox {
                id: sphericalCheck
                text: qsTr("Tag as 360 video")
                checked: true
            }
            Label {
                text: qsTr("Embeds the spherical-video metadata so YouTube and media players show the export as an interactive sphere. Best with a 2:1 \u201c360\u201d resolution above.")
                font.pixelSize: 11
                color: Material.secondaryTextColor
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
                Layout.leftMargin: 8
            }
        }

        Rectangle { Layout.fillWidth: true; height: 1; color: "#444" }

        // ---- Extra stabilization (mutually exclusive options) ----
        // Both options shell out to the ffmpeg CLI for vidstabdetect /
        // vidstabtransform. There is no ffmpeg executable on Android (and no
        // way to ship one usefully), so the whole section is hidden there
        // rather than offering a control that silently fails.
        ColumnLayout {
            visible: Qt.platform.os !== "android"
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
                         && (Qt.platform.os !== "android" || app.folder.hasFolder)
                onClicked: exportDialog.accept()
            }
        }
    }

    onAccepted: {
        // On Android the text field holds a name, not a writable path: nothing
        // can be written by path there. exportDestination() turns it into a
        // document in the granted folder and hands back its content:// URI.
        // On desktop it returns the path unchanged.
        var path = app.exportDestination(ensureSuffix(outputField.text))
        app.exportVideo(path,
                        app.exportWidth, app.exportHeight,
                        app.exportFps,
                        app.exportStart, app.exportEnd,
                        app.exportCodec,
                        app.exportCrf,
                        app.exportBitrate,
                        app.exportVidstab,
                        app.exportVidstabInformed,
                        true,
                        app.projection === 1 && sphericalCheck.checked,
                        audioCheck.checked)
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