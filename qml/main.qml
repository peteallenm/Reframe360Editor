import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Dialogs
import Render360 1.0

ApplicationWindow {
    id: window
    width: 1280
    height: 720
    visible: true
    title: "Reframe360 Editor"
    Material.theme: Material.Dark
    Material.accent: Material.Blue

    // System-bar insets. The app targets SDK 36, so on Android it draws
    // edge-to-edge and content would otherwise sit under the navigation bar
    // (which in landscape is down one SIDE). SafeArea is QtQuick 6.9+, and the
    // desktop builds against Qt 6.4, so the lookup is short-circuited off
    // Android -- an unresolved attached property yields NaN and silently
    // collapses whatever it is used to size.
    readonly property real safeLeft: Qt.platform.os === "android" ? SafeArea.margins.left : 0
    readonly property real safeRight: Qt.platform.os === "android" ? SafeArea.margins.right : 0
    readonly property real safeTop: Qt.platform.os === "android" ? SafeArea.margins.top : 0

    // Side panel folded away to give the viewer the full width.
    property bool panelCollapsed: false

    // Folder of the currently loaded video (where exports default to) and its
    // base name, used to suggest sensible export filenames.
    function videoFolderPath() {
        var p = app.videoPath
        var i = p.lastIndexOf("/")
        return i >= 0 ? p.substring(0, i) : p
    }

    function videoBaseName() {
        // Decode first: a content:// URI percent-encodes the separators
        // (%2F for "/", %3A for ":"), so a raw lastIndexOf("/") finds the one
        // in ".../document/" and returns the whole encoded document id as the
        // "base name". That string then became the export file name.
        var p = decodeURIComponent(app.videoPath.toString())
        var i = Math.max(p.lastIndexOf("/"), p.lastIndexOf(":"))
        var base = i >= 0 ? p.substring(i + 1) : p
        var d = base.lastIndexOf(".")
        return d > 0 ? base.substring(0, d) : base
    }

    // Native save dialogs don't reliably auto-append defaultSuffix, and the
    // exporter needs the extension to pick the container/format — so make sure
    // a bare filename like "myvideo" still becomes "myvideo.mp4".
    function ensureSuffix(path, suffix) {
        var p = path.toString()
        return p.toLowerCase().endsWith("." + suffix) ? p : p + "." + suffix
    }

    header: ToolBar {
        leftPadding: window.safeLeft
        rightPadding: window.safeRight
        topPadding: window.safeTop
        Material.elevation: 4

        RowLayout {
            anchors.fill: parent
            spacing: 0

            ToolButton {
                text: qsTr("Open")
                icon.name: "document-open"
                onClicked: fileDialog.open()
            }

            // Android only: grant a whole folder once (Storage Access
            // Framework tree). Sidecars then resolve by name and keyframes can
            // be written back -- neither is possible with a single-file grant.
            // On desktop the ordinary file dialog already does everything.
            ToolButton {
                visible: app.folder.available
                text: app.folder.hasFolder ? qsTr("Folder: ") + app.folder.folderName
                                           : qsTr("Open Folder")
                onClicked: app.folder.hasFolder ? clipListDialog.open() : app.folder.pickFolder()
                ToolTip.text: qsTr("Pick the folder your clips are in, once. Sidecars (.imu, _thm) are then found automatically and keyframes can be saved.")
                ToolTip.visible: hovered
            }

            ToolSeparator {}

            ToolButton {
                text: app.usePreviewThumbnail ? qsTr("Full Video") : qsTr("Thumbnail")
                icon.name: "view-preview"
                enabled: app.previewThumbnailPath !== ""
                ToolTip.text: qsTr("Preview using the low-res *_thm video — export always uses the full video")
                ToolTip.visible: hovered
                onClicked: app.usePreviewThumbnail = !app.usePreviewThumbnail
            }

            ToolButton {
                text: qsTr("Export Frame")
                icon.name: "camera-photo"
                enabled: app.videoPath !== "" && !app.exportRunning
                onClicked: {
                    exportFrameDialog.currentFile = "file://" + videoFolderPath() + "/" + videoBaseName() + "_frame.png"
                    exportFrameDialog.open()
                }
            }

            ToolButton {
                text: qsTr("Export Video")
                icon.name: "media-record"
                enabled: app.videoPath !== "" && !app.exportRunning
                onClicked: {
                    exportVideoDialog.defaultFolder = videoFolderPath()
                    exportVideoDialog.defaultBase = videoBaseName()
                    exportVideoDialog.open()
                }
            }

            Item { Layout.fillWidth: true }

            Label {
                text: "Reframe360 Editor"
                font.pixelSize: 18
                font.bold: true
                Layout.rightMargin: 16
            }
        }
    }

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: window.safeLeft
        anchors.rightMargin: window.safeRight
        spacing: 0

        ViewerPane {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        // Collapse handle. The panel is 340 px of a phone's ~1200 logical
        // pixels and is only needed while setting a clip up, so it folds away
        // and the viewer takes the space. Drawn chevron, not a glyph: the
        // Edge 40's font lacks several of the obvious characters.
        Rectangle {
            id: panelHandle
            Layout.fillHeight: true
            Layout.preferredWidth: 20
            color: handleHover.hovered ? "#3a3a3a" : "#2b2b2b"

            Canvas {
                id: handleArrow
                anchors.centerIn: parent
                width: 9; height: 14
                property bool pointsRight: !window.panelCollapsed
                onPointsRightChanged: requestPaint()
                onPaint: {
                    var ctx = getContext("2d")
                    ctx.reset()
                    ctx.fillStyle = "#cccccc"
                    ctx.beginPath()
                    if (pointsRight) { ctx.moveTo(1, 0); ctx.lineTo(8, 7); ctx.lineTo(1, 14) }
                    else             { ctx.moveTo(8, 0); ctx.lineTo(1, 7); ctx.lineTo(8, 14) }
                    ctx.closePath(); ctx.fill()
                }
            }

            HoverHandler { id: handleHover }
            TapHandler { onTapped: window.panelCollapsed = !window.panelCollapsed }
            ToolTip.text: window.panelCollapsed ? qsTr("Show the controls") : qsTr("Hide the controls")
            ToolTip.visible: handleHover.hovered
        }

        ControlPanel {
            Layout.fillHeight: true
            Layout.preferredWidth: window.panelCollapsed ? 0 : 340
            visible: Layout.preferredWidth > 0
            clip: true
            Behavior on Layout.preferredWidth {
                NumberAnimation { duration: 140; easing.type: Easing.OutCubic }
            }
        }
    }

    footer: Timeline {
        Layout.fillWidth: true
    }

    // The clips in the granted folder. Chosen by name, so the app can pair each
    // one with its .imu / _thm / .keyframes.json inside the same grant.
    Dialog {
        id: clipListDialog
        title: qsTr("Clips in ") + app.folder.folderName
        modal: true
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(520, (parent ? parent.width : 520) - 24)
        height: Math.min(560, (parent ? parent.height : 560) - 24)
        standardButtons: Dialog.Close

        ColumnLayout {
            anchors.fill: parent
            spacing: 8

            Label {
                text: app.folder.clips.length > 0
                      ? qsTr("%1 clips").arg(app.folder.clips.length)
                      : qsTr("No videos found in this folder.")
                color: "#bbb"
                font.pixelSize: 12
            }

            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: app.folder.clips
                ScrollBar.vertical: ScrollBar {}
                delegate: ItemDelegate {
                    width: ListView.view.width
                    text: modelData
                    onClicked: {
                        app.openClipFromFolder(modelData)
                        clipListDialog.close()
                    }
                }
            }

            Button {
                Layout.fillWidth: true
                text: qsTr("Choose a different folder…")
                onClicked: { clipListDialog.close(); app.folder.pickFolder() }
            }
        }
    }

    CalibrationDialog {
        id: calibrationDialog
    }

    // Progress/status shown while an export is running.
    Dialog {
        id: exportProgressDialog
        modal: true
        visible: app.exportRunning
        closePolicy: Popup.NoAutoClose
        title: qsTr("Export")
        anchors.centerIn: parent
        width: 380
        standardButtons: Dialog.NoButton

        ColumnLayout {
            width: parent.width
            spacing: 12

            ProgressBar {
                Layout.fillWidth: true
                from: 0
                to: 1
                value: app.exportProgress
                indeterminate: app.exportProgress <= 0.0
            }

            Label {
                Layout.fillWidth: true
                text: app.exportStatus
                wrapMode: Text.Wrap
                horizontalAlignment: Text.AlignHCenter
            }
        }
    }

    // Export configuration dialog (resolution, codec, bitrate/CRF, FFmpeg
    // vidstab post-processing). Replaces the old bare save-as dialog.
    ExportDialog {
        id: exportVideoDialog
        defaultFolder: videoFolderPath()
        defaultBase: videoBaseName()
    }

    FileDialog {
        id: fileDialog
        title: qsTr("Open 360 Video (add its .imu and _thm too)")
        // Multi-select, because on Android a picked file arrives as an opaque
        // content:// URI: the app cannot append ".imu" to it, and a single-file
        // grant does not cover siblings, so the sidecars have to be picked
        // alongside the video. Selecting just the video still works exactly as
        // before -- on desktop the sidecars are found by path anyway.
        fileMode: FileDialog.OpenFiles
        nameFilters: ["360 clip and sidecars (*.mp4 *.MP4 *.mov *.MOV *.mkv *.MKV *.imu *.json)",
                      "Video files (*.mp4 *.MP4 *.mov *.MOV *.mkv *.MKV)",
                      "All files (*)"]
        onAccepted: {
            var video = "", imu = "", proxy = "", keyframes = ""
            for (var i = 0; i < selectedFiles.length; ++i) {
                var raw = selectedFiles[i].toString().replace("file://", "")
                // content:// percent-encodes the document id, so decode before
                // matching on the extension.
                var name = decodeURIComponent(raw).toLowerCase()
                if (name.endsWith(".imu"))                       imu = raw
                else if (name.endsWith(".keyframes.json"))        keyframes = raw
                else if (/_thm\.(mp4|mov|mkv)$/.test(name))      proxy = raw
                else                                             video = raw
            }
            // Only a proxy chosen? Then that IS the clip they want to view.
            if (video === "" && proxy !== "") { video = proxy; proxy = "" }
            if (video !== "")
                app.openClip(video, imu, proxy, keyframes)
        }
    }

    // Native save-as dialog (since QT_QUICK_CONTROLS_NATIVE_DIALOGS is no
    // longer forced to 0). fileMode: SaveFile makes this a proper "Save"
    // dialog; currentFile pre-fills a sensible name next to the source video.
    FileDialog {
        id: exportFrameDialog
        title: qsTr("Export Frame")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "png"
        nameFilters: ["PNG images (*.png)", "JPEG images (*.jpg)"]
        onAccepted: {
            app.exportFrame(ensureSuffix(selectedFile.toString().replace("file://", ""), "png"), 1920, 1080)
        }
    }
}
