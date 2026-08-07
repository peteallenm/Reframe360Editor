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
    title: "360 Render"
    Material.theme: Material.Dark
    Material.accent: Material.Blue

    // Folder of the currently loaded video (where exports default to) and its
    // base name, used to suggest sensible export filenames.
    function videoFolderPath() {
        var p = app.videoPath
        var i = p.lastIndexOf("/")
        return i >= 0 ? p.substring(0, i) : p
    }

    function videoBaseName() {
        var p = app.videoPath
        var i = p.lastIndexOf("/")
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
        Material.elevation: 4

        RowLayout {
            anchors.fill: parent
            spacing: 0

            ToolButton {
                text: qsTr("Open")
                icon.name: "document-open"
                onClicked: fileDialog.open()
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
                    exportVideoDialog.currentFile = "file://" + videoFolderPath() + "/" + videoBaseName() + "_export.mp4"
                    exportVideoDialog.open()
                }
            }

            Item { Layout.fillWidth: true }

            Label {
                text: "360 Render"
                font.pixelSize: 18
                font.bold: true
                Layout.rightMargin: 16
            }
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        ViewerPane {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        ControlPanel {
            Layout.fillHeight: true
            Layout.preferredWidth: 320
        }
    }

    footer: Timeline {
        Layout.fillWidth: true
    }

    CalibrationDialog {
        id: calibrationDialog
    }

    // Progress/status shown while an export is running.
    Dialog {
        id: exportDialog
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

    FileDialog {
        id: fileDialog
        title: qsTr("Open 360 Video")
        nameFilters: ["Video files (*.mp4 *.MP4 *.mov *.MOV *.mkv *.MKV)", "All files (*)"]
        onAccepted: {
            app.videoPath = selectedFile.toString().replace("file://", "")
        }
    }

    // Standard save-as dialogs (native, since QT_QUICK_CONTROLS_NATIVE_DIALOGS
    // is no longer forced to 0). fileMode: SaveFile makes these proper "Save"
    // dialogs; currentFile pre-fills a sensible name next to the source video.
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

    FileDialog {
        id: exportVideoDialog
        title: qsTr("Export Video")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "mp4"
        nameFilters: ["MP4 video (*.mp4)"]
        onAccepted: {
            app.exportVideo(ensureSuffix(selectedFile.toString().replace("file://", ""), "mp4"), 1920, 1080, 30.0, 0, app.duration)
        }
    }
}
