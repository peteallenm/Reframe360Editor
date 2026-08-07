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
                onClicked: exportFrameDialog.open()
            }

            ToolButton {
                text: qsTr("Export Video")
                icon.name: "media-record"
                onClicked: exportVideoDialog.open()
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

    FileDialog {
        id: fileDialog
        title: qsTr("Open 360 Video")
        nameFilters: ["Video files (*.mp4 *.MP4 *.mov *.MOV *.mkv *.MKV)", "All files (*)"]
        onAccepted: {
            app.videoPath = selectedFile.toString().replace("file://", "")
        }
    }

    FileDialog {
        id: exportFrameDialog
        title: qsTr("Export Frame")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "png"
        nameFilters: ["PNG images (*.png)", "JPEG images (*.jpg)"]
        onAccepted: {
            app.exportFrame(selectedFile.toString().replace("file://", ""), 1920, 1080)
        }
    }

    FileDialog {
        id: exportVideoDialog
        title: qsTr("Export Video")
        fileMode: FileDialog.SaveFile
        defaultSuffix: "mp4"
        nameFilters: ["MP4 video (*.mp4)"]
        onAccepted: {
            app.exportVideo(selectedFile.toString().replace("file://", ""), 1920, 1080, 30.0, 0, app.duration)
        }
    }
}
