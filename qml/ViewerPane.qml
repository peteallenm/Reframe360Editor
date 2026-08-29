import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Render360 1.0

Item {
    id: viewerPane

    Rectangle {
        anchors.fill: parent
        color: "#1a1a1a"

        LensViewer {
            id: lensViewer
            anchors.fill: parent
            decoder: app.videoDecoder
            yaw: app.yaw
            pitch: app.pitch
            roll: app.roll
            fov: app.fov
            activeLens: app.activeLens
            projection: app.projection
            calibration: app.currentCalibration
            colorGrade: app.colorGrade
            // app.imuOrientation already returns the constant 180 deg video un-flip
            // when stabilization is off; overriding it with identity here put the
            // render back upside down.
            imuOrientation: app.imuOrientation
            flowStitch: app.flowStitch
            flowStrength: app.flowStrength
            flowImage: app.flowImage
            flowEncode: app.flowEncode
            seamStitch: app.seamStitch
            seamStrength: app.seamStrength
            seamImage: app.seamImage
        }

        MouseArea {
            id: dragArea
            anchors.fill: parent
            enabled: !!app.videoPath
            cursorShape: pressed ? Qt.ClosedHandCursor : Qt.OpenHandCursor
            acceptedButtons: Qt.LeftButton

            property point dragOrigin

            onPressed: (mouse) => {
                dragOrigin = Qt.point(mouse.x, mouse.y)
            }
            onPositionChanged: (mouse) => {
                if (!pressed) return
                var dx = mouse.x - dragOrigin.x
                var dy = mouse.y - dragOrigin.y
                if (dx === 0 && dy === 0) return
                var sens = app.fov / Math.max(height, 1)
                app.dragLook(dx * sens, -dy * sens);
                dragOrigin = Qt.point(mouse.x, mouse.y)
            }
        }

        Label {
            anchors.centerIn: parent
            text: qsTr("Open a 360 video to begin")
            font.pixelSize: 24
            color: "#888888"
            visible: !app.videoPath
        }
    }
}
