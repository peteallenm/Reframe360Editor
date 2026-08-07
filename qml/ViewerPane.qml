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
            imuOrientation: app.imuStabilize ? app.imuOrientation : Qt.quaternion(1, 0, 0, 0)
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
