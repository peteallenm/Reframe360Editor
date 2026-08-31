import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import Render360 1.0

Item {
    id: viewerPane

    // Emitted by the empty-state button; main.qml routes it to the platform's
    // open flow (folder on Android, file dialog on desktop).
    signal openRequested()

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
            // named "controller": a property called "app" would shadow the context
            // object of the same name for every other binding in this item.
            controller: app
            flowStitch: app.flowStitch
            flowStrength: app.flowStrength
            flowImage: app.flowImage
            flowEncode: app.flowEncode
            seamStitch: app.seamStitch
            seamStrength: app.seamStrength
            seamImage: app.seamImage
            curveLut: app.colorGrade.curveLut
            curvesActive: app.colorGrade.curvesActive
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
                // Grab semantics: the picture follows the hand. Dragging DOWN
                // must move the scene down, i.e. tilt the view up. The vertical
                // sign was inverted (reported by the user); horizontal was right.
                app.dragLook(dx * sens, dy * sens);
                dragOrigin = Qt.point(mouse.x, mouse.y)
            }
        }

        // Empty state: a first launch used to be a black screen with a
        // passive sentence, and on Android nothing hinted that a FOLDER (not
        // a file) is what gets opened. The action itself is the hint.
        Column {
            anchors.centerIn: parent
            spacing: 16
            visible: !app.videoPath && !app.loadError

            Label {
                anchors.horizontalCenter: parent.horizontalCenter
                text: qsTr("Open a 360 video to begin")
                font.pixelSize: 24
                color: "#888888"
            }
            Label {
                anchors.horizontalCenter: parent.horizontalCenter
                visible: app.folder.available
                width: Math.min(viewerPane.width * 0.8, 460)
                horizontalAlignment: Text.AlignHCenter
                wrapMode: Text.Wrap
                text: app.folder.hasFolder
                      ? qsTr("Your folder \u201c%1\u201d is connected. The sensor data and preview files next to each clip are found automatically.").arg(app.folder.folderName)
                      : qsTr("Pick the folder your clips are in (the camera's DCIM folder, or wherever you copied them). The sensor data and preview files next to each clip are found automatically.")
                font.pixelSize: 13
                color: "#666666"
            }
            Button {
                anchors.horizontalCenter: parent.horizontalCenter
                highlighted: true
                text: app.folder.available
                      ? (app.folder.hasFolder ? qsTr("Choose a clip…") : qsTr("Open folder…"))
                      : qsTr("Open video…")
                onClicked: viewerPane.openRequested()
            }
        }

        // A failed open used to be invisible: the error signal went nowhere and
        // the previous clip's last frame stayed on screen, which reads as a hang.
        Label {
            anchors.centerIn: parent
            width: parent.width * 0.8
            text: app.loadError
            font.pixelSize: 16
            color: "#ff8080"
            horizontalAlignment: Text.AlignHCenter
            wrapMode: Text.Wrap
            visible: app.loadError.length > 0
            padding: 12
            background: Rectangle { color: "#cc000000"; radius: 6 }
        }
    }
}
