import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

Pane {
    id: timeline
    height: 60
    padding: 8

    RowLayout {
        anchors.fill: parent
        spacing: 8

        ToolButton {
            id: playButton
            icon.name: app.isPlaying ? "media-playback-pause" : "media-playback-start"
            onClicked: app.isPlaying = !app.isPlaying
            enabled: app.videoPath !== ""
        }

        Slider {
            id: timeSlider
            Layout.fillWidth: true
            from: 0
            to: app.duration
            value: app.currentTime
            enabled: app.videoPath !== ""

            onMoved: {
                app.currentTime = value
            }
        }

        Label {
            text: {
                var mins = Math.floor(app.currentTime / 60);
                var secs = Math.floor(app.currentTime % 60);
                return mins + ":" + (secs < 10 ? "0" : "") + secs;
            }
            Layout.preferredWidth: 50
        }

        Label {
            text: "/"
            Layout.preferredWidth: 10
        }

        Label {
            text: {
                var mins = Math.floor(app.duration / 60);
                var secs = Math.floor(app.duration % 60);
                return mins + ":" + (secs < 10 ? "0" : "") + secs;
            }
            Layout.preferredWidth: 50
        }
    }
}
