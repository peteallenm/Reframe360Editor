import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts

// Tone-curve editor. Four curves (master luma, red, green, blue) edited one at
// a time; the model lives in app.colorGrade (curvePoints / setCurvePoints /
// curveSamples). Interaction:
//   drag a point        move it (endpoints move vertically only)
//   double-click        add a point on the curve
//   right-click a point remove it (not the endpoints)
// The drawn curve is sampled from the C++ monotone interpolator so what you
// see is exactly what the LUT applies.
ColumnLayout {
    id: editor
    property int channel: 0
    property var points: []          // [{x,y}] in [0,1], sorted by x
    readonly property var channelColors: ["#e0e0e0", "#ef7070", "#70d070", "#70a0ff"]
    readonly property var channelNames: [qsTr("Master"), qsTr("Red"), qsTr("Green"), qsTr("Blue")]

    function reload() {
        var pts = app.colorGrade.curvePoints(channel)
        var arr = []
        for (var i = 0; i < pts.length; ++i) arr.push({ x: pts[i].x, y: pts[i].y })
        points = arr
        canvas.requestPaint()
    }
    function commit() {
        var out = []
        for (var i = 0; i < points.length; ++i) out.push(Qt.point(points[i].x, points[i].y))
        app.colorGrade.setCurvePoints(channel, out)
        canvas.requestPaint()
    }
    onChannelChanged: reload()
    Component.onCompleted: reload()
    Connections {
        target: app.colorGrade
        function onCurvesChanged() { if (!canvas.dragging) editor.reload() }
    }

    RowLayout {
        Layout.fillWidth: true
        Repeater {
            model: 4
            Button {
                text: editor.channelNames[index]
                flat: editor.channel !== index
                highlighted: editor.channel === index
                Material.accent: editor.channelColors[index]
                Layout.fillWidth: true
                font.pixelSize: 11
                onClicked: editor.channel = index
            }
        }
    }

    Canvas {
        id: canvas
        Layout.fillWidth: true
        Layout.preferredHeight: width * 0.7
        property int dragIndex: -1
        property bool dragging: dragIndex >= 0
        readonly property int pad: 8

        function toPx(p) { return Qt.point(pad + p.x * (width - 2 * pad), height - pad - p.y * (height - 2 * pad)) }
        function fromPx(x, y) {
            return { x: Math.max(0, Math.min(1, (x - pad) / (width - 2 * pad))),
                     y: Math.max(0, Math.min(1, (height - pad - y) / (height - 2 * pad))) }
        }
        function hitPoint(x, y) {
            for (var i = 0; i < editor.points.length; ++i) {
                var q = toPx(editor.points[i])
                if (Math.abs(q.x - x) < 9 && Math.abs(q.y - y) < 9) return i
            }
            return -1
        }

        onPaint: {
            var ctx = getContext("2d")
            ctx.reset()
            ctx.fillStyle = "#202020"
            ctx.fillRect(0, 0, width, height)
            // grid
            ctx.strokeStyle = "#3a3a3a"; ctx.lineWidth = 1
            for (var g = 0; g <= 4; ++g) {
                var gx = pad + g / 4 * (width - 2 * pad), gy = pad + g / 4 * (height - 2 * pad)
                ctx.beginPath(); ctx.moveTo(gx, pad); ctx.lineTo(gx, height - pad); ctx.stroke()
                ctx.beginPath(); ctx.moveTo(pad, gy); ctx.lineTo(width - pad, gy); ctx.stroke()
            }
            // identity diagonal
            ctx.strokeStyle = "#505050"
            ctx.beginPath(); ctx.moveTo(pad, height - pad); ctx.lineTo(width - pad, pad); ctx.stroke()
            // curve, sampled from the same interpolator the LUT uses
            var samples = app.colorGrade.curveSamples(editor.channel, 96)
            ctx.strokeStyle = editor.channelColors[editor.channel]; ctx.lineWidth = 2
            ctx.beginPath()
            for (var i = 0; i < samples.length; ++i) {
                var p = toPx({ x: i / (samples.length - 1), y: samples[i] })
                if (i === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y)
            }
            ctx.stroke()
            // points
            for (var k = 0; k < editor.points.length; ++k) {
                var c = toPx(editor.points[k])
                ctx.fillStyle = (k === dragIndex) ? "#ffffff" : editor.channelColors[editor.channel]
                ctx.beginPath(); ctx.arc(c.x, c.y, 5, 0, 2 * Math.PI); ctx.fill()
            }
        }

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton
            hoverEnabled: true
            cursorShape: canvas.hitPoint(mouseX, mouseY) >= 0 ? Qt.SizeAllCursor : Qt.CrossCursor
            onPressed: (mouse) => {
                var i = canvas.hitPoint(mouse.x, mouse.y)
                if (mouse.button === Qt.RightButton) {
                    if (i > 0 && i < editor.points.length - 1) {
                        var pts = editor.points.slice(); pts.splice(i, 1); editor.points = pts; editor.commit()
                    }
                    return
                }
                canvas.dragIndex = i
                canvas.requestPaint()
            }
            onDoubleClicked: (mouse) => {
                if (mouse.button !== Qt.LeftButton || canvas.hitPoint(mouse.x, mouse.y) >= 0) return
                var np = canvas.fromPx(mouse.x, mouse.y)
                var pts = editor.points.slice(); pts.push(np)
                pts.sort(function (a, b) { return a.x - b.x })
                editor.points = pts; editor.commit()
            }
            onPositionChanged: (mouse) => {
                if (canvas.dragIndex < 0) return
                var np = canvas.fromPx(mouse.x, mouse.y)
                var pts = editor.points.slice()
                var i = canvas.dragIndex
                if (i === 0) np.x = 0
                else if (i === pts.length - 1) np.x = 1
                else {
                    // keep ordering: stay strictly between the neighbours
                    np.x = Math.max(pts[i - 1].x + 0.005, Math.min(pts[i + 1].x - 0.005, np.x))
                }
                pts[i] = np
                editor.points = pts
                editor.commit()
            }
            onReleased: { canvas.dragIndex = -1; canvas.requestPaint() }
        }
    }

    RowLayout {
        Layout.fillWidth: true
        Label {
            text: qsTr("Double-click to add a point, right-click to remove")
            font.pixelSize: 10
            color: "#888"
            Layout.fillWidth: true
            elide: Text.ElideRight
        }
        Button {
            text: qsTr("Reset curve")
            font.pixelSize: 11
            onClicked: app.colorGrade.resetCurve(editor.channel)
        }
    }
}
