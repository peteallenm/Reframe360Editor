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

    // --- Histogram, drawn behind the curve ---------------------------------
    // The FILLED histogram is the graded output: the frame put through the
    // colour grade and the tone curves, using the same grade.h maths the
    // shader and the CPU exporter run. It moves as you drag a curve point, so
    // you can see what the curve is doing to the image.
    // The faint OUTLINE is the source frame, before any grading, as a
    // reference for where the tones started.
    // Master shows luma with R/G/B outlined over it; a single channel shows
    // just that channel.
    property bool showHistogram: true
    property var histNorm: null      // { src:{r,g,b,luma}, out:{...} } in 0..1

    function refreshHistogram() {
        if (!showHistogram || !editor.visible) return
        var h = app.frameHistogram(512)
        if (!h || !h.samples) { histNorm = null; canvas.requestPaint(); return }
        var keys = ["r", "g", "b", "luma"]

        // Scaling, and why it is done this way.
        //
        // A tone curve RESAMPLES the histogram: wherever the curve is shallow
        // it packs many input levels into one output level, producing spikes
        // several times taller than anything in the source. Two consequences
        // had to be designed around:
        //
        //  1. Scale each histogram by its OWN peak, never by a peak shared
        //     with the other. Sharing meant a spike in the graded histogram
        //     visibly shrank the SOURCE outline -- data that had not changed
        //     at all -- so the reference moved under you while you dragged.
        //  2. Use a TRIMMED peak: ignore the few tallest bins rather than
        //     taking the true maximum. One narrow curve-induced spike would
        //     otherwise flatten the entire rest of the distribution to nothing
        //     the moment the curve went briefly flat. Bins above the trimmed
        //     peak are clamped, so they read as "off the top", which is the
        //     honest way to show a pile-up.
        //
        // Bins 0 and 255 are excluded from the peak entirely: clipped blacks
        // and whites are single huge spikes that say nothing about the shape.
        var TRIM = 5          // of the 254 interior bins
        function peakOf(set) {
            var all = []
            for (var k = 0; k < keys.length; ++k) {
                var a = set[keys[k]]
                for (var i = 1; i < 255; ++i) if (a[i] > 0) all.push(a[i])
            }
            if (all.length === 0) return 1
            all.sort(function (x, y) { return y - x })
            return Math.max(1, all[Math.min(TRIM, all.length - 1)])
        }
        function norm(set) {
            var peak = peakOf(set)
            var o = {}
            for (var kk = 0; kk < keys.length; ++kk) {
                var src = set[keys[kk]], dst = new Array(256)
                for (var j = 0; j < 256; ++j) dst[j] = Math.min(1, src[j] / peak)
                o[keys[kk]] = dst
            }
            return o
        }

        histNorm = { src: norm(h.source), out: norm(h.graded) }
        canvas.requestPaint()
    }

    onShowHistogramChanged: refreshHistogram()
    onVisibleChanged: if (visible) refreshHistogram()

    // Coalesce: frames arrive at up to 30 fps and each refresh marshals 1024
    // bin counts across the QML boundary. Redrawing ~4x a second is plenty.
    Timer {
        id: histTimer
        interval: 250
        repeat: false
        onTriggered: editor.refreshHistogram()
    }
    Connections {
        target: app
        function onHistogramChanged() { if (editor.showHistogram && editor.visible && !histTimer.running) histTimer.start() }
    }

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
    Component.onCompleted: { reload(); refreshHistogram() }
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

    RowLayout {
        Layout.fillWidth: true
        Item { Layout.fillWidth: true }
        CheckBox {
            id: histToggle
            text: qsTr("Histogram")
            checked: editor.showHistogram
            font.pixelSize: 11
            padding: 0
            onToggled: editor.showHistogram = checked
            ToolTip.text: qsTr("Levels of the current frame. Filled = after the grade and curves, so it moves as you edit; outline = the original frame for reference.")
            ToolTip.visible: hovered
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

            // histogram, behind everything else
            if (editor.showHistogram && editor.histNorm) {
                var hs = editor.histNorm.src, ho = editor.histNorm.out
                var x0 = pad, x1 = width - pad, y1 = height - pad
                var span = x1 - x0, hgt = height - 2 * pad

                function histPath(arr, close) {
                    ctx.beginPath()
                    ctx.moveTo(x0, y1)
                    for (var i = 0; i < 256; ++i)
                        ctx.lineTo(x0 + i / 255 * span, y1 - arr[i] * hgt)
                    ctx.lineTo(x1, y1)
                    if (close) ctx.closePath()
                }

                var ch = editor.channel
                if (ch === 0) {
                    ctx.fillStyle = "#4a4a4a"                 // graded luma
                    histPath(ho.luma, true); ctx.fill()
                    ctx.strokeStyle = "#6f6f6f"; ctx.lineWidth = 1
                    histPath(hs.luma, false); ctx.stroke()    // source luma
                    var cols = ["#a85a5a", "#5aa85a", "#5a7ab8"]
                    var chans = [ho.r, ho.g, ho.b]
                    for (var c = 0; c < 3; ++c) {
                        ctx.strokeStyle = cols[c]
                        histPath(chans[c], false); ctx.stroke()
                    }
                } else {
                    var fills = ["", "#5e3333", "#335e33", "#33415e"]
                    var key = (ch === 1) ? "r" : (ch === 2) ? "g" : "b"
                    ctx.fillStyle = fills[ch]
                    histPath(ho[key], true); ctx.fill()       // graded channel
                    ctx.strokeStyle = "#787878"; ctx.lineWidth = 1
                    histPath(hs[key], false); ctx.stroke()    // source channel
                }
            }

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
