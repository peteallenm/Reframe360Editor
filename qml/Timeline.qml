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

Pane {
    id: timeline
    // Targeting SDK 36 means the app draws edge-to-edge, UNDER the system
    // bars. In landscape the navigation bar sits on one SIDE, and it was
    // covering the play button. SafeArea gives the real insets for whichever
    // rotation, so this is right both ways up rather than a fixed nudge.
    //
    // Guarded on the platform because SafeArea is QtQuick 6.9+, and the
    // desktop here builds against distro Qt 6.4: an unresolved attached
    // property makes the arithmetic NaN, which collapsed the whole footer.
    // The conditional short-circuits before SafeArea is ever looked up.
    // Four things share this strip -- the export trim handles, the scrub
    // slider, the track bars and the keyframe dots -- and a fingertip is wider
    // than any of them. Rather than make the strip taller, each gets its own
    // horizontal band of TOUCH area while the drawing stays exactly where it
    // was: the bands are what the finger hits, the visuals are what the eye
    // sees. Bands are declared here so they cannot drift apart and start
    // overlapping again.
    //
    // The trim handles were the worst of them: 7x12 px with no padding at all,
    // sitting on top of a slider that will happily take the press instead.
    readonly property real grabPad: Qt.platform.os === "android" ? 12 : 7
    readonly property real bandTrimBottom: 18   // trim handles own everything above
    readonly property real bandTrackBottom: 30  // ... then track bars and their grips
                                                // ... then keyframe dots, to the bottom
    readonly property real safeLeft: Qt.platform.os === "android" ? SafeArea.margins.left : 0
    readonly property real safeRight: Qt.platform.os === "android" ? SafeArea.margins.right : 0
    readonly property real safeBottom: Qt.platform.os === "android" ? SafeArea.margins.bottom : 0

    height: 60 + safeBottom
    padding: 8
    leftPadding: 8 + safeLeft
    rightPadding: 8 + safeRight
    bottomPadding: 8 + safeBottom

    Dialog {
        id: clearKeyframesDialog
        title: qsTr("Clear keyframes?")
        modal: true
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(380, (parent ? parent.width : 380) - 24)
        standardButtons: Dialog.Yes | Dialog.No
        onAccepted: app.keyframes.clearKeyframes()
        Label {
            // FIXED width: a sole Label becomes the Dialog's contentItem, so
            // any binding from the dialog's own geometry (parent.width,
            // availableWidth) loops through implicitHeight. The dialog is
            // capped at 380 wide, so 330 always fits inside the padding.
            width: 330
            wrapMode: Text.Wrap
            text: qsTr("Remove all %1 keyframes from this clip? This cannot be undone.")
                    .arg(app.keyframes.count)
        }
    }

    RowLayout {
        anchors.fill: parent
        spacing: 8

        ToolButton {
            id: playButton
            // Drawn, not icon.name and not a glyph:
            //   * icon.name resolves through the freedesktop icon theme, which
            //     exists on desktop Linux and NOT on Android -- the button
            //     rendered completely empty there.
            //   * U+275A (heavy vertical bar) for pause is missing from the
            //     Edge 40's default font and came out as two tofu boxes.
            // Two Rectangles and a Canvas triangle depend on no font or theme.
            implicitWidth: 40
            implicitHeight: 36
            onClicked: app.isPlaying = !app.isPlaying
            enabled: app.videoPath !== ""
            ToolTip.text: app.isPlaying ? qsTr("Pause") : qsTr("Play")
            ToolTip.visible: hovered

            contentItem: Item {
                implicitWidth: 40
                implicitHeight: 36

                Row {
                    anchors.centerIn: parent
                    spacing: 4
                    visible: app.isPlaying
                    Repeater {
                        model: 2
                        Rectangle {
                            width: 5; height: 16; radius: 1
                            color: playButton.enabled ? Material.foreground : Material.hintTextColor
                        }
                    }
                }

                Canvas {
                    anchors.centerIn: parent
                    width: 16; height: 16
                    visible: !app.isPlaying
                    // Repaint when the tint changes (enabled/disabled).
                    property color tint: playButton.enabled ? Material.foreground : Material.hintTextColor
                    onTintChanged: requestPaint()
                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.reset()
                        ctx.fillStyle = tint
                        ctx.beginPath()
                        ctx.moveTo(1, 0); ctx.lineTo(width - 1, height / 2); ctx.lineTo(1, height)
                        ctx.closePath(); ctx.fill()
                    }
                }
            }
        }

        ToolButton {
            text: "+ KF"   // no icon.name: no icon theme on Android (see play button)
            ToolTip.text: "Add keyframe at current time"
            ToolTip.visible: hovered
            onClicked: app.addKeyframeAtCurrent()
            enabled: app.videoPath !== ""
        }

        ToolButton {
            text: qsTr("Clear")
            font.pixelSize: 12
            enabled: app.videoPath !== "" && app.keyframes.count > 0
            onClicked: clearKeyframesDialog.open()
            ToolTip.text: qsTr("Remove every keyframe from this clip")
            ToolTip.visible: hovered
        }

        Item {
            id: markerRow
            Layout.fillWidth: true
            Layout.fillHeight: true

            Slider {
                id: timeSlider
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.verticalCenter: parent.verticalCenter
                from: 0
                to: app.duration
                value: app.currentTime
                enabled: app.videoPath !== ""

                onMoved: {
                    app.currentTime = value
                }
            }

            // ---- Export trim range (in/out markers) ----
            // A shaded band with two draggable handles at the top of the
            // timeline marks the section that will be exported. The handles
            // are positioned by Binding so external changes (loading a new
            // clip, restoring the sidecar's in/out markers) always re-sync
            // them; dragging writes back to app.exportStart/exportEnd, which
            // are persisted in the per-video keyframe sidecar.
            Item {
                id: trimStrip
                anchors.top: parent.top
                anchors.topMargin: 2
                height: 12
                anchors.left: parent.left
                anchors.right: parent.right
                visible: app.videoPath !== ""

                Rectangle {
                    id: rangeShade
                    y: 1
                    height: 6
                    radius: 2
                    color: Material.accent
                    opacity: 0.35
                    Binding {
                        target: rangeShade
                        property: "x"
                        value: trimStrip.width * (app.exportStart / Math.max(app.duration, 0.001))
                    }
                    Binding {
                        target: rangeShade
                        property: "width"
                        value: Math.max(0, trimStrip.width * ((app.exportEnd - app.exportStart) / Math.max(app.duration, 0.001)))
                    }
                }

                Rectangle {
                    id: inHandle
                    objectName: "trimInHandle"
                    width: 7
                    height: 12
                    radius: 2
                    color: Material.accent
                    border.color: "white"
                    border.width: 1
                    onXChanged: {
                        if (dragIn.drag.active)
                            app.exportStart = Math.max(0, Math.min(app.exportEnd, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: inHandle
                        property: "x"
                        when: !dragIn.drag.active
                        value: trimStrip.width * (app.exportStart / Math.max(app.duration, 0.001)) - 3
                    }
                    MouseArea {
                        id: dragIn
                        // Up to the top of the strip and down to the trim
                        // band's edge, and well past the handle sideways. The
                        // slider underneath keeps everything this does not
                        // claim, which is the whole timeline bar a fingertip's
                        // width around each handle.
                        anchors.fill: parent
                        anchors.topMargin: -8
                        anchors.bottomMargin: -(timeline.bandTrimBottom - 14)
                        anchors.leftMargin: -timeline.grabPad
                        anchors.rightMargin: -timeline.grabPad
                        drag.target: parent
                        drag.axis: Drag.XAxis
                        drag.minimumX: -3
                        drag.maximumX: trimStrip.width - 4
                        cursorShape: Qt.SizeHorCursor
                    }
                }

                Rectangle {
                    id: outHandle
                    objectName: "trimOutHandle"
                    width: 7
                    height: 12
                    radius: 2
                    color: Material.accent
                    border.color: "white"
                    border.width: 1
                    onXChanged: {
                        if (dragOut.drag.active)
                            app.exportEnd = Math.max(app.exportStart, Math.min(app.duration, (x + 3) / trimStrip.width * app.duration))
                    }
                    Binding {
                        target: outHandle
                        property: "x"
                        when: !dragOut.drag.active
                        value: trimStrip.width * (app.exportEnd / Math.max(app.duration, 0.001)) - 3
                    }
                    MouseArea {
                        id: dragOut
                        // Up to the top of the strip and down to the trim
                        // band's edge, and well past the handle sideways. The
                        // slider underneath keeps everything this does not
                        // claim, which is the whole timeline bar a fingertip's
                        // width around each handle.
                        anchors.fill: parent
                        anchors.topMargin: -8
                        anchors.bottomMargin: -(timeline.bandTrimBottom - 14)
                        anchors.leftMargin: -timeline.grabPad
                        anchors.rightMargin: -timeline.grabPad
                        drag.target: parent
                        drag.axis: Drag.XAxis
                        drag.minimumX: -3
                        drag.maximumX: trimStrip.width - 4
                        cursorShape: Qt.SizeHorCursor
                    }
                }
            }

            // Keyframe markers
            // Tracks, drawn under the keyframe dots and deliberately unlike
            // them: a bar spanning what the track covers, with a bracket at
            // each end. Keyframes are things you placed; a track is a stretch
            // of time something else is driving, and the two should never be
            // mistaken for each other.
            Repeater {
                id: trackBars
                model: trackSpanList

                Item {
                    id: trackBar
                    required property var modelData
                    required property int index

                    // The ends follow the pointer from here while a bracket is
                    // being dragged; the model is only told on release.
                    // Committing per pixel would re-resolve every track and
                    // rewrite the sidecar on every mouse move.
                    // These start as bindings on the model. A drag assigns to
                    // them, which breaks the binding; releasing restores it, so
                    // the model is the truth again the moment the drag ends --
                    // including when the app REFUSES the trim, where nothing
                    // else would have snapped the bracket back.
                    property real liveStart: modelData.start
                    property real liveEnd: modelData.end
                    function followModel() {
                        liveStart = Qt.binding(() => modelData.start)
                        liveEnd = Qt.binding(() => modelData.end)
                    }

                    readonly property real pxPerSec: (markerRow.width - 8)
                                                     / Math.max(app.duration, 0.001)
                    // Shortest span a trim may leave; matches kMinTrackSpan in
                    // app.cpp, below which a track resolves to a single
                    // keyframe and stops following anything.
                    readonly property real minGapPx: pxPerSec * 0.2
                    readonly property real inX: pxPerSec * (liveStart - modelData.fullStart)
                    readonly property real outX: pxPerSec * (liveEnd - modelData.fullStart)

                    // The item covers everything that was MEASURED. The solid
                    // bar inside covers what is in use, so the gap between them
                    // shows what has been trimmed and that there is still track
                    // there to drag back out.
                    x: pxPerSec * modelData.fullStart
                    y: parent.height - 22
                    width: Math.max(pxPerSec * (modelData.fullEnd - modelData.fullStart), 2) + 8
                    height: 10

                    // What was tracked but is trimmed away.
                    Rectangle {
                        x: 4; y: 4
                        width: Math.max(parent.width - 8, 2)
                        height: 2
                        color: "#f0a030"
                        opacity: 0.25
                        visible: modelData.trimmed
                    }
                    // The span actually driving the view.
                    Rectangle {
                        x: 4 + trackBar.inX
                        y: 3
                        width: Math.max(trackBar.outX - trackBar.inX, 2)
                        height: 4
                        radius: 2
                        color: "#f0a030"
                        opacity: 0.75
                    }

                    MouseArea {
                        anchors.fill: parent
                        anchors.topMargin: -(trackBar.y - timeline.bandTrimBottom)
                        anchors.bottomMargin: -(timeline.bandTrackBottom - (trackBar.y + trackBar.height))
                        anchors.leftMargin: -6
                        anchors.rightMargin: -6
                        cursorShape: Qt.PointingHandCursor
                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        onClicked: (mouse) => {
                            if (mouse.button === Qt.RightButton)
                                timeline.openItemMenu(mapToItem(markerRow, mouse.x, mouse.y).x)
                            else
                                app.currentTime = trackBar.liveStart
                        }
                        // Touch has no right button (same reason as the
                        // keyframe markers).
                        onPressAndHold: (mouse) =>
                            timeline.openItemMenu(mapToItem(markerRow, mouse.x, mouse.y).x)
                        ToolTip.text: modelData.lost
                            ? qsTr("Track %1: %2 s to %3 s, then lost the subject — drag either bracket to trim")
                                  .arg(index + 1).arg(trackBar.liveStart.toFixed(1))
                                  .arg(trackBar.liveEnd.toFixed(1))
                            : qsTr("Track %1: %2 s to %3 s — drag either bracket to trim")
                                  .arg(index + 1).arg(trackBar.liveStart.toFixed(1))
                                  .arg(trackBar.liveEnd.toFixed(1))
                        ToolTip.visible: containsMouse
                        hoverEnabled: true
                    }

                    // The brackets are the handles. Declared after the bar's
                    // own MouseArea, and above it, so a press on a bracket
                    // trims rather than seeking.
                    Rectangle {
                        id: startGrip
                        y: 0; width: 3; height: 10; radius: 1
                        color: "#f0a030"
                        z: 2
                        onXChanged: {
                            if (startDrag.drag.active)
                                trackBar.liveStart = Math.max(trackBar.modelData.fullStart,
                                    trackBar.modelData.fullStart + (x - 2) / trackBar.pxPerSec)
                        }
                        Binding {
                            target: startGrip
                            property: "x"
                            when: !startDrag.drag.active
                            value: 2 + trackBar.inX
                        }
                        MouseArea {
                            id: startDrag
                            anchors.fill: parent
                            // A 3 px bracket needs a fingertip's worth of
                            // target. Bounded to the track band: a keyframe
                            // very often sits exactly where a track ends, and
                            // the dot -- drawn later, so on top -- used to take
                            // the press and nothing would drag.
                            anchors.topMargin: -(trackBar.y - timeline.bandTrimBottom)
                            anchors.bottomMargin: -(timeline.bandTrackBottom - (trackBar.y + trackBar.height))
                            anchors.leftMargin: -timeline.grabPad
                            anchors.rightMargin: -timeline.grabPad
                            drag.target: parent
                            drag.axis: Drag.XAxis
                            drag.minimumX: 2
                            drag.maximumX: Math.max(2, 2 + trackBar.outX - trackBar.minGapPx)
                            cursorShape: Qt.SizeHorCursor
                            preventStealing: true
                            onReleased: {
                                app.setTrackTrim(trackBar.index, trackBar.liveStart,
                                                 trackBar.liveEnd)
                                trackBar.followModel()
                            }
                            onCanceled: trackBar.followModel()
                        }
                    }
                    // End bracket: red when the tracker lost the subject there,
                    // so a track that ran out is visibly different from one
                    // that gave up -- or from one you trimmed yourself.
                    Rectangle {
                        id: endGrip
                        y: 0; width: 3; height: 10; radius: 1
                        color: modelData.lost ? "#e05050" : "#f0a030"
                        z: 2
                        onXChanged: {
                            if (endDrag.drag.active)
                                trackBar.liveEnd = Math.min(trackBar.modelData.fullEnd,
                                    trackBar.modelData.fullStart + (x - 3) / trackBar.pxPerSec)
                        }
                        Binding {
                            target: endGrip
                            property: "x"
                            when: !endDrag.drag.active
                            value: 3 + trackBar.outX
                        }
                        MouseArea {
                            id: endDrag
                            anchors.fill: parent
                            anchors.topMargin: -(trackBar.y - timeline.bandTrimBottom)
                            anchors.bottomMargin: -(timeline.bandTrackBottom - (trackBar.y + trackBar.height))
                            anchors.leftMargin: -timeline.grabPad
                            anchors.rightMargin: -timeline.grabPad
                            drag.target: parent
                            drag.axis: Drag.XAxis
                            drag.minimumX: Math.min(trackBar.width - 5,
                                                    3 + trackBar.inX + trackBar.minGapPx)
                            drag.maximumX: trackBar.width - 5
                            cursorShape: Qt.SizeHorCursor
                            preventStealing: true
                            onReleased: {
                                app.setTrackTrim(trackBar.index, trackBar.liveStart,
                                                 trackBar.liveEnd)
                                trackBar.followModel()
                            }
                            onCanceled: trackBar.followModel()
                        }
                    }
                }
            }

            Repeater {
                model: app.keyframes
                Rectangle {
                    x: (parent.width - 8) * (kfTime / Math.max(app.duration, 0.001))
                    y: parent.height - 14
                    width: 8
                    height: 8
                    radius: 4
                    color: Material.accent
                    border.color: "white"
                    border.width: 1

                    MouseArea {
                        // An 8 px marker is an impossible touch target, so the
                        // hit area extends well past the dot -- but downwards
                        // and sideways only. Reaching up as well is what let a
                        // dot swallow the press meant for a track end sitting
                        // at the same time.
                        anchors.fill: parent
                        anchors.topMargin: 0
                        anchors.bottomMargin: -timeline.grabPad
                        anchors.leftMargin: -timeline.grabPad
                        anchors.rightMargin: -timeline.grabPad
                        cursorShape: Qt.PointingHandCursor
                        acceptedButtons: Qt.LeftButton | Qt.RightButton
                        onClicked: (mouse) => {
                            if (mouse.button === Qt.RightButton)
                                timeline.openItemMenu(mapToItem(markerRow, mouse.x, mouse.y).x)
                            else
                                app.currentTime = kfTime
                        }
                        onDoubleClicked: {
                            kfEditDialog.openFor(index)
                        }
                        // Touch has no right button, so a long press opens the
                        // same menu: without this there was no way at all to
                        // delete a keyframe on Android.
                        onPressAndHold: (mouse) =>
                            timeline.openItemMenu(mapToItem(markerRow, mouse.x, mouse.y).x)
                    }
                }
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

    // Context menu for keyframe markers (right-click)
    // Refreshed wholesale on tracksChanged: a track is added or removed as a
    // unit, never edited in place.
    property var trackSpanList: []
    Connections {
        target: app
        function onTracksChanged() { trackSpanList = app.trackSpans() }
    }
    Component.onCompleted: trackSpanList = app.trackSpans()

    // One menu for everything under the press. A keyframe dot and a track
    // bracket routinely land on the same few pixels, and whichever happened to
    // be on top swallowed the click -- so you could not delete a keyframe that
    // sat on a track, or reach a track whose end a keyframe covered. Rather
    // than make the user guess which one they hit, offer both.
    Menu {
        id: itemMenu
        property var entries: []

        Instantiator {
            model: itemMenu.entries
            delegate: MenuItem {
                text: modelData.label
                onTriggered: itemMenu.run(modelData)
            }
            onObjectAdded: (index, object) => itemMenu.insertItem(index, object)
            onObjectRemoved: (index, object) => itemMenu.removeItem(object)
        }

        function run(e) {
            if (e.act === "kfEdit") kfEditDialog.openFor(e.idx)
            else if (e.act === "kfDel") app.keyframes.removeKeyframe(e.idx)
            else if (e.act === "kfGo") app.currentTime = e.time
            else if (e.act === "trkGo") app.currentTime = e.time
            else if (e.act === "trkReset") app.resetTrackTrim(e.idx)
            else if (e.act === "trkZoom") app.setTrackZoom(e.idx, app.fov)
            else if (e.act === "trkFollow") app.setTrackFollowSize(e.idx, e.on)
            else if (e.act === "trkDel") app.removeTrack(e.idx)
        }
    }

    // Everything whose marker is within a fingertip of `px`, as menu entries.
    function itemsNear(px) {
        const usable = Math.max(markerRow.width - 8, 1)
        const t = px / usable * app.duration
        const slop = 14 / usable * app.duration       // ~a fingertip, in seconds
        let e = []
        for (let i = 0; i < app.keyframes.count; ++i) {
            const kt = app.keyframes.data(app.keyframes.index(i, 0), 0x0101)  // TimeRole
            if (Math.abs(kt - t) > slop) continue
            e.push({ label: qsTr("Keyframe %1 (%2 s) — edit…").arg(i + 1).arg(kt.toFixed(1)),
                     act: "kfEdit", idx: i, time: kt })
            e.push({ label: qsTr("Keyframe %1 — delete").arg(i + 1), act: "kfDel", idx: i, time: kt })
        }
        const spans = app.trackSpans()
        for (let i = 0; i < spans.length; ++i) {
            const s = spans[i]
            // The whole bar counts, not just its ends: a keyframe usually sits
            // where a track stops, which is exactly where both are wanted.
            if (t < s.start - slop || t > s.end + slop) continue
            e.push({ label: qsTr("Track %1 (%2–%3 s) — go to start")
                          .arg(i + 1).arg(s.start.toFixed(1)).arg(s.end.toFixed(1)),
                     act: "trkGo", idx: i, time: s.start })
            if (s.trimmed)
                e.push({ label: qsTr("Track %1 — use the whole track again").arg(i + 1),
                         act: "trkReset", idx: i, time: s.start })
            e.push({ label: qsTr("Track %1 — zoom: use this view (%2° now, track is %3°)")
                          .arg(i + 1).arg(Math.round(app.fov)).arg(Math.round(s.zoom)),
                     act: "trkZoom", idx: i, time: s.start })
            e.push({ label: s.followSize
                        ? qsTr("Track %1 — stop following the subject's size").arg(i + 1)
                        : qsTr("Track %1 — follow the subject's size").arg(i + 1),
                     act: "trkFollow", idx: i, time: s.start, on: !s.followSize })
            e.push({ label: qsTr("Track %1 — delete").arg(i + 1), act: "trkDel", idx: i,
                     time: s.start })
        }
        return e
    }

    function openItemMenu(px) {
        const e = itemsNear(px)
        if (e.length === 0)
            return
        itemMenu.entries = e
        itemMenu.popup()
    }

    // Hidden dialog for editing keyframe
    Dialog {
        id: kfEditDialog
        title: "Edit Keyframe"
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel | Dialog.Discard
        width: 300

        property int editIndex: -1

        function openFor(idx) {
            editIndex = idx
            // Fetch each value by role: the role context properties (kfTime,
            // kfYaw, ...) only exist inside the Repeater delegate scope, not
            // here, so read them through the model's data().
            timeField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0101).toFixed(2)   // TimeRole
            yawField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0102).toFixed(1)     // YawRole
            pitchField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0103).toFixed(1)   // PitchRole
            rollField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0104).toFixed(1)    // RollRole
            fovField.text = app.keyframes.data(app.keyframes.index(idx, 0), 0x0105).toFixed(0)     // FovRole
            open()
        }

        ColumnLayout {
            anchors.fill: parent
            spacing: 8

            // Numbers only: on Android this brings up a numeric keypad instead
            // of a full keyboard, and stops the keyboard trying to
            // autocapitalise and autocorrect a decimal.
            RowLayout { Label { text: "Time:"; Layout.preferredWidth: 60 } TextField { id: timeField; Layout.fillWidth: true; inputMethodHints: Qt.ImhFormattedNumbersOnly } }
            RowLayout { Label { text: "Yaw:"; Layout.preferredWidth: 60 } TextField { id: yawField; Layout.fillWidth: true; inputMethodHints: Qt.ImhFormattedNumbersOnly } }
            RowLayout { Label { text: "Pitch:"; Layout.preferredWidth: 60 } TextField { id: pitchField; Layout.fillWidth: true; inputMethodHints: Qt.ImhFormattedNumbersOnly } }
            RowLayout { Label { text: "Roll:"; Layout.preferredWidth: 60 } TextField { id: rollField; Layout.fillWidth: true; inputMethodHints: Qt.ImhFormattedNumbersOnly } }
            RowLayout { Label { text: "FOV:"; Layout.preferredWidth: 60 } TextField { id: fovField; Layout.fillWidth: true; inputMethodHints: Qt.ImhFormattedNumbersOnly } }
        }

        onAccepted: {
            if (editIndex >= 0) {
                app.keyframes.updateKeyframe(editIndex,
                    parseFloat(timeField.text),
                    parseFloat(yawField.text),
                    parseFloat(pitchField.text),
                    parseFloat(rollField.text),
                    parseFloat(fovField.text))
            }
        }
        onDiscarded: {
            if (editIndex >= 0)
                app.keyframes.removeKeyframe(editIndex)
        }
    }
}
