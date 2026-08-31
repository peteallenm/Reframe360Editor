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
import QtQuick.Layouts

// The licence notice the GPL asks an interactive program to show: what this
// is, that it carries no warranty, and where its source lives. On a store
// build this is the only place a user can read any of that, and it is also
// where Qt's LGPL attribution is discharged.
Dialog {
    id: about

    property string sourceUrl: "https://github.com/peteallenm/Render360"

    modal: true
    title: qsTr("About Reframe360 Editor")
    anchors.centerIn: parent
    width: Math.min(440, (parent ? parent.width : 440) - 32)
    standardButtons: Dialog.Close

    ColumnLayout {
        width: parent.width
        spacing: 10

        Label {
            Layout.fillWidth: true
            text: qsTr("Reframe360 Editor %1").arg(Qt.application.version)
            font.pixelSize: 18
            font.bold: true
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            text: qsTr("Stabilises, reframes and stitches dual-fisheye 360 footage.")
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            textFormat: Text.RichText
            text: qsTr("Free software under the <b>GNU General Public License, version 3 or later</b>. It comes with ABSOLUTELY NO WARRANTY. The complete source, including everything needed to rebuild this app, is at<br><a href=\"%1\">%1</a>").arg(about.sourceUrl)
            onLinkActivated: (link) => Qt.openUrlExternally(link)
        }

        Label {
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            font.pixelSize: 12
            opacity: 0.75
            text: qsTr("Built on Qt 6 (LGPL v3, shipped as replaceable shared libraries), FFmpeg and x264 (GPL v2 or later), and OpenCV (Apache 2.0).")
        }
    }
}
