import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root

    property int currentFrameIndex: -1

    property real tableWidth: 0
    readonly property int minColWidth: 20
    readonly property int handleHit: 6

    function colWidth(i) {
        if (i === columns.count - 1) {
            var s = 0
            for (var k = 0; k < columns.count - 1; k++)
                s += columns.get(k).width
            return Math.max(minColWidth, tableWidth - s)
        }
        return columns.get(i).width
    }

    function colX(i) {
        var x = 0
        for (var k = 0; k < i; k++)
            x += colWidth(k)
        return x
    }

    ListModel {
        id: columns
        ListElement { title: "No.";          role: "no";    width: 50;  hAlign: 2; color: "#9cdcfe" }
        ListElement { title: "Time";         role: "time";  width: 80;  hAlign: 1; color: "#cccccc" }
        ListElement { title: "Ch";           role: "chIdx"; width: 40;  hAlign: 4; color: "#b5cea8" }
	ListElement { title: "RSSI";         role: "rssi";  width: 70;  hAlign: 1; color: "#dcdcaa" }
        ListElement { title: "Protocol";     role: "proto"; width: 70;  hAlign: 1; color: "#ce9178" }
	ListElement { title: "Address";      role: "addr";  width: 110; hAlign: 1; color: "#569cd6" }
	ListElement { title: "Source";       role: "src";   width: 140; hAlign: 1; color: "#cccccc" }
	ListElement { title: "Destination";  role: "dst";   width: 140; hAlign: 1; color: "#cccccc" }
	ListElement { title: "Type";         role: "type";  width: 150; hAlign: 1; color: "#c586c0" }
        ListElement { title: "Info";         role: "info";  width: 200; hAlign: 1; color: "#cccccc" }
    }

    SplitView {
        id: verticalSplit
        anchors.fill: parent
        orientation: Qt.Vertical
        handle: Rectangle {
            implicitHeight: 4
            color: SplitHandle.pressed ? "#444" : (SplitHandle.hovered ? "#333" : "#222")
        }

        Rectangle {
            id: frameListPane
            color: "#1e1e1e"
            SplitView.preferredHeight: parent.height * 0.5
            SplitView.minimumHeight: 80
            onWidthChanged: root.tableWidth = width

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                Rectangle {
                    id: headerRow
                    Layout.fillWidth: true
                    Layout.preferredHeight: 24
                    color: "#2d2d2d"
                    clip: true

                    property int resizeIndex: -1
                    property real startX: 0
                    property var startWidths: []

                    function boundaryX(i) {
                        var x = 0
                        for (var k = 0; k <= i; k++)
                            x += colWidth(k)
                        return x
                    }

                    function beginResize(mx) {
                        var best = -1, bestDist = handleHit
                        for (var i = 0; i < columns.count - 1; i++) {
                            var d = Math.abs(mx - boundaryX(i))
                            if (d < bestDist) { bestDist = d; best = i }
                        }
                        if (best >= 0) {
                            resizeIndex = best
                            startX = mx
                            var w0 = columns.get(best).width
                            var w1 = best < columns.count - 2 ? columns.get(best + 1).width : 0
                            startWidths = [w0, w1]
                        }
                    }

                    function applyResize(mx) {
                        if (resizeIndex < 0) return
                        var dx = mx - startX
                        var w0 = startWidths[0]
                        var newW = w0 + dx
                        if (newW < minColWidth) { newW = minColWidth; dx = minColWidth - w0 }
                        if (resizeIndex < columns.count - 2) {
                            var w1 = startWidths[1]
                            var newW1 = w1 - dx
                            if (newW1 < minColWidth) { newW1 = minColWidth; dx = w1 - minColWidth; newW = w0 + dx }
                            columns.setProperty(resizeIndex, "width", newW)
                            columns.setProperty(resizeIndex + 1, "width", newW1)
                        } else {
                            if (newW > tableWidth - minColWidth) newW = tableWidth - minColWidth
                            columns.setProperty(resizeIndex, "width", newW)
                        }
                    }

                    Repeater {
                        model: columns
                        Rectangle {
                            x: colX(index)
                            width: colWidth(index)
                            height: headerRow.height
                            color: "transparent"
                            Label {
                                anchors.fill: parent
                                text: model.title
                                color: "#cccccc"
                                horizontalAlignment: model.hAlign
                                verticalAlignment: Text.AlignVCenter
                                font.bold: true
                                leftPadding: 8
                                rightPadding: 8
                                elide: Text.ElideRight
                                clip: true
                            }
                            Rectangle {
                                visible: index < columns.count - 1
                                anchors.right: parent.right
                                anchors.top: parent.top
                                anchors.bottom: parent.bottom
                                width: 1
                                color: "#000000"
                            }
                        }
                    }

                    MouseArea {
                        id: headerMA
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: headerRow.resizeIndex >= 0 ? Qt.SplitHCursor : (nearBoundary(mouseX) ? Qt.SplitHCursor : Qt.ArrowCursor)

                        function nearBoundary(mx) {
                            for (var i = 0; i < columns.count - 1; i++)
                                if (Math.abs(mx - headerRow.boundaryX(i)) < handleHit) return true
                            return false
                        }

                        onPressed: function (mouse) {
                            headerRow.beginResize(mouse.x)
                        }
                        onPositionChanged: function (mouse) {
                            if (headerRow.resizeIndex >= 0)
                                headerRow.applyResize(mouse.x)
                        }
                        onReleased: headerRow.resizeIndex = -1
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 1
                    color: "#000000"
                }

                ListView {
                    id: frameListView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    model: ListModel {
                        ListElement { no: 1; time: "0.000000"; chIdx: 37; rssi: "-42.0"; proto: "LE"; addr: "0x9E8B33"; src: "11:22:33:44:55:66"; dst: "Broadcast"; type: "ADV_IND"; info: "Connectable undirected" }
                        ListElement { no: 2; time: "0.001204"; chIdx: 38; rssi: "-55.1"; proto: "LE"; addr: "0x9E8B33"; src: "11:22:33:44:55:66"; dst: "Broadcast"; type: "ADV_NONCONN_IND"; info: "" }
                        ListElement { no: 3; time: "0.002488"; chIdx: 39; rssi: "-61.3"; proto: "LE"; addr: "0x7F3A2C"; src: "AA:BB:CC:DD:00:01"; dst: "11:22:33:44:55:66"; type: "SCAN_REQ"; info: "" }
                        ListElement { no: 4; time: "0.003712"; chIdx: 39; rssi: "-60.8"; proto: "LE"; addr: "0x7F3A2C"; src: "11:22:33:44:55:66"; dst: "AA:BB:CC:DD:00:01"; type: "SCAN_RSP"; info: "" }
                        ListElement { no: 5; time: "0.010033"; chIdx: 37; rssi: "-44.0"; proto: "LE"; addr: "0x7F3A2C"; src: "AA:BB:CC:DD:00:01"; dst: "11:22:33:44:55:66"; type: "CONNECT_IND"; info: "" }
                        ListElement { no: 6; time: "0.012440"; chIdx: 2;  rssi: "-48.2"; proto: "BR/EDR"; addr: "0xAB3D"; src: "Central"; dst: "Peripheral 3"; type: "DM3"; info: "Clock unknown" }
                        ListElement { no: 7; time: "0.014011"; chIdx: 2;  rssi: "-49.0"; proto: "BR/EDR"; addr: "0xAB3D"; src: "Peripheral 3"; dst: "Central"; type: "DM3"; info: "Clock unknown" }
                    }

                    delegate: Rectangle {
                        id: rowDelegate
                        width: frameListView.width
                        height: 22
                        color: frameListView.currentIndex === index ? "#094771" : (index % 2 === 0 ? "#1e1e1e" : "#252525")
                        clip: true

                        property var cells: [no, time, chIdx, rssi, proto, addr, src, dst, type, info]

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                frameListView.currentIndex = index
                                root.currentFrameIndex = index
                            }
                        }

                        Repeater {
                            model: columns
                            Rectangle {
                                x: colX(index)
                                width: colWidth(index)
                                height: rowDelegate.height
                                color: "transparent"
                                Label {
                                    anchors.fill: parent
                                    text: rowDelegate.cells[index]
                                    color: columns.get(index).color
                                    horizontalAlignment: columns.get(index).hAlign
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: 8
                                    rightPadding: 8
                                    elide: Text.ElideRight
                                    clip: true
                                }
                            }
                        }
                    }
                }
            }
        }

        SplitView {
            id: horizontalSplit
            SplitView.preferredHeight: parent.height * 0.5
            SplitView.minimumHeight: 80
            orientation: Qt.Horizontal
            handle: Rectangle {
                implicitWidth: 4
                color: SplitHandle.pressed ? "#444" : (SplitHandle.hovered ? "#333" : "#222")
            }

            Rectangle {
                id: frameInfoPane
                color: "#1e1e1e"
                SplitView.preferredWidth: parent.width * 0.4
                SplitView.minimumWidth: 120

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 0

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 24
                        color: "#2d2d2d"
                        Label {
                            anchors.fill: parent
                            text: "Frame Info"
                            color: "#cccccc"
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 8
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 1
                        color: "#000000"
                    }

                    ListView {
                        id: frameInfoView
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: ListModel {
                            ListElement { field: "PDU Type"; value: "ADV_IND" }
                            ListElement { field: "Channel"; value: "37" }
                            ListElement { field: "Access Address"; value: "9E8B33" }
                            ListElement { field: "AdvA"; value: "11:22:33:44:55:66" }
                            ListElement { field: "TargetA"; value: "—" }
                            ListElement { field: "AdvData Length"; value: "6" }
                            ListElement { field: "CRC"; value: "0xA1B2C3 (PASS)" }
                            ListElement { field: "RSSI"; value: "-42.0 dBm" }
                        }

                        delegate: Rectangle {
                            width: frameInfoView.width
                            height: 22
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: 8

                                Label {
                                    text: field + ":"
                                    color: "#9cdcfe"
                                    Layout.preferredWidth: 140
                                    horizontalAlignment: Text.AlignRight
                                    verticalAlignment: Text.AlignVCenter
                                    rightPadding: 8
                                }
                                Label {
                                    text: value
                                    color: "#cccccc"
                                    Layout.fillWidth: true
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }
                        }
                    }
                }
            }

            Rectangle {
                id: hexPane
                color: "#1e1e1e"
                SplitView.preferredWidth: parent.width * 0.6
                SplitView.minimumWidth: 120

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 0

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 24
                        color: "#2d2d2d"
                        Label {
                            anchors.fill: parent
                            text: "Hex"
                            color: "#cccccc"
                            verticalAlignment: Text.AlignVCenter
                            leftPadding: 8
                            font.bold: true
                        }
                    }

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 1
                        color: "#000000"
                    }

                    ListView {
                        id: hexView
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: ListModel {
                            ListElement { offset: "0000"; hex: "AA BB CC DD EE FF 00 11  22 33 44 55 66 77 88 99"; ascii: "............\"  3DUfw.." }
                            ListElement { offset: "0010"; hex: "01 02 03 04 05 06 07 08  09 0A 0B 0C 0D 0E 0F 10"; ascii: "................" }
                            ListElement { offset: "0020"; hex: "A1 B2 C3 D4 E5 F6 07 18  29 3A 4B 5C 6D 7E 8F 90"; ascii: ".........):K\\.~." }
                            ListElement { offset: "0030"; hex: "FF EE DD CC BB AA 99 88  77 66 55 44 33 22 11 00"; ascii: "........wfUD3\".." }
                        }

                        delegate: Rectangle {
                            width: hexView.width
                            height: 22
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: 0

                                Label {
                                    text: offset
                                    color: "#858585"
                                    Layout.preferredWidth: 50
                                    horizontalAlignment: Text.AlignRight
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                    leftPadding: 8
                                }
                                Label {
                                    text: hex
                                    color: "#d4d4d4"
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                    leftPadding: 16
                                }
                                Label {
                                    text: ascii
                                    color: "#7c7c7c"
                                    Layout.fillWidth: true
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                    leftPadding: 16
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
