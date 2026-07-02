import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root

    property var frameModel: null
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

    function loadDetail(idx) {
        if (!frameModel || idx < 0) {
            frameInfoView.model = []
            hexView.model = []
            return
        }
        frameInfoView.model = frameModel.detailFor(idx)
        hexView.model = frameModel.hexFor(idx)
    }

    function updateAutoFollow() {
        // Considered "at the bottom" when within one row height of the tail.
        var maxContentY = frameListView.contentHeight - frameListView.height
        if (maxContentY < 0)
            maxContentY = 0
        frameListView.autoFollow =
            (frameListView.contentY >= maxContentY - 22)
    }

    ListModel {
        id: columns
        ListElement { title: "No.";          role: "no";    width: 50;  hAlign: 2; color: "#9cdcfe" }
        ListElement { title: "Time";         role: "time";  width: 90;  hAlign: 1; color: "#cccccc" }
        ListElement { title: "RSSI";         role: "rssi";  width: 60;  hAlign: 1; color: "#b5cea8" }
        ListElement { title: "Protocol";     role: "proto"; width: 80;  hAlign: 1; color: "#dcdcaa" }
        ListElement { title: "Ch";           role: "chIdx"; width: 40;  hAlign: 4; color: "#ce9178" }
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
                    model: root.frameModel

                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AsNeeded
                    }

                    // Wireshark-style live follow: stick to the newest frame
                    // unless the user has scrolled away from the live tail.
                    property bool autoFollow: true

                    // Recompute "at bottom" on any scroll (user or programmatic).
                    onContentYChanged: root.updateAutoFollow()
                    onContentHeightChanged: root.updateAutoFollow()

                    onCountChanged: {
                        if (frameListView.autoFollow && frameListView.count > 0)
                            Qt.callLater(frameListView.positionViewAtEnd)
                    }

                    onCurrentIndexChanged: {
                        root.currentFrameIndex = currentIndex
                        root.loadDetail(currentIndex)
                    }

                    delegate: Rectangle {
                        id: rowDelegate
                        width: frameListView.width
                        height: 22
                        color: frameListView.currentIndex === index ? "#094771" : (index % 2 === 0 ? "#1e1e1e" : "#252525")
                        clip: true

                        property var cells: [no, time, rssi, proto, chIdx, addr, src, dst, type, info]

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
                        model: []

                        delegate: Rectangle {
                            width: frameInfoView.width
                            height: 22
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: 8

                                TextEdit {
                                    text: modelData.field + ":"
                                    color: "#9cdcfe"
                                    readOnly: true
                                    selectByMouse: true
                                    Layout.preferredWidth: 140
                                    horizontalAlignment: TextEdit.AlignRight
                                    verticalAlignment: TextEdit.AlignVCenter
                                    rightPadding: 8
                                    wrapMode: TextEdit.NoWrap
                                }
                                TextEdit {
                                    text: modelData.value
                                    color: "#cccccc"
                                    readOnly: true
                                    selectByMouse: true
                                    Layout.fillWidth: true
                                    horizontalAlignment: TextEdit.AlignLeft
                                    verticalAlignment: TextEdit.AlignVCenter
                                    wrapMode: TextEdit.NoWrap
                                    clip: true
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

                // Measure the width of a full 16-byte hex row so partial rows
                // occupy the same column width (TextEdit ignores trailing
                // whitespace in its width calculation).
                TextMetrics {
                    id: hexColMetrics
                    text: "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
                    font.family: "Google Sans Code"
                }

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
                        model: []

                        delegate: Rectangle {
                            width: hexView.width
                            height: 22
                            color: "transparent"

                            RowLayout {
                                anchors.fill: parent
                                spacing: 0

                                TextEdit {
                                    text: modelData.offset
                                    color: "#858585"
                                    readOnly: true
                                    selectByMouse: true
                                    Layout.preferredWidth: 50
                                    horizontalAlignment: TextEdit.AlignRight
                                    verticalAlignment: TextEdit.AlignVCenter
                                    font.family: "Google Sans Code"
                                    leftPadding: 8
                                    wrapMode: TextEdit.NoWrap
                                }
                                TextEdit {
                                    text: modelData.hex
                                    color: "#d4d4d4"
                                    readOnly: true
                                    selectByMouse: true
                                    Layout.preferredWidth: hexColMetrics.width + 16
                                    horizontalAlignment: TextEdit.AlignLeft
                                    verticalAlignment: TextEdit.AlignVCenter
                                    font.family: "Google Sans Code"
                                    leftPadding: 16
                                    wrapMode: TextEdit.NoWrap
                                    clip: true
                                }
                                TextEdit {
                                    text: modelData.ascii
                                    color: "#7c7c7c"
                                    readOnly: true
                                    selectByMouse: true
                                    Layout.fillWidth: true
                                    horizontalAlignment: TextEdit.AlignLeft
                                    verticalAlignment: TextEdit.AlignVCenter
                                    font.family: "Google Sans Code"
                                    leftPadding: 16
                                    wrapMode: TextEdit.NoWrap
                                    clip: true
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
