import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtGraphs

Item {
    id: root

    property var deviceModel: null
    property int currentDeviceIndex: -1

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
        if (!deviceModel || idx < 0) {
            deviceInfoView.model = []
            return
        }
        deviceInfoView.model = deviceModel.detailFor(idx)
    }

    function loadChart(idx) {
        if (!deviceModel || idx < 0) {
            rssiSeries.clear()
            emptyChartLabel.visible = true
            return
        }
        emptyChartLabel.visible = false
        var pts = deviceModel.rssiSeriesFor(idx)
        rssiSeries.clear()
        // GraphsView's append(points) takes a list of QPointF.
        rssiSeries.append(pts)
    }

    ListModel {
        id: columns
        ListElement { title: "RSSI";         role: "rssi";       width: 70;  hAlign: 1; color: "#b5cea8" }
        ListElement { title: "Protocol";     role: "proto";      width: 80;  hAlign: 1; color: "#dcdcaa" }
        ListElement { title: "Address";      role: "addr";       width: 160; hAlign: 1; color: "#569cd6" }
        ListElement { title: "Device";       role: "device";     width: 160; hAlign: 1; color: "#cccccc" }
        ListElement { title: "Last Seen";   role: "lastSeen";    width: 140; hAlign: 1; color: "#cccccc" }
        ListElement { title: "Packets";     role: "packetsSeen"; width: 70;  hAlign: 4; color: "#ce9178" }
        ListElement { title: "Packet Rate";  role: "packetRate";  width: 100; hAlign: 1; color: "#9cdcfe" }
    }

    SplitView {
        id: mainSplit
        anchors.fill: parent
        orientation: Qt.Horizontal
        handle: Rectangle {
            implicitWidth: 4
            color: SplitHandle.pressed ? "#444" : (SplitHandle.hovered ? "#333" : "#222")
        }

        // ---- Table pane ------------------------------------------------
        Rectangle {
            id: deviceListPane
            color: "#1e1e1e"
            SplitView.preferredWidth: parent.width * 0.5
            SplitView.minimumWidth: 200
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
                    id: deviceListView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true
                    model: root.deviceModel

                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AsNeeded
                    }

                    onCurrentIndexChanged: {
                        root.currentDeviceIndex = currentIndex
                        root.loadDetail(currentIndex)
                        root.loadChart(currentIndex)
                    }

                    delegate: Rectangle {
                        id: rowDelegate
                        width: deviceListView.width
                        height: 22
                        color: deviceListView.currentIndex === index ? "#094771" : (index % 2 === 0 ? "#1e1e1e" : "#252525")
                        clip: true

                        property var cells: [rssi, proto, addr, device, lastSeen, packetsSeen, packetRate]

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                deviceListView.currentIndex = index
                                root.currentDeviceIndex = index
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

        // ---- Right column: chart stacked above device info ------------
        SplitView {
            id: rightSplit
            SplitView.preferredWidth: parent.width * 0.5
            SplitView.minimumWidth: 200
            orientation: Qt.Vertical
            handle: Rectangle {
                implicitHeight: 4
                color: SplitHandle.pressed ? "#444" : (SplitHandle.hovered ? "#333" : "#222")
            }

            // ---- RSSI chart pane ---------------------------------------
            Rectangle {
                id: chartPane
                color: "#1e1e1e"
                SplitView.preferredHeight: parent.height * 0.55
                SplitView.minimumHeight: 80

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 0

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 24
                        color: "#2d2d2d"
                        Label {
                            anchors.fill: parent
                            text: "RSSI over time (1s window)"
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

                    GraphsView {
                        id: graphsView
                        Layout.fillWidth: true
                        Layout.fillHeight: true

                        axisX: ValueAxis {
                            id: axisX
                            min: 0.0
                            max: 1.0
                            titleText: "Time (s)"
                            tickInterval: 0.1
                            labelFormat: "%.2f"
                            labelDecimals: 2
                        }

                        axisY: ValueAxis {
                            id: axisY
                            min: -100.0
                            max: -30.0
                            titleText: "RSSI (dBm)"
                            tickInterval: 10
                            labelFormat: "%.0f"
                            labelDecimals: 0
                        }

                        LineSeries {
                            id: rssiSeries
                            color: "#4EC9B0"
                            width: 1.5
                        }

                        Label {
                            id: emptyChartLabel
                            anchors.centerIn: parent
                            text: "Select a device to view its RSSI history"
                            color: "#858585"
                            visible: true
                        }
                    }
                }
            }

            // ---- Device info pane --------------------------------------
            Rectangle {
                id: deviceInfoPane
                color: "#1e1e1e"
                SplitView.preferredHeight: parent.height * 0.45
                SplitView.minimumHeight: 80

                ColumnLayout {
                    anchors.fill: parent
                    spacing: 0

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 24
                        color: "#2d2d2d"
                        Label {
                            anchors.fill: parent
                            text: "Device Info"
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
                        id: deviceInfoView
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        model: []

                        delegate: Rectangle {
                            width: deviceInfoView.width
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
                                    Layout.preferredWidth: 160
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
        }
    }
}