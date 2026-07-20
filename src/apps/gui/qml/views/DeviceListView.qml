import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtGraphs

Item {
    id: root

    property var deviceModel: null
    property int currentDeviceIndex: -1

    // Selectable chart time window (seconds). Defaults to 60s.
    property real chartRangeSec: 60.0

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
            rssiRawSeries.clear()
            axisX.min = 0.0
            axisX.max = chartRangeSec
            emptyChartLabel.visible = true
            return
        }
        emptyChartLabel.visible = false
        var avg = deviceModel.rssiSeriesFor(idx)
        var raw = deviceModel.rssiRawSeriesFor(idx)
        rssiSeries.clear()
        rssiRawSeries.clear()
        // GraphsView's append(points) takes a list of QPointF.
        rssiSeries.append(avg)
        rssiRawSeries.append(raw)

        // Rolling window: anchor the right edge to "now" expressed in the
        // series' frame-relative seconds, slide the left edge by chartRangeSec.
        var firstMs = deviceModel.firstFrameMsFor(idx)
        var nowX = firstMs > 0 ? (Date.now() - firstMs) / 1000.0 : chartRangeSec
        if (nowX < chartRangeSec) nowX = chartRangeSec
        axisX.max = nowX
        axisX.min = nowX - chartRangeSec
        axisX.tickInterval = chartRangeSec >= 120 ? 30
                           : chartRangeSec >= 60 ? 10
                           : chartRangeSec >= 15 ? 5
                           : chartRangeSec >= 5  ? 1
                           : 1
        axisX.labelFormat = chartRangeSec >= 60 ? "%.0f" : "%.1f"
        axisX.labelDecimals = chartRangeSec >= 60 ? 0 : 1

        // Auto-fit Y axis to the points currently inside the visible X
        // window, padded by 10% above max and 10% below min.
        var xMin = axisX.min
        var xMax = axisX.max
        var yMin = Infinity
        var yMax = -Infinity
        function scanForBounds(pts) {
            for (var i = 0; i < pts.length; i++) {
                var p = pts[i]
                if (p.x < xMin || p.x > xMax) continue
                if (p.y < yMin) yMin = p.y
                if (p.y > yMax) yMax = p.y
            }
        }
        scanForBounds(avg)
        scanForBounds(raw)
        if (yMin === Infinity || yMax === -Infinity) {
            // No visible samples — keep previous Y range.
        } else {
            var span = yMax - yMin
            if (span < 1.0) span = 1.0   // avoid a zero/flat range
            axisY.min = Math.floor((yMin - 0.1 * span) * 10) / 10
            axisY.max = Math.ceil((yMax + 0.1 * span) * 10) / 10
        }
    }

    // Periodically re-read the chart series so newly-captured frames are
    // painted and the rolling window keeps sliding forward even when no
    // frames arrive (so the trace visibly advances in time).
    Timer {
        id: chartRefreshTimer
        interval: 500
        repeat: true
        running: root.currentDeviceIndex >= 0
        onTriggered: root.loadChart(root.currentDeviceIndex)
    }

    ListModel {
        id: columns
        ListElement { title: "RSSI";         role: "rssi";        width: 70;  hAlign: 1; color: "#b5cea8" }
        ListElement { title: "Protocol";     role: "proto";       width: 80;  hAlign: 1; color: "#dcdcaa" }
        ListElement { title: "Identifier";   role: "identifier"; width: 220; hAlign: 1; color: "#569cd6" }
        ListElement { title: "Last Seen";     role: "lastSeen";    width: 120; hAlign: 1; color: "#cccccc" }
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

                    Label {
                        anchors.centerIn: parent
                        visible: !deviceModel || deviceModel.count === 0
                        text: "No devices — start a capture to populate"
                        color: "#858585"
                    }

                    delegate: Rectangle {
                        id: rowDelegate
                        width: deviceListView.width
                        height: 22
                        color: deviceListView.currentIndex === index ? "#094771" : (index % 2 === 0 ? "#1e1e1e" : "#252525")
                        clip: true

                        property var cells: [rssi, proto, identifier, lastSeen, packetRate]

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
                        id: chartHeader
                        Layout.fillWidth: true
                        Layout.preferredHeight: 24
                        color: "#2d2d2d"

                        Label {
                            anchors.left: parent.left
                            anchors.verticalCenter: parent.verticalCenter
                            leftPadding: 8
                            text: "RSSI over time"
                            color: "#cccccc"
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                        }

                        Label {
                            id: chartRangeLabel
                            anchors.right: chartRangeCombo.left
                            anchors.verticalCenter: parent.verticalCenter
                            rightPadding: 6
                            text: "Window:"
                            color: "#858585"
                            verticalAlignment: Text.AlignVCenter
                        }

                        ComboBox {
                            id: chartRangeCombo
                            anchors.right: parent.right
                            anchors.verticalCenter: parent.verticalCenter
                            rightPadding: 8
                            width: 90
                            height: 18
                            model: ListModel {
                                id: rangeModel
                                ListElement { label: "5 s";   secs: 5 }
                                ListElement { label: "15 s";  secs: 15 }
                                ListElement { label: "30 s";  secs: 30 }
                                ListElement { label: "1 min"; secs: 60 }
                                ListElement { label: "2 min"; secs: 120 }
                                ListElement { label: "5 min"; secs: 300 }
                            }
                            // Default to "1 min".
                            currentIndex: 3
                            onActivated: function (i) {
                                root.chartRangeSec = rangeModel.get(i).secs
                                root.loadChart(root.currentDeviceIndex)
                            }
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

                        LineSeries {
                            id: rssiRawSeries
                            color: "#9e9e9e"
                            opacity: 0.35
                            width: 1.0
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