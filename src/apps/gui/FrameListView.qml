import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root

    property int currentFrameIndex: -1

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

            ColumnLayout {
                anchors.fill: parent
                spacing: 0

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 24
                    color: "#2d2d2d"

                    RowLayout {
                        anchors.fill: parent
                        spacing: 0

                        Label {
                            text: "No."
                            color: "#cccccc"
                            Layout.preferredWidth: 60
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                        }
                        Label {
                            text: "Time"
                            color: "#cccccc"
                            Layout.preferredWidth: 90
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                        }
                        Label {
                            text: "Source"
                            color: "#cccccc"
                            Layout.preferredWidth: 140
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                            leftPadding: 8
                        }
                        Label {
                            text: "Destination"
                            color: "#cccccc"
                            Layout.preferredWidth: 140
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                            leftPadding: 8
                        }
                        Label {
                            text: "Protocol"
                            color: "#cccccc"
                            Layout.preferredWidth: 90
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                            leftPadding: 8
                        }
                        Label {
                            text: "Length"
                            color: "#cccccc"
                            Layout.preferredWidth: 60
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                        }
                        Label {
                            text: "Info"
                            color: "#cccccc"
                            Layout.fillWidth: true
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                            font.bold: true
                            leftPadding: 8
                        }
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
                        ListElement { no: 1; time: "0.000000"; src: "11:22:33:44:55:66"; dst: "Broadcast"; proto: "BLE"; len: 32; info: "ADV_IND - Connectable undirected" }
                        ListElement { no: 2; time: "0.001204"; src: "11:22:33:44:55:66"; dst: "Broadcast"; proto: "BLE"; len: 16; info: "ADV_NONCONN_IND" }
                        ListElement { no: 3; time: "0.002488"; src: "AA:BB:CC:DD:00:01"; dst: "11:22:33:44:55:66"; proto: "BLE"; len: 18; info: "SCAN_REQ" }
                        ListElement { no: 4; time: "0.003712"; src: "11:22:33:44:55:66"; dst: "AA:BB:CC:DD:00:01"; proto: "BLE"; len: 40; info: "SCAN_RSP" }
                        ListElement { no: 5; time: "0.010033"; src: "AA:BB:CC:DD:00:01"; dst: "11:22:33:44:55:66"; proto: "BLE"; len: 22; info: "CONNECT_IND" }
                    }

                    delegate: Rectangle {
                        width: frameListView.width
                        height: 22
                        color: frameListView.currentIndex === index ? "#094771" : (index % 2 === 0 ? "#1e1e1e" : "#252525")

                        MouseArea {
                            anchors.fill: parent
                            onClicked: {
                                frameListView.currentIndex = index
                                root.currentFrameIndex = index
                            }
                        }

                        RowLayout {
                            anchors.fill: parent
                            spacing: 0

                            Label {
                                text: no
                                color: "#9cdcfe"
                                Layout.preferredWidth: 60
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                            Label {
                                text: time
                                color: "#cccccc"
                                Layout.preferredWidth: 90
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                            Label {
                                text: src
                                color: "#cccccc"
                                Layout.preferredWidth: 140
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: 8
                            }
                            Label {
                                text: dst
                                color: "#cccccc"
                                Layout.preferredWidth: 140
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: 8
                            }
                            Label {
                                text: proto
                                color: "#ce9178"
                                Layout.preferredWidth: 90
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: 8
                            }
                            Label {
                                text: len
                                color: "#b5cea8"
                                Layout.preferredWidth: 60
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }
                            Label {
                                text: info
                                color: "#cccccc"
                                Layout.fillWidth: true
                                horizontalAlignment: Text.AlignLeft
                                verticalAlignment: Text.AlignVCenter
                                leftPadding: 8
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
                                spacing: 16

                                Label {
                                    text: offset
                                    color: "#858585"
                                    Layout.preferredWidth: 50
                                    horizontalAlignment: Text.AlignRight
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                }
                                Label {
                                    text: hex
                                    color: "#d4d4d4"
                                    Layout.fillWidth: true
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                }
                                Label {
                                    text: ascii
                                    color: "#7c7c7c"
                                    Layout.preferredWidth: 180
                                    horizontalAlignment: Text.AlignLeft
                                    verticalAlignment: Text.AlignVCenter
                                    font.family: "Consolas"
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}