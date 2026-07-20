import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Supertooth

ApplicationWindow {
    width: 1200
    height: 800
    visible: true
    title: qsTr("Supertooth")
    color: "grey"

    ReceiverController {
        id: receiverController
    }

    FrameListModel {
        id: frameListModel
    }

    DeviceListModel {
        id: deviceListModel
    }

    Connections {
        target: receiverController
        function onFrameDecoded(row) {
            frameListModel.appendRow(row)
            deviceListModel.onFrameDecoded(row)
        }
        function onErrorOccurred(message) {
            console.error("Supertooth:", message)
        }
        function onRunningChanged() {
            console.log("Supertooth: running =", receiverController.running)
        }
    }

    Binding {
        target: header
        property: "playing"
        value: receiverController.running
    }

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Sidebar {
            id: sidebar
            Layout.fillHeight: true
            onItemSelected: function (index) {
                stack.currentIndex = index
            }
        }

        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 0

            Header {
                id: header
                Layout.fillWidth: true
                onPlayPauseToggled: {
                    console.log("Supertooth: play/pause toggled; running =",
                                receiverController.running,
                                "inputType =", header.inputTypeIndex,
                                "deviceID =", header.deviceID,
                                "sessionType =", channelView.sessionTypeIndex)
                    if (receiverController.running) {
                        receiverController.stop()
                    } else {
                        frameListModel.clear()
                        deviceListModel.clear()
                        receiverController.start(header.inputTypeIndex,
                                                 header.deviceID,
                                                 channelView.sessionTypeIndex)
                    }
                }
            }

            StackLayout {
                id: stack
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: sidebar.selectedIndex

                FrameListView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    frameModel: frameListModel
                }
                Item {
                    Label {
                        anchors.centerIn: parent
                        text: "Topology"
                    }
                }
                DeviceListView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    deviceModel: deviceListModel
                }
                ChannelView {
                    id: channelView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    running: receiverController.running
                }
            }
        }
    }
}
