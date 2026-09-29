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
        }
        function onDevicesUpdated(rows) {
            deviceListModel.setRows(rows)
        }
        function onErrorOccurred(message) {
            console.error("Supertooth:", message)
        }
        function onRunningChanged() {
            console.log("Supertooth: running =", receiverController.running)
            if (receiverController.running)
                deviceListModel.clear()
        }
    }

    Binding {
        target: sidebar
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
            onPlayPauseToggled: {
                // Channel params are passed in the session's native grid:
                // LE RF units for LE sessions, BR/EDR units otherwise.
                var isBle = configView.sessionTypeIndex === 1
                var count = isBle ? configView.backendLeChannelCount
                                  : configView.backendChannelCount
                var bottom = isBle ? configView.bottomLeIndex
                                    : configView.backendBottomChannel
                console.log("Supertooth: play/pause toggled; running =",
                            receiverController.running,
                            "inputType =", configView.inputType,
                            "deviceID =", configView.deviceID,
                            "sessionType =", configView.sessionTypeIndex,
                            "enforceCrc =", configView.enforceCrc,
                            "channels =", count,
                            "bottom =", bottom,
                            "bleAdv =", configView.backendBleAdvChannel,
                            "lna =", configView.hackrfLna,
                            "vga =", configView.hackrfVga,
                            "amp =", configView.hackrfAmp,
                            "bladerfGain =", configView.bladerfGain)
                if (receiverController.running) {
                    receiverController.stop()
                } else {
                    frameListModel.clear()
                    deviceListModel.clear()
                    receiverController.start(configView.inputType,
                                             configView.deviceID,
                                             configView.sessionTypeIndex,
                                             configView.enforceCrc,
                                             count,
                                             bottom,
                                             configView.backendBleAdvChannel,
                                             configView.acErrors,
                                             configView.hackrfLna,
                                             configView.hackrfVga,
                                             configView.hackrfAmp ? 1 : 0,
                                             configView.bladerfGain)
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
            DeviceListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                deviceModel: deviceListModel
            }
            ConfigView {
                id: configView
                Layout.fillWidth: true
                Layout.fillHeight: true
                running: receiverController.running
            }
        }
    }
}
