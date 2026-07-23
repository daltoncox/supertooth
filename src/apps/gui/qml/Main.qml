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

    DeviceListSortProxyModel {
        id: deviceSortProxy
        sourceModel: deviceListModel
        sortRoleName: "rssi"
        sortOrder: Qt.DescendingOrder
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
                // Channel params are passed in the session's native grid:
                // LE RF units for BLE sessions, BR/EDR units otherwise.
                var isBle = captureView.sessionTypeIndex === 1
                var count = isBle ? captureView.backendLeChannelCount
                                  : captureView.backendChannelCount
                var bottom = isBle ? captureView.bottomLeIndex
                                    : captureView.backendBottomChannel
                console.log("Supertooth: play/pause toggled; running =",
                            receiverController.running,
                            "inputType =", header.inputTypeIndex,
                            "deviceID =", header.deviceID,
                            "sessionType =", captureView.sessionTypeIndex,
                            "enforceCrc =", captureView.enforceCrc,
                            "channels =", count,
                            "bottom =", bottom,
                            "leGrid =", captureView.backendLeGrid,
                            "bleAdv =", captureView.backendBleAdvChannel)
                if (receiverController.running) {
                    receiverController.stop()
                } else {
                    frameListModel.clear()
                    deviceListModel.clear()
                    receiverController.start(header.inputTypeIndex,
                                             header.deviceID,
                                             captureView.sessionTypeIndex,
                                             captureView.enforceCrc,
                                             count,
                                             bottom,
                                             captureView.backendLeGrid,
                                             captureView.backendBleAdvChannel)
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
                    deviceModel: deviceSortProxy
                }
                CaptureView {
                    id: captureView
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    running: receiverController.running
                }
            }
        }
    }
}
