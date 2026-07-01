import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Supertooth

Rectangle {
    id: header

    property int inputTypeIndex: 0
    property string deviceID
    property bool playing: false

    signal playPauseToggled()

    RadioDeviceModel {
        id: radioDeviceModel
    }

    Component.onCompleted: {
        radioDeviceModel.refresh(header.inputTypeIndex)
        deviceIdSelector.currentIndex =
            radioDeviceModel.rowCount() > 0 ? 0 : -1
        header.deviceID = deviceIdSelector.currentText
    }

    height: 56
    color: "black"

    RowLayout {
        height: 40
        // Stretches horizontally
        anchors.left: parent.left
        anchors.right: parent.right
        
        // Centers vertically without stretching the height
        anchors.verticalCenter: parent.verticalCenter
        Layout.alignment: Qt.AlignVCenter

        anchors.leftMargin: 12
        anchors.rightMargin: 12
        spacing: 12

        ComboBox {
            id: inputTypeSelector
            Layout.preferredHeight: parent.height

            model: ["HackRF", "File"]

            onActivated: function (index) {
                header.inputTypeIndex = index
                radioDeviceModel.refresh(index)
                deviceIdSelector.currentIndex =
                    radioDeviceModel.rowCount() > 0 ? 0 : -1
                header.deviceID = deviceIdSelector.currentText
            }
        }

        StackLayout {
            id: inputControls
            Layout.preferredHeight: parent.height
            currentIndex: header.inputTypeIndex

            // HackRF Input Options
            RowLayout {
                Layout.preferredHeight: parent.height
                spacing: 8

                ComboBox {
                    id: deviceIdSelector
                    Layout.preferredHeight: parent.height

                    model: radioDeviceModel
                    textRole: "display"

                    onActivated: function (index) {
                        header.deviceID = deviceIdSelector.currentText
                    }
                }

            }

            // File Input Options
            RowLayout {
                Layout.preferredHeight: parent.height
                spacing: 8

                TextField {
                    Layout.preferredHeight: parent.height

                    placeholderText: "/path/to/file"
                }
            }
        }

        Button {
            Layout.preferredHeight: parent.height
            Layout.preferredWidth: parent.height

            Image {
                source: header.playing ? "/images/stop.svg" : "/images/play.svg"
                sourceSize.width: 36
                sourceSize.height: 36
                anchors.centerIn: parent
            }

            onClicked: {
                header.playing = !header.playing
                header.playPauseToggled()
            }
        }
    }
}
