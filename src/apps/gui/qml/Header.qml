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

    height: 64
    color: "black"

    RowLayout {
        height: 48
        // Stretches horizontally
        anchors.left: parent.left
        anchors.right: parent.right
        
        // Centers vertically without stretching the height
        anchors.verticalCenter: parent.verticalCenter
        Layout.alignment: Qt.AlignVCenter

        anchors.leftMargin: 0
        anchors.rightMargin: 12
        spacing: 12

        ComboBox {
            id: inputTypeSelector
            enabled: !header.playing
            Layout.preferredHeight: parent.height
            Layout.preferredWidth: 120

            model: ["HackRF"]

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
            enabled: !header.playing
            Layout.preferredHeight: parent.height
            currentIndex: header.inputTypeIndex

            // HackRF Input Options
            RowLayout {
                Layout.preferredHeight: parent.height
                spacing: 12

                ComboBox {
                    id: deviceIdSelector
                    Layout.preferredHeight: parent.height
		    Layout.preferredWidth: 300

                    model: radioDeviceModel
                    textRole: "display"

                    onActivated: function (index) {
                        header.deviceID = deviceIdSelector.currentText
                    }
                }

                Button {
                    id: refreshButton
		    topInset: 0
		    bottomInset: 0
		    leftInset: 0
		    rightInset: 0

                    enabled: !header.playing
                    Layout.preferredHeight: parent.height
                    Layout.preferredWidth: parent.height

                    Image {
                        source: "/assets/images/refresh.svg"
                        anchors.centerIn: parent
                        width: 30
                        height: 30
                        opacity: refreshButton.enabled ? 1.0 : 0.4
                    }

                    onClicked: {
                        var previousId = deviceIdSelector.currentText
                        radioDeviceModel.refresh(header.inputTypeIndex, true)
                        var idx = radioDeviceModel.indexFromIdentifier(previousId)
                        if (idx >= 0) {
                            deviceIdSelector.currentIndex = idx
                            header.deviceID = previousId
                        } else if (radioDeviceModel.rowCount() > 0) {
                            deviceIdSelector.currentIndex = 0
                            header.deviceID = deviceIdSelector.currentText
                        } else {
                            deviceIdSelector.currentIndex = -1
                            header.deviceID = ""
                        }
                    }
                }
            }
        }

        Button {
	    topInset: 0
	    bottomInset: 0
	    leftInset: 0
	    rightInset: 0
            Layout.preferredHeight: parent.height
            Layout.preferredWidth: parent.height

            Image {
                source: header.playing ? "/assets/images/stop.svg" : "/assets/images/play.svg"
                anchors.centerIn: parent
                width: 30
                height: 30
            }

            onClicked: {
                header.playPauseToggled()
            }
        }
    }
}
