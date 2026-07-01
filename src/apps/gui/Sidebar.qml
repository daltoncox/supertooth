import QtQuick
import QtQuick.Controls

Rectangle {
    id: sidebar

    property int selectedIndex: 0
    signal itemSelected(int index)

    width: 56
    color: "grey"

    Column {
        anchors.horizontalCenter: parent.horizontalCenter
        topPadding: 12
        spacing: 8

        // App Icon
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            width: 28
            height: 28
            radius: 8
            color: "blue"
        }

        // Divider
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            width: 36
            height: 1
            color: "black"
        }

        // Views
        Repeater {
            model: ["A", "B", "C"]

            delegate: Item {
                width: 44
                height: 44

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: 4
                    radius: 8
                    color: sidebar.selectedIndex === index ? "lightgray" : "transparent"

                    Text {
                        anchors.centerIn: parent
                        text: modelData
                        font.pixelSize: 16
                    }
                }

                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        sidebar.selectedIndex = index
                        sidebar.itemSelected(index)
                    }
                }
            }
        }
    }
}
