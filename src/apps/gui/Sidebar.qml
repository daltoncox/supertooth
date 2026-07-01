import QtQuick
import QtQuick.Controls

Rectangle {
    id: sidebar

    property int selectedIndex: 0
    signal itemSelected(int index)

    width: 56
    color: "black"

    Column {
        anchors.horizontalCenter: parent.horizontalCenter
        topPadding: 12
        spacing: 6

        // Main App Icon
        Image {
            anchors.horizontalCenter: parent.horizontalCenter
            source: "/images/Flying_Supertooth_White_512@2x.png"
            width: 28
            height: 28
            sourceSize.width: 64
            sourceSize.height: 64
        }

        // Extra spacing
        Item { 
            width: 1 
            height: 2
        }

        // Divinder
        Rectangle {
            anchors.horizontalCenter: parent.horizontalCenter
            width: 36
            height: 1
        }

        Item { 
            width: 1 
            height: 2
        }

        // View Icon Buttons
        Repeater {
            model: ListModel {
                ListElement { iconPath: "/images/frame_list.svg" }
                ListElement { iconPath: "/images/topolgy.svg" }
                ListElement { iconPath: "/images/devices.svg" }
                ListElement { iconPath: "/images/channel.svg" }
                ListElement { iconPath: "/images/settings.svg" }
            }

            delegate: Button {
                width: 44
                height: 44

                Image {
                    anchors.centerIn: parent
                    source: iconPath
                    width: 20
                    height: 20
                    sourceSize.width: 40
                    sourceSize.height: 40
                }

                onClicked: {
                    sidebar.selectedIndex = index
                    sidebar.itemSelected(index)
                }
            }
        }
    }
}
