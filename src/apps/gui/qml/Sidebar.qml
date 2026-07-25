import QtQuick
import QtQuick.Controls

Rectangle {
    id: sidebar

    property int selectedIndex: 0
    signal itemSelected(int index)

    width: 64
    color: "black"

    Column {
        anchors.horizontalCenter: parent.horizontalCenter
        topPadding: 8
        spacing: 8

        // Main App Icon
        Image {
            anchors.horizontalCenter: parent.horizontalCenter
            source: "/assets/images/Flying_Supertooth_White_512@2x.png"
            width: 48
            height: 48
	    antialiasing: true
        }

        // View Icon Buttons
        Repeater {
            model: ListModel {
                ListElement { iconPath: "/assets/images/frame_list.svg" }
                ListElement { iconPath: "/assets/images/devices.svg" }
                ListElement { iconPath: "/assets/images/channel.svg" }
            }

            delegate: Button {
		topInset: 0
	        bottomInset: 0
       	        leftInset: 0
	        rightInset: 0
                width: 48
                height: 48
                flat: sidebar.selectedIndex !== index

                Image {
                    anchors.centerIn: parent
                    source: iconPath
                    width: 36
                    height: 36
                }

                onClicked: {
                    sidebar.selectedIndex = index
                    sidebar.itemSelected(index)
                }
            }
        }
    }
}
