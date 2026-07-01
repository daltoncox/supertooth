import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    width: 640
    height: 480
    visible: true
    title: qsTr("Supertooth")

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

        StackLayout {
            id: stack
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: sidebar.selectedIndex

            Label { text: "View A" }
            Label { text: "View B" }
            Label { text: "View C" }
        }
    }
}
