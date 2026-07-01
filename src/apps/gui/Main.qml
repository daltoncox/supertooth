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
            }

            StackLayout {
                id: stack
                Layout.fillWidth: true
                Layout.fillHeight: true
                currentIndex: sidebar.selectedIndex

                FrameListView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
                Item {
                    Label {
                        anchors.centerIn: parent
                        text: "Topology"
                    }
                }
                Item {
                    Label {
                        anchors.centerIn: parent
                        text: "Devices"
                    }
                }
                Item {
                    Label {
                        anchors.centerIn: parent
                        text: "Channel"
                    }
                }
                Item {
                    Label {
                        anchors.centerIn: parent
                        text: "Settings"
                    }
                }
            }
        }
    }
}
