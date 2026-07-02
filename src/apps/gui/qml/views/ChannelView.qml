import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    // 0 = Hybrid (default), 1 = BLE, 2 = BR/EDR. Mirrors BACKEND_SESSION_*.
    property int sessionTypeIndex: 0
    property bool running: false

    color: "#1e1e1e"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        Label {
            text: qsTr("Session Type")
            color: "#cccccc"
            font.bold: true
        }

        ComboBox {
            id: sessionTypeSelector
            enabled: !root.running
            model: ["Hybrid", "BLE", "BR/EDR"]
            currentIndex: root.sessionTypeIndex

            onActivated: function (index) {
                root.sessionTypeIndex = index
            }
        }

        Label {
            text: root.running
                  ? qsTr("Stop the running session to change protocol.")
                  : qsTr("Select which receiver pipeline to run. Hybrid captures BR/EDR + BLE from one stream.")
            color: "#858585"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}
