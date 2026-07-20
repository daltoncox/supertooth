import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: root

    // 0 = Hybrid (default), 1 = BLE, 2 = BR/EDR. Mirrors BACKEND_SESSION_*.
    property int sessionTypeIndex: 0
    // Drop BLE frames whose CRC fails. Applies to BLE and hybrid sessions.
    // Default on, matching the CLI's --enforce-crc default.
    property bool enforceCrc: true
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

        Label {
            text: qsTr("CRC Enforcement")
            color: "#cccccc"
            font.bold: true
            Layout.topMargin: 8
        }

        Switch {
            id: enforceCrcSwitch
            enabled: !root.running
            checked: root.enforceCrc
            text: checked ? qsTr("On — drop BLE frames that fail CRC") : qsTr("Off — show all BLE frames")

            onToggled: root.enforceCrc = checked
        }

        Label {
            text: qsTr("When on, BLE frames whose CRC doesn't pass are dropped before display. Applies to BLE and hybrid sessions. Helps suppress spurious devices from bit errors.")
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
