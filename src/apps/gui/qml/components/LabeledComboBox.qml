import QtQuick
import QtQuick.Controls

// ComboBox with a small floating label straddling its top border
// (GroupBox-title style), so rows need no separate side labels.
// The label's fill must match whatever surface the control sits on.
Item {
    id: root

    property alias label: floatingLabel.text
    property color labelFill: "#1e1e1e"
    property alias model: combo.model
    property alias textRole: combo.textRole
    property alias valueRole: combo.valueRole
    property alias currentIndex: combo.currentIndex
    property alias currentText: combo.currentText
    // Note: no `enabled` alias — disabling this Item cascades to the
    // ComboBox via normal QQuickItem enabled-state inheritance.

    signal activated(int index)

    // Same height as a plain ComboBox: the floating label overflows above
    // the top edge (QML doesn't clip by default) instead of adding height.
    // This keeps the box — and every sibling in the row — exactly where a
    // plain ComboBox would sit. Needs a few px of headroom above the row
    // and non-clipping ancestors.
    implicitWidth: combo.implicitWidth
    implicitHeight: combo.implicitHeight

    ComboBox {
        id: combo
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom

        onActivated: function (index) {
            root.activated(index)
        }
    }

    Label {
        id: floatingLabel
        anchors.left: combo.left
        anchors.leftMargin: 12
        y: combo.y - implicitHeight / 2 + 1
        z: 1
        font.pixelSize: 10
        color: "#9d9d9d"
        leftPadding: 4
        rightPadding: 4
        background: Rectangle {
            color: root.labelFill
        }
    }
}
