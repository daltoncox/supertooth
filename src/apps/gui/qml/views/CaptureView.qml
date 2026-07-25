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

    // Channel layout. The capture window is numChannels MHz wide (even,
    // 2..maxChannels) and its bottom edge snaps to one of two grids:
    //   - BR/EDR lock: bottom = BR/EDR channel index, 1 MHz steps, LO at a
    //     half-MHz frequency (e.g. 2411.5, like the current hybrid fixed LO).
    //   - BLE lock: bottom = LE RF channel index, 2 MHz steps, LO at a
    //     whole-MHz frequency.
    // The lock is user-selectable in hybrid mode; BLE-only sessions are
    // always BLE-locked and BR/EDR-only sessions always BR/EDR-locked.
    // CaptureView is the single source of truth — all writes (SpinBoxes,
    // spectrum drags) go through setWindowBredr/setWindowBle so clamping
    // is applied uniformly.
    property int lockReference: 0       // 0 = BR/EDR, 1 = BLE (hybrid only)
    property int bottomChannel: 0       // BR/EDR channel index (BR/EDR lock)
    property int bottomLeIndex: 0       // LE RF channel index (BLE lock)
    property int numChannels: 20
    readonly property int maxChannels: 20   // RECEIVER_BREDR_MAX_CHANNELS

    readonly property bool bleLocked: sessionTypeIndex === 1 ? true
                                    : sessionTypeIndex === 2 ? false
                                    : lockReference === 1

    // Derived helpers shared with the spectrum + summary labels.
    readonly property real windowLeftMhz: bleLocked ? 2401 + 2 * bottomLeIndex
                                                    : 2401.5 + bottomChannel
    // Sample rate mirrors supertooth-rx.c: 4 Msps for 2 channels, else
    // count * 1 Msps.
    readonly property real sampleRateHz: numChannels === 2 ? 4e6 : numChannels * 1e6
    // LO sits at the center of the capture window — a half-MHz frequency
    // when BR/EDR-locked, a whole-MHz frequency when BLE-locked.
    readonly property real loFreqHz: (windowLeftMhz + numChannels / 2.0) * 1e6

    // ---- Channel ranges covered by the window ------------------------------
    // LE: the window is always exactly numChannels/2 LE channels wide
    // (window edges never land on LE centers when BR/EDR-locked; they land
    // on LE edges when BLE-locked).
    readonly property int leFirstRf: bleLocked ? bottomLeIndex
                                               : Math.max(0, Math.ceil((bottomChannel - 0.5) / 2))
    readonly property int leLastRf: leFirstRf + numChannels / 2 - 1
    // BR/EDR: native range when BR/EDR-locked; when BLE-locked, the
    // channels whose centers fall strictly inside the window (channels
    // centered exactly on an edge are half out of band).
    readonly property int brFirstCh: bleLocked ? Math.min(78, bottomLeIndex * 2)
                                               : bottomChannel
    readonly property int brLastCh: bleLocked ? Math.min(78, bottomLeIndex * 2 + numChannels - 2)
                                              : bottomChannel + numChannels - 1

    function rfToLeLabel(rf) {
        if (rf === 0) return "37"
        if (rf === 12) return "38"
        if (rf === 39) return "39"
        if (rf < 12) return String(rf - 1)   // LE 0..10
        return String(rf - 2)                // LE 11..36
    }

    readonly property string brRangeText: brFirstCh === brLastCh
                                          ? String(brFirstCh)
                                          : brFirstCh + "–" + brLastCh
    readonly property string leRangeText: leFirstRf === leLastRf
                                          ? rfToLeLabel(leFirstRf)
                                          : rfToLeLabel(leFirstRf) + "–" + rfToLeLabel(leLastRf)

    readonly property string captureSummary: numChannels + " ch · BR " + brRangeText
                                             + " · LE " + leRangeText
                                             + " · " + (sampleRateHz / 1e6) + " Msps"
                                             + " · LO " + (loFreqHz / 1e6) + " MHz"

    // ---- Backend-ready values ---------------------------------------------
    // What actually gets captured, translated per the lock reference:
    //   - BR/EDR grid: numChannels processors (even), window = numChannels
    //     MHz, LO at a half-MHz.
    //   - LE grid: numChannels-1 processors (odd), window = numChannels MHz,
    //     LO at a whole MHz; the two BR/EDR channels centered on the
    //     Nyquist edges are not processed.
    // bleAdvChannel is the advertising channel whose center lies inside the
    // window (at most one fits a <=20 MHz window), or 0 = none — the hybrid
    // BLE worker idles and BLE-only sessions fall back to ch37.
    readonly property int backendChannelCount: bleLocked ? numChannels - 1
                                                         : numChannels
    readonly property int backendBottomChannel: bleLocked ? Math.min(78, bottomLeIndex * 2)
                                                          : bottomChannel
    // BLE-only sessions take their window in LE RF units instead.
    readonly property int backendLeChannelCount: numChannels / 2
    readonly property int backendLeGrid: bleLocked ? 1 : 0
    readonly property int backendBleAdvChannel: {
        if (leFirstRf <= 0 && leLastRf >= 0) return 37
        if (leFirstRf <= 12 && leLastRf >= 12) return 38
        if (leFirstRf <= 39 && leLastRf >= 39) return 39
        return 0
    }

    // ---- Window clamping (mirrors supertooth-rx.c validation) --------------
    function clampCount(c) {
        c = Math.round(c / 2) * 2
        return Math.max(2, Math.min(maxChannels, c))
    }
    function setWindowBredr(bottom, count) {
        var c = clampCount(count)
        var b = Math.max(0, Math.min(78 - (c - 1), bottom))
        if (c !== numChannels) numChannels = c
        if (b !== bottomChannel) bottomChannel = b
    }
    function setWindowBle(kBottom, count) {
        var c = clampCount(count)
        var k = Math.max(0, Math.min(40 - c / 2, kBottom))
        if (c !== numChannels) numChannels = c
        if (k !== bottomLeIndex) bottomLeIndex = k
    }

    // Re-align the window to the newly active grid when the lock changes
    // (mode switch or hybrid lock toggle). BR/EDR b -> LE k snaps to the
    // nearest LE channel; LE k -> BR/EDR b = 2k keeps the left edge within
    // half a MHz.
    onBleLockedChanged: {
        if (bleLocked)
            setWindowBle(Math.round(bottomChannel / 2), numChannels)
        else
            setWindowBredr(bottomLeIndex * 2, numChannels)
    }

    color: "#1e1e1e"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        Label {
            text: qsTr("Channel Spectrum")
            color: "#cccccc"
            font.bold: true
        }

        // Tuner / spectrum strip. Full 2402-2480 MHz band: LE channels on
        // top (advertising 37/38/39 highlighted), BR/EDR channels below,
        // with a draggable capture window. The zone split follows the
        // session type: hybrid 50/50, BLE all-LE, BR/EDR all-BR/EDR.
        ChannelSpectrum {
            id: spectrum
            Layout.fillWidth: true
            Layout.preferredHeight: 170
            sessionTypeIndex: root.sessionTypeIndex
            bottomChannel: root.bottomChannel
            bottomLeIndex: root.bottomLeIndex
            bleLocked: root.bleLocked
            numChannels: root.numChannels
            maxChannels: root.maxChannels
            running: root.running
            brRangeText: root.brRangeText
            leRangeText: root.leRangeText

            onWindowEdited: function (bottom, count, leGrid) {
                if (leGrid)
                    root.setWindowBle(bottom, count)
                else
                    root.setWindowBredr(bottom, count)
            }
        }

        // ---- Capture-window controls -------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 16

            ColumnLayout {
                spacing: 2

                Label {
                    text: qsTr("Channels")
                    color: "#cccccc"
                    font.bold: true
                }

                SpinBox {
                    id: numChannelsSpin
                    enabled: !root.running
                    from: 2
                    to: root.maxChannels
                    stepSize: 2
                    editable: false

                    Component.onCompleted: value = root.numChannels

                    // No binding on `value` — a binding would be broken by
                    // user interaction. Sync down explicitly and write back
                    // only on user edits via valueModified.
                    onValueModified: {
                        if (root.bleLocked)
                            root.setWindowBle(root.bottomLeIndex, value)
                        else
                            root.setWindowBredr(root.bottomChannel, value)
                        if (value !== root.numChannels)
                            value = root.numChannels
                    }
                    Connections {
                        target: root
                        function onNumChannelsChanged() {
                            if (numChannelsSpin.value !== root.numChannels)
                                numChannelsSpin.value = root.numChannels
                        }
                    }

                    textFromValue: function (value) { return value + " ch" }
                    valueFromText: function (text) {
                        return parseInt(text.replace(" ch", ""))
                    }
                }
            }

            ColumnLayout {
                spacing: 2

                Label {
                    text: root.bleLocked ? qsTr("Bottom (LE)") : qsTr("Bottom (BR/EDR)")
                    color: "#cccccc"
                    font.bold: true
                }

                SpinBox {
                    id: bottomChannelSpin
                    enabled: !root.running
                    from: 0
                    // Can't start lower than the window can fit in the band.
                    to: root.bleLocked ? 40 - root.numChannels / 2
                                       : 78 - (root.numChannels - 1)
                    stepSize: 1
                    editable: false

                    Component.onCompleted: value = root.bleLocked ? root.bottomLeIndex
                                                                  : root.bottomChannel

                    onValueModified: {
                        if (root.bleLocked)
                            root.setWindowBle(value, root.numChannels)
                        else
                            root.setWindowBredr(value, root.numChannels)
                        var cur = root.bleLocked ? root.bottomLeIndex : root.bottomChannel
                        if (value !== cur)
                            value = cur
                    }
                    Connections {
                        target: root
                        function onBottomChannelChanged() {
                            if (!root.bleLocked && bottomChannelSpin.value !== root.bottomChannel)
                                bottomChannelSpin.value = root.bottomChannel
                        }
                        function onBottomLeIndexChanged() {
                            if (root.bleLocked && bottomChannelSpin.value !== root.bottomLeIndex)
                                bottomChannelSpin.value = root.bottomLeIndex
                        }
                        function onBleLockedChanged() {
                            bottomChannelSpin.value = root.bleLocked ? root.bottomLeIndex
                                                                     : root.bottomChannel
                        }
                    }

                    textFromValue: function (value) {
                        return root.bleLocked
                               ? "LE " + root.rfToLeLabel(value) + " (" + (2402 + 2 * value) + " MHz)"
                               : value + " (" + (2402 + value) + " MHz)"
                    }
                    valueFromText: function (text) {
                        return parseInt(text.split(" ")[0])
                    }
                }
            }

            ColumnLayout {
                spacing: 2

                Label {
                    text: qsTr("Lock Reference")
                    color: "#cccccc"
                    font.bold: true
                }

                ComboBox {
                    id: lockCombo
                    // Only user-selectable in hybrid; BLE-only sessions are
                    // always BLE-locked, BR/EDR-only always BR/EDR-locked.
                    enabled: !root.running && root.sessionTypeIndex === 0
                    model: ["BR/EDR", "BLE"]

                    Component.onCompleted: currentIndex = root.bleLocked ? 1 : 0

                    onActivated: function (index) {
                        root.lockReference = index
                    }
                    Connections {
                        target: root
                        function onBleLockedChanged() {
                            var want = root.bleLocked ? 1 : 0
                            if (lockCombo.currentIndex !== want)
                                lockCombo.currentIndex = want
                        }
                    }
                }
            }

            Item { Layout.fillWidth: true }

            ColumnLayout {
                spacing: 2
                Layout.alignment: Qt.AlignRight

                Label {
                    text: qsTr("Capture")
                    color: "#cccccc"
                    font.bold: true
                    horizontalAlignment: Text.AlignRight
                    Layout.alignment: Qt.AlignRight
                }

                Label {
                    text: root.captureSummary
                    color: "#9cdcfe"
                    font.family: "Google Sans Code"
                    horizontalAlignment: Text.AlignRight
                    Layout.alignment: Qt.AlignRight
                }
            }
        }

        Label {
            text: root.running
                  ? qsTr("Stop the running session to change the channel layout.")
                  : qsTr("Drag the highlighted window to retune; drag either edge to resize. In hybrid, the lock reference picks the tuning grid: BR/EDR steps 1 MHz (half-MHz LO), BLE steps 2 MHz (whole-MHz LO).")
            color: "#858585"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
        }

        // ---- Existing session/CRC controls --------------------------------
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 1
            color: "#000000"
            Layout.topMargin: 8
        }

        Label {
            text: qsTr("Session Type")
            color: "#cccccc"
            font.bold: true
            Layout.topMargin: 8
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
