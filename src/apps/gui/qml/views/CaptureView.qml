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
    // Maximum BR/EDR access-code bit errors tolerated by the bitstream decoder.
    // 0 (default) = strict, byte-perfect access-code match.
    property int acErrors: 0
    property bool running: false

    // Channel layout. numChannels is the count of channels in the active
    // grid, and the window's bottom edge snaps to one of two grids:
    //   - BR/EDR lock: numChannels = BR/EDR channels (even, 2..maxChannels),
    //     window = numChannels MHz, LO at a half-MHz frequency (e.g. 2411.5).
    //   - BLE lock: numChannels = BLE channels to capture (2..maxBleChannels)
    //     from bottomLeIndex, window = numChannels*2 MHz, LO at a whole-MHz
    //     frequency.
    // The lock is user-selectable in hybrid mode; BLE-only sessions are
    // always BLE-locked and BR/EDR-only sessions always BR/EDR-locked.
    // CaptureView is the single source of truth — all writes (SpinBoxes,
    // spectrum drags) go through setWindowBredr/setWindowBle so clamping
    // is applied uniformly.
    property int lockReference: 0       // 0 = BR/EDR, 1 = BLE (hybrid only)
    property int bottomChannel: 0       // BR/EDR channel index (BR/EDR lock)
    property int bottomLeIndex: 0       // LE RF channel index (BLE lock)
    property int numChannels: 20
    readonly property int maxChannels: 20        // RECEIVER_BREDR_MAX_CHANNELS
    readonly property int maxBleChannels: 10     // BLE_SESSION_MAX_CHANNELS
    readonly property int windowMaxChannels: bleLocked ? maxBleChannels : maxChannels

    readonly property bool bleLocked: sessionTypeIndex === 1 ? true
                                    : sessionTypeIndex === 2 ? false
                                    : lockReference === 1

    // Derived helpers shared with the spectrum + summary labels.
    readonly property real windowLeftMhz: bleLocked ? 2401 + 2 * bottomLeIndex
                                                    : 2401.5 + bottomChannel
    // Capture-window width in MHz: numChannels for the BR/EDR grid (1 MHz
    // per channel), numChannels*2 for the BLE grid (2 MHz per channel).
    readonly property real windowMhz: bleLocked ? numChannels * 2
                                                : numChannels
    // Sample rate mirrors supertooth-bredr.c: 4 Msps for a 2 MHz window,
    // else window MHz * 1 Msps.
    readonly property real sampleRateHz: windowMhz === 2 ? 4e6 : windowMhz * 1e6
    // LO sits at the center of the capture window — a half-MHz frequency
    // when BR/EDR-locked, a whole-MHz frequency when BLE-locked.
    readonly property real loFreqHz: (windowLeftMhz + windowMhz / 2.0) * 1e6

    // ---- Channel ranges covered by the window ------------------------------
    // LE: when BLE-locked the window is exactly numChannels LE channels
    // wide from bottomLeIndex; when BR/EDR-locked the edges never land on
    // LE centers so the window spans numChannels/2 LE channels.
    readonly property int leFirstRf: bleLocked ? bottomLeIndex
                                               : Math.max(0, Math.ceil((bottomChannel - 0.5) / 2))
    readonly property int leLastRf: bleLocked ? bottomLeIndex + numChannels - 1
                                              : leFirstRf + numChannels / 2 - 1
    // BR/EDR: native range when BR/EDR-locked; when BLE-locked, the
    // channels whose centers fall strictly inside the window (channels
    // centered exactly on an edge are half out of band).
    readonly property int brFirstCh: bleLocked ? Math.min(78, bottomLeIndex * 2)
                                               : bottomChannel
    readonly property int brLastCh: bleLocked ? Math.min(78, bottomLeIndex * 2 + windowMhz - 2)
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
    //   - LE grid: numChannels BLE channels from bottomLeIndex, window =
    //     2*numChannels MHz, LO at a whole MHz; the backend hybrid path
    //     takes an odd channel_count one MHz wider than the window, so
    //     2*numChannels-1 is sent; the two BR/EDR channels centered on the
    //     Nyquist edges are not processed.
    // bleAdvChannel is the advertising channel whose center lies inside the
    // window (at most one fits a <=20 MHz window), or 0 = none — the hybrid
    // BLE worker idles and BLE-only sessions fall back to ch37.
    readonly property int backendChannelCount: bleLocked ? numChannels * 2 - 1
                                                         : numChannels
    readonly property int backendBottomChannel: bleLocked ? Math.min(78, bottomLeIndex * 2)
                                                          : bottomChannel
    // BLE-only sessions take their window in LE RF units (numChannels
    // channels from bottomLeIndex).
    readonly property int backendLeChannelCount: numChannels
    readonly property int backendLeGrid: bleLocked ? 1 : 0
    readonly property int backendBleAdvChannel: {
        if (leFirstRf <= 0 && leLastRf >= 0) return 37
        if (leFirstRf <= 12 && leLastRf >= 12) return 38
        if (leFirstRf <= 39 && leLastRf >= 39) return 39
        return 0
    }

    // ---- Window clamping (mirrors supertooth-bredr.c validation) --------------
    function clampCount(c) {
        if (bleLocked) {
            c = Math.round(c)
            return Math.max(2, Math.min(maxBleChannels, c))
        }
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
        var k = Math.max(0, Math.min(40 - c, kBottom))
        if (c !== numChannels) numChannels = c
        if (k !== bottomLeIndex) bottomLeIndex = k
    }

    // Re-align the window when the lock changes (mode switch or hybrid lock
    // toggle): reset to the new grid's defaults — the lowest bottom channel
    // (0) and the maximum channel count (maxBleChannels/maxChannels).
    onBleLockedChanged: {
        if (bleLocked)
            setWindowBle(0, maxBleChannels)
        else
            setWindowBredr(0, maxChannels)
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
            maxChannels: root.windowMaxChannels
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
                    to: root.bleLocked ? root.maxBleChannels : root.maxChannels
                    stepSize: root.bleLocked ? 1 : 2
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
                            // Defer until the SpinBox's `to`/`stepSize`
                            // bindings have re-evaluated for the new grid.
                            // Otherwise writing `value` while `to` is still
                            // the previous grid's max clamps it (e.g. 10
                            // after switching back to BR/EDR), leaving the
                            // selector out of sync with numChannels.
                            Qt.callLater(function () {
                                if (numChannelsSpin.value !== root.numChannels)
                                    numChannelsSpin.value = root.numChannels
                            })
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
                    to: root.bleLocked ? 40 - root.numChannels
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
                        // All writes are deferred until the SpinBox's
                        // `to`/`stepSize` bindings have re-evaluated for the
                        // new grid. Writing while `to` still reflects the old
                        // grid can clamp `value` (e.g. a BR/EDR bottom clamped
                        // by the transient LE max), leaving the selector stale.
                        function onBottomChannelChanged() {
                            Qt.callLater(function () {
                                if (!root.bleLocked && bottomChannelSpin.value !== root.bottomChannel)
                                    bottomChannelSpin.value = root.bottomChannel
                            })
                        }
                        function onBottomLeIndexChanged() {
                            Qt.callLater(function () {
                                if (root.bleLocked && bottomChannelSpin.value !== root.bottomLeIndex)
                                    bottomChannelSpin.value = root.bottomLeIndex
                            })
                        }
                        function onBleLockedChanged() {
                            Qt.callLater(function () {
                                var target = root.bleLocked ? root.bottomLeIndex
                                                            : root.bottomChannel
                                // QQuickSpinBox::setValue early-returns (and
                                // skips updateDisplayText()) when the value is
                                // unchanged. The reset sets bottom to its default
                                // (0) on every switch, so with the value already
                                // at the default the grid-dependent label would
                                // stay stale. Round-trip the value to force the
                                // display to re-render with the new formatter.
                                if (bottomChannelSpin.value === target) {
                                    var nudge = target < bottomChannelSpin.to ? 1 : -1
                                    bottomChannelSpin.value = target + nudge
                                }
                                bottomChannelSpin.value = target
                            })
                        }
                    }

                    // Qt's SpinBox only recomputes displayText (via
                    // textFromValue) when `value` changes, not when the
                    // formatter or other state changes. So the formatter
                    // itself may close over bleLocked; see onBleLockedChanged
                    // below for forcing a re-render when the reset leaves the
                    // value unchanged.
                    textFromValue: function (value) {
                        return root.bleLocked
                               ? "LE " + root.rfToLeLabel(value) + " (" + (2402 + 2 * value) + " MHz)"
                               : value + " (" + (2402 + value) + " MHz)"
                    }
                    valueFromText: function (text) {
                        var m = text.match(/LE (\d+)/)
                        return m ? parseInt(m[1]) : parseInt(text.split(" ")[0])
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

        Label {
            text: qsTr("Access-Code Errors")
            color: "#cccccc"
            font.bold: true
            Layout.topMargin: 8
        }

        SpinBox {
            id: acErrorsSpin
            enabled: !root.running
            from: 0
            to: 8
            stepSize: 1
            value: root.acErrors

            onValueChanged: root.acErrors = value
        }

        Label {
            text: qsTr("Maximum bit errors tolerated when matching a BR/EDR access code. 0 (default) requires a byte-perfect match and suppresses spurious devices from bit errors; raise it in noisy captures if too few packets are detected.")
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
