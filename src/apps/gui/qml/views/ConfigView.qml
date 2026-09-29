import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import Supertooth

Rectangle {
    id: root

    // 0 = Hybrid (default), 1 = LE, 2 = BR/EDR. Mirrors BACKEND_SESSION_*.
    property int sessionTypeIndex: 0
    // BACKEND_INPUT_* of the selected radio (0 = HackRF, 2 = bladeRF).
    // File replay is hidden from the GUI.
    property int inputType: 0
    property string deviceID: ""
    // Drop LE frames whose CRC fails. Applies to LE and hybrid sessions.
    // Default on, matching the CLI's --enforce-crc default.
    property bool enforceCrc: true
    // Maximum BR/EDR access-code bit errors tolerated by the bitstream decoder.
    // 0 (default) = strict, byte-perfect access-code match.
    property int acErrors: 0
    // RX gains. HackRF: LNA 0,8,...,40 (default 24), VGA even 0-62
    // (default 18), AMP off by default. bladeRF: overall gain 0-60 dB
    // (default 30). Only the fields for the selected inputType are used.
    // Reset to defaults on radio-type switch (see selectInputType).
    property int hackrfLna: 24
    property int hackrfVga: 18
    property bool hackrfAmp: false
    property int bladerfGain: 30
    // Dropdown option lists for the gain boxes. LNA/AMP are small enough
    // to inline in their models below; VGA (even 0-62) and bladeRF (0-60)
    // are built once on startup.
    property var vgaOptions: []
    property var bladerfGainOptions: []
    property bool running: false

    // Device-type selector model, populated from the compiled-in live
    // radios (see RadioDeviceModel.availableInputTypes). Parallel arrays:
    // labels drive the ComboBox, values are the BACKEND_INPUT_* to store.
    property var inputTypeLabels: []
    property var inputTypeValues: []

    // Channel layout. Hybrid and BR/EDR-only sessions always capture on
    // the BR/EDR grid: numChannels = BR/EDR channels (even, 2..maxBredr),
    // window = numChannels MHz, LO at a half-MHz frequency (e.g. 2411.5).
    // LE-only sessions capture on the LE grid: numChannels = LE channels
    // to capture (2..maxBle) from bottomLeIndex, window =
    // numChannels*2 MHz, LO at a whole-MHz frequency.
    // ConfigView is the single source of truth — all writes (spectrum
    // drags) go through setWindowBredr/setWindowBle so clamping is
    // applied uniformly.
    property int bottomChannel: 0       // BR/EDR channel index (hybrid/BR/EDR)
    property int bottomLeIndex: 0       // LE RF channel index (LE-only)
    property int numChannels: 20
    // Per-radio ceilings, shared with the CLI defaults (bladeRF sustains
    // wider windows than HackRF). Refreshed on device-type switch.
    property int maxBredrChannels: 20
    property int maxBleChannels: 10
    // Valid window counts per grid, ascending — the same lane-split set
    // the CLI enforces (not every even count stages: e.g. 22 has no lane
    // split). Drag/resize counts snap down into these. Refreshed on
    // device-type switch.
    property var supportedBredrCounts: []
    property var supportedBleCounts: []
    readonly property int windowMaxChannels: bleLocked ? maxBleChannels : maxBredrChannels

    readonly property bool bleLocked: sessionTypeIndex === 1

    // Derived helpers shared with the spectrum + summary labels.
    readonly property real windowLeftMhz: bleLocked ? 2401 + 2 * bottomLeIndex
                                                     : 2401.5 + bottomChannel
    // Capture-window width in MHz: numChannels for the BR/EDR grid (1 MHz
    // per channel), numChannels*2 for the LE grid (2 MHz per channel).
    readonly property real windowMhz: bleLocked ? numChannels * 2
                                                 : numChannels
    // Sample rate mirrors run_bredr.c: 4 Msps for a 2 MHz window,
    // else window MHz * 1 Msps.
    readonly property real sampleRateHz: windowMhz === 2 ? 4e6 : windowMhz * 1e6
    // LO sits at the center of the capture window — a half-MHz frequency
    // when BR/EDR-locked, a whole-MHz frequency when LE-locked.
    readonly property real loFreqHz: (windowLeftMhz + windowMhz / 2.0) * 1e6

    // ---- Channel ranges covered by the window ------------------------------
    // LE: when LE-locked the window is exactly numChannels LE channels
    // wide from bottomLeIndex; when BR/EDR-locked the edges never land on
    // LE centers so the window spans numChannels/2 LE channels.
    readonly property int leFirstRf: bleLocked ? bottomLeIndex
                                                : Math.max(0, Math.ceil((bottomChannel - 0.5) / 2))
    readonly property int leLastRf: bleLocked ? bottomLeIndex + numChannels - 1
                                               : leFirstRf + numChannels / 2 - 1
    // BR/EDR: native range when BR/EDR-locked; when LE-locked, the
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
    // Hybrid and BR/EDR sessions take numChannels BR/EDR processors and a
    // numChannels-MHz window at a half-MHz LO. LE fans out inside the
    // window from the shared channelizer.
    // bleAdvChannel is the advertising channel whose center lies inside the
    // window (at most one fits a <=20 MHz window), or 0 = none — the hybrid
    // LE worker idles and LE-only sessions fall back to ch37.
    readonly property int backendChannelCount: numChannels
    readonly property int backendBottomChannel: bottomChannel
    // LE-only sessions take their window in LE RF units (numChannels
    // channels from bottomLeIndex).
    readonly property int backendLeChannelCount: numChannels
    readonly property int backendBleAdvChannel: {
        if (leFirstRf <= 0 && leLastRf >= 0) return 37
        if (leFirstRf <= 12 && leLastRf >= 12) return 38
        if (leFirstRf <= 39 && leLastRf >= 39) return 39
        return 0
    }

    // ---- Window clamping (mirrors run_bredr.c validation) --------------
    // Counts snap down into the supported lane-split set so the window the
    // user sees is the window the session actually tunes — the same set
    // the CLI accepts (see backend_supported_counts).
    function snapToList(c, list) {
        c = Math.round(c)
        var best = -1
        for (var i = 0; i < list.length; i++) {
            if (list[i] <= c)
                best = list[i]
            else
                break
        }
        return best >= 0 ? best : (list.length > 0 ? list[0] : c)
    }
    function clampCount(c) {
        if (bleLocked) {
            if (supportedBleCounts.length > 0)
                return Math.max(2, snapToList(c, supportedBleCounts))
            c = Math.round(c)
            return Math.max(2, Math.min(maxBleChannels, c))
        }
        if (supportedBredrCounts.length > 0)
            return snapToList(c, supportedBredrCounts)
        c = Math.round(c / 2) * 2
        return Math.max(2, Math.min(maxBredrChannels, c))
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

    // ---- Device selection -------------------------------------------------
    RadioDeviceModel {
        id: radioDeviceModel
    }

    function updateDeviceCounts() {
        maxBredrChannels = radioDeviceModel.maxBredrCount(root.inputType)
        maxBleChannels = radioDeviceModel.maxBleCount(root.inputType)
        supportedBredrCounts = radioDeviceModel.supportedBredrCounts(root.inputType)
        supportedBleCounts = radioDeviceModel.supportedBleCounts(root.inputType)
    }

    function resetWindowToDefaults() {
        if (root.bleLocked)
            setWindowBle(0, radioDeviceModel.defaultBleCount(root.inputType))
        else
            setWindowBredr(0, radioDeviceModel.defaultBredrCount(root.inputType))
    }

    function refreshDevices() {
        var previousId = deviceIdSelector.currentText
        radioDeviceModel.refresh(root.inputType, true)
        var idx = radioDeviceModel.indexFromIdentifier(previousId)
        if (idx >= 0) {
            deviceIdSelector.currentIndex = idx
            root.deviceID = previousId
        } else if (radioDeviceModel.rowCount() > 0) {
            deviceIdSelector.currentIndex = 0
            root.deviceID = deviceIdSelector.currentText
        } else {
            deviceIdSelector.currentIndex = -1
            root.deviceID = ""
        }
    }

    function selectInputType(value) {
        var idx = root.inputTypeValues.indexOf(value)
        if (idx >= 0)
            inputTypeSelector.currentIndex = idx
        if (value === root.inputType) {
            refreshDevices()
            return
        }
        root.inputType = value
        resetGainsToDefaults()
        updateDeviceCounts()
        resetWindowToDefaults()
        refreshDevices()
    }

    function resetGainsToDefaults() {
        root.hackrfLna = 24
        root.hackrfVga = 18
        root.hackrfAmp = false
        root.bladerfGain = 30
    }

    Component.onCompleted: {
        var vga = []
        for (var v = 0; v <= 62; v += 2)
            vga.push(v)
        root.vgaOptions = vga
        var gains = []
        for (var g = 0; g <= 60; g++)
            gains.push(g)
        root.bladerfGainOptions = gains
        var inputs = radioDeviceModel.availableInputTypes()
        var labels = []
        var values = []
        for (var i = 0; i < inputs.length; i++) {
            labels.push(inputs[i].label)
            values.push(inputs[i].inputType)
        }
        root.inputTypeLabels = labels
        root.inputTypeValues = values
        if (values.length > 0) {
            root.inputType = values[0]
            inputTypeSelector.currentIndex = 0
        }
        updateDeviceCounts()
        resetWindowToDefaults()
        refreshDevices()
    }

    // Re-align the window on session-type switch: reset to the new grid's
    // device-specific defaults.
    onBleLockedChanged: {
        resetWindowToDefaults()
    }

    color: "#1e1e1e"

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8

        // Tuner / spectrum strip. Full 2402-2480 MHz band: LE channels on
        // top (advertising 37/38/39 highlighted), BR/EDR channels below,
        // with a draggable capture window. The zone split follows the
        // session type: hybrid 50/50, LE all-LE, BR/EDR all-BR/EDR.
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
            validCounts: root.bleLocked ? root.supportedBleCounts : root.supportedBredrCounts
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

        Label {
            text: root.captureSummary
            color: "#858585"
            font.family: "Google Sans Code"
            font.pixelSize: 11
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignHCenter
        }

        // ---- Center bar: session + device selection ----------------------
        // Top margin leaves headroom for the floating box labels, which
        // paint just above the row (see LabeledComboBox).
        RowLayout {
            Layout.fillWidth: true
            Layout.topMargin: 6
            spacing: 12

            LabeledComboBox {
                id: sessionTypeSelector
                enabled: !root.running
                Layout.preferredWidth: 120
                label: qsTr("Mode")
                model: ["Hybrid", "LE", "BR/EDR"]
                currentIndex: root.sessionTypeIndex

                onActivated: function (index) {
                    root.sessionTypeIndex = index
                }
            }

            LabeledComboBox {
                id: inputTypeSelector
                enabled: !root.running && root.inputTypeValues.length > 0
                Layout.preferredWidth: 120
                label: qsTr("Device")
                model: root.inputTypeLabels

                onActivated: function (index) {
                    root.selectInputType(root.inputTypeValues[index])
                }
            }

            LabeledComboBox {
                id: deviceIdSelector
                enabled: !root.running
                Layout.fillWidth: true
                Layout.minimumWidth: 140
                label: qsTr("Identifier")

                model: radioDeviceModel
                textRole: "display"

                onActivated: function (index) {
                    root.deviceID = deviceIdSelector.currentText
                }
            }

            // ---- RX gains (right side, before refresh) -------------------
            // HackRF: LNA/VGA/AMP dropdowns. bladeRF: single gain dropdown.
            // Each label floats on its box's top border (LabeledComboBox).
            // Only the active radio's controls are visible; the device
            // selector above holds Layout.fillWidth so it shrinks to fit.
            RowLayout {
                spacing: 4
                visible: root.inputType === 0

                LabeledComboBox {
                    id: lnaGain
                    enabled: !root.running
                    Layout.preferredWidth: 88
                    label: qsTr("LNA")
                    model: [0, 8, 16, 24, 32, 40]
                    currentIndex: model.indexOf(root.hackrfLna)

                    onActivated: function (index) {
                        root.hackrfLna = lnaGain.model[index]
                    }
                }
                LabeledComboBox {
                    id: vgaGain
                    enabled: !root.running
                    Layout.preferredWidth: 88
                    label: qsTr("VGA")
                    model: root.vgaOptions
                    currentIndex: model.indexOf(root.hackrfVga)

                    onActivated: function (index) {
                        root.hackrfVga = vgaGain.model[index]
                    }
                }
                LabeledComboBox {
                    id: ampGain
                    enabled: !root.running
                    Layout.preferredWidth: 100
                    label: qsTr("AMP")
                    model: ["Off", "On"]
                    currentIndex: root.hackrfAmp ? 1 : 0

                    onActivated: function (index) {
                        root.hackrfAmp = (index === 1)
                    }
                }
            }

            RowLayout {
                spacing: 4
                visible: root.inputType === 2

                LabeledComboBox {
                    id: bladerfGainBox
                    enabled: !root.running
                    Layout.preferredWidth: 88
                    label: qsTr("Gain")
                    model: root.bladerfGainOptions
                    currentIndex: model.indexOf(root.bladerfGain)

                    onActivated: function (index) {
                        root.bladerfGain = bladerfGainBox.model[index]
                    }
                }
            }

            Button {
                id: refreshButton
                topInset: 0
                bottomInset: 0
                leftInset: 0
                rightInset: 0

                enabled: !root.running
                Layout.preferredWidth: 40
                Layout.preferredHeight: 40

                Image {
                    source: "/assets/images/refresh.svg"
                    anchors.centerIn: parent
                    width: 24
                    height: 24
                    opacity: refreshButton.enabled ? 1.0 : 0.4
                }

                onClicked: {
                    root.refreshDevices()
                }
            }
        }

        // ---- Protocol-specific settings ----------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            // BR/EDR panel (left). Both panels share the taller column's
            // height so the boxes always match.
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(bredrCol.implicitHeight, leCol.implicitHeight) + 16

                Rectangle {
                    anchors.fill: parent
                    color: "#252525"
                }

                ColumnLayout {
                    id: bredrCol
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    Label {
                        text: qsTr("BR/EDR")
                        color: "#cccccc"
                        font.bold: true
                    }

                    Label {
                        text: qsTr("Access-Code Errors")
                        color: "#cccccc"
                    }

                    SpinBox {
                        id: acErrorsSpin
                        enabled: !root.running && root.sessionTypeIndex !== 1
                        from: 0
                        to: 8
                        stepSize: 1
                        value: root.acErrors

                        onValueChanged: root.acErrors = value
                    }
                }

                Rectangle {
                    id: bredrDim
                    anchors.fill: parent
                    color: "black"
                    opacity: 0.6
                    visible: root.sessionTypeIndex === 1
                }
                MouseArea {
                    anchors.fill: parent
                    enabled: bredrDim.visible
                    onPressed: function (mouse) { mouse.accepted = true }
                }
            }

            // LE panel (right). Height matches the BR/EDR panel (see above).
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: Math.max(bredrCol.implicitHeight, leCol.implicitHeight) + 16

                Rectangle {
                    anchors.fill: parent
                    color: "#252525"
                }

                ColumnLayout {
                    id: leCol
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 6

                    Label {
                        text: qsTr("LE")
                        color: "#cccccc"
                        font.bold: true
                    }

                    Label {
                        text: qsTr("CRC Enforcement")
                        color: "#cccccc"
                    }

                    Switch {
                        id: enforceCrcSwitch
                        enabled: !root.running && root.sessionTypeIndex !== 2
                        checked: root.enforceCrc
                        text: checked ? qsTr("On") : qsTr("Off")

                        onToggled: root.enforceCrc = checked
                    }
                }

                Rectangle {
                    id: leDim
                    anchors.fill: parent
                    color: "black"
                    opacity: 0.6
                    visible: root.sessionTypeIndex === 2
                }
                MouseArea {
                    anchors.fill: parent
                    enabled: leDim.visible
                    onPressed: function (mouse) { mouse.accepted = true }
                }
            }
        }

        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }
}
