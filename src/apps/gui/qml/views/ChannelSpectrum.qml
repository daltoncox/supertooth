import QtQuick
import QtQuick.Controls

// Static channel-spectrum strip spanning the full Bluetooth band
// (2402-2480 MHz).
//
// The bar is split vertically into two zones on a shared frequency axis:
//   - Top:    all 40 LE channels (2 MHz spacing; advertising channels
//             37/38/39 highlighted, data channels 0-36 shown alongside).
//   - Bottom: all 79 BR/EDR channels (1 MHz each), boxes centered on their
//             channel frequency so LE 37/38/39 line up with BR/EDR 0/24/78.
// Hybrid mode splits the bar 50/50; BLE mode expands the LE zone to fill
// the bar; BR/EDR mode expands the BR/EDR zone to fill the bar.
//
// A capture-window overlay ("tuner box") spans numChannels MHz starting at
// the window's left edge, with a dashed LO tick at its center. The window
// snaps to one of two grids depending on the lock reference:
//   - BR/EDR lock: left edge = 2401.5 + bottom (1 MHz drag steps, LO at
//     a half-MHz frequency).
//   - BLE lock:    left edge = 2401 + 2*bottomLeIndex (2 MHz drag steps,
//     LO at a whole-MHz frequency).
//
// Interaction:
//   - Drag the body of the tuner box to move the window (grid-snapped).
//   - Drag the right edge to resize numChannels (even only, 2..maxChannels).
//   - Drag the left edge to resize: bottom channel and count both change,
//     right edge stays fixed.
//   - Click anywhere on the bar to jump the box's left edge there.
//
// Pure presentation: this view never assigns its own properties (doing so
// would break the owner's bindings into it). Edits are reported via the
// windowEdited signal (in whichever grid is active); CaptureView owns the
// channel state and is the single source of truth.
Item {
    id: root

    // 0 = Hybrid, 1 = BLE, 2 = BR/EDR. Mirrors BACKEND_SESSION_*.
    property int sessionTypeIndex: 0
    // Window bottom on the BR/EDR grid (channel index 0..78).
    property int bottomChannel: 0
    // Window bottom on the LE grid (RF channel index 0..39).
    property int bottomLeIndex: 0
    // True when the window snaps to the LE grid (2 MHz steps); false when
    // it snaps to the BR/EDR grid (1 MHz steps).
    property bool bleLocked: false
    // Window width in MHz (even, 2..maxChannels).
    property int numChannels: 20
    // Max simultaneous capture channels supported by the pipeline.
    property int maxChannels: 20
    // True while a session is running — locks the tuner.
    property bool running: false
    // Range labels drawn inside the tuner box (computed by CaptureView).
    property string brRangeText: ""
    property string leRangeText: ""

    // Emitted when the user edits the capture window by dragging. bottom
    // is expressed in the active grid (leGrid=true -> LE RF index).
    signal windowEdited(int bottom, int count, bool leGrid)

    // All 40 LE RF channels (2 MHz spacing) mapped to their LE channel
    // numbers: RF 0 -> adv 37, RF 1..11 -> data 0..10, RF 12 -> adv 38,
    // RF 13..38 -> data 11..36, RF 39 -> adv 39.
    readonly property var leChannels: {
        var arr = []
        for (var rf = 0; rf < 40; rf++) {
            var label
            var adv = false
            if (rf === 0)       { label = "37"; adv = true }
            else if (rf === 12) { label = "38"; adv = true }
            else if (rf === 39) { label = "39"; adv = true }
            else if (rf < 12)   { label = String(rf - 1) }   // LE 0..10
            else                { label = String(rf - 2) }   // LE 11..36
            arr.push({ label: label, freq: 2402 + 2 * rf, adv: adv })
        }
        return arr
    }

    // Fixed band geometry: BR/EDR channel i is centered on 2402 + i MHz
    // (0..78). The x mapping is padded 1 MHz past the band on each side
    // (2401..2481) so the outermost channel boxes (LE 37 spans 2401-2403,
    // BR/EDR 0 spans 2401.5-2402.5, etc.) end flush with the gutters
    // instead of overhanging them — keeping the zone tags clear. The axis
    // line itself is drawn exactly 2402..2480 so both ends land on the
    // outermost labeled ticks.
    readonly property int bandFirstChannel: 0
    readonly property int bandLastChannel: 78
    readonly property int bandChannelCount: bandLastChannel - bandFirstChannel + 1  // 79
    readonly property real bandStartMhz: 2401.0
    readonly property real bandEndMhz: 2481.0

    // Window geometry in MHz, driven by the active grid.
    readonly property real windowLeftMhz: bleLocked ? 2401 + 2 * bottomLeIndex
                                                    : 2401.5 + bottomChannel
    readonly property real windowRightMhz: windowLeftMhz + numChannels
    readonly property real windowCenterMhz: windowLeftMhz + numChannels / 2.0

    // Vertical split between the LE zone (top) and the BR/EDR zone
    // (bottom): 1.0 = all LE, 0.0 = all BR/EDR, 0.5 = even hybrid split.
    property real leFraction: sessionTypeIndex === 1 ? 1.0
                            : sessionTypeIndex === 2 ? 0.0 : 0.5
    Behavior on leFraction {
        NumberAnimation { duration: 160; easing.type: Easing.InOutQuad }
    }
    onLeFractionChanged: canvas.requestPaint()

    // Left/right gutter for the zone tags + axis overhang. Zone labels are
    // right-aligned inside it so they never touch the channel bars.
    readonly property int margin: 48

    // ---- Geometry helpers -------------------------------------------------
    function mhzToX(mhz) {
        var usable = width - 2 * margin
        return margin + (mhz - bandStartMhz) / (bandEndMhz - bandStartMhz) * usable
    }
    function xToMhz(x) {
        var usable = width - 2 * margin
        return bandStartMhz + (x - margin) / usable * (bandEndMhz - bandStartMhz)
    }

    // ---- Clamping (pure — applied before emitting windowEdited) -----------
    function clampCount(count) {
        var c = Math.round(count / 2) * 2
        return Math.max(2, Math.min(maxChannels, c))
    }
    function clampBredrBottom(bottom, count) {
        var maxBottom = bandLastChannel - (count - 1)
        return Math.max(bandFirstChannel, Math.min(maxBottom, bottom))
    }
    function clampLeBottom(k, count) {
        var maxK = 40 - count / 2
        return Math.max(0, Math.min(maxK, k))
    }

    // Vertical geometry shared by the Canvas and the MouseArea.
    function barMetrics() {
        var topPad = 6
        var axisY = height - 20
        var barBottom = axisY - 6
        var areaTop = topPad
        var areaH = barBottom - areaTop
        var leH = areaH * leFraction
        var brH = areaH * (1 - leFraction)
        var gap = (leH > 3 && brH > 3) ? 4 : 0
        leH -= gap / 2
        brH -= gap / 2
        return {
            axisY: axisY,
            areaTop: areaTop,
            areaH: areaH,
            barBottom: barBottom,
            gap: gap,
            leTop: areaTop,
            leH: leH,
            brTop: areaTop + leH + gap,
            brH: brH
        }
    }

    // Repaint whenever any state that affects the drawing changes.
    onSessionTypeIndexChanged: canvas.requestPaint()
    onBottomChannelChanged: canvas.requestPaint()
    onBottomLeIndexChanged: canvas.requestPaint()
    onBleLockedChanged: canvas.requestPaint()
    onNumChannelsChanged: canvas.requestPaint()
    onMaxChannelsChanged: canvas.requestPaint()
    onBrRangeTextChanged: canvas.requestPaint()
    onLeRangeTextChanged: canvas.requestPaint()

    // ---- Painting ---------------------------------------------------------
    Canvas {
        id: canvas
        anchors.fill: parent

        onPaint: {
            var ctx = getContext("2d")
            ctx.reset()

            var w = root.width
            var h = root.height
            var m = root.barMetrics()

            // Background
            ctx.fillStyle = "#1e1e1e"
            ctx.fillRect(0, 0, w, h)

            // ---- LE zone (top) -------------------------------------------
            if (m.leH > 3) {
                for (var i = 0; i < root.leChannels.length; i++) {
                    var le = root.leChannels[i]
                    var lx0 = root.mhzToX(le.freq - 1)
                    var lx1 = root.mhzToX(le.freq + 1)
                    ctx.fillStyle = le.adv ? "#4EC9B0" : "#252525"
                    ctx.fillRect(lx0, m.leTop, Math.max(1, lx1 - lx0 - 1), m.leH)

                    // Channel number inside the box (never clipped).
                    if (m.leH >= 12 && lx1 - lx0 >= 8) {
                        ctx.fillStyle = le.adv ? "#0b2e28" : "#858585"
                        ctx.font = (le.adv ? "bold " : "") + "9px 'Google Sans Code'"
                        ctx.textAlign = "center"
                        ctx.textBaseline = "middle"
                        ctx.fillText(le.label, (lx0 + lx1) / 2, m.leTop + m.leH / 2)
                    }
                }
                ctx.fillStyle = "#5a5a5a"
                ctx.font = "8px 'Google Sans Code'"
                ctx.textAlign = "right"
                ctx.textBaseline = "middle"
                ctx.fillText("LE", root.margin - 8, m.leTop + m.leH / 2)
            }

            // ---- BR/EDR zone (bottom) ------------------------------------
            // Boxes are centered on their channel frequency: channel b
            // spans (2402+b) +/- 0.5 MHz.
            if (m.brH > 3) {
                for (var b = 0; b < root.bandChannelCount; b++) {
                    var bx0 = root.mhzToX(2402 + b - 0.5)
                    var bx1 = root.mhzToX(2402 + b + 0.5)
                    ctx.fillStyle = "#2d2d2d"
                    ctx.fillRect(bx0, m.brTop, Math.max(1, bx1 - bx0 - 1), m.brH)

                    // Every 10th channel number inside the box.
                    if (m.brH >= 12 && (b % 10 === 0 || b === root.bandLastChannel)) {
                        ctx.fillStyle = "#5a5a5a"
                        ctx.font = "8px 'Google Sans Code'"
                        ctx.textAlign = "center"
                        ctx.textBaseline = "middle"
                        ctx.fillText(String(b), (bx0 + bx1) / 2, m.brTop + m.brH / 2)
                    }
                }
                ctx.fillStyle = "#5a5a5a"
                ctx.font = "8px 'Google Sans Code'"
                ctx.textAlign = "right"
                ctx.textBaseline = "middle"
                ctx.fillText("BR/EDR", root.margin - 8, m.brTop + m.brH / 2)
            }
            ctx.textBaseline = "alphabetic"

            // ---- LO tick --------------------------------------------------
            var loX = root.mhzToX(root.windowCenterMhz)
            ctx.strokeStyle = "#dcdcaa"
            ctx.lineWidth = 1
            ctx.setLineDash([3, 3])
            ctx.beginPath()
            ctx.moveTo(loX, m.areaTop)
            ctx.lineTo(loX, m.axisY)
            ctx.stroke()
            ctx.setLineDash([])

            // ---- Capture-window overlay ----------------------------------
            var wx0 = root.mhzToX(root.windowLeftMhz)
            var wx1 = root.mhzToX(root.windowRightMhz)
            var wy = m.areaTop - 2
            var wh = m.areaH + 4

            ctx.fillStyle = "rgba(9, 71, 113, 0.35)"
            ctx.fillRect(wx0, wy, wx1 - wx0, wh)
            ctx.strokeStyle = "#4fc1ff"
            ctx.lineWidth = 1
            ctx.strokeRect(wx0 + 0.5, wy + 0.5, wx1 - wx0 - 1, wh - 1)

            // Channel-range labels centered inside the box: LE range on
            // top, BR/EDR range below. In single-protocol modes only that
            // protocol's range is shown.
            var cx = (wx0 + wx1) / 2
            var cy = m.areaTop + m.areaH / 2
            var showBr = root.sessionTypeIndex !== 1
            var showLe = root.sessionTypeIndex !== 2
            if (wx1 - wx0 >= 64 && m.areaH >= 28 && showBr && showLe) {
                ctx.fillStyle = "#4fc1ff"
                ctx.font = "bold 11px 'Google Sans Code'"
                ctx.textAlign = "center"
                ctx.textBaseline = "middle"
                ctx.fillText("LE " + root.leRangeText, cx, cy - 7)
                ctx.fillText("BR " + root.brRangeText, cx, cy + 8)
                ctx.textBaseline = "alphabetic"
            } else if (wx1 - wx0 >= 40 && m.areaH >= 16) {
                ctx.fillStyle = "#4fc1ff"
                ctx.font = "bold 11px 'Google Sans Code'"
                ctx.textAlign = "center"
                ctx.textBaseline = "middle"
                // Single line: the locked grid's own range.
                ctx.fillText(root.bleLocked ? "LE " + root.leRangeText
                                            : "BR " + root.brRangeText,
                             cx, cy)
                ctx.textBaseline = "alphabetic"
            }

            // ---- Axis baseline + MHz ticks --------------------------------
            // The axis runs exactly 2402..2480 (inset from the gutters by
            // the 1 MHz mapping pad); the bars extend the extra MHz to the
            // gutter edges.
            ctx.strokeStyle = "#3d3d3d"
            ctx.lineWidth = 1
            ctx.beginPath()
            ctx.moveTo(root.mhzToX(2402), m.axisY)
            ctx.lineTo(root.mhzToX(2480), m.axisY)
            ctx.stroke()

            ctx.fillStyle = "#858585"
            ctx.font = "11px 'Google Sans Code'"
            ctx.textAlign = "center"
            var tickMhz = [2402, 2420, 2440, 2460, 2480]
            for (var t = 0; t < tickMhz.length; t++) {
                var mx = root.mhzToX(tickMhz[t])
                ctx.beginPath()
                ctx.moveTo(mx, m.axisY)
                ctx.lineTo(mx, m.axisY + 4)
                ctx.stroke()
                ctx.fillText(tickMhz[t], mx, m.axisY + 16)
            }
        }

        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
    }

    // ---- Interaction ------------------------------------------------------
    MouseArea {
        id: dragArea
        anchors.fill: parent
        enabled: !root.running
        hoverEnabled: true

        readonly property int handleHit: 8

        property int dragMode: 0        // 0=none, 1=move, 2=resize-right, 3=resize-left
        property real grabOffsetMhz: 0  // move: distance from box left edge at press
        property real startLeftMhz: 0   // window edges captured at press
        property real startRightMhz: 0

        function windowLeftX()  { return root.mhzToX(root.windowLeftMhz) }
        function windowRightX() { return root.mhzToX(root.windowRightMhz) }

        function overLeftHandle(mx)  { return Math.abs(mx - windowLeftX()) <= handleHit }
        function overRightHandle(mx) { return Math.abs(mx - windowRightX()) <= handleHit }
        function overBody(mx) { return mx > windowLeftX() + handleHit && mx < windowRightX() - handleHit }

        cursorShape: {
            if (dragMode === 2 || dragMode === 3) return Qt.SizeHorCursor
            if (dragMode === 1) return Qt.ClosedHandCursor
            if (!enabled) return Qt.ArrowCursor
            if (overLeftHandle(mouseX) || overRightHandle(mouseX)) return Qt.SizeHorCursor
            if (mouseX >= windowLeftX() && mouseX <= windowRightX()) return Qt.OpenHandCursor
            return Qt.ArrowCursor
        }

        onPressed: function (mouse) {
            var m = root.barMetrics()
            if (mouse.y > m.barBottom + 2) {
                dragMode = 0
                return
            }
            startLeftMhz = root.windowLeftMhz
            startRightMhz = root.windowRightMhz
            if (overLeftHandle(mouse.x)) {
                dragMode = 3
            } else if (overRightHandle(mouse.x)) {
                dragMode = 2
            } else if (mouse.x >= windowLeftX() && mouse.x <= windowRightX()) {
                dragMode = 1
                grabOffsetMhz = root.xToMhz(mouse.x) - root.windowLeftMhz
            } else {
                // Click outside: jump the box so its left edge lands here.
                moveWindowTo(root.xToMhz(mouse.x))
                dragMode = 0
                return
            }
        }

        onPositionChanged: function (mouse) {
            if (dragMode === 1) {
                moveWindowTo(root.xToMhz(mouse.x) - grabOffsetMhz)
            } else if (dragMode === 2) {
                resizeRightTo(root.xToMhz(mouse.x))
            } else if (dragMode === 3) {
                resizeLeftTo(root.xToMhz(mouse.x))
            }
        }

        onReleased: dragMode = 0
        onCanceled: dragMode = 0

        // Move the window so its left edge sits at leftMhz, snapped to the
        // active grid (1 MHz steps BR/EDR-locked, 2 MHz steps BLE-locked).
        function moveWindowTo(leftMhz) {
            if (root.bleLocked) {
                var k = Math.round((leftMhz - 2401) / 2)
                root.windowEdited(root.clampLeBottom(k, root.numChannels),
                                  root.numChannels, true)
            } else {
                var b = Math.round(leftMhz - 2401.5)
                root.windowEdited(root.clampBredrBottom(b, root.numChannels),
                                  root.numChannels, false)
            }
        }

        // Right edge follows the mouse; left edge stays fixed. Count snaps
        // to even MHz and the window stays inside the band.
        function resizeRightTo(rightMhz) {
            var c = root.clampCount(rightMhz - startLeftMhz)
            var maxForBand = root.bleLocked ? 80 - 2 * root.bottomLeIndex
                                            : root.bandChannelCount - root.bottomChannel
            if (c > maxForBand)
                c = Math.max(2, maxForBand - (maxForBand % 2))
            root.windowEdited(root.bleLocked ? root.bottomLeIndex : root.bottomChannel,
                              c, root.bleLocked)
        }

        // Left edge follows the mouse; right edge stays fixed, so both the
        // bottom channel and the count change. Count snaps to even MHz.
        function resizeLeftTo(leftMhz) {
            var c = root.clampCount(startRightMhz - leftMhz)
            if (root.bleLocked) {
                var k = Math.round((startRightMhz - c - 2401) / 2)
                if (k < 0) {
                    c = root.clampCount(startRightMhz - 2401)
                    k = Math.round((startRightMhz - c - 2401) / 2)
                }
                root.windowEdited(k, c, true)
            } else {
                var b = Math.round(startRightMhz - c - 2401.5)
                if (b < 0) {
                    c = root.clampCount(startRightMhz - 2401.5)
                    b = Math.round(startRightMhz - c - 2401.5)
                }
                root.windowEdited(b, c, false)
            }
        }
    }
}
