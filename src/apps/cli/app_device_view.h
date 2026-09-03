#ifndef APP_DEVICE_VIEW_H
#define APP_DEVICE_VIEW_H

#include <stddef.h>

#include "session.h"

/**
 * @brief Live 1 Hz device/piconet table for the CLI `-v devices` mode.
 *
 * Mirrors the GUI DeviceListView: it polls the core's protocol-agnostic
 * device/piconet snapshots (BR/EDR devices + piconets, BLE devices +
 * piconets), derives the same Type / Identifier columns, and computes a
 * 1-second packet rate from the delta of each entity's total packet count.
 *
 * The table is redrawn in place on a TTY (so it stays put on screen) and
 * appended when stdout is piped (so logs capture every snapshot). Callers
 * must keep the session running (e.g. inside session_run) while the view is
 * active, and early-return from their per-packet callbacks so the two output
 * paths never interleave.
 */
typedef struct app_device_view app_device_view_t;

/* Start the background poller/printer. Returns NULL on allocation failure.
 * @p session must outlive the returned view. */
app_device_view_t *app_device_view_start(session_t *session);

/* Stop the poller, join its thread, and (on a TTY) leave the cursor below
 * the final table so subsequent output (e.g. the session summary) follows
 * cleanly. Safe to call with NULL. */
void app_device_view_stop(app_device_view_t *view);

#endif /* APP_DEVICE_VIEW_H */
