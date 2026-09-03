#ifndef APP_SUMMARY_VIEW_H
#define APP_SUMMARY_VIEW_H

#include "receive_event_models.h"
#include "device_models.h"

/**
 * @brief Unified CLI summary table (mirrors the GUI FrameListView).
 *
 * Columns are the GUI FrameListView columns minus Info:
 *   No. | Time | RSSI | Protocol | Ch | Address | Source | Destination | Type
 *
 * - Time is seconds since capture start (radio_start_sample_index /
 *   radio_sample_rate_hz), formatted %.6f like the GUI backend rows.
 * - Every packet prints exactly one line; fields are width-truncated so
 *   long addresses/names can never wrap to a second line.
 * - All three CLI binaries share these printers so `-v summary` output is
 *   identical regardless of binary.
 */

void app_summary_view_print_header(void);

void app_summary_view_print_ble(unsigned long packet_no,
                                const ble_event_t *event);

void app_summary_view_print_bredr(unsigned long packet_no,
                                  const bredr_event_t *event,
                                  const bredr_piconet_snapshot_t *pnet);

void app_summary_view_print_ble_decode_fail(unsigned long packet_no,
                                            const ble_event_t *event);

#endif /* APP_SUMMARY_VIEW_H */
