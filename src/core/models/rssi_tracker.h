/**
 * @file rssi_tracker.h
 * @brief Sample-ring RSSI tracker with a fixed time window.
 *
 * Replaces the ad-hoc EMA + rolling-window RSSI logic that used to be
 * duplicated across the BR/EDR piconet (combined / central / 8 peripherals)
 * and the BLE advertiser / connection records. Each logical entity owns one
 * rssi_tracker_t that retains the last RSSI_TRACKER_MAX_SAMPLES frames and
 * reports the mean RSSI over the most recent RSSI_TRACKER_WINDOW_SEC of
 * receive time.
 *
 * Timestamps come from the frame metadata (radio_start_sample_index /
 * radio_sample_rate_hz) rather than wall clock, so the window is correct
 * under replay and across multiple source radios. No heap allocation; safe
 * to embed by value.
 */

#ifndef RSSI_TRACKER_H
#define RSSI_TRACKER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "receive_event_models.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of frames retained per tracker (ring buffer depth). */
#define RSSI_TRACKER_MAX_SAMPLES 256u

/** Trailing time window, in seconds, over which the average is computed. */
#define RSSI_TRACKER_WINDOW_SEC 1.0f

/**
 * @brief Fixed-depth ring of recent receive frames used for RSSI averaging.
 *
 * Each entry stores the full rx_metadata_t so the original sample timing and
 * source context are preserved for windowing.
 */
typedef struct
{
    rx_metadata_t frames[RSSI_TRACKER_MAX_SAMPLES];
    size_t head;   /**< index of the next write slot */
    size_t count;  /**< total frames currently stored (<= MAX_SAMPLES) */
} rssi_tracker_t;

/** Initialise an empty tracker. */
void rssi_tracker_init(rssi_tracker_t *tracker);

/** Feed one received frame into the tracker. */
void rssi_tracker_add(rssi_tracker_t *tracker, const rx_metadata_t *frame);

/**
 * @brief Mean RSSI (dBr) over the trailing RSSI_TRACKER_WINDOW_SEC of frames.
 *
 * @param tracker      Must not be NULL.
 * @param out_avg_dbr  Receives the average when at least one in-window frame
 *                     exists for the latest frame's source_id.
 * @return             true if a value was written, false if the tracker is
 *                     empty or no in-window sample matched.
 */
bool rssi_tracker_average(const rssi_tracker_t *tracker, float *out_avg_dbr);

/** Timestamp (seconds) of a frame, derived from its sample metadata. */
double rssi_tracker_frame_timestamp_sec(const rx_metadata_t *meta);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_TRACKER_H */
