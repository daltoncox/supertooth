/**
 * @file rssi_tracker.c
 * @brief Implementation of the sample-ring RSSI tracker (see rssi_tracker.h).
 */

#include "rssi_tracker.h"

#include <math.h>

double rssi_tracker_frame_timestamp_sec(const rx_metadata_t *meta)
{
    if (meta->radio_sample_rate_hz == 0u)
        return 0.0;
    return (double)meta->radio_start_sample_index /
           (double)meta->radio_sample_rate_hz;
}

void rssi_tracker_init(rssi_tracker_t *tracker)
{
    if (!tracker)
        return;
    tracker->head = 0u;
    tracker->count = 0u;
}

void rssi_tracker_add(rssi_tracker_t *tracker, const rx_metadata_t *frame)
{
    if (!tracker || !frame)
        return;

    tracker->frames[tracker->head] = *frame;
    tracker->head = (tracker->head + 1u) % RSSI_TRACKER_MAX_SAMPLES;
    if (tracker->count < RSSI_TRACKER_MAX_SAMPLES)
        tracker->count++;
}

bool rssi_tracker_average(const rssi_tracker_t *tracker, float *out_avg_dbr)
{
    if (!tracker || !out_avg_dbr || tracker->count == 0u)
        return false;

    /* Most recent frame determines the window's end and the source context. */
    size_t latest_idx = (tracker->head + RSSI_TRACKER_MAX_SAMPLES - 1u) %
                        RSSI_TRACKER_MAX_SAMPLES;
    const rx_metadata_t *latest = &tracker->frames[latest_idx];

    double latest_sec = rssi_tracker_frame_timestamp_sec(latest);
    double cutoff_sec = latest_sec - (double)RSSI_TRACKER_WINDOW_SEC;

    float sum = 0.0f;
    size_t valid = 0u;

    for (size_t i = 0u; i < tracker->count; i++)
    {
        size_t idx = (tracker->head + RSSI_TRACKER_MAX_SAMPLES - 1u - i) %
                     RSSI_TRACKER_MAX_SAMPLES;
        const rx_metadata_t *frame = &tracker->frames[idx];

        /* Keep only the same source stream as the latest frame. */
        if (frame->source_id != latest->source_id)
            continue;

        /* Frames are ordered by receive time; once we fall outside the window
         * all earlier frames are also outside, so stop. */
        if (rssi_tracker_frame_timestamp_sec(frame) < cutoff_sec)
            break;

        sum += frame->rssi_dbr;
        valid++;
    }

    if (valid == 0u)
        return false;

    *out_avg_dbr = sum / (float)valid;
    return true;
}
