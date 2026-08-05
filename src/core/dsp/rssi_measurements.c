/**
 * @file dsp/rssi_measurements.c
 * @brief Shared receive-side RSSI measurement helpers.
 */

#include "rssi_measurements.h"

#include <math.h>

#define RSSI_DBM_LIKE_OFFSET_DB 40.0f

static float receiver_rssi_apply_offset(float rssi_db)
{
    return rssi_db - RSSI_DBM_LIKE_OFFSET_DB;
}

float receiver_rssi_from_linear_power(float power, float invalid_value)
{
    if (power <= 0.0f)
        return invalid_value;

    return receiver_rssi_apply_offset(10.0f * log10f(power));
}

float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
                                          float invalid_value)
{
    if (!samples || start_index >= end_index)
        return invalid_value;

    float sum_power = 0.0f;
    for (unsigned int i = start_index; i < end_index; i++)
    {
        float re = crealf(samples[i]);
        float im = cimagf(samples[i]);
        sum_power += re * re + im * im;
    }
    unsigned int sample_count = end_index - start_index;

    return receiver_rssi_from_linear_power(sum_power / (float)sample_count, invalid_value);
}

void receiver_rssi_packet_window(uint64_t block_start_bit_index,
                                 uint64_t packet_start_bit_index,
                                 unsigned int end_sample,
                                 unsigned int samples_per_symbol,
                                 unsigned int available_samples,
                                 unsigned int *out_start,
                                 unsigned int *out_end)
{
    if (!out_start || !out_end)
        return;

    unsigned int end = end_sample + samples_per_symbol;
    if (end > available_samples)
        end = available_samples;

    /* Bit offset within this block -> sample offset within this block. */
    uint64_t rel_bits = (packet_start_bit_index > block_start_bit_index)
                        ? (packet_start_bit_index - block_start_bit_index)
                        : 0u;
    uint64_t start = rel_bits * (uint64_t)samples_per_symbol;

    start = (start > RECEIVER_RSSI_PRETRIGGER_SAMPLES)
            ? start - RECEIVER_RSSI_PRETRIGGER_SAMPLES
            : 0u;

    /* The packet began before this block: average the portion that is here
     * rather than reaching back past the start of the buffer. */
    if (start >= (uint64_t)end)
        start = 0u;

    *out_start = (unsigned int)start;
    *out_end   = end;
}
