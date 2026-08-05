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

float receiver_mean_linear_power(const float complex *samples,
                                unsigned int start_index,
                                unsigned int end_index)
{
    if (!samples || start_index >= end_index)
        return 0.0f;

    float sum_power = 0.0f;
    for (unsigned int i = start_index; i < end_index; i++)
    {
        float re = crealf(samples[i]);
        float im = cimagf(samples[i]);
        sum_power += re * re + im * im;
    }
    return sum_power / (float)(end_index - start_index);
}

float receiver_rssi_signal_dbr(const float complex *samples,
                               unsigned int window_start,
                               unsigned int window_end,
                               unsigned int idle_end,
                               float *noise_floor_linear,
                               unsigned int *noise_floor_initialized,
                               float invalid_value)
{
    if (!samples || window_start >= window_end)
        return invalid_value;

    float signal = receiver_mean_linear_power(samples, window_start, window_end);

    float floor_lin = 0.0f;
    if (noise_floor_linear && noise_floor_initialized && *noise_floor_initialized)
        floor_lin = *noise_floor_linear;

    /* Refresh the floor from the idle prefix when it is long enough to be stable. */
    if (idle_end >= RECEIVER_RSSI_FLOOR_MIN_SAMPLES && noise_floor_linear &&
        noise_floor_initialized)
    {
        float f = receiver_mean_linear_power(samples, 0u, idle_end);
        if (*noise_floor_initialized)
            *noise_floor_linear = RECEIVER_RSSI_FLOOR_EMA_ALPHA * f +
                                 (1.0f - RECEIVER_RSSI_FLOOR_EMA_ALPHA) *
                                     (*noise_floor_linear);
        else
            *noise_floor_linear = f;
        *noise_floor_initialized = 1u;
        floor_lin = *noise_floor_linear;
    }

    if (signal <= floor_lin)
        return invalid_value;

    return receiver_rssi_from_linear_power(signal - floor_lin, invalid_value);
}

void receiver_rssi_packet_window(uint64_t block_start_bit_index,
                                 uint64_t packet_start_bit_index,
                                 unsigned int end_sample,
                                 unsigned int samples_per_symbol,
                                 unsigned int available_samples,
                                 unsigned int *out_start,
                                 unsigned int *out_end,
                                 unsigned int *out_idle_end)
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

    /* Idle prefix available for the noise-floor estimate is everything before
     * the packet start; the caller subtracts its mean power from the window. */
    if (out_idle_end)
    {
        uint64_t idle = rel_bits * (uint64_t)samples_per_symbol;
        *out_idle_end = (idle > (uint64_t)available_samples)
                        ? available_samples
                        : (unsigned int)idle;
    }

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
