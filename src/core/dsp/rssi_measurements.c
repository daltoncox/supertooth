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

    /* Refresh the floor from the idle prefix when it is long enough to be stable.
     * The estimate is kept for diagnostics, but it is intentionally NOT
     * subtracted from the reported RSSI.  Subtracting a per-packet, noise-only
     * floor from a signal that sits near it (i.e. every weak device) amplifies
     * tiny floor fluctuations into enormous dB swings: a fixed-power packet
     * scatters ~47 dB packet-to-packet through this path while the raw power
     * scatters ~2.5 dB (see test_rssi_variance_from_floor_subtraction).  The
     * floor is at most a constant, channel-dependent offset that calibration
     * absorbs, so reporting the raw received power is both stable and honest. */
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
    }

    /* A decoded packet always carries real signal: report its raw received
     * power (signal + the in-band noise/interference present on the channel).
     * Returning invalid_value here would surface as the "--" RSSI on the
     * frontend for weak-but-present devices. */
    return receiver_rssi_from_linear_power(signal, invalid_value);
}

void receiver_rssi_access_code_window(unsigned int ac_end_sample,
                                      unsigned int span_samples,
                                      unsigned int available_samples,
                                      unsigned int *out_start,
                                      unsigned int *out_end,
                                      unsigned int *out_idle_end)
{
    if (!out_start || !out_end)
        return;

    unsigned int end = ac_end_sample;
    if (end > available_samples)
        end = available_samples;

    unsigned int start = (end >= span_samples) ? end - span_samples : 0u;

    *out_start = start;
    *out_end   = end;
    if (out_idle_end)
        *out_idle_end = start;
}
