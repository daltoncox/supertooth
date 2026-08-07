/**
 * @file dsp/rssi_measurements.h
 * @brief Shared receive-side RSSI measurement helpers.
 */

#ifndef RSSI_MEASUREMENTS_H
#define RSSI_MEASUREMENTS_H

#include <complex.h>
#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RECEIVER_RSSI_INVALID ((float)NAN)

/**
 * Minimum length of idle prefix required before it is trusted as a noise-floor
 * estimate.  Shorter prefixes are too noisy to use directly; the caller's
 * per-channel EMA floor is used instead.
 */
#define RECEIVER_RSSI_FLOOR_MIN_SAMPLES 32u

/** EMA smoothing applied to the noise-floor estimate between packets. */
#define RECEIVER_RSSI_FLOOR_EMA_ALPHA 0.05f

/**
 * Demodulator group delay in samples: the GFSK energy for a symbol lags the
 * bit index by this many decimated samples, so that many leading samples are
 * dropped from the averaging window to sit on steady-state signal.
 */
#define RECEIVER_RSSI_DEMOD_DELAY_SAMPLES 2u

float receiver_rssi_from_linear_power(float power, float invalid_value);
float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
                                          float invalid_value);

/** Mean linear power (|sample|^2) over [start,end); 0.0f if empty. */
float receiver_mean_linear_power(const float complex *samples,
                                unsigned int start_index,
                                unsigned int end_index);

/**
 * Packet RSSI as the raw received power over [window_start, window_end).
 *
 * The floor estimated from the idle prefix [0, idle_end) is tracked in the
 * caller-supplied EMA state (updated whenever the prefix is at least
 * RECEIVER_RSSI_FLOOR_MIN_SAMPLES long) for diagnostics, but it is NOT
 * subtracted from the reported value: subtracting a noisy per-packet floor
 * estimate amplifies tiny floor fluctuations into tens of dB of packet-to-
 * packet scatter for signals near the floor (see
 * test_rssi_variance_from_floor_subtraction).  A decoded packet always carries
 * real signal, so the raw received power is reported even when it does not
 * exceed the estimated floor.
 *
 * Returns @p invalid_value only when the window is empty.
 */
float receiver_rssi_signal_dbr(const float complex *samples,
                               unsigned int window_start,
                               unsigned int window_end,
                               unsigned int idle_end,
                               float *noise_floor_linear,
                               unsigned int *noise_floor_initialized,
                               float invalid_value);

/**
 * Half-open sample range `[*out_start, *out_end)` covering the access code of
 * a packet whose detection completed at @p ac_end_sample (the first sample
 * AFTER the symbol whose bit matched the sync word), inside a buffer holding
 * @p available_samples samples.
 *
 * BR/EDR RSSI is measured over the access code rather than the whole packet
 * because the bitstream decoder cannot know the true packet length (the header
 * is whitened until CLK1-6 is recovered) and only signals completion after a
 * fixed maximum-length body.  The access code is always GFSK (constant
 * envelope), even for EDR packets, so a fixed @p span_samples window ending at
 * detection measures the same physical quantity for every packet type.
 *
 * A packet whose access code began before this buffer reports the portion
 * that is present (start clamped to 0).
 *
 * @param ac_end_sample   one past the last sample of the detected access code
 * @param span_samples    access-code span (e.g. BREDR_AC_DETECT_SAMPLES)
 * @param available_samples  samples actually present in the buffer
 * @param out_idle_end    receives *out_start; the idle prefix [0, *out_idle_end)
 *                        seeds the caller's noise-floor estimate
 */
void receiver_rssi_access_code_window(unsigned int ac_end_sample,
                                      unsigned int span_samples,
                                      unsigned int available_samples,
                                      unsigned int *out_start,
                                      unsigned int *out_end,
                                      unsigned int *out_idle_end);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_MEASUREMENTS_H */
