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

/** Samples of slack taken before the detected packet start. */
#define RECEIVER_RSSI_PRETRIGGER_SAMPLES 4u

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
 * Packet RSSI with the in-band noise + interference floor removed.
 *
 * Averages power over [window_start, window_end), estimates the floor from the
 * idle prefix [0, idle_end) when it is at least RECEIVER_RSSI_FLOOR_MIN_SAMPLES
 * long (otherwise the caller-supplied EMA floor is used), subtracts the floor
 * in linear units, and returns the resulting signal power as dBr.  The EMA
 * floor state in @p noise_floor_linear / @p noise_floor_initialized is updated
 * whenever a usable idle prefix is available.
 *
 * Returns @p invalid_value when the window is empty or the signal power does not
 * exceed the estimated floor.
 */
float receiver_rssi_signal_dbr(const float complex *samples,
                               unsigned int window_start,
                               unsigned int window_end,
                               unsigned int idle_end,
                               float *noise_floor_linear,
                               unsigned int *noise_floor_initialized,
                               float invalid_value);

/**
 * Half-open sample range `[*out_start, *out_end)` to average for a packet the
 * bitstream decoder reported at @p packet_start_bit_index, inside a block that
 * began at @p block_start_bit_index.
 *
 * The decoder counts BITS while the demodulated buffer is indexed in SAMPLES,
 * so the block-relative bit offset is scaled UP by @p samples_per_symbol.
 * Getting that conversion backwards silently widens the window by the packet's
 * position within the block, which averages an arbitrary amount of idle
 * channel into the result.
 *
 * @param end_sample         sample index of the symbol that completed the packet
 * @param available_samples  samples actually present in the buffer
 * @param out_idle_end       receives the raw packet-start sample index; the idle
 *                           prefix [0, *out_idle_end) is the noise floor to subtract
 */
void receiver_rssi_packet_window(uint64_t block_start_bit_index,
                                 uint64_t packet_start_bit_index,
                                 unsigned int end_sample,
                                 unsigned int samples_per_symbol,
                                 unsigned int available_samples,
                                 unsigned int *out_start,
                                 unsigned int *out_end,
                                 unsigned int *out_idle_end);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_MEASUREMENTS_H */
