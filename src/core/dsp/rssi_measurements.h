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

float receiver_rssi_from_linear_power(float power, float invalid_value);
float receiver_rssi_from_mean_power_range(const float complex *samples,
                                          unsigned int start_index,
                                          unsigned int end_index,
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
 */
void receiver_rssi_packet_window(uint64_t block_start_bit_index,
                                 uint64_t packet_start_bit_index,
                                 unsigned int end_sample,
                                 unsigned int samples_per_symbol,
                                 unsigned int available_samples,
                                 unsigned int *out_start,
                                 unsigned int *out_end);

#ifdef __cplusplus
}
#endif

#endif /* RSSI_MEASUREMENTS_H */
