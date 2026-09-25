#ifndef BLADERF_WRAPPER_H
#define BLADERF_WRAPPER_H

#include <stddef.h>

#include "radio_common.h"

/* Length in chars (excluding NUL) of the device id reported by a bladeRF:
 * the 32-char lowercase hex serial (see bladerf.c). */
#define BLADERF_ID_LEN 32u

int bladerf_radio_open(void **out_device,
                       const char *device_id,
                       sample_dispatcher_t *dispatcher,
                       int debug_enabled);
int bladerf_radio_configure(void *device, const radio_stream_config_t *config);
int bladerf_radio_start_rx(void *device);
int bladerf_radio_stop_rx(void *device);
void bladerf_radio_close(void *device);

/**
 * Maximum sample rate (Hz) a bladeRF 2.0 Micro can sustain over USB3
 * (61.44 Msps). @p device may be NULL (the limit is hardware-wide, not
 * per-unit). Sessions tune integer-MHz windows at or below this ceiling.
 */
int bladerf_radio_get_max_sample_rate(void *device, uint32_t *out_rate_hz);

/**
 * Enumerate connected bladeRF devices by serial number.
 *
 * On success, `*out_identifiers` points to a freshly allocated array of
 * `*out_count` heap-allocated serial-number strings. The caller owns the
 * array and must release it with radio_free_device_list().
 */
int bladerf_list_devices(char ***out_identifiers, size_t *out_count);

#endif // BLADERF_WRAPPER_H
