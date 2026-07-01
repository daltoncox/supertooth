#ifndef HACKRF_WRAPPER_H
#define HACKRF_WRAPPER_H

#include <stddef.h>

#include "radio_common.h"

/* Length in hex chars (excluding NUL) of the device id reported by a
 * HackRF. See hackrf.c for where it comes from. */
#define HACKRF_ID_LEN 16u

int hackrf_radio_open(void **out_device,
                      const char *device_id,
                      sample_dispatcher_t *dispatcher,
                      int debug_enabled);
int hackrf_radio_configure(void *device, const radio_stream_config_t *config);
int hackrf_radio_start_rx(void *device);
int hackrf_radio_stop_rx(void *device);
void hackrf_radio_close(void *device);

/**
 * Enumerate connected HackRF devices by serial number.
 *
 * On success, `*out_identifiers` points to a freshly allocated array of
 * `*out_count` heap-allocated serial-number strings. The caller owns the
 * array and must release it with radio_free_device_list().
 */
int hackrf_list_devices(char ***out_identifiers, size_t *out_count);

#endif // HACKRF_WRAPPER_H