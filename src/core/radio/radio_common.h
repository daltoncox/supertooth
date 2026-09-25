#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "sample_dispatcher.h"

#define RADIO_SUCCESS 0
#define RADIO_DEVICE_NOT_FOUND (-2)

/* Maximum sample rate the channelization service can stage (80 Msps in four
 * lanes). Individual device types report their own ceiling via
 * radio_get_max_sample_rate_for_type() (HackRF stays at 20 Msps); callers
 * clamp the requested channel count to the selected device before tuning. */
#define RADIO_MAX_SAMPLE_RATE_HZ 80000000u

typedef struct
{
    uint64_t lo_freq_hz;
    uint32_t sample_rate;
    uint32_t lna_gain;
    uint32_t vga_gain;
} radio_stream_config_t;

typedef enum
{
    RADIO_DEVICE_HACKRF = 0,
    RADIO_DEVICE_FILE = 1,
    RADIO_DEVICE_TYPE_COUNT,
} radio_device_type_t;

typedef struct radio_device radio_device_t;

/**
 * Printable name for a device type (e.g. "hackrf"), or NULL when @p type
 * is not a known radio device type. Useful for enumerating all supported
 * radio types by iterating 0..RADIO_DEVICE_TYPE_COUNT-1.
 */
const char *radio_device_type_name(radio_device_type_t type);

/**
 * Open a radio device of the given type.
 *
 * @param device_id  Optional identifier selecting a specific device. When
 *                   NULL, the backend selects a default device. When
 *                   non-NULL, it must match an identifier returned by
 *                   radio_list_devices() for the given device_type.
 */
int radio_open(radio_device_t **out_device,
               radio_device_type_t device_type,
               const char *device_id,
               sample_dispatcher_t *dispatcher,
               int debug_enabled);
int radio_configure(radio_device_t *device, const radio_stream_config_t *config);
int radio_start_rx(radio_device_t *device);
int radio_stop_rx(radio_device_t *device);
void radio_close(radio_device_t *device);

/**
 * 1 when a file-backed device has exhausted its capture (single-pass replay
 * is done), 0 while samples are still flowing or for live radios. Lets the
 * session run loop exit cleanly at EOF without knowing backend types.
 */
int radio_is_finished(radio_device_t *device);

/**
 * Select file-replay behavior (no-op for live radios):
 *   exhaustive != 0 → backpressure, never drop for pacing reasons;
 *   exhaustive == 0 → realtime wall-clock pacing (default).
 * Must be called after radio_open() and before radio_start_rx().
 */
void radio_set_replay_mode(radio_device_t *device, int exhaustive);

/**
 * Maximum sample rate (Hz) a device of @p type can sustain. Used by callers
 * to pick a capture bandwidth that the hardware can actually drive (e.g. a
 * default BR/EDR channel count that fits within the radio's sample-rate
 * ceiling rather than the full 79-channel band).
 *
 * @return RADIO_SUCCESS with *out_rate_hz set, or a negative value if @p type
 *         is unknown.
 */
int radio_get_max_sample_rate_for_type(radio_device_type_t type,
                                       uint32_t *out_rate_hz);

/**
 * Enumerate available devices of the given type.
 *
 * On success, `*out_identifiers` points to a newly allocated array of
 * `*out_count` heap-allocated, NUL-terminated identifier strings. The
 * caller owns the array and must release it with radio_free_device_list().
 *
 * @return RADIO_SUCCESS on success, a negative value on failure.
 */
int radio_list_devices(radio_device_type_t device_type,
                       char ***out_identifiers,
                       size_t *out_count);

/**
 * Free an identifier array previously returned by radio_list_devices().
 * Frees each string and the array itself, and sets the pointer to NULL.
 */
void radio_free_device_list(char ***identifiers, size_t count);

/**
 * Check whether a device with the given id is currently present for the
 * given device type.
 *
 * @return RADIO_SUCCESS          if the device is present.
 *         RADIO_DEVICE_NOT_FOUND if it is not present.
 *         other negative value   on enumeration failure.
 */
int radio_device_exists(radio_device_type_t device_type, const char *device_id);

#endif