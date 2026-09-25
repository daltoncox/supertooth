#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "sample_dispatcher.h"

#ifndef HAVE_HACKRF
#define HAVE_HACKRF 1
#endif
#ifndef HAVE_BLADERF
#define HAVE_BLADERF 0
#endif

#define RADIO_SUCCESS 0
#define RADIO_DEVICE_NOT_FOUND (-2)

/* Maximum sample rate the channelization service can stage (80 Msps in four
 * lanes). Individual device types report their own ceiling via
 * radio_get_max_sample_rate_for_type() (HackRF stays at 20 Msps, bladeRF at
 * 61.44 Msps); callers clamp the requested channel count to the selected
 * device before tuning. */
#define RADIO_MAX_SAMPLE_RATE_HZ 80000000u

/* HackRF gain limits (see hackrf.h). */
#define RADIO_HACKRF_LNA_MIN 0
#define RADIO_HACKRF_LNA_MAX 40
#define RADIO_HACKRF_VGA_MIN 0
#define RADIO_HACKRF_VGA_MAX 62
#define RADIO_HACKRF_LNA_DEFAULT 24
#define RADIO_HACKRF_VGA_DEFAULT 18

/* bladeRF overall RX gain limits (dB, manual gain control). The usable
 * default below is a reasonable starting point; adjust per environment. */
#define RADIO_BLADERF_GAIN_MIN 0
#define RADIO_BLADERF_GAIN_MAX 60
#define RADIO_BLADERF_GAIN_DEFAULT 30

/* Unified per-radio gain selection, parsed from the shared `-g/--gain`
 * CLI flag (see radio_parse_gain_spec). Only the fields for the selected
 * device type are meaningful; the rest are ignored. `present` is 0 when
 * the user passed no `-g` flag (backends apply their defaults). */
typedef struct
{
    int present;
    int hackrf_lna;
    int hackrf_vga;
    int hackrf_amp;
    int bladerf_gain_db;
} radio_gain_spec_t;

typedef struct
{
    uint64_t lo_freq_hz;
    uint32_t sample_rate;
    radio_gain_spec_t gain;
} radio_stream_config_t;

typedef enum
{
    RADIO_DEVICE_HACKRF = 0,
    RADIO_DEVICE_FILE = 1,
    RADIO_DEVICE_BLADERF = 2,
    RADIO_DEVICE_TYPE_COUNT,
} radio_device_type_t;

typedef struct radio_device radio_device_t;

/**
 * Printable name for a device type (e.g. "hackrf"), or NULL when @p type
 * is not a known radio device type *or the backend was compiled out*
 * (see ENABLE_HACKRF / ENABLE_BLADERF). Useful for enumerating all
 * supported radio types by iterating 0..RADIO_DEVICE_TYPE_COUNT-1.
 */
const char *radio_device_type_name(radio_device_type_t type);

/**
 * Whether @p type names a live radio (HackRF/bladeRF when compiled in).
 * File replay is not a live radio.
 */
int radio_device_type_is_live(radio_device_type_t type);

/**
 * Parse a `-g/--gain` argument for @p type into @p out.
 *
 *   hackrf  : "LNA,VGA[,AMP]" (e.g. "24,18,0"); AMP defaults to 0/off.
 *   bladerf : "GAIN_DB"       (e.g. "30").
 *   file    : gains are meaningless; any value is accepted and ignored.
 *
 * @param err  Optional buffer receiving a human-readable reason on failure.
 * @return 0 on success, -1 on malformed/out-of-range specs. On failure @p
 *         out is left untouched and @p err (when non-NULL) is NUL-terminated.
 */
int radio_parse_gain_spec(radio_device_type_t type, const char *str,
                          radio_gain_spec_t *out,
                          char *err, size_t err_len);

/**
 * Fill @p out with the default gains for @p type (used when `-g` is
 * omitted). `present` is 0 on output.
 */
void radio_gain_default(radio_device_type_t type, radio_gain_spec_t *out);

/**
 * One-line usage for the `-g/--gain` flag, and the multi-line
 * device-specific help printed when a gain spec is malformed (both go to
 * stderr from the CLI layer).
 */
void radio_print_gain_usage(void);
void radio_print_gain_help(radio_device_type_t type);

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
 *         is unknown (or compiled out).
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
