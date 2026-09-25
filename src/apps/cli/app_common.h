#ifndef APP_COMMON_H
#define APP_COMMON_H

#include <stddef.h>

#include "radio_common.h"
#include "session.h"

typedef enum
{
    APP_OUTPUT_MODE_FULL = 0,
    APP_OUTPUT_MODE_SUMMARY = 1,
    APP_OUTPUT_MODE_RSSI = 2,
    APP_OUTPUT_MODE_DEVICES = 3
} app_output_mode_t;

typedef struct
{
    app_output_mode_t mode;
    const char *name;
} app_output_mode_option_t;

/* Long-only options (no single-character short form) use val codes that
 * are outside the ASCII range used by the short-option string. Shared by
 * the CLI binaries so they all use the same getopt sentinels. */
enum
{
    APP_OPT_DEBUG = 0x100,
    APP_OPT_ENFORCE_CRC,
    APP_OPT_AC_ERRORS,
    APP_OPT_RECORD,
    APP_OPT_EXHAUSTIVE,
    APP_OPT_GAIN,
};

/* Parsed "<type>:<id>" device spec, e.g. "hackrf:b25062dc22113a0b".
 * @p id points into argv, so it remains valid for the program's lifetime. */
typedef struct
{
    radio_device_type_t type;
    const char *id;
} app_device_spec_t;

int app_parse_output_mode(const char *arg,
                          const app_output_mode_option_t *options,
                          size_t option_count,
                          app_output_mode_t *out_mode);
const app_output_mode_option_t *app_output_mode_option(app_output_mode_t mode,
                                                       const app_output_mode_option_t *options,
                                                       size_t option_count);
const char *app_output_mode_name(app_output_mode_t mode,
                                 const app_output_mode_option_t *options,
                                 size_t option_count);
void app_output_lock(void);
void app_output_unlock(void);
void app_install_sigint_handler(session_t *session_slot);

/**
 * Parse a "<type>:<id>" device spec (e.g. "hackrf:b25062dc22113a0b").
 * @p spec is the raw string; on success @p out is populated (id points
 * into @p spec).
 * @return 0 on success, -1 on malformed/type-unknown specs.
 */
int app_parse_device_spec(const char *spec, app_device_spec_t *out);

/**
 * Print one "<type>:<id>" line per connected radio of every known device
 * type, then return. Used by the bare `-d` / `--device` form.
 * @return EXIT_SUCCESS on completion, EXIT_FAILURE on enumeration error.
 */
int app_print_available_devices(const char *argv0);

/**
 * Verify a device described by @p spec is currently present. On success
 * returns 0. On not-found prints the friendly "Device not found" message
 * (which references @p argv0) and returns non-zero; other enumeration
 * errors also return non-zero.
 */
int app_validate_device_spec(const char *argv0, const app_device_spec_t *spec);

/**
 * Verify a default live device exists (for runs without an explicit
 * `-d <type>:<id>`). Uses session_get_default_device() so any live type
 * is accepted; file replay is not probed here (explicit specs are already
 * validated by app_validate_device_spec()).
 * On success returns 0. On no-device prints the friendly "No devices
 * found" message (which references @p argv0) and returns non-zero.
 */
int app_require_default_device(const char *argv0);

/**
 * Print the `--device` usage line for `print_usage` blocks.
 */
void app_print_device_usage_line(void);

/**
 * Print the `--record` usage line for `print_usage` blocks.
 */
void app_print_record_usage_line(void);

/**
 * Print the `-g/--gain` usage line for `print_usage` blocks.
 */
void app_print_gain_usage_line(void);

/**
 * Default live device type when the user passes no `-d` flag: HackRF when
 * compiled in, otherwise bladeRF, otherwise file (validation-only).
 */
radio_device_type_t app_default_device_type(void);

/**
 * Print the resolved gain selection as a banner line ("Gain : ..."),
 * formatted per device type (raw -g value when explicit, device default
 * otherwise). FILE prints "(n/a - replay)".
 */
void app_print_gain_summary(radio_device_type_t type,
                            const radio_gain_spec_t *spec);

/**
 * Resolve the raw `-g/--gain` argument (NULL = flag omitted → device
 * defaults) into @p out for @p type. On malformed/out-of-range input
 * prints "Invalid --gain ...", the device-specific help, and returns
 * non-zero. @p argv0 is used only for diagnostics.
 */
int app_resolve_gain_spec(const char *argv0, radio_device_type_t type,
                          const char *raw, radio_gain_spec_t *out);

/**
 * Print the `--exhaustive` usage line for `print_usage` blocks.
 */
void app_print_exhaustive_usage_line(void);

/**
 * Print a per-stage breakdown of dropped blocks from a session summary
 * (see session_dropped_blocks_breakdown). Each stage (rf / sub / out)
 * reports producer-side (pool exhausted) vs consumer-side (reader queue
 * full) drops, plus per-lane detail when K > 1, so the user can see WHERE
 * in the pipeline blocks were lost.
 */
void app_print_drop_breakdown(const session_drop_breakdown_t *b);

#endif
