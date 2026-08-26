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
 * Print the `--device` usage line for `print_usage` blocks.
 */
void app_print_device_usage_line(void);

#endif
