#ifndef APP_RECORD_H
#define APP_RECORD_H

#include <stdint.h>

#include "radio_common.h"

/* Record-only raw IQ capture: streams the live radio straight to a
 * self-describing WAV file (see wav.h) with no DSP or decoding.
 *
 * @p out_path is either a file path (used verbatim) or a directory / trailing
 * -slash path, in which case a self-describing filename of the form
 * supertooth_baseband_<LO>Hz_<RATE>Msps_<UTC-timestamp>.wav is generated
 * inside it. Blocks until Ctrl+C. Returns 0 on success, non-zero on error. */
typedef struct
{
    radio_device_type_t device_type; /* must be a live radio, not FILE */
    const char *device_id;           /* optional, NULL for default */
    uint32_t lo_freq_hz;
    uint32_t sample_rate_hz;
    int debug;
} app_record_config_t;

int app_record_run(const app_record_config_t *cfg, const char *out_path);

#endif /* APP_RECORD_H */
