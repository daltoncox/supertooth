#ifndef APP_RECORD_H
#define APP_RECORD_H

#include <stdint.h>

#include "radio_common.h"

/* Record-only raw IQ capture: streams the live radio straight to a
 * self-describing WAV file (see wav.h) with no DSP or decoding.
 *
 * The file is always written to the directory the program was run in
 * (current working directory) with a self-describing filename of the form
 * supertooth_baseband_<LO>Hz_<RATE>Msps_<UTC-timestamp>.wav.
 * Blocks until Ctrl+C. Returns 0 on success, non-zero on error. */
typedef struct
{
    radio_device_type_t device_type; /* must be a live radio, not FILE */
    const char *device_id;           /* optional, NULL for default */
    uint32_t lo_freq_hz;
    uint32_t sample_rate_hz;
    radio_gain_spec_t gain;
    int debug;
} app_record_config_t;

int app_record_run(const app_record_config_t *cfg);

#endif /* APP_RECORD_H */
