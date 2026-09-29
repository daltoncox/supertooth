#ifndef FILE_RADIO_H
#define FILE_RADIO_H

#include <stddef.h>
#include <stdint.h>

#include "radio_common.h"

/* File-backed fake radio: streams a WAV IQ capture (SDR++ baseband format,
 * see wav.h) through a sample dispatcher in HackRF-sized blocks, so the DSP
 * chain cannot tell replay from live capture. Usage mirrors hackrf.h:
 * open → configure → start_rx (spawns reader thread) → stop_rx → close.
 * Replay is single-pass; when the file is exhausted the backend reports
 * finished (see file_radio_is_finished / radio_is_finished) and the session
 * run loop shuts down cleanly. */

int file_radio_open(void **out_device,
                    const char *path,
                    sample_dispatcher_t *dispatcher,
                    int debug_enabled);
int file_radio_configure(void *device, const radio_stream_config_t *config);
int file_radio_start_rx(void *device);
int file_radio_stop_rx(void *device);
void file_radio_close(void *device);

/* 1 once the reader thread has consumed the whole file (or stop was
 * requested), 0 while samples are still flowing. */
int file_radio_is_finished(void *device);

/* Select replay behavior before start_rx (default is realtime):
 *   exhaustive != 0 → push only when every reader has room, so replay never
 *                     drops for pacing reasons (as fast as consumers allow);
 *   exhaustive == 0 → wall-clock paced, live-equivalent timing and drops. */
void file_radio_set_exhaustive(void *device, int exhaustive);

/* Header facts for pre-flight checks and banners (0 when unknown/closed). */
uint32_t file_radio_get_file_sample_rate(void *device);
uint64_t file_radio_get_file_center_hz(void *device);

/* Header-only compatibility check: verifies the WAV at @p path is readable
 * 2ch IQ and (when @p expected_rate_hz != 0) that its sample rate matches the
 * tuned session rate. Prints a specific stderr message on mismatch. Returns
 * 0 when replay can proceed, non-zero otherwise. */
int file_radio_check_compatible(const char *path, uint32_t expected_rate_hz,
                                uint32_t *out_file_rate_hz,
                                uint64_t *out_file_center_hz);

#endif /* FILE_RADIO_H */
