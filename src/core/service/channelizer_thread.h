/**
 * @file service/channelizer_thread.h
 * @brief The channelizer bank's service adapter: threading + dispatchers.
 *
 * This is the threading half of channelization (the DSP half is
 * dsp/channelizer_bank.h, which is thread-free and unit-tested directly).
 * A channelizer owns one bank, pulls RF blocks through it on its worker
 * thread, and publishes frame-major output for the channel processors.
 * Sessions own one of these per bank (a single shared one in hybrid mode).
 *
 * Reads wideband RF blocks from the radio's sample dispatcher, runs them
 * through the bank, and writes frame-major blocks
 * (layout `out[frame * M + bin]`, see channelizer_bank.h) into a second
 * dispatcher.  Each channel processor (BR/EDR or BLE) is a reader of that
 * second dispatcher and pulls its own bin with a uniform stride of M.
 *
 * Instances are configured by grid: the 1 MHz raster
 * (CHANNELIZER_BANK_GRID_BR_EDR_HZ) for BR/EDR and hybrid sessions, the
 * 2 MHz raster (CHANNELIZER_BANK_GRID_BLE_HZ) for BLE-only sessions.
 */

#ifndef CHANNELIZER_THREAD_H
#define CHANNELIZER_THREAD_H

#include <complex.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>

#include "channelizer_bank.h"
#include "sample_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    channelizer_bank_t bank;
    sample_reader_t    rf_reader;   /**< registered on the radio (RF) dispatcher */
    sample_dispatcher_t *out;       /**< frame-major output dispatcher */
    const _Atomic unsigned int *shutdown;
    int debug;
    int active;

    /* Exhaustive replay: wait for pool blocks / reader room instead of
     * dropping (see sample_dispatcher_acquire_blocking/push_blocking).
     * Set by the session from its config; 0 preserves live behavior. */
    int exhaustive;

    /* The bank (channelizer_bank_t) keeps its own internal carry
     * (q->carry) across calls, so no worker-side RF buffer is needed: raw RF
     * blocks are fed to the bank in place, in max_in-sized sub-chunks, and the
     * bank bridges the sub-chunk boundaries to keep the demodulator's sample
     * grid phase-locked.  max_in bounds the output of a single bank call so it
     * never exceeds the output block capacity. */
    size_t max_in;                  /**< max RF samples fed per bank call */
} channelizer_t;

/**
 * Build the channelizer.  Registers @p rf_reader on the radio dispatcher so
 * RF blocks are delivered here.  The channelized output is pushed to @p out,
 * whose readers are the channel processors.
 *
 * @param grid_hz  channel raster: CHANNELIZER_BANK_GRID_BR_EDR_HZ (1 MHz) for
 *                  BR/EDR or CHANNELIZER_BANK_GRID_BLE_HZ (2 MHz) for BLE.
 * @return 0 on success, -1 on failure (caller should treat as fatal).
 */
int channelizer_init(channelizer_t *c,
                     sample_dispatcher_t *rf,
                     sample_dispatcher_t *out,
                     unsigned int sample_rate_hz,
                     uint32_t lo_hz,
                     uint32_t grid_hz,
                     int debug);

void channelizer_destroy(channelizer_t *c);

/** Worker entry point (run on its own thread by the session). */
void *channelizer_worker(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* CHANNELIZER_THREAD_H */
