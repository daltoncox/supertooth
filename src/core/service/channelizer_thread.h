/**
 * @file service/channelizer_thread.h
 * @brief Polyphase channelizer worker (shared by BR/EDR and BLE).
 *
 * Reads wideband RF blocks from the radio's sample dispatcher, runs them
 * through a single `channelizer_bank_t`, and writes frame-major blocks
 * (layout `out[frame * M + bin]`, see channelizer_bank.h) into a second
 * dispatcher.  Each channel processor (BR/EDR or BLE) is a reader of that
 * second dispatcher and pulls its own bin with a uniform stride of M.
 *
 * BR/EDR and BLE instances are the same object configured with different
 * grids: BR/EDR uses the 1 MHz raster (CHANNELIZER_BANK_GRID_BR_EDR_HZ), BLE
 * uses the 2 MHz raster (CHANNELIZER_BANK_GRID_BLE_HZ).  Each session owns two
 * of these — one per protocol — driven by the same worker entry point.
 */

#ifndef CHANNELIZER_THREAD_H
#define CHANNELIZER_THREAD_H

#include <stdatomic.h>
#include <stdint.h>

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
