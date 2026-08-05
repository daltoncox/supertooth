/**
 * @file service/channelizer_thread.h
 * @brief BR/EDR polyphase channelizer worker.
 *
 * Reads wideband RF blocks from the radio's sample dispatcher, runs them
 * through a single `channelizer_bank_t`, and writes frame-major blocks
 * (layout `out[frame * M + bin]`, see channelizer_bank.h) into a second
 * dispatcher.  Each BR/EDR channel processor is a reader of that second
 * dispatcher and pulls its own bin with a uniform stride of M.
 *
 * This is the BR/EDR side of the hybrid runtime: BLE (when enabled) keeps
 * reading raw RF blocks from the radio dispatcher on the legacy per-channel
 * NCO + firdecim path.
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
} bredr_channelizer_t;

/**
 * Build the channelizer.  Registers @p rf_reader on the radio dispatcher so
 * RF blocks are delivered here.  The channelized output is pushed to @p out,
 * whose readers are the BR/EDR channel processors.
 *
 * @return 0 on success, -1 on failure (caller should fall back to legacy).
 */
int bredr_channelizer_init(bredr_channelizer_t *c,
                           sample_dispatcher_t *rf,
                           sample_dispatcher_t *out,
                           unsigned int sample_rate_hz,
                           uint32_t lo_hz,
                           uint32_t grid_hz,
                           int debug);

void bredr_channelizer_destroy(bredr_channelizer_t *c);

/** Worker entry point (run on its own thread by the session). */
void *bredr_channelizer_worker(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* CHANNELIZER_THREAD_H */
