/**
 * @file service/ble_channelizer.h
 * @brief BLE polyphase channelizer worker.
 *
 * Reads wideband RF blocks from the radio's sample dispatcher, runs them
 * through a single `channelizer_bank_t` with 2 MHz grid, and writes frame-major
 * blocks (layout `out[frame * M + bin]`, see channelizer_bank.h) into a second
 * dispatcher.  Each BLE channel processor is a reader of that second
 * dispatcher and pulls its own bin with a uniform stride of M.
 */

#ifndef BLE_CHANNELIZER_H
#define BLE_CHANNELIZER_H

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
    sample_reader_t    rf_reader;
    sample_dispatcher_t *out;
    const _Atomic unsigned int *shutdown;
    int debug;
    int active;
} ble_channelizer_t;

int ble_channelizer_init(ble_channelizer_t *c,
                         sample_dispatcher_t *rf,
                         sample_dispatcher_t *out,
                         unsigned int sample_rate_hz,
                         uint32_t lo_hz,
                         uint32_t grid_hz,
                         int debug);

void ble_channelizer_destroy(ble_channelizer_t *c);

void *ble_channelizer_worker(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CHANNELIZER_H */
