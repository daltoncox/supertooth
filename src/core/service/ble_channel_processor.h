#ifndef BLE_CHANNEL_PROCESSOR_NEW_H
#define BLE_CHANNEL_PROCESSOR_NEW_H

#include <pthread.h>
#include <stdint.h>

#include <liquid/liquid.h>

#include "ble_bitstream_decoder.h"
#include "channelizer_service.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SESSION_SAMPLES_PER_SYMBOL 2u
/* Max LE RF channels per session: the full 40-channel band (80 Msps staged
 * over 4 lanes on wideband-capable devices; HackRF stays at 10). */
#define BLE_SESSION_MAX_CHANNELS 40u
#define BLE_SESSION_LNA_GAIN 24u
#define BLE_SESSION_VGA_GAIN 18u

struct session;

typedef struct {
    uint16_t rf_channel_index;
    int32_t frequency_offset_hz;
    uint32_t center_frequency_hz;

    sample_reader_t reader;

    cpfskdem demodulator;
    unsigned int samples_per_symbol;
    unsigned int input_decimation; /**< cached from reader.view_decimation */
    float rssi_cal_db;             /**< cached from reader.view_rssi_cal_db */

    ble_bitstream_decoder_t decoder;

    uint64_t block_start_decim_sample;
    long pkt_start_decim_sample;
    ble_status_t prev_state;

    /* Per-channel noise/interference floor estimate (linear mean power),
     * tracked for diagnostics only -- NOT subtracted from reported RSSI (see
     * receiver_rssi_signal_dbr).  Seeded from the idle prefix that precedes a
     * packet and smoothed across packets via an EMA. */
    float noise_floor_linear;
    unsigned int noise_floor_initialized;

    unsigned long valid_packets;
    unsigned int dbg_blocks_seen;

    struct session *session;

    _Bool active;
} ble_channel_processor_t;

/* The reader is configured by the channelizer service before init (bin,
 * stride, centre and friends live on reader.view_*); init only wires the
 * demodulator/decoder and caches stream scalars. */
int ble_channel_processor_init(ble_channel_processor_t *proc,
                               uint16_t rf_index);

void ble_channel_processor_destroy(ble_channel_processor_t *proc);

void *ble_channel_worker(void *arg);

/* Demodulate one gathered stream (as returned by sample_reader_next):
 * @p samples holds @p count contiguous 2 Msps samples, @p base_radio is
 * the block's RF-domain base. */
int ble_channel_processor_process_stream(ble_channel_processor_t *proc,
                                         const float complex *samples,
                                         unsigned int count,
                                         uint64_t base_radio);

#ifdef __cplusplus
}
#endif

#endif