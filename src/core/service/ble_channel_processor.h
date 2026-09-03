#ifndef BLE_CHANNEL_PROCESSOR_NEW_H
#define BLE_CHANNEL_PROCESSOR_NEW_H

#include <pthread.h>
#include <stdint.h>

#include <liquid/liquid.h>

#include "ble_bitstream_decoder.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SESSION_SAMPLES_PER_SYMBOL 2u
#define BLE_SESSION_MAX_CHANNELS 10u
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
    unsigned int input_decimation;

    unsigned int bin;
    unsigned int bank_M;
    unsigned int frame_stride;   /**< frames to skip per output sample (grid/1MHz) */
    float rssi_cal_db;

    float complex *decimated;
    size_t buf_cap_samples;

    unsigned int abs_sample_scale;

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

int ble_channel_processor_init(ble_channel_processor_t *proc,
                               sample_dispatcher_t *dispatcher,
                               uint16_t rf_index,
                               uint32_t center_frequency_hz,
                               unsigned int sample_rate_hz,
                               unsigned int chan_bin,
                               unsigned int bank_M,
                               unsigned int bank_M2,
                                unsigned int frame_stride,
                                float rssi_cal_db);

void ble_channel_processor_destroy(ble_channel_processor_t *proc);

void *ble_channel_worker(void *arg);

int ble_channel_processor_process_block(ble_channel_processor_t *proc, sample_block_t *blk);

#ifdef __cplusplus
}
#endif

#endif