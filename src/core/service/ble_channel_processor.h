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

    nco_crcf nco;
    firdecim_crcf decimator;
    cpfskdem demodulator;
    unsigned int samples_per_symbol;
    unsigned int input_decimation;

    float complex *mixed_buf;
    float complex *decimated;
    size_t buf_cap_samples;

    unsigned int abs_sample_scale;

    ble_bitstream_decoder_t decoder;

    long pkt_start_decim_sample;
    ble_status_t prev_state;

    unsigned long valid_packets;
    unsigned int dbg_blocks_seen;

    struct session *session;

    _Bool active;
} ble_channel_processor_t;

int ble_channel_processor_init(ble_channel_processor_t *proc,
                               sample_dispatcher_t *dispatcher,
                               uint16_t rf_index,
                               int32_t frequency_offset_hz,
                               uint32_t center_frequency_hz,
                               unsigned int sample_rate_hz,
                               struct ble_piconet_store *store);

void ble_channel_processor_destroy(ble_channel_processor_t *proc);

void *ble_channel_worker(void *arg);

int ble_channel_processor_process_block(ble_channel_processor_t *proc, sample_block_t *blk);

#ifdef __cplusplus
}
#endif

#endif