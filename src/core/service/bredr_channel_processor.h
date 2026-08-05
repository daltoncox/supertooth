#ifndef BREDR_CHANNEL_PROCESSOR_NEW_H
#define BREDR_CHANNEL_PROCESSOR_NEW_H

#include <pthread.h>
#include <stdint.h>

#include <liquid/liquid.h>

#include "bredr_bitstream_decoder.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BREDR_SESSION_SAMPLES_PER_SYMBOL 2u
#define BREDR_SESSION_SYMBOL_STEP        2u
#define BREDR_SESSION_MAX_CHANNELS       79u

struct session;

typedef struct {
    uint16_t rf_channel_index;
    int32_t  frequency_offset_hz;
    uint32_t center_frequency_hz;

    sample_reader_t reader;

    nco_crcf nco;
    firdecim_crcf decimator;
    cpfskdem demodulator;
    unsigned int input_decimation;
    unsigned int samps_per_symbol;

    /** When non-zero, `process_block` reads its bin from a frame-major
     *  channelizer block (stride `bank_M`) instead of mix+decimating RF. */
    int uses_channelizer;
    unsigned int bin;        /**< channelizer bin index for this channel */
    unsigned int bank_M;     /**< number of bins in the channelizer bank */
    float rssi_cal_db;       /**< added to RSSI to match the legacy chain */

    float complex *mixed_buf;
    float complex *decimated;
    size_t buf_cap_samples;

    /* Per-channel BR/EDR bitstream decoder (owns its own packet state). */
    bredr_bitstream_decoder_t decoder;

    /* Decimated-sample indices tracking for RSSI + metadata. */
    uint64_t block_start_decim_sample;
    uint64_t block_start_bit_index;
    uint64_t pkt_start_decim_sample;
    bredr_status_t prev_state;

    unsigned long valid_packets;
    unsigned int  dbg_blocks_seen;

    struct session *session;

    _Bool active;
} bredr_channel_processor_t;

int  bredr_channel_processor_init(bredr_channel_processor_t *proc,
                                 sample_dispatcher_t *dispatcher,
                                 uint16_t rf_channel_index,
                                 int32_t frequency_offset_hz,
                                 uint32_t center_frequency_hz,
                                 unsigned int sample_rate_hz);

void bredr_channel_processor_destroy(bredr_channel_processor_t *proc);

/**
 * Initialise a processor that consumes frame-major channelizer output.
 *
 * @param dispatcher      the frame-major dispatcher (channelizer output)
 * @param chan_bin        this channel's bin index in the bank
 * @param bank_M          number of bins in the bank (the strided read stride)
 * @param bank_M2         bank decimation (input samples per output frame)
 * @param rssi_cal_db     RSSI calibration to add (CHANNELIZER_BANK_RSSI_CAL_DB)
 */
int  bredr_channel_processor_init_channelizer(bredr_channel_processor_t *proc,
                                              sample_dispatcher_t *dispatcher,
                                              uint16_t rf_channel_index,
                                              uint32_t center_frequency_hz,
                                              unsigned int sample_rate_hz,
                                              unsigned int chan_bin,
                                              unsigned int bank_M,
                                              unsigned int bank_M2,
                                              float rssi_cal_db);

void *bredr_channel_worker(void *arg);

int  bredr_channel_processor_process_block(bredr_channel_processor_t *proc,
                                           sample_block_t *blk);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CHANNEL_PROCESSOR_NEW_H */
