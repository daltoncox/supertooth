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

    cpfskdem demodulator;
    unsigned int input_decimation;
    unsigned int samps_per_symbol;

    unsigned int bin;
    unsigned int bank_M;
    float rssi_cal_db;

    float complex *decimated;
    size_t buf_cap_samples;

    /* Per-channel BR/EDR bitstream decoder (owns its own packet state). */
    bredr_bitstream_decoder_t decoder;

    bredr_status_t prev_state;

    /* RSSI measured over the access code at detection time, consumed when the
     * packet completes.  The decoder cannot know the true packet length (the
     * header is whitened until CLK1-6 is recovered) and only completes after
     * a fixed maximum-length body, so averaging at completion time would
     * dilute short packets with up to 5 slots of post-packet channel content.
     * The access code is constant-envelope GFSK for every packet type, so an
     * AC-span window at detect time measures the same quantity per packet. */
    float pending_rssi_dbr;
    _Bool pending_rssi_valid;

    /* Per-channel noise/interference floor estimate (linear mean power),
     * tracked for diagnostics only -- NOT subtracted from reported RSSI (see
     * receiver_rssi_signal_dbr).  Seeded from the idle prefix that precedes a
     * packet and smoothed across packets via an EMA. */
    float noise_floor_linear;
    unsigned int noise_floor_initialized;

    unsigned long valid_packets;
    unsigned int  dbg_blocks_seen;

    struct session *session;

    _Bool active;
} bredr_channel_processor_t;

int  bredr_channel_processor_init(bredr_channel_processor_t *proc,
                                  sample_dispatcher_t *dispatcher,
                                  uint16_t rf_channel_index,
                                  uint32_t center_frequency_hz,
                                  unsigned int sample_rate_hz,
                                  unsigned int chan_bin,
                                  unsigned int bank_M,
                                  unsigned int bank_M2,
                                  float rssi_cal_db);

void bredr_channel_processor_destroy(bredr_channel_processor_t *proc);

void *bredr_channel_worker(void *arg);

int  bredr_channel_processor_process_block(bredr_channel_processor_t *proc,
                                           sample_block_t *blk);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CHANNEL_PROCESSOR_NEW_H */
