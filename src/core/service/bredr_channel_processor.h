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

    /* Absolute radio-sample index of the current packet's header start,
     * latched when its access code is detected and consumed when the packet
     * completes (same lifetime as pending_rssi_*).
     *
     * The bitstream decoder only completes after a fixed maximum-length body
     * (the header is whitened until CLK1-6 is recovered, so the true length
     * is unknowable up front).  Timestamping at completion would place the
     * stamp a fixed ~3 ms after the header -- plus any wall-clock holes from
     * RF blocks dropped under load that the collection spanned.  Clock
     * recovery compares the header against the central clock derived from
     * this stamp, so a hole-shifted stamp validates at the wrong CLK1-6 and
     * actively decays the lock (see recover_clock_drift): drops would not
     * just remove packets, they would poison the survivors.  Latching the
     * stamp at the access code keeps the header-to-stamp distance fixed
     * (68 us) regardless of downstream drops, so drops only ever remove
     * packets and tracking rides through bottlenecks. */
    uint64_t pending_header_abs_radio;
    _Bool pending_header_valid;

    /* Trailing edge of the previously processed channel block, so an access
     * code straddling a block boundary can still be located.  Zero until the
     * first block has been seen. */
    uint64_t prev_block_end_radio;
    _Bool has_prev_block;

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
