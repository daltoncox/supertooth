#include "bredr_channel_processor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "radio_common.h"
#include "rssi_measurements.h"
#include "channelizer_bank.h"
#include "session.h"

#ifndef RECEIVER_SOURCE_ID_DEFAULT
#define RECEIVER_SOURCE_ID_DEFAULT 0u
#endif

static rx_metadata_t bredr_make_metadata(uint64_t radio_start_sample_index,
                                         uint32_t radio_sample_rate_hz,
                                         uint32_t center_frequency_hz,
                                         uint16_t channel_index,
                                         float rssi_dbr)
{
    rx_metadata_t meta = {
        .source_id = RECEIVER_SOURCE_ID_DEFAULT,
        .radio_start_sample_index = radio_start_sample_index,
        .radio_sample_rate_hz = radio_sample_rate_hz,
        .center_frequency_hz = center_frequency_hz,
        .channel_index = channel_index,
        .rssi_dbr = rssi_dbr,
    };
    return meta;
}

int bredr_channel_processor_init(bredr_channel_processor_t *proc,
                                 uint16_t rf_channel_index)
{
    if (!proc)
        return -1;
    /* Stream layout lives on the pre-configured reader (service fills it
     * before this runs). No memset here: it would wipe that configuration. */
    if (proc->reader.view_stride == 0u)
        return -1;

    proc->rf_channel_index    = rf_channel_index;
    proc->frequency_offset_hz = 0;
    proc->center_frequency_hz = proc->reader.view_center_hz;
    proc->samps_per_symbol    = BREDR_SESSION_SAMPLES_PER_SYMBOL;

    /* Cached from the reader for the hot loop. */
    proc->input_decimation = proc->reader.view_decimation;
    proc->rssi_cal_db      = proc->reader.view_rssi_cal_db;

    proc->demodulator = cpfskdem_create(1u, 0.5f, proc->samps_per_symbol,
                                        3u, 0.5f, LIQUID_CPFSK_GMSK);
    if (!proc->demodulator)
    {
        bredr_channel_processor_destroy(proc);
        return -1;
    }

    bredr_bitstream_decoder_init(&proc->decoder);

    proc->prev_state               = BREDR_SEARCHING;
    proc->noise_floor_linear       = 0.0f;
    proc->noise_floor_initialized  = 0u;
    proc->pending_rssi_dbr         = RECEIVER_RSSI_INVALID;
    proc->pending_rssi_valid       = 0;
    proc->pending_header_abs_radio = 0u;
    proc->pending_header_valid     = 0;
    proc->prev_block_end_radio     = 0u;
    proc->has_prev_block           = 0;
    proc->valid_packets            = 0ul;
    proc->dbg_blocks_seen          = 0u;
    proc->active                   = 1;
    /* session + reader left as the caller set them. */
    return 0;
}

void bredr_channel_processor_destroy(bredr_channel_processor_t *proc)
{
    if (!proc) return;
    if (proc->demodulator) cpfskdem_destroy(proc->demodulator);
    proc->demodulator = NULL;
    sample_reader_destroy(&proc->reader);
    memset(proc, 0, sizeof(*proc));
}

static int emit_frame(bredr_channel_processor_t *proc,
                      unsigned int end_decim_sample,
                      unsigned int decim_out,
                      unsigned long abs_block_base_radio)
{
    bredr_frame_t frame;
    if (bredr_bitstream_decoder_get_frame(&proc->decoder, &frame) != 0) return -1;

    /* RSSI was measured over the access code when it was detected (see
     * process_block): a fixed-length, signal-only window on constant-envelope
     * GFSK, identical for every packet type.  Measuring here at completion
     * instead would span the decoder's fixed maximum-length collection body
     * (5 slots) and dilute the average with whatever post-packet content
     * (idle floor, interference, block-end clamping) filled the rest of the
     * window -- measured at ~30 dB of packet-to-packet scatter for one
     * stationary device. */
    float rssi_dbr = proc->pending_rssi_valid ? proc->pending_rssi_dbr
                                              : RECEIVER_RSSI_INVALID;
    proc->pending_rssi_dbr   = RECEIVER_RSSI_INVALID;
    proc->pending_rssi_valid = 0;
    rssi_dbr += proc->rssi_cal_db;

    /* Radio sample index = the latched header-start stamp when available (see
     * the field docs in the header): it sits a fixed 68 us after the access
     * code regardless of RF holes spanned during collection.  Without it
     * (unlocatable access code, e.g. straddling a dropped block), fall back
     * to the collection-end stamp. */
    uint64_t abs_radio;
    if (proc->pending_header_valid)
        abs_radio = proc->pending_header_abs_radio;
    else
    {
        unsigned int end_sample = end_decim_sample + proc->samps_per_symbol;
        if (end_sample > decim_out)
            end_sample = decim_out;
        abs_radio = abs_block_base_radio +
            (uint64_t)end_sample * (uint64_t)proc->input_decimation;
    }
    proc->pending_header_valid = 0;

    rx_metadata_t meta = bredr_make_metadata(
        abs_radio,
        proc->input_decimation * 2000000u,
        proc->center_frequency_hz,
        proc->rf_channel_index,
        rssi_dbr);

    if (proc->session && proc->session->config.debug)
    {
        uint32_t rxclk = (uint32_t)((abs_radio * 1600u + 10000000u) / 20000000u);
        fprintf(stderr, "[timing ch=%u] abs_radio=%llu rxclk=%u\n",
                proc->rf_channel_index,
                (unsigned long long)abs_radio, rxclk);
    }

    bredr_event_t event = { .meta = meta, .frame = frame };

    if (proc->session)
    {
        session_t *s = proc->session;
        if (s->bredr_cfg.lap_filter_enabled &&
            ((frame.lap & 0xFFFFFFu) != s->bredr_cfg.lap_filter))
            return 0;
        /* Hand the decoded event to the BR/EDR collector thread; the worker
         * releases its sample block immediately after, decoupling block-pool
         * lifetime from tracking + presentation cost. */
        collector_submit(&s->bredr_collector, &event);
    }

    return 0;
}

int bredr_channel_processor_process_stream(bredr_channel_processor_t *proc,
                                            const float complex *samples,
                                            unsigned int count,
                                            uint64_t base_radio)
{
    if (!proc || !proc->active || !samples) return -1;

    unsigned int decim_out = count;

    int dbg = proc->session ? proc->session->config.debug : 0;
    if (dbg && proc->dbg_blocks_seen < 4u)
        fprintf(stderr,
                "[bredr_proc ch=%u] block #%u: decim_out=%u\n",
                proc->rf_channel_index, proc->dbg_blocks_seen,
                decim_out);
    proc->dbg_blocks_seen++;

    unsigned int num_bits = decim_out / proc->samps_per_symbol;
    for (unsigned int s = 0u; s < num_bits; s++)
    {
        unsigned int sample_index = s * proc->samps_per_symbol;
        /* Liquid's demodulate omits const (it only reads); our stream is const. */
        uint32_t raw_sym_val = cpfskdem_demodulate(proc->demodulator,
                                    (float complex *)&samples[sample_index]);
        uint8_t bit = (uint8_t)(raw_sym_val & 0x1u);

        bredr_status_t status = bredr_bitstream_decoder_push_bit(&proc->decoder, bit);

        if (proc->prev_state == BREDR_SEARCHING && status != BREDR_SEARCHING)
        {
            /* Access-code detection completed on this bit: the AC occupies
             * the BREDR_AC_DETECT_SAMPLES samples ending one symbol past
             * sample_index.  Measure RSSI now, over that fixed signal-only
             * window; emit_frame reports this pending value when the packet
             * completes. */
            unsigned int ac_end = sample_index + proc->samps_per_symbol;
            unsigned int i_start = 0u, i_end = 0u, i_idle = 0u;
            receiver_rssi_access_code_window(ac_end, BREDR_AC_DETECT_SAMPLES,
                                             decim_out,
                                             &i_start, &i_end, &i_idle);
            proc->pending_rssi_dbr = receiver_rssi_signal_dbr(
                samples, i_start, i_end, i_idle,
                &proc->noise_floor_linear, &proc->noise_floor_initialized,
                RECEIVER_RSSI_INVALID);
            proc->pending_rssi_valid = !isnan(proc->pending_rssi_dbr);

            /* Latch the header-start timestamp now, for the same reason:
             * the 64-bit sync word ends at ac_end, so it starts at
             * ac_end - 64 (decimated samples) and the header follows the
             * 4-bit trailer 68 samples later.  emit_frame stamps the packet
             * with this instead of the collection-end position, keeping the
             * header-to-stamp distance hole-independent (see field docs). */
            int sync_start = (int)ac_end - 64;
            if (sync_start >= 0)
            {
                uint64_t abs_sync = base_radio +
                    (uint64_t)sync_start * (uint64_t)proc->input_decimation;
                proc->pending_header_abs_radio =
                    abs_sync + 68u * (uint64_t)proc->input_decimation;
                proc->pending_header_valid = 1;
            }
            else if (proc->has_prev_block &&
                     proc->prev_block_end_radio == base_radio)
            {
                proc->pending_header_abs_radio =
                    proc->prev_block_end_radio -
                    (uint64_t)(-sync_start) * (uint64_t)proc->input_decimation +
                    68u * (uint64_t)proc->input_decimation;
                proc->pending_header_valid = 1;
            }
            else
            {
                proc->pending_header_valid = 0;
            }
        }

        if (status == BREDR_ERROR)
        {
            proc->pending_rssi_dbr   = RECEIVER_RSSI_INVALID;
            proc->pending_rssi_valid = 0;
            proc->pending_header_valid = 0;
        }

        proc->prev_state = status;

        if (status == BREDR_VALID_PACKET)
        {
            proc->valid_packets++;
            emit_frame(proc, sample_index, decim_out, base_radio);
        }
    }
    /* Remember this block's trailing edge so an access code detected at the
     * top of the next block can still be located (see the latch above). */
    proc->prev_block_end_radio = base_radio +
        (uint64_t)decim_out * (uint64_t)proc->input_decimation;
    proc->has_prev_block = 1;
    return 0;
}

void *bredr_channel_worker(void *arg)
{
    bredr_channel_processor_t *proc = (bredr_channel_processor_t *)arg;
    if (!proc || !proc->session) return NULL;

    const _Atomic unsigned int *shutdown = &proc->session->shutdown_requested;

    for (;;)
    {
        const float complex *samples;
        unsigned int count;
        uint64_t base;

        /* The reader's single call pops, gathers and holds the block. */
        if (sample_reader_next(&proc->reader, shutdown,
                               &samples, &count, &base) != 0)
            break;

        bredr_channel_processor_process_stream(proc, samples, count, base);
    }

    return NULL;
}
