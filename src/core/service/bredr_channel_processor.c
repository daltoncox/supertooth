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
                                 sample_dispatcher_t *dispatcher,
                                 uint16_t rf_channel_index,
                                 uint32_t center_frequency_hz,
                                 unsigned int sample_rate_hz,
                                 unsigned int chan_bin,
                                 unsigned int bank_M,
                                 unsigned int bank_M2,
                                 float rssi_cal_db)
{
    if (!proc || !dispatcher) return -1;
    memset(proc, 0, sizeof(*proc));

    proc->rf_channel_index    = rf_channel_index;
    proc->frequency_offset_hz = 0;
    proc->center_frequency_hz  = center_frequency_hz;
    proc->samps_per_symbol     = BREDR_SESSION_SAMPLES_PER_SYMBOL;

    proc->bin              = chan_bin;
    proc->bank_M           = bank_M;
    proc->input_decimation = bank_M2;
    proc->rssi_cal_db      = rssi_cal_db;

    if (sample_reader_init(&proc->reader, dispatcher) != 0)
    {
        bredr_channel_processor_destroy(proc);
        return -1;
    }

    proc->demodulator = cpfskdem_create(1u, 0.5f, proc->samps_per_symbol,
                                        3u, 0.5f, LIQUID_CPFSK_GMSK);
    if (!proc->demodulator) { bredr_channel_processor_destroy(proc); return -1; }

    /* Frame-major block holds at most SAMPLE_BLOCK_SAMPLE_CAPACITY / M frames. */
    proc->buf_cap_samples = SAMPLE_BLOCK_SAMPLE_CAPACITY / (size_t)bank_M + 16u;
    proc->decimated = malloc(sizeof(float complex) * proc->buf_cap_samples);
    if (!proc->decimated) { bredr_channel_processor_destroy(proc); return -1; }

    bredr_bitstream_decoder_init(&proc->decoder, BREDR_AC_ERRORS_DEFAULT);

    proc->prev_state               = BREDR_SEARCHING;
    proc->pkt_start_decim_sample   = 0u;
    proc->block_start_decim_sample = 0u;
    proc->block_start_bit_index    = 0u;
    proc->noise_floor_linear       = 0.0f;
    proc->noise_floor_initialized  = 0u;
    proc->active                   = 1;
    return 0;
}

void bredr_channel_processor_destroy(bredr_channel_processor_t *proc)
{
    if (!proc) return;
    if (proc->demodulator) cpfskdem_destroy(proc->demodulator);
    free(proc->decimated);
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

    /* RSSI is averaged over the packet span in the decimated buffer: from the
     * packet start sample (derived from the decoder's absolute bit index) to
     * the end of the current symbol, with a small pre-trigger guard.  The idle
     * prefix before the packet seeds a per-channel noise-floor estimate that is
     * subtracted so the reported value reflects signal alone. */
    unsigned int i_start = 0u, i_end = 0u, i_idle = 0u;
    receiver_rssi_packet_window(proc->block_start_bit_index,
                                frame.start_bit_index,
                                end_decim_sample,
                                proc->samps_per_symbol,
                                decim_out,
                                &i_start, &i_end, &i_idle);

    /* Skip the demodulator group-delay region at the head of the packet. */
    if (i_end > i_start + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES)
        i_start += RECEIVER_RSSI_DEMOD_DELAY_SAMPLES;

    float rssi_dbr = receiver_rssi_signal_dbr(
        proc->decimated, i_start, i_end, i_idle,
        &proc->noise_floor_linear, &proc->noise_floor_initialized,
        RECEIVER_RSSI_INVALID);
    rssi_dbr += proc->rssi_cal_db;

    /* Radio sample index = block base (input samples) + decimated offset
     * scaled back up to the input rate by input_decimation. */
    uint64_t abs_radio = abs_block_base_radio +
        (uint64_t)i_end * (uint64_t)proc->input_decimation;

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
        session_process_bredr_event(s, &event);
    }

    return 0;
}

int bredr_channel_processor_process_block(bredr_channel_processor_t *proc, sample_block_t *blk)
{
    if (!proc || !proc->active || !blk) return -1;

    proc->block_start_bit_index = proc->decoder.total_bits_seen;

    unsigned int decim_out;

    unsigned int frames = blk->num_samples / proc->bank_M;
    if (frames > proc->buf_cap_samples)
        frames = (unsigned int)proc->buf_cap_samples;
    for (unsigned int k = 0u; k < frames; k++)
        proc->decimated[k] = blk->samples[proc->bin + (size_t)k * proc->bank_M];
    decim_out = frames;
    proc->block_start_decim_sample = blk->block_base_sample / proc->input_decimation;

    int dbg = proc->session ? proc->session->config.debug : 0;
    if (dbg && proc->dbg_blocks_seen < 4u)
        fprintf(stderr,
                "[bredr_proc ch=%u] block #%u: num_samples=%u decim_out=%u\n",
                proc->rf_channel_index, proc->dbg_blocks_seen,
                blk->num_samples, decim_out);
    proc->dbg_blocks_seen++;

    unsigned int num_bits = decim_out / proc->samps_per_symbol;
    for (unsigned int s = 0u; s < num_bits; s++)
    {
        unsigned int sample_index = s * proc->samps_per_symbol;
        uint32_t raw_sym_val = cpfskdem_demodulate(proc->demodulator,
                                                   &proc->decimated[sample_index]);
        uint8_t bit = (uint8_t)(raw_sym_val & 0x1u);

        bredr_status_t status = bredr_bitstream_decoder_push_bit(&proc->decoder, bit);

        if (proc->prev_state == BREDR_SEARCHING && status != BREDR_SEARCHING)
            proc->pkt_start_decim_sample = proc->block_start_decim_sample + sample_index;

        proc->prev_state = status;

        if (status == BREDR_VALID_PACKET)
        {
            proc->valid_packets++;
            if (dbg)
                fprintf(stderr,
                        "[bredr_proc ch=%u] VALID_PACKET #%lu (decim_out=%u)\n",
                        proc->rf_channel_index, proc->valid_packets, decim_out);
            emit_frame(proc, sample_index, decim_out, blk->block_base_sample);
        }
    }
    return 0;
}

void *bredr_channel_worker(void *arg)
{
    bredr_channel_processor_t *proc = (bredr_channel_processor_t *)arg;
    if (!proc || !proc->session) return NULL;

    const _Atomic unsigned int *shutdown = &proc->session->shutdown_requested;
    sample_block_t *block = NULL;

    for (;;)
    {
        if (sample_reader_wait_pop(&proc->reader, shutdown, &block) != 0)
            break;

        if (block)
        {
            bredr_channel_processor_process_block(proc, block);
            sample_block_release(block);
            block = NULL;
        }
    }

    return NULL;
}
