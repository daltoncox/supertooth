#include "bredr_channel_processor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "radio_common.h"
#include "rssi_measurements.h"
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
                                 int32_t frequency_offset_hz,
                                 uint32_t center_frequency_hz,
                                 unsigned int sample_rate_hz)
{
    if (!proc || !dispatcher) return -1;
    memset(proc, 0, sizeof(*proc));

    proc->rf_channel_index    = rf_channel_index;
    proc->frequency_offset_hz = frequency_offset_hz;
    proc->center_frequency_hz  = center_frequency_hz;
    proc->samps_per_symbol     = BREDR_SESSION_SAMPLES_PER_SYMBOL;

    if (sample_reader_init(&proc->reader, dispatcher) != 0)
    {
        bredr_channel_processor_destroy(proc);
        return -1;
    }

    double rate_d   = (double)sample_rate_hz;
    int decimation  = (int)(rate_d / (2.0 * 1e6));
    if (decimation < 1) decimation = 1;
    proc->input_decimation = (unsigned int)decimation;

    proc->nco = nco_crcf_create(LIQUID_NCO);
    if (!proc->nco) { bredr_channel_processor_destroy(proc); return -1; }
    double omega_nco = ((double)frequency_offset_hz / rate_d) * 2.0 * M_PI;
    nco_crcf_set_frequency(proc->nco, (float)omega_nco);

    proc->decimator = firdecim_crcf_create_kaiser((unsigned int)decimation, 7u, 60.0f);
    if (!proc->decimator) { bredr_channel_processor_destroy(proc); return -1; }

    unsigned int m_taps   = 3u;
    proc->demodulator = cpfskdem_create(1u, 0.5f, proc->samps_per_symbol,
                                        m_taps, 0.5f, LIQUID_CPFSK_GMSK);
    if (!proc->demodulator) { bredr_channel_processor_destroy(proc); return -1; }

    proc->buf_cap_samples = SAMPLE_BLOCK_SAMPLE_CAPACITY;
    proc->mixed_buf       = malloc(sizeof(float complex) * proc->buf_cap_samples);
    size_t decim_out_cap  = proc->buf_cap_samples / (size_t)decimation + 16u;
    proc->decimated       = malloc(sizeof(float complex) * decim_out_cap);
    if (!proc->mixed_buf || !proc->decimated) { bredr_channel_processor_destroy(proc); return -1; }

    bredr_bitstream_decoder_init(&proc->decoder, BREDR_AC_ERRORS_DEFAULT);

    proc->prev_state                = BREDR_SEARCHING;
    proc->pkt_start_decim_sample    = 0u;
    proc->block_start_decim_sample  = 0u;
    proc->block_start_bit_index     = 0u;
    proc->active                    = 1;
    return 0;
}

void bredr_channel_processor_destroy(bredr_channel_processor_t *proc)
{
    if (!proc) return;
    if (proc->demodulator) cpfskdem_destroy(proc->demodulator);
    if (proc->decimator)   firdecim_crcf_destroy(proc->decimator);
    if (proc->nco)         nco_crcf_destroy(proc->nco);
    free(proc->mixed_buf);
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
     * the end of the current symbol, with a small pre-trigger guard. */
    unsigned long rel_start = 0u;
    if (frame.start_bit_index > proc->block_start_bit_index)
        rel_start = (unsigned long)(frame.start_bit_index - proc->block_start_bit_index);
    unsigned int i_start = (unsigned int)(rel_start / proc->samps_per_symbol);
    if (i_start > 4u) i_start -= 4u;
    unsigned int i_end = end_decim_sample + proc->samps_per_symbol;
    if (i_end > decim_out) i_end = decim_out;

    float rssi_dbr = receiver_rssi_from_mean_power_range(
        proc->decimated, i_start, i_end, RECEIVER_RSSI_INVALID);

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

    unsigned int input_count = blk->num_samples;
    unsigned int decim_count = input_count / proc->input_decimation;

    nco_crcf_mix_block_down(proc->nco, blk->samples, proc->mixed_buf, input_count);

    firdecim_crcf_execute_block(proc->decimator, proc->mixed_buf,
                               decim_count, proc->decimated);
    unsigned int decim_out = decim_count;

    proc->block_start_decim_sample = blk->block_base_sample / proc->input_decimation;

    int dbg = proc->session ? proc->session->config.debug : 0;
    if (dbg && proc->dbg_blocks_seen < 4u)
        fprintf(stderr,
                "[bredr_proc ch=%u] block #%u: num_samples=%u decim_count=%u "
                "decim_out=%u\n",
                proc->rf_channel_index, proc->dbg_blocks_seen,
                input_count, decim_count, decim_out);
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
        if (sample_reader_wait_pop(&proc->reader,
                                   (const unsigned int *)shutdown, &block) != 0)
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
