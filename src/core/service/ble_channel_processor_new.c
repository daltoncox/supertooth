#include "ble_channel_processor_new.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ble_codec.h"
#include "ble_piconet.h"
#include "session.h"
#include "rssi_measurements.h"

int ble_channel_processor_init(ble_channel_processor_t *proc,
                               sample_dispatcher_t *dispatcher,
                               uint16_t rf_channel_index,
                               int32_t frequency_offset_hz,
                               uint32_t center_frequency_hz,
                               unsigned int sample_rate_hz,
                               struct ble_piconet_store *store)
{
    if (!proc || !dispatcher || rf_channel_index >= BLE_RF_CHANNEL_COUNT) return -1;
    memset(proc, 0, sizeof(*proc));

    proc->rf_channel_index    = rf_channel_index;
    proc->frequency_offset_hz = frequency_offset_hz;
    proc->center_frequency_hz = center_frequency_hz;
    proc->samples_per_symbol  = BLE_SESSION_SAMPLES_PER_SYMBOL;

    if (sample_reader_init(&proc->reader, dispatcher) != 0) { ble_channel_processor_destroy(proc); return -1; }

    double rate_d   = (double)sample_rate_hz;
    int decimation  = (int)(rate_d / (2.0 * 1e6));
    if (decimation < 1) decimation = 1;
    proc->input_decimation = (unsigned int)decimation;

    proc->nco                 = nco_crcf_create(LIQUID_NCO);
    if (!proc->nco) { ble_channel_processor_destroy(proc); return -1; }
    double omega_nco          = ((double)frequency_offset_hz / rate_d) * 2.0 * M_PI;
    nco_crcf_set_frequency(proc->nco, (float)omega_nco);

    proc->decimator = firdecim_crcf_create_kaiser((unsigned int)decimation, 7u, 60.0f);
    if (!proc->decimator) { ble_channel_processor_destroy(proc); return -1; }

    unsigned int m_taps   = 3u;
    proc->demodulator = cpfskdem_create(1u, 0.5f, proc->samples_per_symbol,
                                        m_taps, 0.5f, LIQUID_CPFSK_GMSK);
    if (!proc->demodulator) { ble_channel_processor_destroy(proc); return -1; }

    proc->buf_cap_samples = SAMPLE_BLOCK_SAMPLE_CAPACITY;
    proc->mixed_buf       = malloc(sizeof(float complex) * proc->buf_cap_samples);
    size_t decim_out_cap  = proc->buf_cap_samples / (size_t)decimation + 16u;
    proc->decimated       = malloc(sizeof(float complex) * decim_out_cap);
    if (!proc->mixed_buf || !proc->decimated) { ble_channel_processor_destroy(proc); return -1; }

    proc->abs_sample_scale = (sample_rate_hz > 0u)
                             ? (uint64_t)(1000000000ull / sample_rate_hz)
                             : 1u;

    uint8_t le_ch = ble_rf_to_le_channel(rf_channel_index);
    ble_bitstream_decoder_init(&proc->decoder, le_ch, store);

    proc->prev_state       = BLE_SEARCHING;
    proc->pkt_start_decim_sample = -1L;
    proc->active           = 1;
    return 0;
}

void ble_channel_processor_destroy(ble_channel_processor_t *proc)
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

static int emit_frame(ble_channel_processor_t *proc,
                      unsigned long block_start_decim_sample,
                      unsigned int end_decim_sample,
                      unsigned int decim_out,
                      unsigned long abs_block_base_radio)
{
    ble_frame_t frame;
    if (ble_bitstream_decoder_get_frame(&proc->decoder, &frame) != 0) return -1;

    /* RSSI is averaged over the packet symbols in the decimated buffer: from
     * the detected packet start (in decimated samples) to the end of the
     * current symbol. Indices bound by decim_out (the actual returned count). */
    unsigned int i_start_ui = 0u;
    unsigned int i_end = 0u;
    if (proc->pkt_start_decim_sample >= 0)
    {
        long rel_start_l = (long)proc->pkt_start_decim_sample
                           - (long)block_start_decim_sample;
        i_start_ui = (rel_start_l > 0) ? (unsigned)rel_start_l : 0u;
        i_end = end_decim_sample + proc->samples_per_symbol;
        if (i_end > decim_out)
            i_end = decim_out;
    }

    float rssi_dbr = receiver_rssi_from_mean_power_range(
        proc->decimated, i_start_ui, i_end, RECEIVER_RSSI_INVALID);

    /* Radio sample index = block base (input samples) + decimated-sample offset
     * scaled back up to the input rate by input_decimation. Mirrors the old
     * path: (decim_buf_start + sample_index) * sample_scale. */
    rx_metadata_t meta;
    memset(&meta, 0, sizeof(meta));
    meta.source_id                = 0u;
    meta.radio_start_sample_index = abs_block_base_radio
                                   + (unsigned long)end_decim_sample
                                     * (unsigned long)proc->input_decimation;
    meta.radio_sample_rate_hz     = proc->input_decimation * 2000000u;
    meta.center_frequency_hz      = proc->center_frequency_hz;
    meta.channel_index            = ble_rf_to_le_channel(proc->rf_channel_index);
    meta.rssi_dbr                 = rssi_dbr;

    ble_event_t event = { .meta = meta, .frame = frame };

    if (proc->session)
        session_process_ble_event(proc->session, &event);

    return 0;
}

int ble_channel_processor_process_block(ble_channel_processor_t *proc, sample_block_t *blk)
{
    if (!proc || !proc->active || !blk) return -1;
    unsigned int input_count = blk->num_samples;
    unsigned int decim_count = input_count / proc->input_decimation;

    nco_crcf_mix_block_down(proc->nco, blk->samples, proc->mixed_buf, input_count);

    firdecim_crcf_execute_block(proc->decimator, proc->mixed_buf,
                                decim_count, proc->decimated);
    /* firdecim_crcf_execute_block(q, x, _n, y) takes _n = number of OUTPUT
     * samples and reads _n*_M inputs; in this liquid build it returns 0 (not
     * the count). So the produced sample count is exactly the _n we passed,
     * which is decim_count (= input_count / input_decimation). */
    unsigned int decim_out = decim_count;

    unsigned long block_start_decim_sample = blk->block_base_sample / proc->input_decimation;

    int dbg = proc->session ? proc->session->config.debug : 0;
    if (dbg && proc->dbg_blocks_seen < 4u)
        fprintf(stderr,
                "[ble_proc rf=%u] block #%u: num_samples=%u decim_count=%u "
                "decim_out=%u num_bits=%u\n",
                proc->rf_channel_index, proc->dbg_blocks_seen,
                input_count, decim_count, decim_out, decim_out / proc->samples_per_symbol);
    proc->dbg_blocks_seen++;

    /* liquid's cpfskdem_demodulate consumes exactly _k samples per call
     * (samples_per_symbol) and returns one symbol; it carries phase/state
     * across calls, so the window must advance by _k each iteration. */
    unsigned int num_bits = decim_out / proc->samples_per_symbol;
    for (unsigned int s = 0u; s < num_bits; s++)
    {
        unsigned int sample_index = s * proc->samples_per_symbol;
        uint32_t raw_sym_val = cpfskdem_demodulate(proc->demodulator,
                                                    &proc->decimated[sample_index]);
        uint8_t bit = (uint8_t)(raw_sym_val & 0x1u);

        ble_status_t status = ble_bitstream_decoder_push_bit(&proc->decoder, bit);

        if (proc->prev_state == BLE_SEARCHING && status != BLE_SEARCHING)
            proc->pkt_start_decim_sample = (long)(block_start_decim_sample + sample_index);

        proc->prev_state = status;

        if (status == BLE_VALID_PACKET)
        {
            proc->valid_packets++;
            if (dbg)
                fprintf(stderr,
                        "[ble_proc rf=%u] VALID_PACKET #%lu (decim_out=%u)\n",
                        proc->rf_channel_index, proc->valid_packets, decim_out);
            emit_frame(proc, block_start_decim_sample, sample_index,
                       decim_out, blk->block_base_sample);
        }
    }
    return 0;
}

void *ble_channel_worker(void *arg)
{
    ble_channel_processor_t *proc = (ble_channel_processor_t *)arg;
    if (!proc || !proc->session) return NULL;

    const _Atomic unsigned int *shutdown = &proc->session->shutdown_requested;
    sample_block_t *block = NULL;

    for (;;)
    {
        if (sample_reader_wait_pop(&proc->reader, shutdown, &block) != 0)
            break;

        if (block)
        {
            ble_channel_processor_process_block(proc, block);
            sample_block_release(block);
            block = NULL;
        }
    }

    return NULL;
}