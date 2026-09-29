#include "ble_channel_processor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ble_codec.h"
#include "session.h"
#include "rssi_measurements.h"

int ble_channel_processor_init(ble_channel_processor_t *proc,
                               uint16_t rf_channel_index)
{
    if (!proc || rf_channel_index >= BLE_RF_CHANNEL_COUNT)
        return -1;
    /* Stream layout lives on the pre-configured reader (service fills it
     * before this runs). No memset here: it would wipe that configuration. */
    if (proc->reader.view_stride == 0u)
        return -1;

    proc->rf_channel_index    = rf_channel_index;
    proc->frequency_offset_hz = 0;
    proc->center_frequency_hz = proc->reader.view_center_hz;
    proc->samples_per_symbol  = BLE_SESSION_SAMPLES_PER_SYMBOL;

    /* Cached from the reader for the hot loop. */
    proc->input_decimation = proc->reader.view_decimation;
    proc->rssi_cal_db      = proc->reader.view_rssi_cal_db;

    proc->demodulator = cpfskdem_create(1u, 0.5f, proc->samples_per_symbol,
                                        3u, 0.5f, LIQUID_CPFSK_GMSK);
    if (!proc->demodulator)
    {
        ble_channel_processor_destroy(proc);
        return -1;
    }

    uint8_t le_ch = ble_rf_to_le_channel(rf_channel_index);
    ble_bitstream_decoder_init(&proc->decoder, le_ch);

    proc->block_start_decim_sample = 0u;
    proc->pkt_start_decim_sample   = -1L;
    proc->prev_state               = BLE_SEARCHING;
    proc->noise_floor_linear       = 0.0f;
    proc->noise_floor_initialized  = 0u;
    proc->valid_packets            = 0ul;
    proc->dbg_blocks_seen          = 0u;
    proc->active                   = 1;
    /* session + reader left as the caller set them. */
    return 0;
}

void ble_channel_processor_destroy(ble_channel_processor_t *proc)
{
    if (!proc) return;
    if (proc->demodulator) cpfskdem_destroy(proc->demodulator);
    proc->demodulator = NULL;
    sample_reader_destroy(&proc->reader);
    memset(proc, 0, sizeof(*proc));
}

static int emit_frame(ble_channel_processor_t *proc,
                      unsigned long block_start_decim_sample,
                      unsigned int end_decim_sample,
                      unsigned int decim_out,
                      unsigned long abs_block_base_radio,
                      const float complex *samples)
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

    /* The idle prefix [0, i_start_ui) before the packet seeds the per-channel
     * noise-floor estimate (tracked for diagnostics, not subtracted).  Skip
     * the demodulator group-delay region at the packet head. */
    unsigned int sig_start = i_start_ui;
    if (i_end > sig_start + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES)
        sig_start += RECEIVER_RSSI_DEMOD_DELAY_SAMPLES;

    float rssi_dbr = receiver_rssi_signal_dbr(
        samples, sig_start, i_end, i_start_ui,
        &proc->noise_floor_linear, &proc->noise_floor_initialized,
        RECEIVER_RSSI_INVALID);
    rssi_dbr += proc->rssi_cal_db;

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
        /* Hand the decoded event to the BLE collector thread; the worker
         * releases its sample block immediately after, decoupling block-pool
         * lifetime from tracking + presentation cost. */
        collector_submit(&proc->session->ble_collector, &event);

    return 0;
}

int ble_channel_processor_process_stream(ble_channel_processor_t *proc,
                                         const float complex *samples,
                                         unsigned int count,
                                         uint64_t base_radio)
{
    if (!proc || !proc->active || !samples) return -1;

    unsigned int decim_out = count;

    proc->block_start_decim_sample = base_radio / proc->input_decimation;

    int dbg = proc->session ? proc->session->config.debug : 0;
    if (dbg && proc->dbg_blocks_seen < 4u)
        fprintf(stderr,
                "[ble_proc rf=%u] block #%u: decim_out=%u\n",
                proc->rf_channel_index, proc->dbg_blocks_seen,
                decim_out);
    proc->dbg_blocks_seen++;

    unsigned long block_start_decim_sample = proc->block_start_decim_sample;

    unsigned int num_bits = decim_out / proc->samples_per_symbol;
    for (unsigned int s = 0u; s < num_bits; s++)
    {
        unsigned int sample_index = s * proc->samples_per_symbol;
        /* Liquid's demodulate omits const (it only reads); our stream is const. */
        uint32_t raw_sym_val = cpfskdem_demodulate(proc->demodulator,
                                    (float complex *)&samples[sample_index]);
        uint8_t bit = (uint8_t)(raw_sym_val & 0x1u);

        ble_status_t status = ble_bitstream_decoder_push_bit(&proc->decoder, bit);

        if (proc->prev_state == BLE_SEARCHING && status != BLE_SEARCHING)
            proc->pkt_start_decim_sample = (long)(block_start_decim_sample + sample_index);

        proc->prev_state = status;

        if (status == BLE_VALID_PACKET)
        {
            proc->valid_packets++;
            emit_frame(proc, block_start_decim_sample, sample_index,
                       decim_out, base_radio, samples);
        }
    }
    return 0;
}

void *ble_channel_worker(void *arg)
{
    ble_channel_processor_t *proc = (ble_channel_processor_t *)arg;
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

        ble_channel_processor_process_stream(proc, samples, count, base);
    }

    return NULL;
}