#include "ble_channel_processor.h"
#include "rssi_measurements.h"

#include <math.h>
#include <string.h>

static void receiver_ble_emit_event(ble_channel_processor_t *ble,
                                    const ble_event_t *event)
{
    receiver_session_t *session = ble->session;

    if (ble->pipeline == RECEIVER_BLE_PIPELINE_HYBRID)
    {
        receiver_hybrid_callbacks_t callbacks;

        pthread_mutex_lock(&session->decoded_packet_mutex);
        session->hybrid_total_packets++;
        callbacks = session->hybrid_callbacks;
        pthread_mutex_unlock(&session->decoded_packet_mutex);

        if (callbacks.on_ble_packet)
            callbacks.on_ble_packet(event, callbacks.user);
        return;
    }

    if (session->ble_callbacks.on_packet)
        session->ble_callbacks.on_packet(event, session->ble_callbacks.user);
}

void receiver_ble_update_layout(receiver_session_t *session)
{
    unsigned int count = session->ble_config.le_channel_count;
    /* The capture span is the LE window width: 2 MHz per channel. */
    unsigned int span_mhz = 2u * count;
    session->ble_sample_rate = span_mhz * 1000000u;
    if (span_mhz == 2u)
        session->ble_sample_rate = 4000000u;
    session->ble_decim_factor = session->ble_sample_rate / 2000000u;
    /* LO = window center = 2401 + 2*bottom + count MHz (a whole MHz). LE
     * channels then sit at odd-MHz offsets +/-1, +/-3, ... around it. */
    session->ble_lo_freq_hz =
        (uint64_t)(2401u + 2u * session->ble_config.bottom_le_channel + count) *
        1000000ULL;
}

/* Full DSP setup for one channel: NCO mix from offset_hz down to baseband,
 * decimate to 2 Msps, demodulate, decode. */
static int receiver_ble_setup_channel_dsp(ble_channel_processor_t *ble,
                                          double offset_hz,
                                          unsigned int input_sample_rate,
                                          unsigned int decimation)
{
    /* The channel must be fully inside the capture span: |offset| + half
     * the channel width <= Nyquist. */
    double nyquist_hz = (double)input_sample_rate / 2.0;
    if (fabs(offset_hz) + 500000.0 > nyquist_hz)
        return -1;

    ble->nco = nco_crcf_create(LIQUID_NCO);
    ble->firdec = firdecim_crcf_create_kaiser(decimation, 7, 60.0f);
    if (!ble->nco || !ble->firdec)
        return -1;
    ble->input_decimation = decimation;
    ble->sample_scale = decimation;
    ble->input_sample_rate_hz = input_sample_rate;
    ble->active = 1u;
    nco_crcf_set_frequency(ble->nco,
                           2.0f * (float)M_PI * (float)offset_hz /
                               (float)input_sample_rate);

    ble->demod = cpfskdem_create(1u, 0.5f, RECEIVER_BLE_SAMPLES_PER_SYMBOL, 3u, 0.5f,
                                 LIQUID_CPFSK_GMSK);
    if (!ble->demod)
        return -1;

    ble_bitstream_decoder_init(&ble->proc, (uint8_t)ble->channel_index);
    ble->prev_status = BLE_SEARCHING;
    ble->pkt_start_sample = -1;
    if (sample_reader_init(&ble->reader, &ble->session->sample_dispatcher) != 0)
        return -1;
    ble->reader_initialized = 1u;

    return 0;
}

static int receiver_ble_setup_hybrid_channel(receiver_session_t *session)
{
    ble_channel_processor_t *ble = &session->ble_ctx[0];
    memset(ble, 0, sizeof(*ble));
    ble->session = session;
    ble->pipeline = RECEIVER_BLE_PIPELINE_HYBRID;
    session->ble_ctx_count = 1u;

    /* Derive everything from the bredr layout (LO + sample rate) and the
     * configured advertising channel instead of fixed constants, so the
     * worker follows the movable capture window. */
    uint8_t ble_channel = session->hybrid_config.ble_channel;
    uint32_t adv_freq_hz = BLE_CH37_FREQ_HZ;
    if (ble_channel == BLE_CH38_INDEX)
        adv_freq_hz = BLE_CH38_FREQ_HZ;
    else if (ble_channel == BLE_CH39_INDEX)
        adv_freq_hz = BLE_CH39_FREQ_HZ;

    unsigned int decimation = session->bredr_decim_factor;
    if (decimation == 0u)
        return -1;

    double offset_hz = (double)adv_freq_hz - (double)session->bredr_lo_freq_hz;
    ble->center_frequency_hz = adv_freq_hz;
    ble->channel_index = (uint16_t)ble_channel;
    return receiver_ble_setup_channel_dsp(ble, offset_hz,
                                          session->bredr_sample_rate,
                                          decimation);
}

static int receiver_ble_setup_session_channels(receiver_session_t *session)
{
    unsigned int count = session->ble_config.le_channel_count;
    unsigned int bottom = session->ble_config.bottom_le_channel;
    if (count < 1u || count > RECEIVER_BLE_MAX_CHANNELS)
        return -1;
    if (bottom >= BLE_RF_CHANNEL_COUNT || bottom + count > BLE_RF_CHANNEL_COUNT)
        return -1;

    receiver_ble_update_layout(session);

    session->ble_ctx_count = 0u;
    for (unsigned int i = 0; i < count; i++)
    {
        unsigned int rf = bottom + i;
        ble_channel_processor_t *ble = &session->ble_ctx[i];
        memset(ble, 0, sizeof(*ble));
        ble->session = session;
        ble->pipeline = RECEIVER_BLE_PIPELINE_SESSION;
        ble->channel_index = ble_channel_number_for_rf(rf);
        ble->center_frequency_hz = ble_rf_channel_freq_hz(rf);

        if (!ble_rf_is_advertising(rf))
        {
            /* Data channel: no DSP work for now. It still gets a reader so
             * its worker can drain and release blocks. */
            ble->active = 0u;
            if (sample_reader_init(&ble->reader, &session->sample_dispatcher) != 0)
                return -1;
            ble->reader_initialized = 1u;
            session->ble_ctx_count++;
            continue;
        }

        double offset_hz =
            (double)ble->center_frequency_hz - (double)session->ble_lo_freq_hz;
        if (receiver_ble_setup_channel_dsp(ble, offset_hz,
                                           session->ble_sample_rate,
                                           session->ble_decim_factor) != 0)
            return -1;
        session->ble_ctx_count++;
    }

    return 0;
}

int receiver_ble_channel_processor_setup(receiver_session_t *session,
                                         receiver_ble_pipeline_t pipeline)
{
    if (!session || !session->ble_ctx)
        return -1;

    if (pipeline == RECEIVER_BLE_PIPELINE_HYBRID)
        return receiver_ble_setup_hybrid_channel(session);

    return receiver_ble_setup_session_channels(session);
}

void receiver_ble_channel_processor_destroy(receiver_session_t *session)
{
    if (!session || !session->ble_ctx)
        return;

    for (unsigned int i = 0; i < session->ble_ctx_count; i++)
    {
        ble_channel_processor_t *ble = &session->ble_ctx[i];
        if (ble->demod)
        {
            cpfskdem_destroy(ble->demod);
            ble->demod = NULL;
        }
        if (ble->firdec)
        {
            firdecim_crcf_destroy(ble->firdec);
            ble->firdec = NULL;
        }
        if (ble->nco)
        {
            nco_crcf_destroy(ble->nco);
            ble->nco = NULL;
        }
        if (ble->reader_initialized)
            sample_reader_destroy(&ble->reader);
    }
    memset(session->ble_ctx, 0,
           RECEIVER_BLE_MAX_CHANNELS * sizeof(*session->ble_ctx));
    session->ble_ctx_count = 0u;
}

void receiver_ble_channel_processor_process(ble_channel_processor_t *ble,
                                            sample_block_t *blk)
{
    /* Data channels do no work: their worker just drains and releases. */
    if (!ble->active)
        return;

    float complex *samples = blk->samples;
    unsigned int sample_count = blk->num_samples;
    unsigned long long buf_start = blk->block_base_sample;

    /* SESSION and HYBRID are both wideband channelized pipelines: mix the
     * channel to baseband and decimate to 2 Msps. */
    nco_crcf_mix_block_down(ble->nco, blk->samples, ble->mixed, blk->num_samples);
    sample_count = blk->num_samples / ble->input_decimation;
    firdecim_crcf_execute_block(ble->firdec, ble->mixed, sample_count, ble->decimated);
    samples = ble->decimated;
    buf_start /= ble->input_decimation;

    unsigned int num_bits = sample_count / RECEIVER_BLE_SAMPLES_PER_SYMBOL;
    for (unsigned int s = 0; s < num_bits; s++)
    {
        unsigned int sample_index = s * RECEIVER_BLE_SAMPLES_PER_SYMBOL;
        unsigned int sym = cpfskdem_demodulate(ble->demod, &samples[sample_index]);
        uint8_t bit = (uint8_t)(sym & 0x01u);
        ble_status_t status = ble_bitstream_decoder_push_bit(&ble->proc, bit);

        if (ble->prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            ble->pkt_start_sample = (long long)(buf_start + sample_index);
        ble->prev_status = status;

        if (status != BLE_VALID_PACKET)
            continue;

        unsigned int i_start = 0u;
        unsigned int i_end = 0u;
        if (ble->pkt_start_sample >= 0)
        {
            long long rel_start = ble->pkt_start_sample - (long long)buf_start;
            i_start = (rel_start < 0) ? 0u : (unsigned int)rel_start;
            i_end = (s + 1u) * RECEIVER_BLE_SAMPLES_PER_SYMBOL;
            if (i_end > sample_count)
                i_end = sample_count;
        }

        ble_frame_t frame;
        if (ble_bitstream_decoder_get_frame(&ble->proc, &frame) != 0)
            continue;

        float rssi_dbr =
            receiver_rssi_from_mean_power_range(samples, i_start, i_end,
                                                RECEIVER_RSSI_INVALID);
        unsigned long long abs_sample_index =
            (buf_start + sample_index) * (unsigned long long)ble->sample_scale;
        rx_metadata_t meta =
            receiver_make_metadata(abs_sample_index,
                                   ble->input_sample_rate_hz,
                                   ble->center_frequency_hz,
                                   ble->channel_index,
                                   rssi_dbr);
        ble_event_t event = {
            .meta = meta,
            .frame = frame,
        };
        receiver_ble_emit_event(ble, &event);
    }
}
