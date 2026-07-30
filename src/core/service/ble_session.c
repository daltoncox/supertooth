#include "ble_session.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ble_piconet.h"
#include "radio_common.h"

void ble_session_process_ble_event(ble_session_t *session, const ble_event_t *event)
{
    if (!session || !event || !session->config.packet_cb.cb) return;
    session->config.packet_cb.cb(event, session->config.packet_cb.user);
}

int ble_session_init(ble_session_t *session, const ble_session_config_t *cfg)
{
    memset(session, 0, sizeof(*session));

    if (!cfg || cfg->sample_rate_hz == 0u || !cfg->dispatcher
        || cfg->le_channel_count == 0u
        || cfg->le_channel_count > BLE_SESSION_MAX_CHANNELS)
        return -1;

    session->config            = *cfg;
    session->lo_frequency_hz   = cfg->lo_frequency_hz;
    session->sample_rate_hz    = cfg->sample_rate_hz;
    session->bottom_le_channel = cfg->bottom_le_channel;
    session->le_channel_count  = cfg->le_channel_count;
    session->dispatcher        = cfg->dispatcher;

    ble_piconet_store_init(&session->piconet_store);

    return 0;
}

static void session_signal_readers(ble_session_t *session)
{
    if (!session || !session->channels) return;
    for (size_t w = 0u; w < session->channel_count; w++)
        sample_reader_signal(&session->channels[w].reader);
}

int ble_session_run(ble_session_t *session)
{
    if (!session || session->workers_running) return -1;

    unsigned int count = session->le_channel_count;

    session->channels = calloc(count, sizeof(ble_channel_processor_t));
    if (!session->channels) return -1;

    session->channel_count = 0u;

    for (unsigned int i = 0u; i < count; i++)
    {
        uint16_t rf_channel_index = (uint16_t)(session->bottom_le_channel + i);
        if (rf_channel_index >= BLE_RF_CHANNEL_COUNT) continue;

        int32_t freq_offset_hz  = (int32_t)(ble_rf_channel_freq_hz(rf_channel_index))
                                  - (int32_t)session->lo_frequency_hz;
        uint32_t chan_center_hz = ble_rf_channel_freq_hz(rf_channel_index);

        ble_channel_processor_t *proc = &session->channels[session->channel_count];

        if (ble_channel_processor_init(proc, session->dispatcher, rf_channel_index,
                                        freq_offset_hz, chan_center_hz,
                                        session->sample_rate_hz,
                                        &session->piconet_store) != 0)
        {
            continue;
        }

        /* Set this AFTER init(): init memsets the whole struct, so assigning
         * it before init would be wiped and every worker thread would exit
         * immediately on its `!proc->session` guard. */
        proc->session = session;

        session->channel_count++;
    }

    if (session->channel_count == 0u)
    {
        ble_session_destroy(session);
        return -1;
    }

    if (session->config.debug)
    {
        fprintf(stderr,
                "[ble_session] configured %zu channel processor(s): "
                "lo=%u Hz sample_rate=%u Hz bottom_rf=%u dispatcher_readers=%u\n",
                session->channel_count, session->lo_frequency_hz,
                session->sample_rate_hz, session->bottom_le_channel,
                session->dispatcher ? session->dispatcher->reader_count : 0u);
        for (size_t w = 0u; w < session->channel_count; w++)
        {
            ble_channel_processor_t *p = &session->channels[w];
            fprintf(stderr,
                    "[ble_session]   ch[%zu] rf=%u le_ch=%u center=%u Hz "
                    "offset=%d Hz decim=%u\n",
                    w, p->rf_channel_index, ble_rf_to_le_channel(p->rf_channel_index),
                    p->center_frequency_hz, p->frequency_offset_hz,
                    p->input_decimation);
        }
    }

    session->worker_threads = calloc(session->channel_count, sizeof(pthread_t));
    if (!session->worker_threads)
    {
        ble_session_destroy(session);
        return -1;
    }

    unsigned int threads_started = 0u;
    for (size_t w = 0u; w < session->channel_count; w++)
    {
        if (pthread_create(&session->worker_threads[threads_started], NULL,
                           ble_channel_worker, &session->channels[w]) != 0)
            break;
        threads_started++;
    }

    if (threads_started == 0u)
    {
        ble_session_destroy(session);
        return -1;
    }

    session->workers_running = 1;

    int result = radio_open(&session->device, session->config.device_type,
                            session->config.device_id,
                            session->dispatcher,
                            session->config.debug);
    if (result != RADIO_SUCCESS)
    {
        ble_session_destroy(session);
        return result;
    }

    radio_stream_config_t radio_config = {
        .lo_freq_hz = session->lo_frequency_hz,
        .sample_rate = session->sample_rate_hz,
        .lna_gain = session->config.lna_gain,
        .vga_gain = session->config.vga_gain,
    };

    result = radio_configure(session->device, &radio_config);
    if (result == RADIO_SUCCESS)
        result = radio_start_rx(session->device);

    if (result != RADIO_SUCCESS)
    {
        ble_session_destroy(session);
        return result;
    }

    while (!session->shutdown_requested)
    {
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 50000000L };
        nanosleep(&ts, NULL);
    }

    ble_session_destroy(session);
    return 0;
}

void ble_session_request_stop(ble_session_t *session)
{
    if (!session) return;
    session->shutdown_requested = 1u;
    session_signal_readers(session);
}

int ble_session_destroy(ble_session_t *session)
{
    if (!session) return -1;

    if (session->workers_running)
        ble_session_request_stop(session);

    if (session->worker_threads)
    {
        for (unsigned int w = 0u; w < session->channel_count; w++)
            pthread_join(session->worker_threads[w], NULL);
        free(session->worker_threads);
        session->worker_threads = NULL;
    }

    if (session->device)
    {
        radio_stop_rx(session->device);
        radio_close(session->device);
        session->device = NULL;
    }

    if (session->channels)
    {
        for (size_t w = 0u; w < session->channel_count; w++)
            ble_channel_processor_destroy(&session->channels[w]);
        free(session->channels);
        session->channels = NULL;
    }

    session->channel_count = 0u;
    session->workers_running = 0;
    session->shutdown_requested = 0u;

    ble_piconet_store_free(&session->piconet_store);
    return 0;
}