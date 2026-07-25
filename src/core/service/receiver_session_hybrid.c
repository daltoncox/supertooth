#include "ble_channel_processor.h"
#include "bredr_channel_processor.h"
#include "radio_common.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void *receiver_hybrid_bredr_worker(void *arg)
{
    bredr_channel_processor_t *ctx = (bredr_channel_processor_t *)arg;
    receiver_session_t *session = ctx->session;
    for (;;)
    {
        sample_block_t *block = NULL;
        if (sample_reader_wait_pop(&ctx->reader,
                                   &session->hybrid_shutdown_requested,
                                   &block) != 0)
            break;
        receiver_bredr_channel_processor_process(ctx, block);
        sample_block_release(block);
    }
    return NULL;
}

static void *receiver_hybrid_ble_worker(void *arg)
{
    ble_channel_processor_t *ble = (ble_channel_processor_t *)arg;
    receiver_session_t *session = ble->session;
    for (;;)
    {
        sample_block_t *block = NULL;
        if (sample_reader_wait_pop(&ble->reader,
                                   &session->hybrid_shutdown_requested,
                                   &block) != 0)
            break;
        receiver_ble_channel_processor_process(ble, block);
        sample_block_release(block);
    }
    return NULL;
}

static void receiver_hybrid_bredr_bridge(const bredr_event_t *event,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user)
{
    receiver_session_t *session = (receiver_session_t *)user;
    if (!session || !event)
        return;

    receiver_hybrid_callbacks_t callbacks;
    receiver_bredr_piconet_snapshot_t snapshot;
    const receiver_bredr_piconet_snapshot_t *snapshot_ptr = NULL;

    pthread_mutex_lock(&session->decoded_packet_mutex);
    session->hybrid_total_packets++;
    callbacks = session->hybrid_callbacks;
    if (piconet)
    {
        snapshot = *piconet;
        snapshot_ptr = &snapshot;
    }
    pthread_mutex_unlock(&session->decoded_packet_mutex);

    if (callbacks.on_bredr_packet)
        callbacks.on_bredr_packet(event, snapshot_ptr, callbacks.user);
}

static int receiver_hybrid_start_thread_pool(receiver_session_t *session)
{
    unsigned int bredr_count = session->bredr_config.channel_count;

    session->hybrid_shutdown_requested = 0u;
    session->hybrid_worker_count = 0u;
    session->hybrid_worker_threads =
        (pthread_t *)calloc(bredr_count + 1u, sizeof(pthread_t));
    if (!session->hybrid_worker_threads)
        return -1;

    for (unsigned int i = 0; i < bredr_count; i++)
    {
        if (pthread_create(&session->hybrid_worker_threads[i], NULL,
                           receiver_hybrid_bredr_worker, &session->bredr_ctx[i]) != 0)
            return -1;
        session->hybrid_worker_count++;
    }

    if (session->hybrid_ble_enabled)
    {
        if (pthread_create(&session->hybrid_worker_threads[bredr_count], NULL,
                           receiver_hybrid_ble_worker, session->ble_ctx) != 0)
            return -1;
        session->hybrid_worker_count++;
    }

    return 0;
}

static void receiver_hybrid_stop_thread_pool(receiver_session_t *session)
{
    session->hybrid_shutdown_requested = 1u;
    for (unsigned int i = 0; i < session->bredr_config.channel_count; i++)
        sample_reader_signal(&session->bredr_ctx[i].reader);
    if (session->hybrid_ble_enabled)
        sample_reader_signal(&session->ble_ctx->reader);
    for (unsigned int i = 0; i < session->hybrid_worker_count; i++)
        pthread_join(session->hybrid_worker_threads[i], NULL);
    free(session->hybrid_worker_threads);
    session->hybrid_worker_threads = NULL;
    session->hybrid_worker_count = 0u;
}


int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out)
{
    if (!session || !config)
        return -1;

    receiver_bredr_config_t bredr_config = {
        .channel_count = config->channel_count,
        .bottom_channel = config->bottom_channel,
        .le_grid = config->le_grid,
        .rssi_averaging_window = RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW,
        .lap_filter = 0u,
        .lap_filter_enabled = 0,
        .debug = config->debug,
    };
    receiver_bredr_callbacks_t bredr_callbacks = {
        .on_packet = receiver_hybrid_bredr_bridge,
        .user = session,
    };
    session->hybrid_config = *config;
    session->hybrid_total_packets = 0;
    session->hybrid_shutdown_requested = 0;
    session->hybrid_worker_threads = NULL;
    session->hybrid_worker_count = 0u;
    /* The BLE worker only runs when an advertising channel's center lies
     * inside the capture window (ble_channel != 0). */
    session->hybrid_ble_enabled =
        (config->ble_channel >= 37u && config->ble_channel <= 39u) ? 1u : 0u;
    if (callbacks)
        session->hybrid_callbacks = *callbacks;
    else
        memset(&session->hybrid_callbacks, 0, sizeof(session->hybrid_callbacks));
    receiver_bredr_session_init(session, &bredr_config, &bredr_callbacks);
    if (receiver_bredr_channel_processor_setup(session) != 0)
    {
        bredr_piconet_store_free(&session->bredr_store);
        receiver_bredr_channel_processor_destroy(session);
        return -1;
    }
    if (session->hybrid_ble_enabled &&
        receiver_ble_channel_processor_setup(session, RECEIVER_BLE_PIPELINE_HYBRID) != 0)
    {
        receiver_ble_channel_processor_destroy(session);
        receiver_bredr_channel_processor_destroy(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }

    if (receiver_hybrid_start_thread_pool(session) != 0)
    {
        if (session->hybrid_worker_threads)
            receiver_hybrid_stop_thread_pool(session);
        if (session->hybrid_ble_enabled)
            receiver_ble_channel_processor_destroy(session);
        receiver_bredr_channel_processor_destroy(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }

    radio_device_t *device = NULL;
    int result = radio_open(&device, config->device_type, config->device_id,
                            &session->sample_dispatcher, session->debug);
    if (result == RADIO_SUCCESS)
    {
        /* LO/sample rate come from the bredr layout (window center + span),
         * so the capture follows the configured channel window and grid. */
        radio_stream_config_t radio_config = {
            .lo_freq_hz = session->bredr_lo_freq_hz,
            .sample_rate = session->bredr_sample_rate,
            .lna_gain = RECEIVER_BREDR_LNA_GAIN,
            .vga_gain = RECEIVER_BREDR_VGA_GAIN,
        };
        result = radio_configure(device, &radio_config);
        if (result == RADIO_SUCCESS)
            result = radio_start_rx(device);
        if (result == RADIO_SUCCESS)
        {
            /* Block until receiver_session_request_stop() signals stop_cv. */
            pthread_mutex_lock(&session->stop_mutex);
            while (!session->stop_requested)
            {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_nsec += 200000000L;
                if (ts.tv_nsec >= 1000000000L)
                {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000L;
                }
                pthread_cond_timedwait(&session->stop_cv, &session->stop_mutex, &ts);
            }
            pthread_mutex_unlock(&session->stop_mutex);
            radio_stop_rx(device);
        }
        radio_close(device);
    }

    receiver_hybrid_stop_thread_pool(session);

    if (stats_out)
    {
        memset(stats_out, 0, sizeof(*stats_out));
        stats_out->total_packets = session->hybrid_total_packets;
        stats_out->bredr_channel_count = session->bredr_config.channel_count;
    }

    if (session->hybrid_ble_enabled)
        receiver_ble_channel_processor_destroy(session);
    receiver_bredr_channel_processor_destroy(session);
    return result;
}
