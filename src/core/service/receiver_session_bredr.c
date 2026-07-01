#include "bredr_channel_processor.h"
#include "radio_common.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void *receiver_bredr_worker(void *arg)
{
    bredr_channel_processor_t *ctx = (bredr_channel_processor_t *)arg;
    receiver_session_t *session = ctx->session;
    for (;;)
    {
        sample_block_t *block = NULL;
        if (sample_reader_wait_pop(&ctx->reader,
                                   &session->bredr_shutdown_requested,
                                   &block) != 0)
            break;

        receiver_bredr_channel_processor_process(ctx, block);
        sample_block_release(block);
    }
    return NULL;
}

static int receiver_bredr_init_thread_pool(receiver_session_t *session)
{
    session->bredr_shutdown_requested = 0u;
    session->bredr_worker_count = 0u;
    session->bredr_worker_threads =
        (pthread_t *)calloc(session->bredr_config.channel_count, sizeof(pthread_t));
    if (!session->bredr_worker_threads)
        return -1;

    for (unsigned int i = 0; i < session->bredr_config.channel_count; i++)
    {
        if (pthread_create(&session->bredr_worker_threads[i], NULL, receiver_bredr_worker,
                           &session->bredr_ctx[i]) != 0)
            return -1;
        session->bredr_worker_count++;
    }
    return 0;
}

static void receiver_bredr_stop_thread_pool(receiver_session_t *session)
{
    session->bredr_shutdown_requested = 1u;
    for (unsigned int i = 0; i < session->bredr_worker_count; i++)
        sample_reader_signal(&session->bredr_ctx[i].reader);
    for (unsigned int i = 0; i < session->bredr_worker_count; i++)
        pthread_join(session->bredr_worker_threads[i], NULL);
    free(session->bredr_worker_threads);
    session->bredr_worker_threads = NULL;
    session->bredr_worker_count = 0u;
}


int receiver_session_run_bredr(receiver_session_t *session,
                               const receiver_bredr_config_t *config,
                               const receiver_bredr_callbacks_t *callbacks,
                               receiver_bredr_stats_t *stats_out)
{
    if (!session || !config)
        return -1;

    receiver_bredr_session_init(session, config, callbacks);

    if (receiver_bredr_channel_processor_setup(session) != 0)
    {
        bredr_piconet_store_free(&session->bredr_store);
        receiver_bredr_channel_processor_destroy(session);
        return -1;
    }
    if (receiver_bredr_init_thread_pool(session) != 0)
    {
        if (session->bredr_worker_threads)
            receiver_bredr_stop_thread_pool(session);
        receiver_bredr_channel_processor_destroy(session);
        bredr_piconet_store_free(&session->bredr_store);
        return -1;
    }

    radio_device_t *device = NULL;
    int result = radio_open(&device, config->device_type, config->device_id,
                            &session->sample_dispatcher, session->debug);
    if (result == RADIO_SUCCESS)
    {
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

    receiver_bredr_stop_thread_pool(session);
    receiver_bredr_channel_processor_destroy(session);

    if (stats_out)
    {
        memset(stats_out, 0, sizeof(*stats_out));
        stats_out->total_bits = session->bredr_total_bits;
        stats_out->total_packets = session->bredr_total_packets;
        stats_out->header_packets = session->bredr_header_packets;
        stats_out->id_packets = session->bredr_id_packets;
        stats_out->channel_count = session->bredr_config.channel_count;
    }

    return result;
}
