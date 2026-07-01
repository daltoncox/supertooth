#include "ble_channel_processor.h"
#include "radio_common.h"

#include <string.h>
#include <time.h>

static void *receiver_ble_worker(void *arg)
{
    ble_channel_processor_t *ble = (ble_channel_processor_t *)arg;
    receiver_session_t *session = ble->session;
    for (;;)
    {
        sample_block_t *block = NULL;
        if (sample_reader_wait_pop(&ble->reader,
                                   &session->ble_shutdown_requested,
                                   &block) != 0)
            break;

        receiver_ble_channel_processor_process(ble, block);
        sample_block_release(block);
    }
    return NULL;
}

static int receiver_ble_start_worker(receiver_session_t *session)
{
    session->ble_shutdown_requested = 0u;
    session->ble_worker_running = 0u;
    if (pthread_create(&session->ble_worker_thread, NULL, receiver_ble_worker,
                       session->ble_ctx) != 0)
        return -1;
    session->ble_worker_running = 1u;
    return 0;
}

static void receiver_ble_stop_worker(receiver_session_t *session)
{
    session->ble_shutdown_requested = 1u;
    if (session->ble_ctx)
        sample_reader_signal(&session->ble_ctx->reader);
    if (session->ble_worker_running)
    {
        pthread_join(session->ble_worker_thread, NULL);
        session->ble_worker_running = 0u;
    }
}

int receiver_session_run_ble(receiver_session_t *session,
                             const receiver_ble_config_t *config,
                             const receiver_ble_callbacks_t *callbacks)
{
    if (!session || !config)
        return -1;

    session->stop_requested = 0;
    session->debug = config->debug;
    session->ble_config = *config;
    session->ble_worker_running = 0u;
    session->ble_shutdown_requested = 0u;
    sample_dispatcher_reset(&session->sample_dispatcher);
    if (callbacks)
        session->ble_callbacks = *callbacks;
    else
        memset(&session->ble_callbacks, 0, sizeof(session->ble_callbacks));

    if (receiver_ble_channel_processor_setup(session, RECEIVER_BLE_PIPELINE_DIRECT) != 0)
        return -1;
    if (receiver_ble_start_worker(session) != 0)
    {
        receiver_ble_channel_processor_destroy(session);
        return -1;
    }

    radio_device_t *device = NULL;
    int result = radio_open(&device, RADIO_DEVICE_HACKRF, NULL,
                            &session->sample_dispatcher, session->debug);
    if (result != RADIO_SUCCESS)
    {
        receiver_ble_stop_worker(session);
        receiver_ble_channel_processor_destroy(session);
        return result;
    }

    radio_stream_config_t radio_config = {
        .lo_freq_hz = config->lo_freq_hz,
        .sample_rate = RECEIVER_BLE_SAMPLE_RATE,
        .lna_gain = RECEIVER_BLE_LNA_GAIN,
        .vga_gain = RECEIVER_BLE_VGA_GAIN,
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
    receiver_ble_stop_worker(session);

    receiver_ble_channel_processor_destroy(session);

    return result;
}
