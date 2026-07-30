#include "session.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ble_piconet.h"
#include "bredr_piconet.h"
#include "radio_common.h"

static void session_signal_readers(session_t *session)
{
    if (!session) return;
    for (size_t w = 0u; w < session->ble_channel_count; w++)
        sample_reader_signal(&session->ble_channels[w].reader);
    for (size_t w = 0u; w < session->bredr_channel_count; w++)
        sample_reader_signal(&session->bredr_channels[w].reader);
}

int session_init(session_t *session, const session_config_t *cfg)
{
    if (!session || !cfg) return -1;
    memset(session, 0, sizeof(*session));

    session->config = *cfg;

    session->torn_down = 0;
    session->stopped_cb = NULL;
    session->stopped_user = NULL;
    atomic_store_explicit(&session->shutdown_requested, 0u, memory_order_release);

    session->dispatcher = (sample_dispatcher_t *)calloc(1, sizeof(*session->dispatcher));
    if (!session->dispatcher) return -1;
    if (sample_dispatcher_init(session->dispatcher) != 0)
    {
        free(session->dispatcher);
        session->dispatcher = NULL;
        return -1;
    }

    ble_piconet_store_init(&session->ble_piconet_store);
    bredr_piconet_store_init(&session->bredr_piconet_store);
    bredr_piconet_store_set_rssi_averaging(&session->bredr_piconet_store,
                                           BREDR_SESSION_DEFAULT_RSSI_AVERAGING_WINDOW);
    pthread_mutex_init(&session->bredr_mutex, NULL);

    return 0;
}

void session_enable_ble(session_t *session,
                        const session_ble_config_t *cfg,
                        session_ble_packet_fn cb, void *user)
{
    if (!session) return;
    if (cfg) session->ble_cfg = *cfg;
    session->ble_cb   = cb;
    session->ble_user = user;
    session->ble_enabled = 1;
}

void session_enable_bredr(session_t *session,
                           const session_bredr_config_t *cfg,
                           session_bredr_packet_fn cb, void *user)
{
    if (!session) return;
    if (cfg) session->bredr_cfg = *cfg;
    session->bredr_cb   = cb;
    session->bredr_user = user;
    session->bredr_enabled = 1;
}

void session_set_stopped_callback(session_t *session,
                                  void (*cb)(void *user), void *user)
{
    if (!session) return;
    session->stopped_cb   = cb;
    session->stopped_user = user;
}

int session_tune(session_t *session,
                 session_protocol_ref_t ref,
                 unsigned int bottom_channel,
                 unsigned int channel_count)
{
    if (!session || channel_count == 0u) return -1;

    double lo_mhz, bredr_span_mhz, ble_span_mhz;
    unsigned int bredr_rate_mhz;

    if (ref == SESSION_REF_BLE)
    {
        if (channel_count > BLE_RF_CHANNEL_COUNT ||
            bottom_channel + channel_count > BLE_RF_CHANNEL_COUNT)
            return -1;
        lo_mhz        = 2401.0 + 2.0 * (double)bottom_channel + (double)channel_count;
        /* BLE channels are 2 MHz apart, so an N-channel BLE window needs a
         * 2*N MHz span; the LO is already a whole-MHz frequency. */
        ble_span_mhz  = 2.0 * (double)channel_count;
        bredr_span_mhz = ble_span_mhz;
        bredr_rate_mhz = (unsigned int)bredr_span_mhz;
        if (bredr_rate_mhz < 4u) bredr_rate_mhz = 4u;
    }
    else
    {
        if (channel_count > BREDR_SESSION_MAX_CHANNELS ||
            bottom_channel + channel_count > BREDR_SESSION_MAX_CHANNELS)
            return -1;
        lo_mhz         = 2402.0 + (double)bottom_channel + ((double)channel_count - 1.0) / 2.0;
        bredr_span_mhz = (double)channel_count;
        bredr_rate_mhz = (unsigned int)bredr_span_mhz;
        if (bredr_rate_mhz < 4u) bredr_rate_mhz = 4u;
        ble_span_mhz   = bredr_span_mhz;
    }

    session->tune_ref        = ref;
    session->tune_bottom     = bottom_channel;
    session->tune_count      = channel_count;
    session->lo_frequency_hz = (uint32_t)((uint64_t)(lo_mhz * 1e6));
    session->sample_rate_hz  = bredr_rate_mhz * 1000000u;
    session->decimation      = session->sample_rate_hz / 2000000u;
    if (session->decimation < 1u) session->decimation = 1u;

    (void)ble_span_mhz;

    /* Reject capture windows the radio cannot sustain (e.g. >20 BR/EDR channels
     * at 1 MHz each would request >20 Msps, beyond the HackRF ceiling). */
    if (session->sample_rate_hz > RADIO_MAX_SAMPLE_RATE_HZ)
        return -1;

    return 0;
}

static int session_create_channels(session_t *session)
{
    int debug = session->config.debug;
    size_t total = 0u;

    if (session->ble_enabled)
    {
        session->ble_channel_count = 0u;
        session->ble_channels = calloc(BLE_RF_CHANNEL_COUNT, sizeof(ble_channel_processor_t));
        if (!session->ble_channels) return -1;

        for (unsigned int rf = 0u; rf < BLE_RF_CHANNEL_COUNT; rf++)
        {
            uint32_t center = ble_rf_channel_freq_hz(rf);
            int32_t offset  = (int32_t)center - (int32_t)session->lo_frequency_hz;
            /* BLE processor is only meaningful inside the capture span. */
            if (labs((long)offset) > (int32_t)(session->sample_rate_hz / 2u))
                continue;

            ble_channel_processor_t *proc = &session->ble_channels[session->ble_channel_count];
            if (ble_channel_processor_init(proc, session->dispatcher, rf, offset, center,
                                           session->sample_rate_hz,
                                           &session->ble_piconet_store) != 0)
                continue;
            proc->session = session;
            session->ble_channel_count++;
        }
    }

    if (session->bredr_enabled)
    {
        session->bredr_channel_count = 0u;
        session->bredr_channels = calloc(BREDR_SESSION_MAX_CHANNELS, sizeof(bredr_channel_processor_t));
        if (!session->bredr_channels) return -1;

        for (unsigned int c = 0u; c < BREDR_SESSION_MAX_CHANNELS; c++)
        {
            uint32_t center = (uint32_t)(2402000000ull + (uint64_t)c * 1000000ull);
            int32_t offset  = (int32_t)center - (int32_t)session->lo_frequency_hz;
            if (labs((long)offset) > (int32_t)(session->sample_rate_hz / 2u))
                continue;

            bredr_channel_processor_t *proc = &session->bredr_channels[session->bredr_channel_count];
            if (bredr_channel_processor_init(proc, session->dispatcher, (uint16_t)c, offset, center,
                                            session->sample_rate_hz) != 0)
                continue;
            proc->session = session;
            session->bredr_channel_count++;
        }
    }

    total = session->ble_channel_count + session->bredr_channel_count;
    if (debug)
    {
        fprintf(stderr,
                "[session] lo=%u Hz rate=%u Hz decim=%u : %zu BLE + %zu BR/EDR processors\n",
                session->lo_frequency_hz, session->sample_rate_hz, session->decimation,
                session->ble_channel_count, session->bredr_channel_count);
    }
    if (total == 0u) return -1;
    return 0;
}

static void *session_ble_worker_shim(void *arg)
{
    return ble_channel_worker(arg);
}

static void *session_bredr_worker_shim(void *arg)
{
    return bredr_channel_worker(arg);
}

int session_run(session_t *session)
{
    if (!session || session->workers_running) return -1;

    if (session_create_channels(session) != 0)
    {
        session_destroy(session);
        return -1;
    }

    size_t total = session->ble_channel_count + session->bredr_channel_count;
    session->worker_threads = calloc(total, sizeof(pthread_t));
    if (!session->worker_threads)
    {
        session_destroy(session);
        return -1;
    }

    size_t started = 0u;
    for (size_t w = 0u; w < session->ble_channel_count; w++)
    {
        if (pthread_create(&session->worker_threads[started], NULL,
                           session_ble_worker_shim, &session->ble_channels[w]) != 0)
            break;
        started++;
    }
    for (size_t w = 0u; w < session->bredr_channel_count; w++)
    {
        if (pthread_create(&session->worker_threads[started], NULL,
                           session_bredr_worker_shim, &session->bredr_channels[w]) != 0)
            break;
        started++;
    }

    if (started == 0u)
    {
        session_destroy(session);
        return -1;
    }
    session->worker_count   = started;
    session->workers_running = 1;

    int result = radio_open(&session->device, session->config.device_type,
                            session->config.device_id,
                            session->dispatcher,
                            session->config.debug);
    if (result != RADIO_SUCCESS)
    {
        session_destroy(session);
        return result;
    }

    uint32_t lna  = session->bredr_enabled ? SESSION_BREDR_LNA_GAIN : SESSION_BLE_LNA_GAIN;
    uint32_t vga  = session->bredr_enabled ? SESSION_BREDR_VGA_GAIN : SESSION_BLE_VGA_GAIN;

    radio_stream_config_t radio_config = {
        .lo_freq_hz  = session->lo_frequency_hz,
        .sample_rate = session->sample_rate_hz,
        .lna_gain    = lna,
        .vga_gain    = vga,
    };

    result = radio_configure(session->device, &radio_config);
    if (result == RADIO_SUCCESS)
        result = radio_start_rx(session->device);

    if (result != RADIO_SUCCESS)
    {
        session_destroy(session);
        return result;
    }

    while (atomic_load_explicit(&session->shutdown_requested, memory_order_acquire) == 0u)
    {
        /* Relative 50ms poll. NOTE: nanosleep's request is a *duration*, not an
         * absolute time, so we must not feed it a CLOCK_REALTIME timestamp. */
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 50000000L };
        nanosleep(&ts, NULL);
    }
    /* The capture loop has ended; notify the owner (UI) now, before the
     * potentially blocking radio teardown below, so the UI can flip to the
     * stopped state immediately instead of waiting on device shutdown. */
    if (session->stopped_cb)
        session->stopped_cb(session->stopped_user);

    session_destroy(session);
    return 0;
}

void session_request_stop(session_t *session)
{
    if (!session) return;
    atomic_store_explicit(&session->shutdown_requested, 1u, memory_order_release);
    session_signal_readers(session);
}

int session_destroy(session_t *session)
{
    if (!session) return -1;

    /* Idempotent: session_run() tears the session down on its worker thread,
     * and the owner may also call this (e.g. on destruction). Guard against a
     * double free / use-after-free. */
    if (session->torn_down)
        return 0;

    session->torn_down = 1;

    if (session->workers_running)
        session_request_stop(session);

    if (session->worker_threads)
    {
        for (size_t w = 0u; w < session->worker_count; w++)
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

    if (session->ble_channels)
    {
        for (size_t w = 0u; w < session->ble_channel_count; w++)
            ble_channel_processor_destroy(&session->ble_channels[w]);
        free(session->ble_channels);
        session->ble_channels = NULL;
    }
    session->ble_channel_count = 0u;

    if (session->bredr_channels)
    {
        for (size_t w = 0u; w < session->bredr_channel_count; w++)
            bredr_channel_processor_destroy(&session->bredr_channels[w]);
        free(session->bredr_channels);
        session->bredr_channels = NULL;
    }
    session->bredr_channel_count = 0u;

    session->worker_count    = 0u;
    session->workers_running = 0;
    atomic_store_explicit(&session->shutdown_requested, 0u, memory_order_release);

    ble_piconet_store_free(&session->ble_piconet_store);
    bredr_piconet_store_free(&session->bredr_piconet_store);
    pthread_mutex_destroy(&session->bredr_mutex);
    sample_dispatcher_destroy(session->dispatcher);
    return 0;
}

void session_process_ble_event(session_t *session, const ble_event_t *event)
{
    if (!session || !event || !session->ble_cb) return;
    session->ble_cb(event, session->ble_user);
}

void session_process_bredr_event(session_t *session, const bredr_event_t *event)
{
    if (!session || !event) return;

    pthread_mutex_lock(&session->bredr_mutex);
    int packet_is_newest = 0;
    bredr_piconet_t *pnet = bredr_piconet_store_add_packet(&session->bredr_piconet_store,
                                                          event, &packet_is_newest);
    bredr_piconet_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    const bredr_piconet_snapshot_t *snapshot_ptr = NULL;
    if (pnet)
    {
        snapshot.lap            = pnet->lap;
        snapshot.uap_found      = pnet->uap_found;
        snapshot.uap            = pnet->uap;
        snapshot.clk_known      = pnet->clk_known;
        snapshot.central_clk_1_6        = pnet->central_clk_1_6;
        snapshot.last_successful_rx_clk_1600 = pnet->last_successful_rx_clk_1600;
        snapshot.tracking_state = pnet->tracking_state;
        snapshot.total_packets  = pnet->total_packets;
        snapshot.combined_rssi_seen = pnet->combined_rssi_seen;
        snapshot.combined_rssi      = pnet->combined_rssi;
        snapshot.master_rssi_seen    = pnet->master_rssi_seen;
        snapshot.master_rssi         = pnet->master_rssi;
        memcpy(snapshot.slave_rssi_seen, pnet->slave_rssi_seen, sizeof(snapshot.slave_rssi_seen));
        memcpy(snapshot.slave_rssi, pnet->slave_rssi, sizeof(snapshot.slave_rssi));
        if (!packet_is_newest)
            snapshot.clk_known = 0;
        snapshot_ptr = &snapshot;
    }
    pthread_mutex_unlock(&session->bredr_mutex);

    if (session->bredr_cb)
        session->bredr_cb(event, snapshot_ptr, session->bredr_user);
}

size_t session_bredr_piconet_count(const session_t *session)
{
    if (!session) return 0u;
    return bredr_piconet_store_count(&session->bredr_piconet_store);
}

int session_bredr_piconet_snapshot(const session_t *session,
                                  size_t index,
                                  bredr_piconet_snapshot_t *out)
{
    if (!session || !out) return -1;
    const bredr_piconet_t *pnet = bredr_piconet_store_get(&session->bredr_piconet_store, index);
    if (!pnet) return -1;
    memset(out, 0, sizeof(*out));
    out->lap            = pnet->lap;
    out->uap_found      = pnet->uap_found;
    out->uap            = pnet->uap;
    out->clk_known      = pnet->clk_known;
    out->central_clk_1_6        = pnet->central_clk_1_6;
    out->last_successful_rx_clk_1600 = pnet->last_successful_rx_clk_1600;
    out->tracking_state = pnet->tracking_state;
    out->total_packets  = pnet->total_packets;
    out->combined_rssi_seen = pnet->combined_rssi_seen;
    out->combined_rssi      = pnet->combined_rssi;
    out->master_rssi_seen    = pnet->master_rssi_seen;
    out->master_rssi         = pnet->master_rssi;
    memcpy(out->slave_rssi_seen, pnet->slave_rssi_seen, sizeof(out->slave_rssi_seen));
    memcpy(out->slave_rssi, pnet->slave_rssi, sizeof(out->slave_rssi));
    return 0;
}

unsigned long session_dropped_blocks(const session_t *session)
{
    if (!session) return 0ul;
    return session->dispatcher->dropped_blocks;
}

int session_create_channels_for_test(session_t *session,
                                     size_t *ble_count,
                                     size_t *bredr_count)
{
    if (!session) return -1;
    if (session_create_channels(session) != 0)
        return -1;
    if (ble_count) *ble_count = session->ble_channel_count;
    if (bredr_count) *bredr_count = session->bredr_channel_count;
    return 0;
}
