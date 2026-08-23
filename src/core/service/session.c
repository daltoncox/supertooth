#include "session.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ble_piconet.h"
#include "bredr_piconet.h"
#include "ble_codec.h"
#include "bt_assigned_numbers.h"
#include "channelizer_bank.h"
#include "radio_common.h"

static void session_signal_readers(session_t *session)
{
    if (!session) return;
    for (size_t w = 0u; w < session->ble_channel_count; w++)
        sample_reader_signal(&session->ble_channels[w].reader);
    for (size_t w = 0u; w < session->bredr_channel_count; w++)
        sample_reader_signal(&session->bredr_channels[w].reader);
    if (session->ble_channelizer_running)
        sample_reader_signal(&session->ble_channelizer.rf_reader);
    if (session->bredr_channelizer_running)
        sample_reader_signal(&session->bredr_channelizer.rf_reader);
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

    session->bredr_chan_dispatcher = (sample_dispatcher_t *)calloc(1, sizeof(*session->bredr_chan_dispatcher));
    if (!session->bredr_chan_dispatcher) return -1;
    if (sample_dispatcher_init(session->bredr_chan_dispatcher) != 0)
    {
        free(session->bredr_chan_dispatcher);
        session->bredr_chan_dispatcher = NULL;
        return -1;
    }
    memset(&session->bredr_channelizer, 0, sizeof(session->bredr_channelizer));
    session->bredr_channelizer_running = 0;

    session->ble_chan_dispatcher = (sample_dispatcher_t *)calloc(1, sizeof(*session->ble_chan_dispatcher));
    if (!session->ble_chan_dispatcher) return -1;
    if (sample_dispatcher_init(session->ble_chan_dispatcher) != 0)
    {
        free(session->ble_chan_dispatcher);
        session->ble_chan_dispatcher = NULL;
        return -1;
    }
    memset(&session->ble_channelizer, 0, sizeof(session->ble_channelizer));
    session->ble_channelizer_running = 0;

    ble_tracker_init(&session->ble_tracker);
    bredr_tracker_init(&session->bredr_tracker);
    pthread_mutex_init(&session->bredr_mutex, NULL);

    return 0;
}

void session_enable_ble(session_t *session,
                        const session_ble_config_t *cfg,
                        session_ble_packet_fn cb, void *user)
{
    if (!session) return;
    if (cfg) session->ble_cfg = *cfg;
    ble_tracker_set_enforce_crc(&session->ble_tracker,
                                cfg ? (int)cfg->enforce_crc : 0);
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

        /* Prefer the 2 MHz BLE raster (one bin per BLE channel => efficient),
         * but firpfbch2 needs an even bin count, so fall back to the 1 MHz
         * raster when 2 MHz would yield an odd M (e.g. a 10 MHz window). */
        uint32_t ble_grid = CHANNELIZER_BANK_GRID_BLE_HZ;
        if (channelizer_bank_bins_for_rate(session->sample_rate_hz, ble_grid) == 0u)
            ble_grid = CHANNELIZER_BANK_GRID_BR_EDR_HZ;
        unsigned int ble_frame_stride = ble_grid / 1000000u;

        if (channelizer_init(&session->ble_channelizer,
                              session->dispatcher,
                              session->ble_chan_dispatcher,
                              session->sample_rate_hz,
                              session->lo_frequency_hz,
                              ble_grid,
                              debug) != 0)
        {
            if (debug)
                fprintf(stderr, "[session] BLE channelizer init failed\n");
            return -1;
        }

        for (unsigned int rf = 0u; rf < BLE_RF_CHANNEL_COUNT; rf++)
        {
            uint32_t center = ble_rf_channel_freq_hz(rf);
            int32_t offset  = (int32_t)center - (int32_t)session->lo_frequency_hz;
            if (labs((long)offset) >= (int32_t)(session->sample_rate_hz / 2u))
                continue;

            ble_channel_processor_t *proc = &session->ble_channels[session->ble_channel_count];
            int bin = (ble_grid == CHANNELIZER_BANK_GRID_BLE_HZ)
                ? channelizer_bank_bin_for_center_ble(
                      session->ble_channelizer.bank.M,
                      session->ble_channelizer.bank.lo_eff_hz, center)
                : (int)channelizer_bank_bin_for_center(
                      session->ble_channelizer.bank.M,
                      session->ble_channelizer.bank.lo_eff_hz,
                      center, CHANNELIZER_BANK_GRID_BR_EDR_HZ);
            if (bin < 0)
                continue;

            /* The decoder CRC-gates data candidates against the tracker's
             * piconet store (shared per-session; the store serializes its
             * own access). */
            int ok = ble_channel_processor_init(
                proc, session->ble_chan_dispatcher, rf, center,
                session->sample_rate_hz, (unsigned int)bin,
                session->ble_channelizer.bank.M,
                session->ble_channelizer.bank.M2,
                ble_frame_stride,
                CHANNELIZER_BANK_RSSI_CAL_DB,
                &session->ble_tracker.conn_store);

            if (ok != 0)
                continue;
            proc->session = session;
            session->ble_channel_count++;
        }

        if (session->ble_channel_count == 0u)
        {
            channelizer_destroy(&session->ble_channelizer);
            return -1;
        }
        session->ble_channelizer.active = 1;
    }

    if (session->bredr_enabled)
    {
        session->bredr_channel_count = 0u;
        session->bredr_channels = calloc(BREDR_SESSION_MAX_CHANNELS, sizeof(bredr_channel_processor_t));
        if (!session->bredr_channels) return -1;

        if (channelizer_init(&session->bredr_channelizer,
                              session->dispatcher,
                              session->bredr_chan_dispatcher,
                              session->sample_rate_hz,
                              session->lo_frequency_hz,
                              CHANNELIZER_BANK_GRID_BR_EDR_HZ,
                              debug) != 0)
        {
            if (debug)
                fprintf(stderr, "[session] channelizer init failed\n");
            return -1;
        }

        for (unsigned int c = 0u; c < BREDR_SESSION_MAX_CHANNELS; c++)
        {
            uint32_t center = (uint32_t)(2402000000ull + (uint64_t)c * 1000000ull);
            int32_t offset  = (int32_t)center - (int32_t)session->lo_frequency_hz;
            if (labs((long)offset) >= (int32_t)(session->sample_rate_hz / 2u))
                continue;

            bredr_channel_processor_t *proc = &session->bredr_channels[session->bredr_channel_count];
            int bin = channelizer_bank_bin_for_center(
                session->bredr_channelizer.bank.M,
                session->bredr_channelizer.bank.lo_eff_hz,
                center, CHANNELIZER_BANK_GRID_BR_EDR_HZ);
            if (bin < 0)
                continue;
            
            int ok = bredr_channel_processor_init(
                proc, session->bredr_chan_dispatcher, (uint16_t)c, center,
                session->sample_rate_hz, (unsigned int)bin,
                session->bredr_channelizer.bank.M,
                session->bredr_channelizer.bank.M2,
                CHANNELIZER_BANK_RSSI_CAL_DB);
            
            if (ok != 0)
                continue;
            proc->session = session;
            session->bredr_channel_count++;
        }

        if (session->bredr_channel_count == 0u)
        {
            channelizer_destroy(&session->bredr_channelizer);
            return -1;
        }
        session->bredr_channelizer.active = 1;
    }

    total = session->ble_channel_count + session->bredr_channel_count +
            (session->bredr_channelizer.active ? 1u : 0u);
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

    size_t total = session->ble_channel_count + session->bredr_channel_count +
                   (session->ble_channelizer.active ? 1u : 0u) +
                   (session->bredr_channelizer.active ? 1u : 0u);
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

    if (session->ble_channelizer.active)
    {
        session->ble_channelizer.shutdown = &session->shutdown_requested;
        if (pthread_create(&session->worker_threads[started], NULL,
                           channelizer_worker, &session->ble_channelizer) != 0)
        {
            session_destroy(session);
            return -1;
        }
        session->ble_channelizer_thread = session->worker_threads[started];
        session->ble_channelizer_running = 1;
        started++;
    }

    if (session->bredr_channelizer.active)
    {
        session->bredr_channelizer.shutdown = &session->shutdown_requested;
        if (pthread_create(&session->worker_threads[started], NULL,
                            channelizer_worker, &session->bredr_channelizer) != 0)
        {
            /* Channelizer thread failed to start: keep BR/EDR workers but they
             * would starve, so treat it as a hard failure. */
            session_destroy(session);
            return -1;
        }
        session->bredr_channelizer_thread = session->worker_threads[started];
        session->bredr_channelizer_running = 1;
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

    /* Snapshot the drop counters now: the dispatcher resets below zero them,
     * and session_run() calls session_destroy() before returning, so any
     * post-run query must read this snapshot rather than the live (now-zero)
     * counters. */
    session->dropped_blocks_total = sample_dispatcher_total_dropped(session->dispatcher);
    if (session->ble_chan_dispatcher)
        session->dropped_blocks_total += sample_dispatcher_total_dropped(session->ble_chan_dispatcher);
    if (session->bredr_chan_dispatcher)
        session->dropped_blocks_total += sample_dispatcher_total_dropped(session->bredr_chan_dispatcher);

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
    session->ble_channelizer_running = 0;
    session->bredr_channelizer_running = 0;
    atomic_store_explicit(&session->shutdown_requested, 0u, memory_order_release);

    ble_tracker_free(&session->ble_tracker);
    bredr_tracker_free(&session->bredr_tracker);
    pthread_mutex_destroy(&session->bredr_mutex);
    
    channelizer_destroy(&session->ble_channelizer);
    if (session->ble_chan_dispatcher)
    {
        sample_dispatcher_destroy(session->ble_chan_dispatcher);
        free(session->ble_chan_dispatcher);
        session->ble_chan_dispatcher = NULL;
    }
    
    channelizer_destroy(&session->bredr_channelizer);
    if (session->bredr_chan_dispatcher)
    {
        sample_dispatcher_destroy(session->bredr_chan_dispatcher);
        free(session->bredr_chan_dispatcher);
        session->bredr_chan_dispatcher = NULL;
    }
    sample_dispatcher_destroy(session->dispatcher);
    return 0;
}

/* --- BLE advertiser tracking: owned by the BLE tracker (see ble_tracker) --- */

void session_process_ble_event(session_t *session, const ble_event_t *event)
{
    if (!session || !event) return;

    /* The tracker owns all BLE correlation: it parses advertising PDUs
     * (advertiser name/manufacturer, CONNECT_IND linkage) and CRC-gates
     * data frames against its piconet store. It returns whether the frame
     * is surfaced to presentation layers (advertising, or a CRC-valid data
     * frame); pure correlation frames are consumed silently. */
    int surface = ble_tracker_submit_frame(&session->ble_tracker, event);
    if (surface && session->ble_cb)
        session->ble_cb(event, session->ble_user);
}

void session_process_bredr_event(session_t *session, const bredr_event_t *event)
{
    if (!session || !event) return;

    pthread_mutex_lock(&session->bredr_mutex);
    int packet_is_newest = 0;
    bredr_piconet_t *pnet = bredr_tracker_add_packet(&session->bredr_tracker,
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
        snapshot.central_clk_1_6        = bredr_piconet_central_clk_1_6(pnet, pnet->last_seen);
        snapshot.tracking_state = pnet->tracking_state;
        snapshot.total_packets  = pnet->total_packets;
        snapshot.combined_rssi_seen =
            rssi_tracker_average(&pnet->combined_rssi_track, &snapshot.combined_rssi);
        snapshot.master_rssi_seen =
            rssi_tracker_average(&pnet->master_rssi_track, &snapshot.master_rssi);
        for (int lt = 0; lt < 8; lt++)
            snapshot.slave_rssi_seen[lt] =
                rssi_tracker_average(&pnet->slave_rssi_track[lt],
                                     &snapshot.slave_rssi[lt]);
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
    return bredr_piconet_store_count(&session->bredr_tracker.store);
}

int session_bredr_piconet_snapshot(const session_t *session,
                                  size_t index,
                                  bredr_piconet_snapshot_t *out)
{
    if (!session || !out) return -1;
    const bredr_piconet_t *pnet = bredr_piconet_store_get(&session->bredr_tracker.store, index);
    if (!pnet) return -1;
    memset(out, 0, sizeof(*out));
    out->lap            = pnet->lap;
    out->uap_found      = pnet->uap_found;
    out->uap            = pnet->uap;
    out->clk_known      = pnet->clk_known;
    out->central_clk_1_6        = bredr_piconet_central_clk_1_6(pnet, pnet->last_seen);
    out->tracking_state = pnet->tracking_state;
    out->total_packets  = pnet->total_packets;
    out->combined_rssi_seen =
        rssi_tracker_average(&pnet->combined_rssi_track, &out->combined_rssi);
    out->master_rssi_seen =
        rssi_tracker_average(&pnet->master_rssi_track, &out->master_rssi);
    for (int lt = 0; lt < 8; lt++)
        out->slave_rssi_seen[lt] =
            rssi_tracker_average(&pnet->slave_rssi_track[lt],
                                 &out->slave_rssi[lt]);
    return 0;
}

size_t session_get_bredr_devices(const session_t *session,
                                 bredr_device_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return bredr_tracker_get_devices(&session->bredr_tracker, out, max);
}

size_t session_get_bredr_piconets(const session_t *session,
                                  bredr_piconet_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return bredr_tracker_get_piconets(&session->bredr_tracker, out, max);
}

size_t session_get_ble_devices(const session_t *session,
                               ble_device_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return ble_tracker_get_devices(&session->ble_tracker, out, max);
}

size_t session_get_ble_piconets(const session_t *session,
                                ble_piconet_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return ble_tracker_get_piconets(&session->ble_tracker, out, max);
}

unsigned long session_dropped_blocks(const session_t *session)
{
    if (!session) return 0ul;
    /* After teardown the live counters have been reset to zero, so report the
     * snapshot taken just before reset (see session_destroy). */
    if (session->torn_down)
        return session->dropped_blocks_total;
    unsigned long total = sample_dispatcher_total_dropped(session->dispatcher);
    if (session->ble_chan_dispatcher)
        total += sample_dispatcher_total_dropped(session->ble_chan_dispatcher);
    if (session->bredr_chan_dispatcher)
        total += sample_dispatcher_total_dropped(session->bredr_chan_dispatcher);
    return total;
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
