#include "session.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ble_codec.h"
#include "bt_assigned_numbers.h"
#include "channelizer_bank.h"
#include "radio_common.h"
#include "collector.h"

/* Allocate + init one sample dispatcher (pool + reader table). */
static int alloc_dispatcher(sample_dispatcher_t **out)
{
    *out = (sample_dispatcher_t *)calloc(1, sizeof(**out));
    if (!*out)
        return -1;
    if (sample_dispatcher_init(*out) != 0)
    {
        free(*out);
        *out = NULL;
        return -1;
    }
    return 0;
}

/* Tear down + free one dispatcher; NULL-safe. */
static void free_dispatcher(sample_dispatcher_t **dispatcher)
{
    if (!dispatcher || !*dispatcher)
        return;
    sample_dispatcher_destroy(*dispatcher);
    free(*dispatcher);
    *dispatcher = NULL;
}

/* A missing dispatcher is idle by definition. Service dispatchers are
 * never NULL once the service is up, but the helper keeps drain checks
 * total (e.g. pre-init service with K == 0). */
static int dispatcher_idle_or_absent(const sample_dispatcher_t *dispatcher)
{
    return !dispatcher || sample_dispatcher_all_free(dispatcher);
}

/* Every service-owned dispatcher is idle (quiescence probe for drain). */
static int service_idle_or_absent(const channelizer_service_t *svc)
{
    if (!svc || svc->K == 0u)
        return 1;
    size_t n = channelizer_service_dispatcher_count(svc);
    for (size_t i = 0u; i < n; i++)
    {
        if (!dispatcher_idle_or_absent(
                channelizer_service_dispatcher_at(svc, i)))
            return 0;
    }
    return 1;
}

static void session_signal_readers(session_t *session)
{
    if (!session) return;
    for (size_t w = 0u; w < session->ble_channel_count; w++)
        sample_reader_signal(&session->ble_channels[w].reader);
    for (size_t w = 0u; w < session->bredr_channel_count; w++)
        sample_reader_signal(&session->bredr_channels[w].reader);
    if (session->chan_svc_running)
        channelizer_service_signal(&session->chan_svc);
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

    session->dispatcher = NULL;
    if (alloc_dispatcher(&session->dispatcher) != 0)
        return -1;
    memset(&session->chan_svc, 0, sizeof(session->chan_svc));
    session->chan_svc_running = 0;

    ble_registry_init(&session->ble_registry, NULL);
    bredr_registry_init(&session->bredr_registry, NULL);

    /* Collector queues: drained by dedicated per-protocol threads spawned in
     * session_run(). Bounded at 4096 events; overwrite-oldest on overflow keeps
     * the newest packets (the ones the tracker prefers). */
    if (collector_init(&session->ble_collector, sizeof(ble_event_t), 4096u,
                       &session->shutdown_requested) != 0)
        return -1;
    if (collector_init(&session->bredr_collector, sizeof(bredr_event_t), 4096u,
                       &session->shutdown_requested) != 0)
    {
        collector_destroy(&session->ble_collector);
        return -1;
    }

    return 0;
}

void session_enable_ble(session_t *session,
                        const session_ble_config_t *cfg,
                        session_ble_packet_fn cb, void *user)
{
    if (!session) return;
    if (cfg) session->ble_cfg = *cfg;
    ble_registry_set_enforce_crc(&session->ble_registry,
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

    /* Hybrid sessions (BLE + BR/EDR) run a single shared 1 MHz channelizer
     * anchored to the BR/EDR grid: BLE workers are fed center-bin slices of
     * the same bank, so a BLE-grid tune has no meaning anymore. BLE-only
     * sessions keep SESSION_REF_BLE. Fail loudly so stale callers (e.g. an
     * old --tune-ref ble invocation) cannot silently mis-tune. */
    if (session->ble_enabled && session->bredr_enabled &&
        ref != SESSION_REF_BREDR)
        return -1;

    double lo_mhz;
    unsigned int rate_mhz;

    if (ref == SESSION_REF_BLE)
    {
        if (channel_count > BLE_RF_CHANNEL_COUNT ||
            bottom_channel + channel_count > BLE_RF_CHANNEL_COUNT)
            return -1;
        lo_mhz = 2401.0 + 2.0 * (double)bottom_channel + (double)channel_count;
        /* BLE channels are 2 MHz apart, so an N-channel BLE window needs a
         * 2*N MHz span; the LO is already a whole-MHz frequency. */
        rate_mhz = 2u * channel_count;
    }
    else
    {
        if (channel_count > BREDR_SESSION_MAX_CHANNELS ||
            bottom_channel + channel_count > BREDR_SESSION_MAX_CHANNELS)
            return -1;
        lo_mhz   = 2402.0 + (double)bottom_channel + ((double)channel_count - 1.0) / 2.0;
        rate_mhz = channel_count;
    }
    if (rate_mhz < 4u) rate_mhz = 4u;

    session->lo_frequency_hz = (uint32_t)((uint64_t)(lo_mhz * 1e6));
    session->sample_rate_hz  = rate_mhz * 1000000u;

    /* Reject capture windows no stage can channelize (e.g. counts with no
     * even <=20 MHz lane split: 22, 26, 30, ...). The lane planner encodes
     * the supported set; the grid here mirrors the service that
     * session_create_channels will build (2 MHz desired for BLE-only). */
    {
        uint32_t grid = (ref == SESSION_REF_BLE)
            ? CHANNELIZER_BANK_GRID_BLE_HZ
            : CHANNELIZER_BANK_GRID_BR_EDR_HZ;
        if (!channelizer_service_valid_sample_rate(session->sample_rate_hz,
                                                    grid))
            return -1;
    }

    /* Reject capture windows the selected device cannot sustain. The generic
     * ceiling covers file replay up to 80 Msps; live radios report their own
     * (HackRF stays at 20 Msps). */
    {
        uint32_t max_rate_hz = RADIO_MAX_SAMPLE_RATE_HZ;
        if (radio_get_max_sample_rate_for_type(session->config.device_type,
                                               &max_rate_hz) == RADIO_SUCCESS &&
            session->sample_rate_hz > max_rate_hz)
            return -1;
        else if (session->sample_rate_hz > RADIO_MAX_SAMPLE_RATE_HZ)
            return -1;
    }

    return 0;
}

static int session_create_channels(session_t *session)
{
    int debug = session->config.debug;
    /* One service for every mode. BLE-only requests the 2 MHz grid (one bin
     * per BLE channel, stride 2); the service transparently falls back to
     * 1 MHz when 2 MHz would yield an odd bin count. Hybrid and BR/EDR
     * sessions share a single 1 MHz service (BLE fans out over the same
     * lanes with stride 1). */
    int ble_only = session->ble_enabled && !session->bredr_enabled;
    uint32_t grid = ble_only ? CHANNELIZER_BANK_GRID_BLE_HZ
                             : CHANNELIZER_BANK_GRID_BR_EDR_HZ;

    channelizer_service_config_t svc_cfg;
    memset(&svc_cfg, 0, sizeof(svc_cfg));
    svc_cfg.sample_rate_hz = session->sample_rate_hz;
    svc_cfg.lo_hz          = session->lo_frequency_hz;
    svc_cfg.grid_hz        = grid;
    svc_cfg.debug          = debug;
    svc_cfg.exhaustive     = session->config.file_exhaustive;
    svc_cfg.shutdown       = &session->shutdown_requested;

    if (channelizer_service_init(&session->chan_svc, session->dispatcher,
                                  &svc_cfg) != 0)
    {
        if (debug)
            fprintf(stderr, "[session] channelizer service init failed\n");
        return -1;
    }

    /* BR/EDR processors from the service descriptors. */
    if (session->bredr_enabled)
    {
        channelizer_channel_t desc[BREDR_SESSION_MAX_CHANNELS];
        size_t n = channelizer_service_get_bredr_channels(
            &session->chan_svc, desc,
            sizeof(desc) / sizeof(desc[0]));

        session->bredr_channel_count = 0u;
        session->bredr_channels = calloc(BREDR_SESSION_MAX_CHANNELS,
                                         sizeof(bredr_channel_processor_t));
        if (!session->bredr_channels)
        {
            channelizer_service_destroy(&session->chan_svc);
            return -1;
        }
        for (size_t i = 0u; i < n; i++)
        {
            /* Descriptor order follows the band (channel c at index c for
             * the in-span subset); recover the RF index from the centre. */
            uint16_t c = (uint16_t)((desc[i].center_hz -
                                     2402000000u) / 1000000u);
            bredr_channel_processor_t *proc =
                &session->bredr_channels[session->bredr_channel_count];
            if (bredr_channel_processor_init(proc, &desc[i], c) != 0)
                continue;
            proc->session = session;
            session->bredr_channel_count++;
        }
        if (session->bredr_channel_count == 0u)
        {
            channelizer_service_destroy(&session->chan_svc);
            return -1;
        }
    }

    /* BLE processors from the service descriptors (shared 1 MHz lanes in
     * hybrid mode, dedicated 2 MHz lanes when BLE-only). */
    if (session->ble_enabled)
    {
        channelizer_channel_t desc[BLE_RF_CHANNEL_COUNT];
        size_t n = channelizer_service_get_ble_channels(
            &session->chan_svc, desc,
            sizeof(desc) / sizeof(desc[0]));

        session->ble_channel_count = 0u;
        session->ble_channels = calloc(BLE_RF_CHANNEL_COUNT,
                                       sizeof(ble_channel_processor_t));
        if (!session->ble_channels)
        {
            channelizer_service_destroy(&session->chan_svc);
            return -1;
        }
        for (size_t i = 0u; i < n; i++)
        {
            uint16_t rf = (uint16_t)((desc[i].center_hz -
                                      2402000000u) / 2000000u);
            ble_channel_processor_t *proc =
                &session->ble_channels[session->ble_channel_count];
            if (ble_channel_processor_init(proc, &desc[i], rf) != 0)
                continue;
            proc->session = session;
            session->ble_channel_count++;
        }
        if (session->ble_channel_count == 0u)
        {
            channelizer_service_destroy(&session->chan_svc);
            return -1;
        }
    }

    if (session->ble_channel_count == 0u && session->bredr_channel_count == 0u)
    {
        channelizer_service_destroy(&session->chan_svc);
        return -1;
    }
    if (debug)
    {
        fprintf(stderr,
                "[session] lo=%u Hz rate=%u Hz K=%u grid=%u : "
                "%zu BLE + %zu BR/EDR processors\n",
                session->lo_frequency_hz, session->sample_rate_hz,
                session->chan_svc.K, session->chan_svc.grid_actual_hz,
                session->ble_channel_count, session->bredr_channel_count);
    }
    return 0;
}

/* Spawn one worker thread, tracked for join-on-teardown via worker_threads.
 * Returns 0 when started, -1 on pthread failure (callers treat that as
 * fatal and tear the session down). */
static int spawn_worker(session_t *session, size_t *started,
                        void *(*fn)(void *), void *arg)
{
    if (!session || !started || !fn)
        return -1;
    if (pthread_create(&session->worker_threads[*started], NULL, fn, arg) != 0)
        return -1;
    (*started)++;
    return 0;
}

/* Drains the BLE collector queue and runs the tracker + presentation callback
 * for each event. Single consumer, so the BLE tracker has exactly one writer
 * and never contends with BR/EDR or with the per-channel workers. */
static void *session_ble_collector_shim(void *arg)
{
    session_t *s = (session_t *)arg;
    ble_event_t ev;
    while (collector_pop(&s->ble_collector, &ev) == 0)
        session_process_ble_event(s, &ev);
    return NULL;
}

/* Same as the BLE collector but for BR/EDR events. */
static void *session_bredr_collector_shim(void *arg)
{
    session_t *s = (session_t *)arg;
    bredr_event_t ev;
    while (collector_pop(&s->bredr_collector, &ev) == 0)
        session_process_bredr_event(s, &ev);
    return NULL;
}

/* Quiescence bound for the exhaustive EOF drain: a wedged pipeline warns and
 * tears down instead of hanging forever. */
#define SESSION_DRAIN_TIMEOUT_NS 30000000000ull

static uint64_t session_mono_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Wait until every in-flight sample block is released and both event queues
 * are empty (i.e. the whole capture has been processed and surfaced), the
 * drain times out, or shutdown is requested. Exhaustive file replay only;
 * realtime/live teardown discards in-flight data like a Ctrl+C would. */
static void session_drain_exhaustive(session_t *session)
{
    uint64_t start_ns = session_mono_ns();

    for (;;)
    {
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 10000000L};

        if (atomic_load_explicit(&session->shutdown_requested,
                                 memory_order_acquire) != 0u)
            return;

        if (sample_dispatcher_all_free(session->dispatcher) &&
            service_idle_or_absent(&session->chan_svc) &&
            collector_count(&session->ble_collector) == 0u &&
            collector_count(&session->bredr_collector) == 0u)
            return;

        if (session_mono_ns() - start_ns > SESSION_DRAIN_TIMEOUT_NS)
        {
            if (session->config.debug)
                fprintf(stderr,
                        "[session] exhaustive drain timed out with data in "
                        "flight; tearing down anyway\n");
            return;
        }

        nanosleep(&ts, NULL);
    }
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
                    (session->ble_enabled ? 1u : 0u) +
                    (session->bredr_enabled ? 1u : 0u);
    session->worker_threads = calloc(total, sizeof(pthread_t));
    if (!session->worker_threads)
    {
        session_destroy(session);
        return -1;
    }

    size_t started = 0u;
    for (size_t w = 0u; w < session->ble_channel_count; w++)
        if (spawn_worker(session, &started,
                         ble_channel_worker, &session->ble_channels[w]) != 0)
            break;
    for (size_t w = 0u; w < session->bredr_channel_count; w++)
        if (spawn_worker(session, &started,
                         bredr_channel_worker, &session->bredr_channels[w]) != 0)
            break;

    /* The service owns its DDC + PFB threads (started here, joined in
     * session_destroy via channelizer_service_stop). Without them the
     * channel workers would starve, so a start failure is fatal. */
    if (channelizer_service_start(&session->chan_svc) != 0)
    {
        session_destroy(session);
        return -1;
    }
    session->chan_svc_running = 1;

    /* Collector threads are spawned last (joined last) so they drain any
     * remaining events after the channel workers have stopped producing. */
    if (session->ble_enabled)
    {
        if (spawn_worker(session, &started,
                         session_ble_collector_shim, session) != 0)
        {
            session_destroy(session);
            return -1;
        }
    }
    if (session->bredr_enabled)
    {
        if (spawn_worker(session, &started,
                         session_bredr_collector_shim, session) != 0)
        {
            session_destroy(session);
            return -1;
        }
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

    /* File replay mode (realtime default, exhaustive on request); no-op for
     * live radios. Applied before configure/start so the reader thread
     * observes it from its first block. Collector blocking follows the same
     * flag so decoded events also apply backpressure instead of
     * overwrite-oldest. */
    radio_set_replay_mode(session->device, session->config.file_exhaustive);
    collector_set_blocking(&session->ble_collector,
                           session->config.file_exhaustive);
    collector_set_blocking(&session->bredr_collector,
                           session->config.file_exhaustive);

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
        /* Single-pass file replay ends here: the backend has exhausted its
         * capture, so exit the loop and run the normal teardown + summary. */
        if (radio_is_finished(session->device))
            break;
    }

    /* Exhaustive file replay only (never realtime/live): the radio is done
     * but blocks/events may still be in flight. Drain every stage before the
     * teardown below stops the workers, so nothing sampled is lost. A
     * shutdown request (Ctrl+C) skips the drain and tears down immediately. */
    if (atomic_load_explicit(&session->shutdown_requested,
                             memory_order_acquire) == 0u &&
        session->config.file_exhaustive &&
        session->config.device_type == RADIO_DEVICE_FILE &&
        radio_is_finished(session->device))
        session_drain_exhaustive(session);
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
    collector_wake(&session->ble_collector);
    collector_wake(&session->bredr_collector);
}

/* Accumulate a dispatcher's drop counts into pool-exhausted / consumer-full
 * subtotals (the latter is the sum of every reader's dropped_blocks). */
static void dispatcher_accumulate(const sample_dispatcher_t *d,
                                  unsigned long *pool_exhausted,
                                  unsigned long *consumer_full)
{
    if (!d)
        return;
    *pool_exhausted += d->dropped_blocks;
    for (unsigned int i = 0u; i < d->reader_count; i++)
        *consumer_full += d->readers[i]->dropped_blocks;
}

/* Accumulate one service dispatcher's drops into the session bucket that
 * owns it: BLE-only sessions report service output under ble_out_*, every
 * other mode under bredr_out_* (hybrid parity with the old shared bank). */
static void service_accumulate(const channelizer_service_t *svc, int ble_only,
                               session_drop_breakdown_t *out,
                               unsigned long *total)
{
    if (!svc || svc->K == 0u)
        return;
    size_t n = channelizer_service_dispatcher_count(svc);
    for (size_t i = 0u; i < n; i++)
    {
        sample_dispatcher_t *d = channelizer_service_dispatcher_at(svc, i);
        if (!d)
            continue;
        if (total)
            *total += sample_dispatcher_total_dropped(d);
        if (!out)
            continue;
        if (ble_only)
            dispatcher_accumulate(d, &out->ble_out_pool_exhausted,
                                  &out->ble_out_consumer_full);
        else
            dispatcher_accumulate(d, &out->bredr_out_pool_exhausted,
                                  &out->bredr_out_consumer_full);
    }
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
    int ble_only =
        session->ble_enabled && !session->bredr_enabled;

    /* Snapshot the drop counters now: the dispatcher resets below zero them,
     * and session_run() calls session_destroy() before returning, so any
     * post-run query must read this snapshot rather than the live (now-zero)
     * counters. */
    session->dropped_blocks_total =
        sample_dispatcher_total_dropped(session->dispatcher);
    service_accumulate(&session->chan_svc, ble_only, NULL,
                       &session->dropped_blocks_total);

    /* Snapshot the per-pool breakdown so the session summary can report WHERE
     * blocks were dropped (the live counters are zeroed by the reset below). */
    memset(&session->dropped_breakdown, 0, sizeof(session->dropped_breakdown));
    dispatcher_accumulate(session->dispatcher,
                           &session->dropped_breakdown.rf_pool_exhausted,
                           &session->dropped_breakdown.rf_consumer_full);
    service_accumulate(&session->chan_svc, ble_only,
                       &session->dropped_breakdown, NULL);

    if (session->workers_running || session->chan_svc_running)
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
    session->chan_svc_running = 0;
    atomic_store_explicit(&session->shutdown_requested, 0u, memory_order_release);

    ble_registry_free(&session->ble_registry);
    bredr_registry_free(&session->bredr_registry);
    session->ble_collector_dropped =
        collector_dropped(&session->ble_collector);
    session->bredr_collector_dropped =
        collector_dropped(&session->bredr_collector);
    collector_destroy(&session->ble_collector);
    collector_destroy(&session->bredr_collector);

    /* Service stop joins DDC + PFB threads (request_stop already set the
     * flag and signalled them); destroy frees banks, readers, dispatchers. */
    channelizer_service_stop(&session->chan_svc);
    channelizer_service_destroy(&session->chan_svc);
    free_dispatcher(&session->dispatcher);
    return 0;
}

/* --- BLE device/connection correlation: owned by the BLE registry --- */

void session_process_ble_event(session_t *session, const ble_event_t *event)
{
    if (!session || !event) return;

    /* The registry owns all BLE correlation: it parses advertising PDUs
     * (advertiser name/manufacturer, CONNECT_IND linkage) and CRC-gates
     * data frames for CRCInit recovery. It returns whether the frame
     * is surfaced to presentation layers (advertising, or a CRC-valid data
     * frame); pure correlation frames are consumed silently. */
    session->ble_frames_emitted++;
    int surface = ble_registry_submit(&session->ble_registry, event, NULL);
    if (surface)
        session->ble_frames_confirmed++;
    if (surface && session->ble_cb)
        session->ble_cb(event, session->ble_user);
}

void session_process_bredr_event(session_t *session, const bredr_event_t *event)
{
    if (!session || !event) return;

    /* Sole writer is the BR/EDR collector thread, so no mutex is needed here;
     * the registry's own lock still guards the GUI poll readers. The touched
     * connection snapshot is filled by value under the registry lock (never
     * a raw pointer), so it is safe to hand to the presentation callback. */
    session->bredr_frames_emitted++;
    int packet_is_newest = 0;
    bredr_connection_snapshot_t snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    const bredr_connection_snapshot_t *snapshot_ptr = NULL;
    if (bredr_registry_submit(&session->bredr_registry, event,
                              &snapshot, &packet_is_newest) == 0 &&
        snapshot.id != 0u)
    {
        if (!packet_is_newest)
            snapshot.clk_known = 0;
        snapshot_ptr = &snapshot;
    }

    if (session->bredr_cb)
        session->bredr_cb(event, snapshot_ptr, session->bredr_user);
}

size_t session_get_bredr_devices(const session_t *session,
                                 bredr_device_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return bredr_registry_get_devices(&session->bredr_registry, out, max);
}

size_t session_get_bredr_connections(const session_t *session,
                                     bredr_connection_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return bredr_registry_get_connections(&session->bredr_registry, out, max);
}

size_t session_get_ble_devices(const session_t *session,
                               ble_device_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return ble_registry_get_devices(&session->ble_registry, out, max);
}

size_t session_get_ble_connections(const session_t *session,
                                   ble_connection_snapshot_t *out, size_t max)
{
    if (!session) return 0u;
    return ble_registry_get_connections(&session->ble_registry, out, max);
}

unsigned long session_dropped_blocks(const session_t *session)
{
    if (!session) return 0ul;
    /* After teardown the live counters have been reset to zero, so report the
     * snapshot taken just before reset (see session_destroy). */
    if (session->torn_down)
        return session->dropped_blocks_total;
    unsigned long total =
        sample_dispatcher_total_dropped(session->dispatcher);
    service_accumulate(&session->chan_svc,
                       session->ble_enabled && !session->bredr_enabled,
                       NULL, &total);
    return total;
}

void session_ble_frame_counts(const session_t *session,
                              unsigned long *emitted,
                              unsigned long *confirmed)
{
    if (emitted) *emitted = session ? session->ble_frames_emitted : 0ul;
    if (confirmed) *confirmed = session ? session->ble_frames_confirmed : 0ul;
}

unsigned long session_bredr_frame_count(const session_t *session)
{
    return session ? session->bredr_frames_emitted : 0ul;
}

void session_collector_dropped(const session_t *session,
                               unsigned long *ble,
                               unsigned long *bredr)
{
    if (!session)
    {
        if (ble)
            *ble = 0ul;
        if (bredr)
            *bredr = 0ul;
        return;
    }
    /* After teardown the queues are gone, so report the snapshot (same
     * pattern as the block-drop counters). */
    if (session->torn_down)
    {
        if (ble)
            *ble = session->ble_collector_dropped;
        if (bredr)
            *bredr = session->bredr_collector_dropped;
        return;
    }
    if (ble)
        *ble = collector_dropped(&session->ble_collector);
    if (bredr)
        *bredr = collector_dropped(&session->bredr_collector);
}

void session_dropped_blocks_breakdown(const session_t *session,
                                      session_drop_breakdown_t *out)
{
    if (!session || !out) return;
    /* After teardown the live counters have been reset to zero, so report the
     * snapshot taken just before reset (see session_destroy). */
    if (session->torn_down)
        *out = session->dropped_breakdown;
    else
    {
        memset(out, 0, sizeof(*out));
        dispatcher_accumulate(session->dispatcher,
                               &out->rf_pool_exhausted, &out->rf_consumer_full);
        service_accumulate(&session->chan_svc,
                           session->ble_enabled && !session->bredr_enabled,
                           out, NULL);
    }
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
