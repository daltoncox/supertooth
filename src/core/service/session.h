#ifndef SESSION_H
#define SESSION_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include <liquid/liquid.h>

#include "ble_channel_processor.h"
#include "bredr_channel_processor.h"
#include "channelizer_service.h"
#include "ble_registry.h"
#include "bredr_registry.h"
#include "bredr_display.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"
#include "radio_common.h"
#include "collector.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SESSION_BREDR_LNA_GAIN 24u
#define SESSION_BREDR_VGA_GAIN 18u
#define SESSION_BLE_LNA_GAIN   24u
#define SESSION_BLE_VGA_GAIN   18u

#define BREDR_SESSION_MAX_CHANNELS 79u

/** Protocol whose channel window defines the radio tuning.
 *  SESSION_REF_BLE is for BLE-only sessions. Hybrid sessions always use
 *  SESSION_REF_BREDR on a single shared 1 MHz channelizer (session_tune()
 *  rejects BLE-ref tunes once both protocols are enabled). */
typedef enum {
    SESSION_REF_BLE   = 0,  /**< Window is an LE RF channel range (BLE-only). */
    SESSION_REF_BREDR = 1,  /**< Window is a BR/EDR channel range; BLE fans out inside it. */
} session_protocol_ref_t;

typedef void (*session_ble_packet_fn)(const ble_event_t *event, void *user);
typedef void (*session_bredr_packet_fn)(const bredr_event_t *event,
                                         const bredr_connection_snapshot_t *connection,
                                         void *user);

typedef struct {
    uint32_t lna_gain;
    uint32_t vga_gain;

    radio_device_type_t device_type;
    const char *device_id;
    int debug;

    /* File replay only: 1 = exhaustive (backpressure, never drop for pacing
     * reasons), 0 = realtime wall-clock pacing (default). Ignored for live
     * radios. */
    int file_exhaustive;
} session_config_t;

typedef struct {
    unsigned int enforce_crc;  /* 1 = drop frames whose BLE CRC fails (default on) */
} session_ble_config_t;

typedef struct {
    uint32_t lap_filter;
    int lap_filter_enabled;
} session_bredr_config_t;

/** Per-stage breakdown of where blocks were dropped during a session. Read this
 *  (not the live dispatcher counters) after a run: session_run() resets the
 *  live counters on teardown, and the snapshot is taken just before that.
 *  The single shared channelizer service has three stages:
 *    - rf  : radio dispatcher (radio -> K DDC lanes).
 *    - sub : K intermediate dispatchers (DDC lane -> PFB lane).
 *    - out : K partitioned output dispatchers (PFB lane -> channel workers).
 *  For each stage, two sub-reasons are tracked:
 *    - *_pool_exhausted : a producer could not allocate a block from the pool
 *      (e.g. the radio could not allocate an RF block, or a DDC/PFB lane
 *      could not allocate its output block).
 *    - *_consumer_full   : a reader's queue was full, i.e. a consumer reading
 *      from this stage was not keeping up (a DDC lane for the RF stage, the
 *      PFB lane for the sub stage, or a channel worker for the out stage).
 *  Per-lane arrays pinpoint which lane stalled; aggregates are the sum over
 *  lanes. lane_count is K at snapshot time (0 when the service never ran). */
typedef struct {
    unsigned long rf_pool_exhausted;
    unsigned long rf_consumer_full;
    unsigned long sub_pool_exhausted;
    unsigned long sub_consumer_full;
    unsigned long out_pool_exhausted;
    unsigned long out_consumer_full;
    unsigned int lane_count;
    unsigned long sub_pool_exhausted_lane[CHANNELIZER_SERVICE_MAX_LANES];
    unsigned long sub_consumer_full_lane[CHANNELIZER_SERVICE_MAX_LANES];
    unsigned long out_pool_exhausted_lane[CHANNELIZER_SERVICE_MAX_LANES];
    unsigned long out_consumer_full_lane[CHANNELIZER_SERVICE_MAX_LANES];
} session_drop_breakdown_t;

typedef struct session {
    uint32_t lo_frequency_hz;
    uint32_t sample_rate_hz;

    int ble_enabled;
    int bredr_enabled;

    session_ble_config_t   ble_cfg;
    session_ble_packet_fn  ble_cb;
    void *ble_user;

    /* Invoked (on the session worker thread) the moment the run loop exits,
     * BEFORE the blocking radio teardown. Lets the UI flip to "stopped"
     * immediately rather than waiting on device shutdown. */
    void (*stopped_cb)(void *user);
    void *stopped_user;

    session_bredr_config_t  bredr_cfg;
    session_bredr_packet_fn bredr_cb;
    void *bredr_user;

    sample_dispatcher_t *dispatcher;

    /**
     * Channelization service (DDC + PFB lanes, owned dispatchers, worker
     * threads). Single instance for every mode: 1 MHz grid for BR/EDR and
     * hybrid sessions (BLE fans out over the same lanes), 2 MHz grid for
     * BLE-only sessions (power saver, transparent 1 MHz fallback).
     */
    channelizer_service_t chan_svc;
    int                   chan_svc_running;

    ble_registry_t      ble_registry;  /**< owns BLE devices + connections. */
    bredr_registry_t    bredr_registry;

    /** Per-protocol collector: channel processors submit decoded events here; a
     *  dedicated thread drains the queue, runs the tracker, and invokes the
     *  presentation callback. Keeps BLE and BR/EDR from contending on a shared
     *  tracker mutex and decouples block lifetime from presentation cost. */
    collector_t         ble_collector;
    collector_t         bredr_collector;

    ble_channel_processor_t   *ble_channels;
    size_t                     ble_channel_count;
    bredr_channel_processor_t *bredr_channels;
    size_t                     bredr_channel_count;

    pthread_t *worker_threads;
    size_t     worker_count;
    int        workers_running;

    /* Written by session_request_stop() (often from another thread) and read by
     * the run loop / worker threads; atomic + acquire/release so the stop is
     * observed promptly on all architectures (incl. ARM64 where plain volatile
     * does not guarantee cross-thread visibility). */
    _Atomic unsigned int shutdown_requested;

    session_config_t config;

    radio_device_t *device;

    /* Set once teardown has run so session_destroy() is idempotent (it may be
     * called both from within session_run() and by the owner afterwards). */
    int torn_down;

    /* Snapshot of the total dropped-block count, taken just before the
     * dispatchers are reset during session_destroy().  The live counters are
     * zeroed by the reset, so this is what callers must read after a run. */
    unsigned long dropped_blocks_total;

    /* Per-stage breakdown of dropped blocks (see session_drop_breakdown_t).
     * Snapshotted at the same point as dropped_blocks_total. */
    session_drop_breakdown_t dropped_breakdown;

    /* Collector overwrite-oldest drops, snapshotted in session_destroy()
     * before the queues are torn down (same pattern as the block counters
     * above). Always 0 in exhaustive mode short of a shutdown race. */
    unsigned long ble_collector_dropped;
    unsigned long bredr_collector_dropped;

    /* BLE frame counts: emitted = every event reaching
     * session_process_ble_event (i.e. every VALID_PACKET the BLE processors
     * forwarded); confirmed = subset the registry surfaced
     * (ble_registry_submit returned 1: accepted advertising frame or
     * CRC-gated data frame). Sole writer is the single BLE collector thread. */
    unsigned long ble_frames_emitted;
    unsigned long ble_frames_confirmed;

    /* BR/EDR frame count: emitted = every event reaching
     * session_process_bredr_event (i.e. every valid packet the BR/EDR
     * processors forwarded). No confirmation stage exists for BR/EDR, so
     * emitted is the only metric. Sole writer is the single BR/EDR
     * collector thread. */
    unsigned long bredr_frames_emitted;
} session_t;

int  session_init(session_t *session, const session_config_t *cfg);
void session_enable_ble(session_t *session,
                         const session_ble_config_t *cfg,
                         session_ble_packet_fn cb, void *user);
void session_enable_bredr(session_t *session,
                           const session_bredr_config_t *cfg,
                           session_bredr_packet_fn cb, void *user);
/* Register a callback fired on the session worker thread just before the
 * blocking radio teardown begins (i.e. once the capture loop has ended). */
void session_set_stopped_callback(session_t *session,
                                  void (*cb)(void *user), void *user);
int  session_tune(session_t *session,
                  session_protocol_ref_t ref,
                  unsigned int bottom_channel,
                  unsigned int channel_count);

/* Pre-init layout check: is (device, enables, ref, bottom, count) tuneable?
 * Pure function of its arguments — needs no session object. Apps call this
 * (instead of reaching into the channelizer/radio layers) to validate CLI /
 * UI values early with specific diagnostics. session_tune() enforces the
 * identical checks, so validation can never drift from tuning. */
typedef enum {
    SESSION_LAYOUT_OK = 0,
    SESSION_LAYOUT_BAD_RANGE,     /* count 0, band overflow, bad ref */
    SESSION_LAYOUT_RATE_EXCEEDED, /* device cannot sustain the sample rate */
    SESSION_LAYOUT_NO_LANE_SPLIT, /* no even <=20 MHz lane split */
} session_layout_status_t;

session_layout_status_t session_validate_layout(
    radio_device_type_t device_type,
    int ble_enabled, int bredr_enabled,
    session_protocol_ref_t ref,
    unsigned int bottom_channel,
    unsigned int channel_count);

/* Thin wrappers so apps never touch the lane planner or radio ceilings
 * directly: snap a BR/EDR count down to the nearest supported lane split,
 * the default BR/EDR count for a device, and a device's max sample rate
 * (for user-facing diagnostics). */
unsigned int session_snap_bredr_count(unsigned int count);
unsigned int session_default_bredr_count(radio_device_type_t device_type);
uint32_t session_device_max_rate_hz(radio_device_type_t device_type);
/* Printable device-type name for diagnostics (e.g. "hackrf"). */
const char *session_device_type_name(radio_device_type_t device_type);
int  session_run(session_t *session);
void session_request_stop(session_t *session);
int  session_destroy(session_t *session);

void session_process_ble_event(session_t *session, const ble_event_t *event);
void session_process_bredr_event(session_t *session, const bredr_event_t *event);

/** Snapshot polling API for the device/connection list (caller-provided
 *  array, core fills up to @p max entries and returns the number written). */
size_t session_get_bredr_devices(const session_t *session,
                                 bredr_device_snapshot_t *out, size_t max);
size_t session_get_bredr_connections(const session_t *session,
                                     bredr_connection_snapshot_t *out, size_t max);
size_t session_get_ble_devices(const session_t *session,
                               ble_device_snapshot_t *out, size_t max);
size_t session_get_ble_connections(const session_t *session,
                                   ble_connection_snapshot_t *out, size_t max);

unsigned long session_dropped_blocks(const session_t *session);
void session_dropped_blocks_breakdown(const session_t *session,
                                       session_drop_breakdown_t *out);

void session_ble_frame_counts(const session_t *session,
                              unsigned long *emitted,
                              unsigned long *confirmed);

unsigned long session_bredr_frame_count(const session_t *session);

/* Collector overwrite-oldest drops per protocol (see
 * session_t.ble/bredr_collector_dropped). After teardown these read the
 * snapshot taken in session_destroy(); pass NULL for either side. */
void session_collector_dropped(const session_t *session,
                               unsigned long *ble,
                               unsigned long *bredr);

/* Test-only helper: build the BLE/BR/EDR channel processors for the current
 * tune + enable state (without opening the radio) and report the counts.
 * Returns 0 on success; *ble_count / *bredr_count receive the processor
 * counts. Used by tests/service/test_channel_layout.c. */
int session_create_channels_for_test(session_t *session,
                                     size_t *ble_count,
                                     size_t *bredr_count);

#ifdef __cplusplus
}
#endif

#endif /* SESSION_H */
