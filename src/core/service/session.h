#ifndef SESSION_H
#define SESSION_H

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdatomic.h>

#include <liquid/liquid.h>

#include "ble_channel_processor.h"
#include "bredr_channel_processor.h"
#include "channelizer_thread.h"
#include "ble_piconet.h"
#include "bredr_piconet.h"
#include "bredr_display.h"
#include "bredr_piconet_store.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"
#include "radio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SESSION_BREDR_LNA_GAIN 24u
#define SESSION_BREDR_VGA_GAIN 18u
#define SESSION_BLE_LNA_GAIN   24u
#define SESSION_BLE_VGA_GAIN   18u

#define BREDR_SESSION_DEFAULT_RSSI_AVERAGING_WINDOW 16u
#define BREDR_SESSION_MAX_CHANNELS 79u

/** Protocol whose channel window defines the radio tuning. */
typedef enum {
    SESSION_REF_BLE   = 0,  /**< Window is an LE RF channel range; BR/EDR fans out inside it. */
    SESSION_REF_BREDR = 1,  /**< Window is a BR/EDR channel range; BLE fans out inside it. */
} session_protocol_ref_t;

typedef void (*session_ble_packet_fn)(const ble_event_t *event, void *user);
typedef void (*session_bredr_packet_fn)(const bredr_event_t *event,
                                         const bredr_piconet_snapshot_t *piconet,
                                         void *user);

typedef struct {
    uint32_t lna_gain;
    uint32_t vga_gain;

    radio_device_type_t device_type;
    const char *device_id;
    int debug;
} session_config_t;

typedef struct {
    unsigned int enforce_crc;  /* 1 = drop frames whose BLE CRC fails (default on) */
} session_ble_config_t;

typedef struct {
    unsigned int rssi_averaging_window;
    uint32_t lap_filter;
    int lap_filter_enabled;
} session_bredr_config_t;

typedef struct session {
    uint32_t lo_frequency_hz;
    uint32_t sample_rate_hz;
    unsigned int decimation;

    session_protocol_ref_t tune_ref;
    unsigned int tune_bottom;
    unsigned int tune_count;

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

    /** Frame-major channelizer output (BR/EDR channel processors read here). */
    sample_dispatcher_t *bredr_chan_dispatcher;
    bredr_channelizer_t  bredr_channelizer;
    pthread_t            bredr_channelizer_thread;
    int                  bredr_channelizer_running;

    ble_piconet_store_t   ble_piconet_store;
    bredr_piconet_store_t bredr_piconet_store;
    pthread_mutex_t       bredr_mutex;

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
int  session_run(session_t *session);
void session_request_stop(session_t *session);
int  session_destroy(session_t *session);

void session_process_ble_event(session_t *session, const ble_event_t *event);
void session_process_bredr_event(session_t *session, const bredr_event_t *event);

size_t       session_bredr_piconet_count(const session_t *session);
int          session_bredr_piconet_snapshot(const session_t *session,
                                           size_t index,
                                           bredr_piconet_snapshot_t *out);
unsigned long session_dropped_blocks(const session_t *session);

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
