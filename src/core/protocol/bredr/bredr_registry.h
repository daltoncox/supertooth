/**
 * @file bredr_registry.h
 * @brief BR/EDR registry: sole owner of connection + device tables.
 *
 * Sole owner of BR/EDR connection + device tables. One registry
 * owns:
 *   - conns[]   : dynamic LAP-keyed connections (DSP link + stable ids +
 *                 idle-reset state). The DSP link (bredr_link_t, incl. the
 *                 128-event queue) stays heap-allocated per connection.
 *   - devices[] : dynamic materialized member rows (central + per-LT_ADDR
 *                 peripherals) plus standalone rows (connection_id == 0) for
 *                 out-of-band discovery (e.g. chipset inquiry hits).
 *   - pending[] : static bounded tally of first-sight LAPs so one-off junk
 *                 never allocates a 60KB link (mirrors the BLE registry's
 *                 promote-on-promise gate; threshold 1 preserves legacy
 *                 create-on-first-sight behavior).
 *
 * Threading: single internal mutex. The collector thread calls
 * bredr_registry_submit(); the poll thread calls get_connections/get_devices.
 * Getters are pure reads (ids are assigned at ingest, never at poll).
 */

#ifndef BREDR_REGISTRY_H
#define BREDR_REGISTRY_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>

#include "bredr_link.h"
#include "receive_event_models.h"
#include "device_models.h"
#include "rssi_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initial capacity of the dynamic connections table. */
#define BREDR_REGISTRY_CONNS_INIT_CAP 8u
/** Initial capacity of the dynamic devices table. */
#define BREDR_REGISTRY_DEVICES_INIT_CAP 16u
/** Static pending-tally capacity (bounded, no alloc under noise). */
#define BREDR_REGISTRY_PENDING_CAP 128u

typedef struct {
    uint32_t idle_reset_clkn;   /**< LAP silence before recovery reset (default 16384, 0=disable). */
    unsigned int promote_threshold; /**< first-sight LAP frames before a slot is earned (default 1). */
} bredr_registry_config_t;

/** One LAP-keyed connection: DSP link + presentation linkage. */
typedef struct {
    bredr_link_t *link;      /**< heap DSP state incl. queue; NULL only on alloc failure. */
    uint64_t connection_id;     /**< stable id for this connection. */
    uint64_t device_id_central; /**< linkage into devices[] (0 = none yet). */
    uint64_t device_id_peripheral[8];
    uint32_t last_clkn;
    int has_last_clkn;
} bredr_connection_t;

/** One member device row (materialized at ingest; poll just copies).
 * Member rows (connection_id != 0) resolve RSSI/timestamps/UAP from the
 * owning connection link at poll time; standalone rows (connection_id == 0)
 * use the stored observation fields below. */
typedef struct {
    uint64_t device_id;
    uint64_t connection_id;     /**< owning connection (0 = standalone). */
    uint8_t lt_addr;            /**< 255 = central, 0..7 = peripheral slot. */
    uint32_t lap;               /**< 24-bit LAP of the owning connection. */
    uint8_t uap;
    int uap_valid;
    float rssi_db;
    int rssi_valid;
    uint64_t first_seen_ms, last_seen_ms;
    unsigned long total_packets;
} bredr_device_entry_t;

/** First-sight LAP tally (static, bounded). */
typedef struct {
    uint32_t lap;
    unsigned long count;
    uint64_t seq;
} bredr_pending_t;

/** Out-of-band device observation (e.g. chipset inquiry hit). */
typedef struct {
    uint32_t lap;               /**< 24-bit LAP. */
    uint8_t uap;
    int uap_valid;
    float rssi_db;
    int rssi_valid;
} bredr_device_obs_t;

typedef struct bredr_registry {
    bredr_connection_t *conns;
    size_t conns_count, conns_cap;
    bredr_device_entry_t *devices;
    size_t devices_count, devices_cap;
    bredr_pending_t pending[BREDR_REGISTRY_PENDING_CAP];
    size_t pending_count;
    uint64_t pending_seq;
    uint64_t next_id;           /**< monotonic id allocator (starts at 1). */
    uint64_t epoch_base_ms;     /**< wall-clock at first packet. */
    uint32_t clk_base_1600;     /**< rx_clk_1600 at first packet. */
    int has_epoch_base;
    bredr_registry_config_t cfg;
    pthread_mutex_t lock;
    FILE *frame_dump;           /**< owned by caller; forwarded to recovery. */
} bredr_registry_t;

void bredr_registry_init(bredr_registry_t *r, const bredr_registry_config_t *cfg);
void bredr_registry_free(bredr_registry_t *r);

/**
 * Ingest one BR/EDR event: route to its LAP connection (pending-gated),
 * drive idle-reset + DSP link, materialize member device rows, assign ids.
 *
 * @param touched_out  Optional: filled by value (under lock) with the
 *                     connection snapshot for this packet. Never a pointer.
 * @param is_newest_out Optional: packet_is_newest from the link queue.
 * @return 0 on success, -1 on error.
 */
int bredr_registry_submit(bredr_registry_t *r, const bredr_event_t *event,
                          bredr_connection_snapshot_t *touched_out,
                          int *is_newest_out);

/** Ingest an out-of-band device (standalone, connection_id == 0). Returns 0 on success. */
int bredr_registry_add_device(bredr_registry_t *r, const bredr_device_obs_t *obs);

/** Frame-dump handle (owned by caller, NULL disables). */
void bredr_registry_set_frame_dump(bredr_registry_t *r, FILE *file);

/** Fill out[] with at most max connection snapshots; returns count. */
size_t bredr_registry_get_connections(const bredr_registry_t *r,
                                      bredr_connection_snapshot_t *out,
                                      size_t max);

/** Fill out[] with at most max member-device snapshots; returns count. */
size_t bredr_registry_get_devices(const bredr_registry_t *r,
                                  bredr_device_snapshot_t *out,
                                  size_t max);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_REGISTRY_H */
