/**
 * @file ble_registry.h
 * @brief BLE registry: sole owner of connection + device tables.
 *
 * Sole owner of BLE connection + device tables. One registry owns:
 *   - conns[]   : dynamic AA-keyed connections (CRCInit recovery link +
 *                 stable ids + RSSI + timestamps). Grows dynamically; LRU
 *                 victim eviction preserves confirmed connections.
 *   - devices[] : dynamic advertiser rows plus standalone rows
 *                 (connection_id == 0) for out-of-band discovery.
 *   - pending[] : static bounded tally (1024) of first-sight AAs so one-off
 *                 junk never allocates a connection slot. No heap is touched
 *                 on the junk path, by design: LE processors emit thousands
 *                 of false positives per second.
 *
 * Threading: single internal mutex. The collector thread calls
 * ble_registry_submit(); the poll thread calls get_connections/get_devices.
 * Getters are pure reads (ids and linkage are assigned at ingest).
 */

#ifndef BLE_REGISTRY_H
#define BLE_REGISTRY_H

#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#include "ble_codec.h"
#include "receive_event_models.h"
#include "device_models.h"
#include "rssi_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initial capacity of the dynamic connections table. */
#define BLE_REGISTRY_CONNS_INIT_CAP 16u
/** Initial capacity of the dynamic devices table. */
#define BLE_REGISTRY_DEVICES_INIT_CAP 16u
/** Static pending-tally capacity (bounded, no alloc under noise). */
#define BLE_REGISTRY_PENDING_CAP 1024u
/** CRCInit candidates kept per access address (dedup, FIFO). */
#define BLE_REGISTRY_MAX_CANDIDATES 4u
/** Frames an AA must accumulate before it earns a connection slot. */
#define BLE_REGISTRY_PROMOTE_THRESHOLD 3u

typedef enum {
    BLE_CONNECTION_COLLECTING = 0,
    BLE_CONNECTION_CONFIRMED = 1,
} ble_connection_state_t;

typedef struct {
    int enforce_crc;            /**< drop CRC-failing adv frames; hide unconfirmed conns. */
} ble_registry_config_t;

/** CRCInit recovery link for one access address. */
typedef struct {
    uint32_t access_address;
    ble_connection_state_t state;
    uint32_t crc_init;          /**< valid when state == CONFIRMED. */
    uint32_t candidates[BLE_REGISTRY_MAX_CANDIDATES];
    unsigned int candidate_count;
    unsigned long packets_seen;
    unsigned long packets_accepted;
    uint64_t last_used_seq;     /**< LRU clock. */
} ble_link_t;

/** First-sight AA tally (static, bounded). */
typedef struct {
    uint32_t aa;
    unsigned long count;
    uint64_t seq;
} ble_pending_t;

/** One AA-keyed connection: recovery link + presentation state. */
typedef struct {
    ble_link_t link;
    uint64_t connection_id;
    uint64_t device_id_central, device_id_peripheral;
    rssi_tracker_t rssi;        /**< data-frame RSSI. */
    uint64_t first_seen_ms, last_seen_ms;
} ble_connection_t;

/** One advertiser device row. */
typedef struct {
    uint64_t device_id;
    uint64_t connection_id;     /**< linked AA's connection (0 = standalone). */
    uint64_t adv_addr;          /**< 48-bit advertiser address. */
    char addr_type[DEVICE_ADDRTYPE_MAX];
    char name[DEVICE_NAME_MAX];
    char manufacturer[DEVICE_MANUF_MAX];
    rssi_tracker_t rssi;
    uint64_t first_seen_ms, last_seen_ms;
    unsigned long total_packets;
    uint32_t linked_aa;
    int has_linked_aa;
} ble_device_entry_t;

/** Out-of-band device observation (e.g. chipset discovery). */
typedef struct {
    uint64_t adv_addr;          /**< 48-bit advertiser address. */
    const char *addr_type;
    const char *name;
    const char *manufacturer;
    float rssi_db;
    int rssi_valid;
} ble_device_obs_t;

/** Decoupled advertiser observation (internal ingest detail). */
typedef struct {
    uint64_t adv_addr;
    const char *addr_type;
    const char *name;
    const char *manufacturer;
    float rssi_db;
    int rssi_valid;
    int is_connect_ind;
    uint32_t conn_access_address;
    uint64_t conn_initiator_addr;
    uint64_t conn_advertiser_addr;
    uint32_t conn_crc_init;
    int crc_ok;
} ble_advertiser_event_t;

typedef struct ble_registry {
    ble_connection_t *conns;
    size_t conns_count, conns_cap;
    uint64_t seq;               /**< connection LRU clock. */
    ble_device_entry_t *devices;
    size_t devices_count, devices_cap;
    ble_pending_t pending[BLE_REGISTRY_PENDING_CAP];
    size_t pending_count;
    uint64_t pending_seq;
    uint64_t next_id;           /**< monotonic id allocator (starts at 1). */
    ble_registry_config_t cfg;
    pthread_mutex_t lock;
} ble_registry_t;

void ble_registry_init(ble_registry_t *r, const ble_registry_config_t *cfg);
void ble_registry_free(ble_registry_t *r);

/** Enable/disable CRC enforcement. */
void ble_registry_set_enforce_crc(ble_registry_t *r, int on);

/**
 * Ingest one BLE event: advertising frames register advertisers (+
 * CONNECT_IND linkage); data frames are CRC-gated for CRCInit recovery.
 *
 * @param touched_out Optional: filled by value with the touched connection
 *                    snapshot (data frames only, zeroed otherwise).
 * @return 1 if surface-worthy (advertising, or CRC-verified data),
 *         0 if consumed for correlation only.
 */
int ble_registry_submit(ble_registry_t *r, const ble_event_t *event,
                        ble_connection_snapshot_t *touched_out);

/** Ingest an out-of-band device (standalone, connection_id == 0). */
int ble_registry_add_device(ble_registry_t *r, const ble_device_obs_t *obs);

/** Force-confirm an AA/CRCInit pair (test setup). */
void ble_registry_confirm_for_test(ble_registry_t *r, uint32_t access_address,
                                   uint32_t crc_init);

/** Copy the recovery link for @p access_address (test introspection).
 * Returns 0 when found, -1 otherwise. */
int ble_registry_find_link_for_test(const ble_registry_t *r,
                                    uint32_t access_address,
                                    ble_link_t *out);

/** Fill out[] with at most max device snapshots; returns count. */
size_t ble_registry_get_devices(const ble_registry_t *r,
                                ble_device_snapshot_t *out, size_t max);

/** Fill out[] with at most max connection snapshots; returns count. */
size_t ble_registry_get_connections(const ble_registry_t *r,
                                    ble_connection_snapshot_t *out, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* BLE_REGISTRY_H */
