/**
 * @file ble_tracker.h
 * @brief BLE tracker: owns connection (piconet) recovery via the CRC gate and
 *        adds net-new advertiser tracking, exposing device + piconet
 *        snapshots for polling.
 *
 * A ble_tracker_t embeds a ble_piconet_store_t for connection/CRCInit recovery
 * and adds a separate advertiser registry keyed by advertiser address. A
 * CONNECT_IND advertiser PDU links an advertiser device to the connection it
 * spawns (so the piconet snapshot can reference its two peer device ids).
 *
 * The CRC gate is called from the decoder per-bit path, so all entry points
 * are serialized by an internal mutex and avoid allocation in the hot path.
 */

#ifndef BLE_TRACKER_H
#define BLE_TRACKER_H

#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#include "ble_piconet.h"
#include "receive_event_models.h"
#include "device_models.h"
#include "rssi_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Decoupled advertiser observation fed by the session/processor after the
 *  BLE codec has decoded an advertising-channel PDU. */
typedef struct {
    uint64_t adv_addr;          /**< 48-bit advertiser address */
    const char *addr_type;      /**< "PUBLIC"/"RANDOM-STATIC"/... or NULL */
    const char *name;           /**< LE local name (or NULL) */
    const char *manufacturer;   /**< AD Company ID name (or NULL) */
    float rssi_db;
    int    rssi_valid;

    /* CONNECT_IND linkage (all zero/false when not a connect indication). */
    int      is_connect_ind;
    uint32_t conn_access_address;
    uint64_t conn_initiator_addr;
    uint64_t conn_advertiser_addr;
    uint32_t conn_crc_init;
} ble_advertiser_event_t;

/** Advertiser registry entry (owned by the tracker). */
typedef struct {
    uint64_t adv_addr;
    int      used;
    char     addr_type[DEVICE_ADDRTYPE_MAX];
    char     name[DEVICE_NAME_MAX];
    char     manufacturer[DEVICE_MANUF_MAX];
    rssi_tracker_t rssi_track;   /**< rolling 1 s average source */
    uint64_t first_seen_ms;
    uint64_t last_seen_ms;
    unsigned long total_packets;
    uint64_t device_id;
    uint32_t linked_aa;
    int      has_linked_aa;
} ble_adv_record_t;

/** Connection (piconet) registry entry (mirrors the recovery store). */
typedef struct {
    uint32_t aa;
    int      used;
    uint64_t piconet_id;
    uint64_t device_id_master;
    uint64_t device_id_slave;
    uint32_t crc_init;
    int      crc_init_confirmed;
    int      state;
    uint64_t first_seen_ms;
    uint64_t last_seen_ms;
    rssi_tracker_t rssi_track;   /**< rolling RSSI from accepted data
                                        frames (every data packet carries an
                                        RSSI), used to populate the piconet
                                        snapshot's signal strength */
} ble_conn_record_t;

typedef struct ble_tracker_t {
    ble_piconet_store_t conn_store;   /**< connection / CRCInit recovery
                                           (owned by the tracker) */
    ble_adv_record_t *adv;            /**< advertiser registry */
    size_t adv_count;
    size_t adv_cap;
    ble_conn_record_t *conn;          /**< connection registry */
    size_t conn_count;
    size_t conn_cap;
    uint64_t next_id;
    int      enforce_crc;   /**< drop CRC-failing advertising frames and
                                 suppress unconfirmed connection candidates
                                 from the device list */
    pthread_mutex_t lock;
} ble_tracker_t;

void ble_tracker_init(ble_tracker_t *t);
void ble_tracker_free(ble_tracker_t *t);

/** Enable/disable CRC enforcement (see `enforce_crc` above). */
void ble_tracker_set_enforce_crc(ble_tracker_t *t, int on);

/**
 * Ingest one fully-decoded BLE event from the session collector.
 *
 * Advertising frames are parsed (name/manufacturer/CONNECT_IND linkage) and
 * registered as advertisers; data frames are CRC-gated against the tracker's
 * piconet store to recover/confirm the connection CRCInit.
 *
 * @return 1 if the frame is "surface-worthy" (advertising, or a data frame
 *         whose CRC verified) and should be forwarded to presentation/UI
 *         layers; 0 if it was consumed for correlation only (e.g. a
 *         CRC-invalid data candidate).
 */
int ble_tracker_submit_frame(ble_tracker_t *t, const ble_event_t *event);

/** CRC gate (data-channel packets) — used internally by submit_frame. */
ble_gate_result_t ble_tracker_gate_data_pdu(ble_tracker_t *t,
                                            uint32_t access_address,
                                            const uint8_t *pdu,
                                            unsigned int pdu_bytes,
                                            uint32_t rx_crc,
                                            uint32_t *crc_init_used_out);

/** Seed a CRCInit candidate from a decoded CONNECT_IND. */
void ble_tracker_seed_candidate(ble_tracker_t *t,
                                uint32_t access_address,
                                uint32_t crc_init);

/** Ingest a decoded advertising-channel PDU (advertiser + optional link). */
void ble_tracker_add_advertiser(ble_tracker_t *t,
                                const ble_advertiser_event_t *ev);

size_t ble_tracker_get_devices(const ble_tracker_t *t,
                               ble_device_snapshot_t *out,
                               size_t max);
size_t ble_tracker_get_piconets(const ble_tracker_t *t,
                                ble_piconet_snapshot_t *out,
                                size_t max);

#ifdef __cplusplus
}
#endif

#endif /* BLE_TRACKER_H */
