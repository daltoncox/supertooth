/**
 * @file device_models.h
 * @brief Protocol-agnostic entity snapshots shared between the core library
 *        and the applications (GUI / CLI).
 *
 * Overview
 * --------
 * The core tracks two classes of radio entity, per protocol:
 *
 *   - Devices  : a single observed radio entity (an LE advertiser, or a
 *                BR/EDR piconet member — master or a specific slave slot).
 *   - Piconets : a network / link relationship (a BR/EDR piconet identified
 *                by LAP, or an LE connection identified by access address).
 *
 * Both classes share a common leading field block (ENTITY_COMMON_FIELDS) so
 * an application can merge devices and piconets into a single list, copy the
 * common fields in one pass, then branch on `kind` for the class-specific
 * tail. Each protocol also has its own snapshot struct so the field sets stay
 * honest (a piconet carries clock / per-slave RSSI that a device never does).
 *
 * Passing convention
 * ------------------
 * Applications poll via the tracker getters, supplying a pre-allocated array
 * of the appropriate snapshot struct and a capacity. The core fills the array
 * under the tracker lock and returns the number of entries written (capped at
 * the supplied capacity). No heap is allocated on the read path and the
 * structs are plain-old-data, so they are safe to memcpy and own no pointers.
 *
 * Timestamps
 * ----------
 * first_seen_ms / last_seen_ms are epoch milliseconds (system clock at the
 * time the sample was processed). For BR/EDR the core converts the piconet's
 * slot clock (rx_clk_1600) to epoch ms via a session-start anchor.
 */

#ifndef DEVICE_MODELS_H
#define DEVICE_MODELS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Entity classification
 * ---------------------------------------------------------------------------*/

typedef enum {
    ENTITY_BREDR_DEVICE = 0,   /**< BR/EDR piconet member (master or slave). */
    ENTITY_BREDR_PICONET,      /**< BR/EDR piconet identified by LAP.        */
    ENTITY_BLE_DEVICE,         /**< LE advertiser (broadcasting device).     */
    ENTITY_BLE_PICONET,        /**< LE connection (identified by AA).        */
} entity_kind_t;

/* ---------------------------------------------------------------------------
 * Shared field block (first member of every snapshot struct)
 * ---------------------------------------------------------------------------*/

/** Address string buffer (e.g. "0x123456" or "A1:B2:C3:D4:E5:F6"). */
#define DEVICE_ADDR_STR_MAX 24u
/** Display label buffer (e.g. "Central", "LT_ADDR 2", "piconet", "Advertiser"). */
#define DEVICE_LABEL_MAX    24u
/** BLE local name buffer (truncated if longer). */
#define DEVICE_NAME_MAX     64u
/** BLE advertiser address-type buffer (PUBLIC/STATIC/RESOLVABLE/...). */
#define DEVICE_ADDRTYPE_MAX 16u
/** BLE manufacturer (Company ID -> name) buffer. */
#define DEVICE_MANUF_MAX    32u

#define ENTITY_COMMON_FIELDS \
    uint64_t       id;            /**< device_id or piconet_id (stable) */ \
    entity_kind_t  kind;          /**< one of entity_kind_t */ \
    char           addr_str[DEVICE_ADDR_STR_MAX]; \
    char           label[DEVICE_LABEL_MAX]; \
    float          rssi_db;       /**< EMA / most-recent signal */ \
    int            rssi_valid;    /**< 0 => no signal yet (GUI sentinel) */ \
    uint64_t       first_seen_ms; /**< epoch ms, 0 = unknown */ \
    uint64_t       last_seen_ms;  /**< epoch ms, 0 = unknown */ \
    unsigned long  total_packets; /**< packets attributed to this entity */ \
    unsigned int   packet_rate    /**< core-computed packets/sec window */

/* ---------------------------------------------------------------------------
 * BR/EDR device (piconet member)
 * ---------------------------------------------------------------------------*/

typedef struct {
    ENTITY_COMMON_FIELDS;

    uint8_t  lt_addr;       /**< 255 = master; 0..7 = slave slot */
    uint32_t lap;           /**< 24-bit Lower Address Part */
    uint8_t  uap;           /**< valid iff uap_found */
    int      uap_found;

    uint64_t piconet_id;    /**< owning piconet (linkage) */
} bredr_device_snapshot_t;

/* ---------------------------------------------------------------------------
 * BR/EDR piconet (network / link)
 * ---------------------------------------------------------------------------*/

typedef struct {
    ENTITY_COMMON_FIELDS;

    uint32_t lap;           /**< 24-bit Lower Address Part */
    uint8_t  uap;           /**< valid iff uap_found */
    int      uap_found;

    int      clk_known;             /**< nonzero once CLK1-6 established */
    uint8_t  central_clk_1_6;       /**< best central CLK1-6 (0..63) */
    uint32_t last_successful_rx_clk_1600; /**< ts of last HEC-verified pkt */
    int      tracking_state;        /**< clock confidence (-1..5) */

    float    master_rssi;           /**< RSSI for master (CLK1 == 0) */
    int      master_rssi_seen;
    float    slave_rssi[8];         /**< RSSI per slave LT_ADDR (0..7) */
    int      slave_rssi_seen[8];

    int      combined_rssi_seen;    /**< nonzero once any member RSSI seen */
    float    combined_rssi;         /**< combined (master+slave) RSSI */

    uint64_t master_device_id;      /**< linkage to member device */
    uint64_t slave_device_id[8];    /**< linkage to member devices */
} bredr_piconet_snapshot_t;

/* ---------------------------------------------------------------------------
 * LE device (advertiser)
 * ---------------------------------------------------------------------------*/

typedef struct {
    ENTITY_COMMON_FIELDS;

    uint64_t adv_addr;      /**< 48-bit advertiser address */
    char     addr_type[DEVICE_ADDRTYPE_MAX];
    char     name[DEVICE_NAME_MAX];        /**< LE local name (if known) */
    char     manufacturer[DEVICE_MANUF_MAX]; /**< AD Company ID name (if known) */
} ble_device_snapshot_t;

/* ---------------------------------------------------------------------------
 * LE piconet (connection)
 * ---------------------------------------------------------------------------*/

typedef struct {
    ENTITY_COMMON_FIELDS;

    uint32_t access_address;       /**< 32-bit connection access address */
    int      crc_init_confirmed;   /**< nonzero once CRCInit proven */
    uint32_t crc_init;             /**< confirmed CRCInit */
    int      state;                /**< collecting / confirmed */
    unsigned int candidate_count;  /**< distinct CRCInit candidates accumulated */

    uint64_t device_id_master;     /**< linkage to advertiser device */
    uint64_t device_id_slave;      /**< linkage to advertiser device */
} ble_piconet_snapshot_t;

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_MODELS_H */
