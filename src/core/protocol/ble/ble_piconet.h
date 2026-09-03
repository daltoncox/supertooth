/**
 * @file ble_piconet.h
 * @brief Per-connection (access-address-keyed) BLE tracking with CRC gating.
 *
 * Overview
 * --------
 * Data-channel packets cannot be matched on their access address up front
 * (each connection owns a random one) and a bare preamble is far too common
 * to accept on its own.  Acceptance therefore requires a confirmed CRC,
 * which in turn requires the connection's 24-bit CRCInit.
 *
 * A `ble_piconet_store_t` tracks connections by access address and drives
 * CRCInit recovery:
 *
 *  - A 0-length data packet for an AA yields a CRCInit *candidate* by
 *    reversing the CRC (see ble_crc_reverse_init_len2()).
 *  - CONNECT_IND PDUs decoded on advertising channels also seed candidates
 *    (ble_piconet_store_seed_candidate()) — but no candidate is trusted:
 *    it must prove itself by verifying a non-0-length packet's CRC.
 *  - The first candidate that verifies a packet becomes the confirmed
 *    CRCInit for that AA; recovery then stops for that AA.
 *
 * Threading
 * ---------
 * Unlike the BR/EDR piconet store (whose callers hold a session mutex),
 * this store owns an internal mutex: the CRC gate runs deep inside the
 * bitstream decoder's per-bit path where no caller can wrap the access.
 * All API functions lock internally; the gate is atomic.
 */

#ifndef BLE_PICONET_H
#define BLE_PICONET_H

#include <pthread.h>
#include <stdint.h>

#include "ble_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/

/** Maximum number of tracked connections (LRU eviction beyond this). */
#define BLE_PICONET_STORE_CAPACITY 512u
/** CRCInit candidates kept per access address (dedup, FIFO). */
#define BLE_PICONET_MAX_CANDIDATES 4u
/** Pending (not-yet-promoted) access addresses tallied before they earn a
 *  store slot (promote-on-promise gating). Bounded to bound memory. */
#define BLE_PICONET_PENDING_CAP 1024u
/** An access address must be seen this many times (or seed a CRCInit
 *  candidate via CONNECT_IND) before it may occupy a store entry. */
#define BLE_PICONET_PROMOTE_THRESHOLD 3u

/* ---------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------------*/

typedef enum
{
    BLE_PICONET_COLLECTING = 0,  /* candidates only, none proven */
    BLE_PICONET_CONFIRMED = 1,   /* crc_init proven on a real packet */
} ble_piconet_state_t;

typedef struct
{
    uint32_t access_address;
    ble_piconet_state_t state;
    uint32_t crc_init;           /* valid when state == BLE_PICONET_CONFIRMED */
    uint32_t candidates[BLE_PICONET_MAX_CANDIDATES];
    unsigned int candidate_count;
    unsigned long packets_seen;
    unsigned long packets_accepted;
    uint64_t last_used_seq;      /* LRU clock */
} ble_piconet_t;

/** Lightweight tally for an access address not yet promoted to a store entry.
 *  Counts raw frames so one-off junk never consumes a slot. */
typedef struct
{
    uint32_t aa;
    unsigned long count;
    uint64_t seq;   /**< LRU clock for pending eviction */
} ble_piconet_pending_t;

typedef struct ble_piconet_store
{
    ble_piconet_t entries[BLE_PICONET_STORE_CAPACITY];
    unsigned int count;
    uint64_t seq;
    ble_piconet_pending_t pending[BLE_PICONET_PENDING_CAP];
    unsigned int pending_count;
    uint64_t pending_seq;
    pthread_mutex_t lock;
    unsigned int initialized;
} ble_piconet_store_t;

typedef enum
{
    BLE_GATE_REJECT = 0,
    BLE_GATE_ACCEPT = 1,
} ble_gate_result_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_piconet_store_init(ble_piconet_store_t *store);
void ble_piconet_store_free(ble_piconet_store_t *store);

/**
 * The complete CRC gate for a captured data-channel candidate, atomically:
 *
 *  1. find (or create, LRU-evicting) the entry for @p access_address;
 *  2. confirmed CRCInit  -> accept iff it verifies the packet;
 *  3. no confirmed CRCInit, len == 0
 *                       -> reverse the CRC into a new candidate and reject
 *                          (a 0-length packet can produce a candidate but
 *                          never prove one);
 *  4. candidates, len > 0
 *                       -> the first candidate that verifies the packet is
 *                          confirmed and the packet is accepted;
 *  5. otherwise         -> reject.
 *
 * @param pdu         Dewhitened PDU bytes (header + payload, CRC excluded).
 * @param pdu_bytes   2 + payload_len.
 * @param rx_crc      Received CRC (ble_extract_crc() form).
 * @param crc_init_used_out  On BLE_GATE_ACCEPT, the CRCInit that verified
 *                           the packet. May be NULL.
 */
ble_gate_result_t ble_piconet_store_gate_data_pdu(
        ble_piconet_store_t *store, uint32_t access_address,
        const uint8_t *pdu, unsigned int pdu_bytes,
        uint32_t rx_crc, uint32_t *crc_init_used_out);

/** Seed a CRCInit candidate from a decoded CONNECT_IND. No-op when the AA
 * is already confirmed (the candidate still has to prove itself). */
void ble_piconet_store_seed_candidate(ble_piconet_store_t *store,
                                      uint32_t access_address,
                                      uint32_t crc_init);

/** Force-confirm an AA/CRCInit pair (test setup). */
void ble_piconet_store_confirm(ble_piconet_store_t *store,
                               uint32_t access_address,
                               uint32_t crc_init);

unsigned int ble_piconet_store_count(const ble_piconet_store_t *store);

/** Snapshot entry @p index (0..count-1) into @p out. Returns 0 on success. */
int ble_piconet_store_get(ble_piconet_store_t *store,
                          unsigned int index,
                          ble_piconet_t *out);

/** Snapshot the entry for @p access_address into @p out.
 * Returns 0 when found, -1 otherwise. */
int ble_piconet_store_find(ble_piconet_store_t *store,
                           uint32_t access_address,
                           ble_piconet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_PICONET_H */
