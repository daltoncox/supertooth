/**
 * @file ble_piconet.c
 * @brief Per-connection BLE tracking with CRC-gated CRCInit recovery.
 *
 * See ble_piconet.h for the design overview and threading notes.
 */

#include "ble_piconet.h"

#include <string.h>

/* ---------------------------------------------------------------------------
 * Internal helpers (store lock must be held)
 * ---------------------------------------------------------------------------*/

static ble_piconet_t *ble_piconet_find(ble_piconet_store_t *store, uint32_t aa)
{
    for (unsigned int i = 0; i < store->count; i++)
        if (store->entries[i].access_address == aa)
            return &store->entries[i];
    return NULL;
}

/* Pick the eviction victim: the least "promising" entry, so that real
 * connections survive long enough for their CRCInit to confirm while junk
 * access addresses (misread AAs that never recur) churn out instead.
 *
 * Confirmed entries are never evicted (a confirmed CRCInit is expensive to
 * relearn). Among unconfirmed entries the eviction priority is:
 *   1. no CRCInit candidate  <  has candidate      (junk first)
 *   2. fewer packets_seen    <  more packets_seen  (one-off first)
 *   3. older last_used_seq   <  newer last_used_seq (LRU tiebreak)
 * A real connection quickly gains a candidate (from a 0-length packet or a
 * CONNECT_IND) and/or accumulates packets, so it is retained while pure junk
 * (1 packet, no candidate) is discarded. */
static unsigned int ble_piconet_victim_index(const ble_piconet_store_t *store)
{
    unsigned int best = 0u;
    uint64_t best_seq = UINT64_MAX;
    unsigned int best_pkts = 0u;
    int best_has_cand = 1;

    for (unsigned int i = 0; i < store->count; i++)
    {
        const ble_piconet_t *e = &store->entries[i];
        if (e->state == BLE_PICONET_CONFIRMED)
            continue; /* never evict a confirmed connection */

        int has_cand = (e->candidate_count > 0u) ? 1 : 0;
        unsigned int pkts = (unsigned int)e->packets_seen;

        int better = 0;
        if (has_cand != best_has_cand)
        {
            /* Prefer to evict the entry without a candidate. */
            if (best_has_cand && !has_cand)
                better = 1;
        }
        else if (pkts != best_pkts)
        {
            if (pkts < best_pkts)
                better = 1;
        }
        else if (e->last_used_seq < best_seq)
        {
            better = 1;
        }

        if (best_seq == UINT64_MAX || better)
        {
            best = i;
            best_seq = e->last_used_seq;
            best_pkts = pkts;
            best_has_cand = has_cand;
        }
    }

    /* Every entry confirmed (only possible if the table is entirely confirmed
     * and still full): fall back to LRU over all entries. */
    if (best_seq == UINT64_MAX)
    {
        for (unsigned int i = 0; i < store->count; i++)
            if (store->entries[i].last_used_seq < best_seq)
            {
                best = i;
                best_seq = store->entries[i].last_used_seq;
            }
    }
    return best;
}

/* Pending-tally helpers (store lock must be held). */

static int ble_piconet_pending_index(const ble_piconet_store_t *store,
                                     uint32_t aa)
{
    for (unsigned int i = 0; i < store->pending_count; i++)
        if (store->pending[i].aa == aa)
            return (int)i;
    return -1;
}

static void ble_piconet_pending_remove(ble_piconet_store_t *store,
                                       unsigned int idx)
{
    if (idx >= store->pending_count)
        return;
    store->pending[idx] = store->pending[store->pending_count - 1u];
    store->pending_count--;
}

/* Record one more raw frame for an as-yet-unpromoted access address, evicting
 * the LRU pending entry when the tally is full. Returns the new count. */
static unsigned long ble_piconet_pending_bump(ble_piconet_store_t *store,
                                              uint32_t aa)
{
    int idx = ble_piconet_pending_index(store, aa);
    if (idx >= 0)
    {
        store->pending[idx].count++;
        store->pending[idx].seq = ++store->pending_seq;
        return store->pending[idx].count;
    }

    if (store->pending_count >= BLE_PICONET_PENDING_CAP)
    {
        unsigned int victim = 0u;
        uint64_t vseq = UINT64_MAX;
        for (unsigned int i = 0; i < store->pending_count; i++)
            if (store->pending[i].seq < vseq)
            {
                vseq = store->pending[i].seq;
                victim = i;
            }
        ble_piconet_pending_remove(store, victim);
    }

    ble_piconet_pending_t *p = &store->pending[store->pending_count++];
    p->aa = aa;
    p->count = 1u;
    p->seq = ++store->pending_seq;
    return 1u;
}

/* Allocate a fresh store entry (LRU-evicting if the table is full). The
 * caller must hold the store lock. */
static ble_piconet_t *ble_piconet_create_entry(ble_piconet_store_t *store,
                                               uint32_t aa)
{
    ble_piconet_t *entry = ble_piconet_find(store, aa);
    if (entry)
        return entry;

    if (store->count < BLE_PICONET_STORE_CAPACITY)
        entry = &store->entries[store->count++];
    else
        entry = &store->entries[ble_piconet_victim_index(store)];

    memset(entry, 0, sizeof(*entry));
    entry->access_address = aa;
    entry->state = BLE_PICONET_COLLECTING;
    return entry;
}

/* Acquire the store entry for @p aa under promote-on-promise gating:
 *  - an existing entry is returned as-is;
 *  - @p force (a CONNECT_IND seed / explicit confirm) promotes immediately;
 *  - otherwise a brand-new AA must accumulate BLE_PICONET_PROMOTE_THRESHOLD
 *    raw frames (tracked in the pending tally) before it earns a slot. A
 *    one-off misread access address therefore never occupies a store entry,
 *    so real connections are not evicted by noise. */
static ble_piconet_t *ble_piconet_acquire(ble_piconet_store_t *store,
                                          uint32_t aa, int force)
{
    ble_piconet_t *entry = ble_piconet_find(store, aa);
    if (entry)
        return entry;

    if (force)
    {
        int idx = ble_piconet_pending_index(store, aa);
        if (idx >= 0)
            ble_piconet_pending_remove(store, (unsigned int)idx);
        return ble_piconet_create_entry(store, aa);
    }

    unsigned long cnt = ble_piconet_pending_bump(store, aa);
    if (cnt >= (unsigned long)BLE_PICONET_PROMOTE_THRESHOLD)
    {
        int idx = ble_piconet_pending_index(store, aa);
        if (idx >= 0)
            ble_piconet_pending_remove(store, (unsigned int)idx);
        return ble_piconet_create_entry(store, aa);
    }
    return NULL;
}

static void ble_piconet_add_candidate(ble_piconet_t *entry, uint32_t crc_init)
{
    crc_init &= 0xFFFFFFu;

    for (unsigned int i = 0; i < entry->candidate_count; i++)
        if (entry->candidates[i] == crc_init)
            return;

    if (entry->candidate_count < BLE_PICONET_MAX_CANDIDATES)
    {
        entry->candidates[entry->candidate_count++] = crc_init;
        return;
    }

    /* FIFO: drop the oldest unproven candidate. */
    memmove(&entry->candidates[0], &entry->candidates[1],
            (BLE_PICONET_MAX_CANDIDATES - 1u) * sizeof(entry->candidates[0]));
    entry->candidates[BLE_PICONET_MAX_CANDIDATES - 1u] = crc_init;
}

static void ble_piconet_confirm_entry(ble_piconet_t *entry, uint32_t crc_init)
{
    entry->state = BLE_PICONET_CONFIRMED;
    entry->crc_init = crc_init & 0xFFFFFFu;
    entry->candidate_count = 0u;
}

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_piconet_store_init(ble_piconet_store_t *store)
{
    if (!store)
        return;

    memset(store, 0, sizeof(*store));
    pthread_mutex_init(&store->lock, NULL);
    store->initialized = 1u;
    ble_crc_tables_init();
}

void ble_piconet_store_free(ble_piconet_store_t *store)
{
    if (!store || !store->initialized)
        return;

    pthread_mutex_destroy(&store->lock);
    memset(store, 0, sizeof(*store));
}

ble_gate_result_t ble_piconet_store_gate_data_pdu(
        ble_piconet_store_t *store, uint32_t access_address,
        const uint8_t *pdu, unsigned int pdu_bytes,
        uint32_t rx_crc, uint32_t *crc_init_used_out)
{
    if (!store || !store->initialized || !pdu || pdu_bytes < 2u)
        return BLE_GATE_REJECT;

    ble_gate_result_t result = BLE_GATE_REJECT;
    uint32_t used = 0u;

    pthread_mutex_lock(&store->lock);
    ble_crc_tables_init();   /* idempotent; safe under the store lock */

    ble_piconet_t *entry = ble_piconet_acquire(store, access_address, 0);
    if (!entry)
    {
        /* Not yet promoted (one-off / unconfirmed) — drop without consuming a
         * store slot. The pending tally was bumped inside acquire(). */
        pthread_mutex_unlock(&store->lock);
        return BLE_GATE_REJECT;
    }
    entry->packets_seen++;
    entry->last_used_seq = ++store->seq;

    if (entry->state == BLE_PICONET_CONFIRMED)
    {
        if (ble_crc_calc(pdu, pdu_bytes, entry->crc_init) ==
            (rx_crc & 0xFFFFFFu))
        {
            entry->packets_accepted++;
            used = entry->crc_init;
            result = BLE_GATE_ACCEPT;
        }
        goto done;
    }

    if (pdu_bytes == 2u)
    {
        /* 0-length LL data PDU: reverse the CRC into a CRCInit candidate.
         * A 0-length packet can produce a candidate but never prove one,
         * so it is always rejected until a CRCInit is confirmed. */
        ble_piconet_add_candidate(entry,
                                  ble_crc_reverse_init_len2(pdu, rx_crc));
        goto done;
    }

    for (unsigned int i = 0; i < entry->candidate_count; i++)
    {
        if (ble_crc_calc(pdu, pdu_bytes, entry->candidates[i]) ==
            (rx_crc & 0xFFFFFFu))
        {
            /* A candidate proved itself on a real packet: keep it. */
            used = entry->candidates[i];
            ble_piconet_confirm_entry(entry, used);
            entry->packets_accepted++;
            result = BLE_GATE_ACCEPT;
            goto done;
        }
    }

done:
    pthread_mutex_unlock(&store->lock);

    if (result == BLE_GATE_ACCEPT && crc_init_used_out)
        *crc_init_used_out = used;
    return result;
}

void ble_piconet_store_seed_candidate(ble_piconet_store_t *store,
                                      uint32_t access_address,
                                      uint32_t crc_init)
{
    if (!store || !store->initialized)
        return;

    pthread_mutex_lock(&store->lock);
    ble_piconet_t *entry = ble_piconet_acquire(store, access_address, 1);
    if (!entry)
    {
        pthread_mutex_unlock(&store->lock);
        return;
    }
    entry->last_used_seq = ++store->seq;
    if (entry->state != BLE_PICONET_CONFIRMED)
        ble_piconet_add_candidate(entry, crc_init);
    pthread_mutex_unlock(&store->lock);
}

void ble_piconet_store_confirm(ble_piconet_store_t *store,
                               uint32_t access_address,
                               uint32_t crc_init)
{
    if (!store || !store->initialized)
        return;

    pthread_mutex_lock(&store->lock);
    ble_piconet_t *entry = ble_piconet_acquire(store, access_address, 1);
    if (!entry)
    {
        pthread_mutex_unlock(&store->lock);
        return;
    }
    entry->last_used_seq = ++store->seq;
    ble_piconet_confirm_entry(entry, crc_init);
    pthread_mutex_unlock(&store->lock);
}

unsigned int ble_piconet_store_count(const ble_piconet_store_t *store)
{
    if (!store || !store->initialized)
        return 0u;

    return store->count;
}

int ble_piconet_store_get(ble_piconet_store_t *store,
                          unsigned int index,
                          ble_piconet_t *out)
{
    if (!store || !store->initialized || !out || index >= store->count)
        return -1;

    pthread_mutex_lock(&store->lock);
    if (index >= store->count)
    {
        pthread_mutex_unlock(&store->lock);
        return -1;
    }
    memcpy(out, &store->entries[index], sizeof(*out));
    pthread_mutex_unlock(&store->lock);
    return 0;
}

int ble_piconet_store_find(ble_piconet_store_t *store,
                           uint32_t access_address,
                           ble_piconet_t *out)
{
    if (!store || !store->initialized || !out)
        return -1;

    pthread_mutex_lock(&store->lock);
    ble_piconet_t *entry = ble_piconet_find(store, access_address);
    if (entry)
        memcpy(out, entry, sizeof(*out));
    pthread_mutex_unlock(&store->lock);
    return entry ? 0 : -1;
}
