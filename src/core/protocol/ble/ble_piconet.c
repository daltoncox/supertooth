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

/* Pick the eviction victim: the least-recently-used entry, preferring
 * unconfirmed ones (a confirmed CRCInit is expensive to relearn). */
static unsigned int ble_piconet_victim_index(const ble_piconet_store_t *store)
{
    unsigned int best = 0u;
    uint64_t best_seq = UINT64_MAX;

    for (unsigned int i = 0; i < store->count; i++)
    {
        if (store->entries[i].state != BLE_PICONET_CONFIRMED &&
            store->entries[i].last_used_seq < best_seq)
        {
            best = i;
            best_seq = store->entries[i].last_used_seq;
        }
    }
    if (best_seq != UINT64_MAX)
        return best;

    for (unsigned int i = 0; i < store->count; i++)
        if (store->entries[i].last_used_seq < best_seq)
        {
            best = i;
            best_seq = store->entries[i].last_used_seq;
        }
    return best;
}

static ble_piconet_t *ble_piconet_find_or_create(ble_piconet_store_t *store,
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

    ble_piconet_t *entry = ble_piconet_find_or_create(store, access_address);
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
    ble_piconet_t *entry = ble_piconet_find_or_create(store, access_address);
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
    ble_piconet_t *entry = ble_piconet_find_or_create(store, access_address);
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
