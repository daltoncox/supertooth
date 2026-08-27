/**
 * @file bredr_piconet_store.c
 * @brief Dynamic piconet store with repository-owned BR/EDR recovery plumbing.
 */

#include "bredr_piconet_store.h"
#include "bredr_codec.h"
#include "bredr_bitstream_decoder.h"
#include "bredr_clock_recovery.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Reset recovery state if a LAP is idle for too long.
 *  CLKN ticks at 312.5 µs, so 16384 ticks ≈ 5.1 s. */
#define BTBB_LAP_IDLE_RESET_CLKN 16384u

/* ---------------------------------------------------------------------------
 * Frame dump forwarding
 *
 * The actual frame recording lives in the recovery module
 * (bredr_clock_recovery.c); the store merely forwards the FILE handle.
 * --------------------------------------------------------------------------- */

void bredr_piconet_store_set_frame_dump(bredr_piconet_store_t *store, FILE *file)
{
    (void)store;
    bredr_recovery_set_frame_dump(file);
}

/* ---------------------------------------------------------------------------
 * Internal entry struct
 * ---------------------------------------------------------------------------*/

struct bredr_piconet_store_entry
{
    bredr_piconet_t *pnet;
    uint32_t last_clkn;
    int has_last_clkn;
};

/* ---------------------------------------------------------------------------
 * Store helpers
 * ---------------------------------------------------------------------------*/

static bredr_piconet_store_entry_t *find_entry(bredr_piconet_store_t *store,
                                               uint32_t lap)
{
    for (size_t i = 0; i < store->count; i++)
    {
        if ((store->entries[i].pnet->lap & 0xFFFFFFu) == lap)
            return &store->entries[i];
    }
    return NULL;
}

static bredr_piconet_store_entry_t *create_entry(bredr_piconet_store_t *store,
                                                 uint32_t lap)
{
    if (store->count >= store->capacity)
    {
        size_t new_cap = store->capacity * 2u;
        bredr_piconet_store_entry_t *resized =
            (bredr_piconet_store_entry_t *)realloc(
                store->entries, new_cap * sizeof(*resized));
        if (!resized)
            return NULL;
        store->entries = resized;
        store->capacity = new_cap;
    }

    bredr_piconet_t *pnet = (bredr_piconet_t *)malloc(sizeof(bredr_piconet_t));
    if (!pnet)
        return NULL;

    bredr_piconet_init(pnet, lap);

    bredr_piconet_store_entry_t *entry = &store->entries[store->count++];
    entry->pnet = pnet;
    entry->last_clkn = 0u;
    entry->has_last_clkn = 0;
    return entry;
}

/* ---------------------------------------------------------------------------
 * Time conversion helpers
 * ---------------------------------------------------------------------------*/

static uint32_t sample_to_clkn(uint64_t radio_start_sample_index,
                                unsigned int radio_sample_rate_hz)
{
    if (radio_sample_rate_hz == 0u)
        return 0u;

    uint64_t num = radio_start_sample_index * 3200u +
                   (uint64_t)(radio_sample_rate_hz / 2u);
    return (uint32_t)(num / (uint64_t)radio_sample_rate_hz);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

void bredr_piconet_store_init(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    memset(store, 0, sizeof(*store));
    store->capacity = BREDR_PICONET_STORE_INIT_CAP;
    store->entries = (bredr_piconet_store_entry_t *)calloc(
        store->capacity, sizeof(*store->entries));
}

void bredr_piconet_store_free(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    for (size_t i = 0; i < store->count; i++)
    {
        free(store->entries[i].pnet);
    }

    free(store->entries);
    memset(store, 0, sizeof(*store));
}

bredr_piconet_t *bredr_piconet_store_add_packet(bredr_piconet_store_t *store,
                                                const bredr_event_t *event,
                                                int *packet_is_newest_out)
{
    int packet_is_newest = 0;
    unsigned int radio_sample_rate_hz;

    if (!store || !event || !store->entries)
        return NULL;
    const bredr_frame_t *frame = &event->frame;
    radio_sample_rate_hz = event->meta.radio_sample_rate_hz;
    if (radio_sample_rate_hz == 0u)
        return NULL;

    uint32_t clkn = sample_to_clkn(event->meta.radio_start_sample_index,
                                    radio_sample_rate_hz);

    uint32_t lap = frame->lap & 0xFFFFFFu;

    bredr_piconet_store_entry_t *entry = find_entry(store, lap);
    if (!entry)
        entry = create_entry(store, lap);
    if (!entry)
        return NULL;

    /* Idle reset: if this LAP has been silent long enough, restart the
     * recovery state so UAP recovery begins fresh when it reappears.
     *
     * Guard against occasional non-monotonic timestamp regressions by only
     * applying the idle test when clkn advances. */
    if (entry->has_last_clkn)
    {
        if (clkn >= entry->last_clkn)
        {
            uint32_t idle = clkn - entry->last_clkn;
            if (idle > BTBB_LAP_IDLE_RESET_CLKN)
                bredr_recovery_reset(entry->pnet);

            entry->last_clkn = clkn;
        }
    }
    else
        entry->last_clkn = clkn;

    entry->has_last_clkn = 1;

    packet_is_newest = bredr_piconet_add_packet(entry->pnet, event);

    if (packet_is_newest_out)
        *packet_is_newest_out = packet_is_newest;

    return entry->pnet;
}

size_t bredr_piconet_store_count(const bredr_piconet_store_t *store)
{
    return store ? store->count : 0u;
}

const bredr_piconet_t *bredr_piconet_store_get(const bredr_piconet_store_t *store,
                                               size_t index)
{
    if (!store || index >= store->count || !store->entries)
        return NULL;
    return store->entries[index].pnet;
}
