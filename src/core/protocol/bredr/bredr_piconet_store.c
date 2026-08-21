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
 * Optional frame dump for offline recovery replay (Phase 3)
 *
 * When enabled, every frame fed to UAP acquisition is appended to an open FILE
 * in a simple binary format so it can be replayed through the recovery backend
 * from a real capture.  The format is intentionally trivial and self-describing
 * so a replay tool/test can validate it without parsing the radio stack.
 * --------------------------------------------------------------------------- */

#define FRAME_DUMP_MAGIC   0x53544C44u /* "STLD" */
#define FRAME_DUMP_VERSION 1u

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t lap;
    int32_t channel;
    uint32_t clkn;
    uint64_t header_raw;
    uint32_t air_payload_bits;
    uint8_t air_payload[BR_MAX_AIR_PAYLOAD_BYTES];
} frame_dump_rec_t;

static FILE *g_frame_dump = NULL;

void bredr_piconet_store_set_frame_dump(bredr_piconet_store_t *store, FILE *file)
{
    (void)store;
    g_frame_dump = file;
}

/* ---------------------------------------------------------------------------
 * Internal entry struct
 * ---------------------------------------------------------------------------*/

struct bredr_piconet_store_entry
{
    bredr_piconet_t *pnet;
    bredr_recovery_state_t *recovery;
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

    bredr_recovery_state_t *recovery = bredr_recovery_state_create(lap);
    if (!recovery)
        fprintf(stderr, "[bredr_piconet_store] OOM: UAP recovery disabled for LAP 0x%06x\n", lap);

    bredr_piconet_init(pnet, lap);

    bredr_piconet_store_entry_t *entry = &store->entries[store->count++];
    entry->pnet = pnet;
    entry->recovery = recovery;
    entry->last_clkn = 0u;
    entry->has_last_clkn = 0;
    return entry;
}

/* ---------------------------------------------------------------------------
 * UAP/clock acquisition
 * ---------------------------------------------------------------------------*/

static uint32_t sample_to_ticks(uint64_t radio_start_sample_index,
                                unsigned int radio_sample_rate_hz,
                                unsigned int ticks_per_second)
{
    if (radio_sample_rate_hz == 0u)
        return 0u;

    uint64_t num = radio_start_sample_index * (uint64_t)ticks_per_second +
                   (uint64_t)(radio_sample_rate_hz / 2u);
    return (uint32_t)(num / (uint64_t)radio_sample_rate_hz);
}

static uint32_t sample_to_rx_clk_1600(uint64_t radio_start_sample_index,
                                      unsigned int radio_sample_rate_hz)
{
    return sample_to_ticks(radio_start_sample_index, radio_sample_rate_hz, 1600u);
}

static uint32_t sample_to_clkn(uint64_t radio_start_sample_index,
                               unsigned int radio_sample_rate_hz)
{
    return sample_to_ticks(radio_start_sample_index, radio_sample_rate_hz, 3200u);
}

/** Maximum age (rx_clk_1600 ticks) for historical packets used in CLK1-6
 *  narrowing.  625 µs × 8000 ≈ 5 seconds. */
#define CLK1_6_HISTORY_CUTOFF_CLK1600 8000u

/**
 * Narrows a list of CLK1-6 candidates using the piconet's historical packets.
 *
 * Iterates backwards through the ring buffer (most-recent-first, skipping the
 * current packet which was just added), eliminating any candidate whose
 * time-adjusted CLK1-6 does not produce a valid HEC on that historical packet.
 *
 * Stops early when only one candidate remains, no more usable history exists,
 * or a packet older than CLK1_6_HISTORY_CUTOFF_CLK1600 ticks is reached.
 *
 * @param pnet        Piconet whose queue[] to search.
 * @param cur_pkt     The packet that seeded the candidate list.
 * @param uap         UAP to use for HEC checks.
 * @param candidates  In/out: candidate CLK1-6 values; filtered in-place.
 * @param n           Number of candidates on entry.
 * @return            Number of surviving candidates.
 */
static int narrow_clk6_candidates(const bredr_piconet_t *pnet,
                                  const bredr_event_t *cur_event,
                                  uint8_t uap,
                                  int candidates[64],
                                  int n)
{
    uint32_t cur_clk;

    if (n <= 1 || pnet->queue_fill < 2)
        return n;

    cur_clk = sample_to_rx_clk_1600(cur_event->meta.radio_start_sample_index,
                                    cur_event->meta.radio_sample_rate_hz);

    /* The queue is maintained in start-sample order. The current packet is at
     * the logical tail, so walk backwards through older history and stop once
     * packets fall outside the history window. */
    for (unsigned int i = 1; i < pnet->queue_fill; i++)
    {
        unsigned int idx =
            (pnet->queue_head + BREDR_PICONET_QUEUE_SIZE - 1u - i) % BREDR_PICONET_QUEUE_SIZE;
        const bredr_event_t *hist_event;
        const bredr_frame_t *hist_frame;
        uint32_t hist_clk;

        hist_event = &pnet->queue[idx];
        hist_frame = &hist_event->frame;
        hist_clk = sample_to_rx_clk_1600(hist_event->meta.radio_start_sample_index,
                         hist_event->meta.radio_sample_rate_hz);

        /* Older entries only get farther away in time as we walk backward. */
        if ((cur_clk - hist_clk) > CLK1_6_HISTORY_CUTOFF_CLK1600)
            break;

        /* Skip packets without a decodable header inside configured AC tolerance. */
        if (!hist_frame->has_header || hist_frame->ac_errors > BREDR_AC_ERRORS_DEFAULT)
            continue;

        /* CLK1-6 advances one tick per rx_clk_1600 slot.  The CLK1-6 at the
         * historical packet is: (c_current - delta) mod 64. */
        uint32_t delta_mod64 = (cur_clk - hist_clk) & 0x3Fu;

        int j = 0;
        for (int k = 0; k < n; k++)
        {
            uint8_t c_at_hist =
                (uint8_t)((candidates[k] - (int)delta_mod64 + 64) & 0x3F);
            if (bredr_hec_ok_for_clk6(hist_frame, uap, c_at_hist))
                candidates[j++] = candidates[k];
        }
        n = j;

        if (n <= 1)
            break;
    }

    return n;
}

static void try_uap_acquisition(bredr_piconet_store_entry_t *entry,
                                const bredr_event_t *event,
                                uint32_t clkn,
                                uint32_t rx_clk_1600)
{
    const bredr_frame_t *frame = &event->frame;
    bredr_piconet_t *pnet = entry->pnet;

    if (!pnet || !frame->has_header || frame->ac_errors > BREDR_AC_ERRORS_DEFAULT)
        return;
    if (pnet->uap_found && pnet->clk_known)
        return;
    if (!entry->recovery)
        return;

    if (g_frame_dump && frame->has_header)
    {
        frame_dump_rec_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.magic = FRAME_DUMP_MAGIC;
        rec.version = FRAME_DUMP_VERSION;
        rec.lap = frame->lap & 0xFFFFFFu;
        rec.channel = (int32_t)event->meta.channel_index;
        rec.clkn = clkn;
        rec.header_raw = frame->header_raw;
        rec.air_payload_bits = frame->air_payload_bits;
        if (rec.air_payload_bits > BR_MAX_AIR_PAYLOAD_BITS)
            rec.air_payload_bits = BR_MAX_AIR_PAYLOAD_BITS;
        unsigned int nbytes = (rec.air_payload_bits + 7u) / 8u;
        if (nbytes > sizeof(rec.air_payload))
            nbytes = sizeof(rec.air_payload);
        memcpy(rec.air_payload, frame->air_payload, nbytes);
        if (fwrite(&rec, sizeof(rec), 1u, g_frame_dump) == 1u)
            fflush(g_frame_dump);
    }

    bredr_recovery_result_t result = {0};
    if (bredr_recovery_process_packet(entry->recovery, frame, (int)event->meta.channel_index, clkn, &result))
    {
        uint8_t uap = result.uap;
        uint8_t btbb_clk6 = result.clk6_hint; /* recovery backend CLK1-6 hint */
        if (!pnet->uap_found)
            bredr_piconet_set_uap_only(pnet, uap);

        /* Collect all CLK1-6 values that produce a valid HEC for this packet. */
        int valid_clk[64];
        int valid_n = 0;
        for (int c = 0; c < 64; c++)
        {
            if (bredr_hec_ok_for_clk6(frame, uap, (uint8_t)c))
                valid_clk[valid_n++] = c;
        }

        /* Narrow the candidates using historical packets in the ring buffer. */
        valid_n = narrow_clk6_candidates(pnet, event, uap, valid_clk, valid_n);

        if (valid_n == 1)
        {
            /* Unambiguous — use directly. */
            bredr_piconet_set_uap(pnet, uap, (uint8_t)valid_clk[0],
                                  rx_clk_1600);
        }
        else if (valid_n > 1)
        {
            /* Still ambiguous after history scan — fall back to the candidate
             * closest to the recovery backend's clock-offset hint. */
            int best = valid_clk[0];
            int best_dist = 64;
            for (int i = 0; i < valid_n; i++)
            {
                int d = valid_clk[i] - (int)btbb_clk6;
                if (d < 0)
                    d = -d;
                if (d > 32)
                    d = 64 - d;
                if (d < best_dist)
                {
                    best_dist = d;
                    best = valid_clk[i];
                }
            }
            bredr_piconet_set_uap(pnet, uap, (uint8_t)best, rx_clk_1600);
        }
        /* valid_n == 0: UAP may be wrong — leave state unchanged and let the
         * recovery backend accumulate more packets before trying again. */
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

void bredr_piconet_store_set_rssi_averaging(bredr_piconet_store_t *store,
                                            unsigned int window)
{
    if (!store)
        return;

    store->rssi_avg_window = window;
    if (window == 0u)
    {
        store->rssi_avg_alpha = 1.0f;
        store->rssi_avg_one_minus_alpha = 0.0f;
    }
    else
    {
        store->rssi_avg_alpha = 2.0f / ((float)window + 1.0f);
        store->rssi_avg_one_minus_alpha = 1.0f - store->rssi_avg_alpha;
    }
}

void bredr_piconet_store_init(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    memset(store, 0, sizeof(*store));
    store->capacity = BREDR_PICONET_STORE_INIT_CAP;
    store->entries = (bredr_piconet_store_entry_t *)calloc(
        store->capacity, sizeof(*store->entries));

    bredr_recovery_global_init(BREDR_AC_ERRORS_DEFAULT);
    bredr_piconet_store_set_rssi_averaging(store, 16u);
}

void bredr_piconet_store_free(bredr_piconet_store_t *store)
{
    if (!store)
        return;

    for (size_t i = 0; i < store->count; i++)
    {
        bredr_recovery_state_destroy(store->entries[i].recovery);
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
    uint32_t rx_clk_1600 = sample_to_rx_clk_1600(event->meta.radio_start_sample_index,
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
    if (entry->has_last_clkn && entry->recovery)
    {
        if (clkn >= entry->last_clkn)
        {
            uint32_t idle = clkn - entry->last_clkn;
            if (idle > BTBB_LAP_IDLE_RESET_CLKN)
                bredr_recovery_state_reset(entry->recovery, lap);

            entry->last_clkn = clkn;
        }
    }
    else
        entry->last_clkn = clkn;

    entry->has_last_clkn = 1;

    packet_is_newest = bredr_piconet_add_packet(entry->pnet, event,
                                                store->rssi_avg_window,
                                                store->rssi_avg_alpha,
                                                store->rssi_avg_one_minus_alpha);
    if (packet_is_newest)
        try_uap_acquisition(entry, event, clkn, rx_clk_1600);

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
