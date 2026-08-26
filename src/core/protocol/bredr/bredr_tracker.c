/**
 * @file bredr_tracker.c
 * @brief BR/EDR tracker implementation (see bredr_tracker.h).
 */

#include "bredr_tracker.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>

/* ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------*/

static uint64_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000u + (uint64_t)tv.tv_usec / 1000u;
}

static uint32_t sample_to_rx_clk_1600(uint64_t radio_start_sample_index,
                                      unsigned int rate)
{
    if (rate == 0u)
        return 0u;
    uint64_t num = radio_start_sample_index * 1600u + (uint64_t)(rate / 2u);
    return (uint32_t)(num / (uint64_t)rate);
}

static void format_addr_bredr(char *buf, size_t n, uint32_t lap,
                              uint8_t uap, int uap_found)
{
    uint8_t used = uap;
    int known = uap_found;

    /* The General/Limited Inquiry Access Codes (GIAC 0x9E8B33, LIAC 0x9E8B00)
     * use the well-known DCI UAP (0x00); they are broadcast discovery LAPs, not
     * recovered addresses, so always render with a known UAP regardless of the
     * recovery state. */
    if (lap == 0x9E8B33u || lap == 0x9E8B00u)
    {
        used = 0x00u;
        known = 1;
    }

    if (known)
        snprintf(buf, n, "0x%02X%06X", (unsigned)used,
                 (unsigned)(lap & 0xFFFFFFu));
    else
        snprintf(buf, n, "0x??%06X", (unsigned)(lap & 0xFFFFFFu));
}

/** Convert a piconet slot-clock value (rx_clk_1600) to epoch ms. */
static uint64_t clk_to_ms(const bredr_tracker_t *t, uint32_t clk)
{
    if (!t->has_epoch_base)
        return 0u;
    int32_t d = (int32_t)(clk - t->clk_base_1600);
    return t->epoch_base_ms + (uint64_t)((int64_t)d * 625 / 1000);
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------------*/

void bredr_tracker_init(bredr_tracker_t *t)
{
    if (!t)
        return;
    memset(t, 0, sizeof(*t));
    bredr_piconet_store_init(&t->store);
    t->aux = NULL;
    t->aux_cap = 0;
    t->next_id = 1u;
    pthread_mutex_init(&t->lock, NULL);
}

void bredr_tracker_free(bredr_tracker_t *t)
{
    if (!t)
        return;
    bredr_piconet_store_free(&t->store);
    free(t->aux);
    t->aux = NULL;
    t->aux_cap = 0;
    pthread_mutex_destroy(&t->lock);
    memset(t, 0, sizeof(*t));
}

/* ---------------------------------------------------------------------------
 * Internal: aux array + id allocation
 * ---------------------------------------------------------------------------*/

/** Grow aux[] to match the store's capacity. Caller holds the lock. */
static void ensure_aux_cap(bredr_tracker_t *t)
{
    if (t->store.count <= t->aux_cap)
        return;
    size_t new_cap = t->store.capacity ? t->store.capacity
                                       : (t->store.count + 8u);
    bredr_tracker_piconet_aux_t *na =
        (bredr_tracker_piconet_aux_t *)realloc(
            t->aux, new_cap * sizeof(*na));
    if (!na)
        return;
    memset(na + t->aux_cap, 0, (new_cap - t->aux_cap) * sizeof(*na));
    t->aux = na;
    t->aux_cap = new_cap;
}

/** Assign stable ids for a piconet and its observed members. Caller holds lock. */
static void ensure_piconet_ids(bredr_tracker_t *t, size_t i)
{
    bredr_tracker_piconet_aux_t *a = &t->aux[i];
    const bredr_piconet_t *p = bredr_piconet_store_get(&t->store, i);

    if (a->piconet_id == 0u)
        a->piconet_id = t->next_id++;
    if (p->master_pkts > 0u && a->master_device_id == 0u)
        a->master_device_id = t->next_id++;
    for (int lt = 0; lt < 8; lt++)
        if (p->slave_pkts[lt] > 0u && a->slave_device_id[lt] == 0u)
            a->slave_device_id[lt] = t->next_id++;
}

/* ---------------------------------------------------------------------------
 * Ingest
 * ---------------------------------------------------------------------------*/

bredr_piconet_t *bredr_tracker_add_packet(bredr_tracker_t *t,
                                          const bredr_event_t *event,
                                          int *packet_is_newest_out)
{
    if (!t || !event)
        return NULL;

    pthread_mutex_lock(&t->lock);

    if (!t->has_epoch_base)
    {
        t->clk_base_1600 = sample_to_rx_clk_1600(
            event->meta.radio_start_sample_index,
            event->meta.radio_sample_rate_hz);
        t->epoch_base_ms = now_ms();
        t->has_epoch_base = 1;
    }

    size_t prev_count = t->store.count;
    bredr_piconet_t *p =
        bredr_piconet_store_add_packet(&t->store, event, packet_is_newest_out);
    ensure_aux_cap(t);
    if (t->store.count > prev_count && t->store.count <= t->aux_cap)
    {
        size_t new_i = t->store.count - 1u;
        if (t->aux[new_i].piconet_id == 0u)
            t->aux[new_i].piconet_id = t->next_id++;
    }

    pthread_mutex_unlock(&t->lock);
    return p;
}

/* ---------------------------------------------------------------------------
 * Poll: piconets
 * ---------------------------------------------------------------------------*/

size_t bredr_tracker_get_piconets(const bredr_tracker_t *t,
                                  bredr_piconet_snapshot_t *out,
                                  size_t max)
{
    if (!t || !out || max == 0u)
        return 0u;

    pthread_mutex_lock((pthread_mutex_t *)&t->lock);
    size_t n = 0u;
    for (size_t i = 0; i < t->store.count && n < max; i++)
    {
        ensure_piconet_ids((bredr_tracker_t *)t, i);
        const bredr_piconet_t *p = bredr_piconet_store_get(&t->store, i);

        /* When the piconet is broken out into tracked slaves, the per-member
         * (Central / LT_ADDR N) rows already represent it; suppress this
         * aggregate "piconet" row so the GUI does not surface a redundant
         * "CONN" entry alongside them. */
        int broken_out = 0;
        for (int lt = 0; lt < 8; lt++)
        {
            if (p->slave_pkts[lt] > 0u)
            {
                broken_out = 1;
                break;
            }
        }
        if (broken_out)
            continue;

        bredr_piconet_snapshot_t *s = &out[n++];
        memset(s, 0, sizeof(*s));

        s->id = t->aux[i].piconet_id;
        s->kind = ENTITY_BREDR_PICONET;
        format_addr_bredr(s->addr_str, sizeof(s->addr_str),
                          p->lap, p->uap, p->uap_found);
        /* The General/Limited Inquiry Access Codes (GIAC 0x9E8B33,
         * LIAC 0x9E8B00) are broadcast LAPs used by device discovery; label
         * them as INQUIRY rather than a piconet/connection. */
        if (p->lap == 0x9E8B33u || p->lap == 0x9E8B00u)
            snprintf(s->label, sizeof(s->label), "INQUIRY");
        else
            snprintf(s->label, sizeof(s->label), "piconet");

        /* Pull the trailing-1 s RSSI averages out of the trackers. */
        s->combined_rssi_seen =
            rssi_tracker_average(&p->combined_rssi_track, &s->combined_rssi);
        s->master_rssi_seen =
            rssi_tracker_average(&p->master_rssi_track, &s->master_rssi);
        for (int lt = 0; lt < 8; lt++)
        {
            s->slave_rssi_seen[lt] =
                rssi_tracker_average(&p->slave_rssi_track[lt],
                                     &s->slave_rssi[lt]);
        }

        /* Entity-level RSSI prefers the aggregate, then central. */
        if (s->combined_rssi_seen)
        {
            s->rssi_db = s->combined_rssi;
            s->rssi_valid = 1;
        }
        else if (s->master_rssi_seen)
        {
            s->rssi_db = s->master_rssi;
            s->rssi_valid = 1;
        }
        else
            s->rssi_valid = 0;

        s->first_seen_ms = clk_to_ms(t, p->first_seen);
        s->last_seen_ms = clk_to_ms(t, p->last_seen);
        s->total_packets = p->total_packets;
        s->packet_rate = 0u;

        s->lap = p->lap;
        s->uap = p->uap;
        s->uap_found = p->uap_found;
        s->clk_known = p->clk_known;
        s->central_clk_1_6 = bredr_piconet_central_clk_1_6(p, p->last_seen);
        s->tracking_state = p->tracking_state;


        s->master_device_id = t->aux[i].master_device_id;
        memcpy(s->slave_device_id, t->aux[i].slave_device_id,
               sizeof(s->slave_device_id));
    }
    pthread_mutex_unlock((pthread_mutex_t *)&t->lock);
    return n;
}

/* ---------------------------------------------------------------------------
 * Poll: devices (derived piconet members)
 * ---------------------------------------------------------------------------*/

size_t bredr_tracker_get_devices(const bredr_tracker_t *t,
                                 bredr_device_snapshot_t *out,
                                 size_t max)
{
    if (!t || !out || max == 0u)
        return 0u;

    pthread_mutex_lock((pthread_mutex_t *)&t->lock);
    size_t n = 0u;
    for (size_t i = 0; i < t->store.count; i++)
    {
        ensure_piconet_ids((bredr_tracker_t *)t, i);
        const bredr_piconet_t *p = bredr_piconet_store_get(&t->store, i);
        uint64_t pid = t->aux[i].piconet_id;

        if (p->master_pkts > 0u && n < max)
        {
            bredr_device_snapshot_t *s = &out[n++];
            memset(s, 0, sizeof(*s));
            s->id = t->aux[i].master_device_id;
            s->kind = ENTITY_BREDR_DEVICE;
            format_addr_bredr(s->addr_str, sizeof(s->addr_str),
                              p->lap, p->uap, p->uap_found);
            snprintf(s->label, sizeof(s->label), "Central");
            s->rssi_valid =
                rssi_tracker_average(&p->master_rssi_track, &s->rssi_db);
            s->first_seen_ms = clk_to_ms(t, p->first_seen);
            s->last_seen_ms = clk_to_ms(t, p->last_seen);
            s->total_packets = p->master_pkts;
            s->packet_rate = 0u;
            s->lt_addr = 255u;
            s->lap = p->lap;
            s->uap = p->uap;
            s->uap_found = p->uap_found;
            s->piconet_id = pid;
        }

        for (int lt = 0; lt < 8 && n < max; lt++)
        {
            if (p->slave_pkts[lt] == 0u)
                continue;
            bredr_device_snapshot_t *s = &out[n++];
            memset(s, 0, sizeof(*s));
            s->id = t->aux[i].slave_device_id[lt];
            s->kind = ENTITY_BREDR_DEVICE;
            format_addr_bredr(s->addr_str, sizeof(s->addr_str),
                              p->lap, p->uap, p->uap_found);
            snprintf(s->label, sizeof(s->label), "LT_ADDR %d", lt);
            s->rssi_valid =
                rssi_tracker_average(&p->slave_rssi_track[lt], &s->rssi_db);
            s->first_seen_ms = clk_to_ms(t, p->first_seen);
            s->last_seen_ms = clk_to_ms(t, p->last_seen);
            s->total_packets = p->slave_pkts[lt];
            s->packet_rate = 0u;
            s->lt_addr = (uint8_t)lt;
            s->lap = p->lap;
            s->uap = p->uap;
            s->uap_found = p->uap_found;
            s->piconet_id = pid;
        }
    }
    pthread_mutex_unlock((pthread_mutex_t *)&t->lock);
    return n;
}
