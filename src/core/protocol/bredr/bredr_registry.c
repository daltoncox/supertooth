/**
 * @file bredr_registry.c
 * @brief BR/EDR registry implementation (see bredr_registry.h).
 */

#include "bredr_registry.h"

#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "bredr_clock_recovery.h"

#define BREDR_REGISTRY_DEFAULT_IDLE_RESET_CLKN 16384u

/* ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------*/

static uint64_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000u + (uint64_t)tv.tv_usec / 1000u;
}

static uint32_t sample_to_rx_clk_1600(uint64_t idx, unsigned int rate)
{
    if (rate == 0u)
        return 0u;
    uint64_t num = idx * 1600u + (uint64_t)(rate / 2u);
    return (uint32_t)(num / (uint64_t)rate);
}

static uint32_t sample_to_clkn(uint64_t idx, unsigned int rate)
{
    if (rate == 0u)
        return 0u;
    uint64_t num = idx * 3200u + (uint64_t)(rate / 2u);
    return (uint32_t)(num / (uint64_t)rate);
}

static void format_addr_bredr(char *buf, size_t n, uint32_t lap,
                              uint8_t uap, int uap_valid)
{
    uint8_t used = uap;
    int known = uap_valid;

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

static uint64_t clk_to_ms(const bredr_registry_t *r, uint32_t clk)
{
    if (!r->has_epoch_base)
        return 0u;
    int32_t d = (int32_t)(clk - r->clk_base_1600);
    return r->epoch_base_ms + (uint64_t)((int64_t)d * 625 / 1000);
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------------*/

void bredr_registry_init(bredr_registry_t *r, const bredr_registry_config_t *cfg)
{
    if (!r)
        return;
    memset(r, 0, sizeof(*r));
    if (cfg)
        r->cfg = *cfg;
    else
    {
        r->cfg.idle_reset_clkn = BREDR_REGISTRY_DEFAULT_IDLE_RESET_CLKN;
        r->cfg.promote_threshold = 1u;
    }
    if (r->cfg.promote_threshold == 0u)
        r->cfg.promote_threshold = 1u;
    r->next_id = 1u;
    pthread_mutex_init(&r->lock, NULL);
}

void bredr_registry_free(bredr_registry_t *r)
{
    if (!r)
        return;
    for (size_t i = 0; i < r->conns_count; i++)
        free(r->conns[i].link);
    free(r->conns);
    free(r->devices);
    pthread_mutex_destroy(&r->lock);
    memset(r, 0, sizeof(*r));
}

void bredr_registry_set_frame_dump(bredr_registry_t *r, FILE *file)
{
    if (!r)
    {
        bredr_recovery_set_frame_dump(file);
        return;
    }
    pthread_mutex_lock(&r->lock);
    r->frame_dump = file;
    pthread_mutex_unlock(&r->lock);
    bredr_recovery_set_frame_dump(file);
}

/* ---------------------------------------------------------------------------
 * Internal tables (caller holds the lock)
 * ---------------------------------------------------------------------------*/

static bredr_connection_t *conn_find(bredr_registry_t *r, uint32_t lap)
{
    for (size_t i = 0; i < r->conns_count; i++)
        if ((r->conns[i].link->lap & 0xFFFFFFu) == lap)
            return &r->conns[i];
    return NULL;
}

static int pending_index(const bredr_registry_t *r, uint32_t lap)
{
    for (size_t i = 0; i < r->pending_count; i++)
        if (r->pending[i].lap == lap)
            return (int)i;
    return -1;
}

static void pending_remove(bredr_registry_t *r, size_t idx)
{
    if (idx >= r->pending_count)
        return;
    r->pending[idx] = r->pending[r->pending_count - 1u];
    r->pending_count--;
}

/* Returns the new tally count for lap. */
static unsigned long pending_bump(bredr_registry_t *r, uint32_t lap)
{
    int idx = pending_index(r, lap);
    if (idx >= 0)
    {
        r->pending[idx].count++;
        r->pending[idx].seq = ++r->pending_seq;
        return r->pending[idx].count;
    }
    if (r->pending_count >= BREDR_REGISTRY_PENDING_CAP)
    {
        size_t victim = 0u;
        uint64_t vseq = UINT64_MAX;
        for (size_t i = 0; i < r->pending_count; i++)
            if (r->pending[i].seq < vseq)
            {
                vseq = r->pending[i].seq;
                victim = i;
            }
        pending_remove(r, victim);
    }
    bredr_pending_t *p = &r->pending[r->pending_count++];
    p->lap = lap;
    p->count = 1u;
    p->seq = ++r->pending_seq;
    return 1u;
}

static bredr_connection_t *conn_create(bredr_registry_t *r, uint32_t lap)
{
    if (r->conns_count >= r->conns_cap)
    {
        size_t new_cap = r->conns_cap ? r->conns_cap * 2u : BREDR_REGISTRY_CONNS_INIT_CAP;
        bredr_connection_t *na = (bredr_connection_t *)realloc(
            r->conns, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + r->conns_cap, 0, (new_cap - r->conns_cap) * sizeof(*na));
        r->conns = na;
        r->conns_cap = new_cap;
    }
    bredr_link_t *link = (bredr_link_t *)malloc(sizeof(*link));
    if (!link)
        return NULL;
    bredr_link_init(link, lap);
    bredr_connection_t *c = &r->conns[r->conns_count++];
    memset(c, 0, sizeof(*c));
    c->link = link;
    c->connection_id = r->next_id++;
    return c;
}

static bredr_device_entry_t *device_find(bredr_registry_t *r,
                                         uint64_t connection_id,
                                         uint8_t lt_addr)
{
    for (size_t i = 0; i < r->devices_count; i++)
        if (r->devices[i].connection_id == connection_id &&
            r->devices[i].lt_addr == lt_addr)
            return &r->devices[i];
    return NULL;
}

static bredr_device_entry_t *device_ensure(bredr_registry_t *r,
                                           bredr_connection_t *conn,
                                           uint8_t lt_addr)
{
    bredr_device_entry_t *d = device_find(r, conn->connection_id, lt_addr);
    if (d)
        return d;
    if (r->devices_count >= r->devices_cap)
    {
        size_t new_cap = r->devices_cap ? r->devices_cap * 2u : BREDR_REGISTRY_DEVICES_INIT_CAP;
        bredr_device_entry_t *na = (bredr_device_entry_t *)realloc(
            r->devices, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + r->devices_cap, 0, (new_cap - r->devices_cap) * sizeof(*na));
        /* realloc may move the arrays: re-resolve conn pointer. */
        size_t ci = (size_t)(conn - r->conns);
        r->devices = na;
        r->devices_cap = new_cap;
        conn = &r->conns[ci];
        (void)conn;
    }
    d = &r->devices[r->devices_count++];
    memset(d, 0, sizeof(*d));
    d->device_id = r->next_id++;
    d->connection_id = conn->connection_id;
    d->lt_addr = lt_addr;
    d->lap = conn->link->lap & 0xFFFFFFu;
    return d;
}

/* Fill a single connection snapshot from a connection entry. Caller holds lock. */
static void fill_connection_snapshot(const bredr_registry_t *r,
                                     const bredr_connection_t *c,
                                     bredr_connection_snapshot_t *s)
{
    const bredr_link_t *p = c->link;
    memset(s, 0, sizeof(*s));
    s->id = c->connection_id;
    s->kind = ENTITY_BREDR_CONNECTION;
    format_addr_bredr(s->addr_str, sizeof(s->addr_str), p->lap, p->uap, p->uap_valid);
    if (p->lap == 0x9E8B33u || p->lap == 0x9E8B00u)
        snprintf(s->label, sizeof(s->label), "INQUIRY");
    else
        snprintf(s->label, sizeof(s->label), "connection");

    s->combined_rssi_seen =
        rssi_tracker_average(&p->combined_rssi_track, &s->combined_rssi);
    s->central_rssi_seen =
        rssi_tracker_average(&p->central_rssi_track, &s->central_rssi);
    for (int lt = 0; lt < 8; lt++)
        s->peripheral_rssi_seen[lt] =
            rssi_tracker_average(&p->peripheral_rssi_track[lt], &s->peripheral_rssi[lt]);

    if (s->combined_rssi_seen)
    {
        s->rssi_db = s->combined_rssi;
        s->rssi_valid = 1;
    }
    else if (s->central_rssi_seen)
    {
        s->rssi_db = s->central_rssi;
        s->rssi_valid = 1;
    }
    else
        s->rssi_valid = 0;

    s->first_seen_ms = clk_to_ms(r, p->first_seen);
    s->last_seen_ms = clk_to_ms(r, p->last_seen);
    s->total_packets = p->total_packets;
    s->packet_rate = 0u;

    s->lap = p->lap;
    s->uap = p->uap;
    s->uap_valid = p->uap_valid;
    s->clk_known = p->clk_known;
    s->central_clk_1_6 = bredr_link_central_clk_1_6(p, p->last_seen);
    s->tracking_state = p->tracking_state;

    s->central_device_id = c->device_id_central;
    memcpy(s->peripheral_device_id, c->device_id_peripheral, sizeof(s->peripheral_device_id));
}

/* ---------------------------------------------------------------------------
 * Ingest
 * ---------------------------------------------------------------------------*/

int bredr_registry_submit(bredr_registry_t *r, const bredr_event_t *event,
                          bredr_connection_snapshot_t *touched_out,
                          int *is_newest_out)
{
    if (!r || !event)
        return -1;
    if (event->meta.radio_sample_rate_hz == 0u)
        return -1;

    pthread_mutex_lock(&r->lock);

    if (!r->has_epoch_base)
    {
        r->clk_base_1600 = sample_to_rx_clk_1600(
            event->meta.radio_start_sample_index,
            event->meta.radio_sample_rate_hz);
        r->epoch_base_ms = now_ms();
        r->has_epoch_base = 1;
    }

    uint32_t lap = event->frame.lap & 0xFFFFFFu;
    bredr_connection_t *c = conn_find(r, lap);
    if (!c)
    {
        unsigned long tally = pending_bump(r, lap);
        if (tally < r->cfg.promote_threshold)
        {
            if (is_newest_out)
                *is_newest_out = 0;
            pthread_mutex_unlock(&r->lock);
            return 0;
        }
        int pi = pending_index(r, lap);
        if (pi >= 0)
            pending_remove(r, (size_t)pi);
        c = conn_create(r, lap);
        if (!c)
        {
            pthread_mutex_unlock(&r->lock);
            return -1;
        }
    }

    uint32_t clkn = sample_to_clkn(event->meta.radio_start_sample_index,
                                   event->meta.radio_sample_rate_hz);
    if (c->has_last_clkn)
    {
        if (clkn >= c->last_clkn)
        {
            uint32_t idle = clkn - c->last_clkn;
            if (r->cfg.idle_reset_clkn && idle > r->cfg.idle_reset_clkn)
                bredr_recovery_reset(c->link);
            c->last_clkn = clkn;
        }
    }
    else
        c->last_clkn = clkn;
    c->has_last_clkn = 1;

    int is_newest = bredr_link_add_packet(c->link, event);
    if (is_newest_out)
        *is_newest_out = is_newest;

    /* Materialize member device rows + linkage at ingest (poll is pure read). */
    if (c->link->central_pkts > 0u)
    {
        bredr_device_entry_t *d = device_ensure(r, c, 255u);
        if (d && c->device_id_central == 0u)
            c->device_id_central = d->device_id;
        else if (d)
            c->device_id_central = d->device_id;
    }
    for (int lt = 0; lt < 8; lt++)
    {
        if (c->link->peripheral_pkts[lt] == 0u)
            continue;
        bredr_device_entry_t *d = device_ensure(r, c, (uint8_t)lt);
        if (d)
            c->device_id_peripheral[lt] = d->device_id;
    }
    /* Re-resolve c: device_ensure may have realloc'd conns via devices path?
     * devices realloc never moves conns, but conn_create may have; c is still
     * valid here (no conn realloc after lookup). Safe. */

    if (touched_out)
        fill_connection_snapshot(r, c, touched_out);

    pthread_mutex_unlock(&r->lock);
    return 0;
}

int bredr_registry_add_device(bredr_registry_t *r, const bredr_device_obs_t *obs)
{
    if (!r || !obs)
        return -1;
    pthread_mutex_lock(&r->lock);
    uint32_t lap = obs->lap & 0xFFFFFFu;
    for (size_t i = 0; i < r->devices_count; i++)
    {
        bredr_device_entry_t *d = &r->devices[i];
        if (d->connection_id == 0u && d->lap == lap)
        {
            d->uap = obs->uap;
            d->uap_valid = obs->uap_valid;
            if (obs->rssi_valid)
            {
                d->rssi_db = obs->rssi_db;
                d->rssi_valid = 1;
            }
            uint64_t now = now_ms();
            if (d->total_packets == 0u)
                d->first_seen_ms = now;
            d->last_seen_ms = now;
            d->total_packets++;
            pthread_mutex_unlock(&r->lock);
            return 0;
        }
    }
    if (r->devices_count >= r->devices_cap)
    {
        size_t new_cap = r->devices_cap ? r->devices_cap * 2u : BREDR_REGISTRY_DEVICES_INIT_CAP;
        bredr_device_entry_t *na = (bredr_device_entry_t *)realloc(
            r->devices, new_cap * sizeof(*na));
        if (!na)
        {
            pthread_mutex_unlock(&r->lock);
            return -1;
        }
        memset(na + r->devices_cap, 0, (new_cap - r->devices_cap) * sizeof(*na));
        r->devices = na;
        r->devices_cap = new_cap;
    }
    bredr_device_entry_t *d = &r->devices[r->devices_count++];
    memset(d, 0, sizeof(*d));
    d->device_id = r->next_id++;
    d->connection_id = 0u;
    d->lt_addr = 255u;
    d->lap = lap;
    d->uap = obs->uap;
    d->uap_valid = obs->uap_valid;
    if (obs->rssi_valid)
    {
        d->rssi_db = obs->rssi_db;
        d->rssi_valid = 1;
    }
    uint64_t now = now_ms();
    d->first_seen_ms = now;
    d->last_seen_ms = now;
    d->total_packets = 1u;
    pthread_mutex_unlock(&r->lock);
    return 0;
}

/* ---------------------------------------------------------------------------
 * Poll (pure reads)
 * ---------------------------------------------------------------------------*/

size_t bredr_registry_get_connections(const bredr_registry_t *r,
                                      bredr_connection_snapshot_t *out,
                                      size_t max)
{
    if (!r || !out || max == 0u)
        return 0u;
    pthread_mutex_lock((pthread_mutex_t *)&r->lock);
    size_t n = 0u;
    for (size_t i = 0; i < r->conns_count && n < max; i++)
        fill_connection_snapshot(r, &r->conns[i], &out[n++]);
    pthread_mutex_unlock((pthread_mutex_t *)&r->lock);
    return n;
}

size_t bredr_registry_get_devices(const bredr_registry_t *r,
                                  bredr_device_snapshot_t *out,
                                  size_t max)
{
    if (!r || !out || max == 0u)
        return 0u;
    pthread_mutex_lock((pthread_mutex_t *)&r->lock);
    size_t n = 0u;
    for (size_t i = 0; i < r->devices_count && n < max; i++)
    {
        const bredr_device_entry_t *d = &r->devices[i];
        bredr_device_snapshot_t *s = &out[n++];
        memset(s, 0, sizeof(*s));
        s->kind = ENTITY_BREDR_DEVICE;
        s->lt_addr = d->lt_addr;
        s->lap = d->lap;
        s->connection_id = d->connection_id;
        if (d->connection_id != 0u)
        {
            /* Member row: resolve presentation from the owning link. */
            const bredr_connection_t *owner = NULL;
            for (size_t k = 0; k < r->conns_count; k++)
                if (r->conns[k].connection_id == d->connection_id)
                {
                    owner = &r->conns[k];
                    break;
                }
            if (!owner)
            {
                n--; /* skip dangling row; keep output dense. */
                continue;
            }
            const bredr_link_t *p = owner->link;
            s->id = d->device_id;
            /* Member rows keep the last known UAP even after tracking loss. */
            format_addr_bredr(s->addr_str, sizeof(s->addr_str), p->lap, p->uap, 1);
            if (d->lt_addr == 255u)
            {
                snprintf(s->label, sizeof(s->label), "Central");
                s->rssi_valid =
                    rssi_tracker_average(&p->central_rssi_track, &s->rssi_db);
                s->total_packets = p->central_pkts;
            }
            else
            {
                snprintf(s->label, sizeof(s->label), "LT_ADDR %d", d->lt_addr);
                s->rssi_valid =
                    rssi_tracker_average(&p->peripheral_rssi_track[d->lt_addr], &s->rssi_db);
                s->total_packets = p->peripheral_pkts[d->lt_addr];
            }
            s->first_seen_ms = clk_to_ms(r, p->first_seen);
            s->last_seen_ms = clk_to_ms(r, p->last_seen);
            s->uap = p->uap;
            s->uap_valid = p->uap_valid;
        }
        else
        {
            /* Standalone (chipset) row: use stored observation. */
            s->id = d->device_id;
            format_addr_bredr(s->addr_str, sizeof(s->addr_str), d->lap, d->uap, d->uap_valid);
            snprintf(s->label, sizeof(s->label), "Device");
            s->rssi_valid = d->rssi_valid;
            s->rssi_db = d->rssi_db;
            s->first_seen_ms = d->first_seen_ms;
            s->last_seen_ms = d->last_seen_ms;
            s->total_packets = d->total_packets;
            s->uap = d->uap;
            s->uap_valid = d->uap_valid;
        }
        s->packet_rate = 0u;
    }
    pthread_mutex_unlock((pthread_mutex_t *)&r->lock);
    return n;
}
