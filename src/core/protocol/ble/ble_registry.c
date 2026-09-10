/**
 * @file ble_registry.c
 * @brief BLE registry implementation (see ble_registry.h).
 */

#include "ble_registry.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/time.h>

/** Hard ceiling for the dynamic connections table (then LRU-evict). */
#define BLE_REGISTRY_CONNS_MAX 512u

/* ---------------------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------------------*/

static uint64_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000u + (uint64_t)tv.tv_usec / 1000u;
}

static const char *ble_adv_addr_subtype(ble_addr_kind_t kind,
                                        const uint8_t addr[BLE_ADDR_LEN])
{
    if (kind == BLE_ADDR_PUBLIC)
        return "PUBLIC";
    switch ((addr[5] >> 6) & 0x03u)
    {
    case 0x03u: return "STATIC";
    case 0x02u: return "RESERVED";
    case 0x01u: return "RESOLVABLE";
    default:    return "NONRESOLVABLE";
    }
}

static void format_addr_bytes(uint64_t addr, char *buf, size_t n)
{
    uint8_t b[6];
    for (int i = 0; i < 6; i++)
        b[i] = (uint8_t)((addr >> (8 * i)) & 0xFFu);
    snprintf(buf, n, "%02X:%02X:%02X:%02X:%02X:%02X",
             b[5], b[4], b[3], b[2], b[1], b[0]);
}

static void format_aa(uint32_t aa, char *buf, size_t n)
{
    snprintf(buf, n, "0x%08X", (unsigned)aa);
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------------*/

void ble_registry_init(ble_registry_t *r, const ble_registry_config_t *cfg)
{
    if (!r)
        return;
    memset(r, 0, sizeof(*r));
    if (cfg)
        r->cfg = *cfg;
    r->next_id = 1u;
    pthread_mutex_init(&r->lock, NULL);
    ble_crc_tables_init();
}

void ble_registry_set_enforce_crc(ble_registry_t *r, int on)
{
    if (!r)
        return;
    pthread_mutex_lock(&r->lock);
    r->cfg.enforce_crc = on ? 1 : 0;
    pthread_mutex_unlock(&r->lock);
}

void ble_registry_free(ble_registry_t *r)
{
    if (!r)
        return;
    free(r->conns);
    free(r->devices);
    pthread_mutex_destroy(&r->lock);
    memset(r, 0, sizeof(*r));
}

/* ---------------------------------------------------------------------------
 * Internal connection table (caller holds the lock)
 * ---------------------------------------------------------------------------*/

static ble_connection_t *conn_find(ble_registry_t *r, uint32_t aa)
{
    for (size_t i = 0; i < r->conns_count; i++)
        if (r->conns[i].link.access_address == aa)
            return &r->conns[i];
    return NULL;
}

static size_t conn_victim_index(ble_registry_t *r)
{
    size_t best = 0u;
    uint64_t best_seq = UINT64_MAX;
    unsigned int best_pkts = 0u;
    int best_has_cand = 1;

    for (size_t i = 0; i < r->conns_count; i++)
    {
        const ble_link_t *e = &r->conns[i].link;
        if (e->state == BLE_CONNECTION_CONFIRMED)
            continue;

        int has_cand = (e->candidate_count > 0u) ? 1 : 0;
        unsigned int pkts = (unsigned int)e->packets_seen;

        int better = 0;
        if (has_cand != best_has_cand)
        {
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

    if (best_seq == UINT64_MAX)
    {
        for (size_t i = 0; i < r->conns_count; i++)
            if (r->conns[i].link.last_used_seq < best_seq)
            {
                best = i;
                best_seq = r->conns[i].link.last_used_seq;
            }
    }
    return best;
}

static ble_connection_t *conn_create(ble_registry_t *r, uint32_t aa)
{
    ble_connection_t *c = conn_find(r, aa);
    if (c)
        return c;

    if (r->conns_count < r->conns_cap)
        c = &r->conns[r->conns_count++];
    else if (r->conns_cap < BLE_REGISTRY_CONNS_MAX)
    {
        size_t new_cap = r->conns_cap ? r->conns_cap * 2u : BLE_REGISTRY_CONNS_INIT_CAP;
        if (new_cap > BLE_REGISTRY_CONNS_MAX)
            new_cap = BLE_REGISTRY_CONNS_MAX;
        ble_connection_t *na = (ble_connection_t *)realloc(
            r->conns, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + r->conns_cap, 0, (new_cap - r->conns_cap) * sizeof(*na));
        r->conns = na;
        r->conns_cap = new_cap;
        c = &r->conns[r->conns_count++];
    }
    else
        c = &r->conns[conn_victim_index(r)];

    uint64_t now = now_ms();
    memset(c, 0, sizeof(*c));
    c->link.access_address = aa;
    c->link.state = BLE_CONNECTION_COLLECTING;
    c->connection_id = r->next_id++;
    c->first_seen_ms = now;
    c->last_seen_ms = now;
    return c;
}

static int pending_index(const ble_registry_t *r, uint32_t aa)
{
    for (size_t i = 0; i < r->pending_count; i++)
        if (r->pending[i].aa == aa)
            return (int)i;
    return -1;
}

static void pending_remove(ble_registry_t *r, size_t idx)
{
    if (idx >= r->pending_count)
        return;
    r->pending[idx] = r->pending[r->pending_count - 1u];
    r->pending_count--;
}

static unsigned long pending_bump(ble_registry_t *r, uint32_t aa)
{
    int idx = pending_index(r, aa);
    if (idx >= 0)
    {
        r->pending[idx].count++;
        r->pending[idx].seq = ++r->pending_seq;
        return r->pending[idx].count;
    }

    if (r->pending_count >= BLE_REGISTRY_PENDING_CAP)
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

    ble_pending_t *p = &r->pending[r->pending_count++];
    p->aa = aa;
    p->count = 1u;
    p->seq = ++r->pending_seq;
    return 1u;
}

static ble_connection_t *conn_acquire(ble_registry_t *r, uint32_t aa, int force)
{
    ble_connection_t *c = conn_find(r, aa);
    if (c)
        return c;

    if (force)
    {
        int idx = pending_index(r, aa);
        if (idx >= 0)
            pending_remove(r, (size_t)idx);
        return conn_create(r, aa);
    }

    unsigned long cnt = pending_bump(r, aa);
    if (cnt >= (unsigned long)BLE_REGISTRY_PROMOTE_THRESHOLD)
    {
        int idx = pending_index(r, aa);
        if (idx >= 0)
            pending_remove(r, (size_t)idx);
        return conn_create(r, aa);
    }
    return NULL;
}

static void link_add_candidate(ble_link_t *link, uint32_t crc_init)
{
    crc_init &= 0xFFFFFFu;

    for (unsigned int i = 0; i < link->candidate_count; i++)
        if (link->candidates[i] == crc_init)
            return;

    if (link->candidate_count < BLE_REGISTRY_MAX_CANDIDATES)
    {
        link->candidates[link->candidate_count++] = crc_init;
        return;
    }

    memmove(&link->candidates[0], &link->candidates[1],
            (BLE_REGISTRY_MAX_CANDIDATES - 1u) * sizeof(link->candidates[0]));
    link->candidates[BLE_REGISTRY_MAX_CANDIDATES - 1u] = crc_init;
}

static void link_confirm(ble_link_t *link, uint32_t crc_init)
{
    link->state = BLE_CONNECTION_CONFIRMED;
    link->crc_init = crc_init & 0xFFFFFFu;
    link->candidate_count = 0u;
}

/* CRC gate for one data PDU. Caller holds the lock. Returns 1 on accept. */
static int gate_data_pdu(ble_registry_t *r, uint32_t aa,
                         const uint8_t *pdu, unsigned int pdu_bytes,
                         uint32_t rx_crc, uint32_t *crc_init_used_out)
{
    ble_crc_tables_init();

    ble_connection_t *c = conn_acquire(r, aa, 0);
    if (!c)
        return 0;
    ble_link_t *link = &c->link;
    link->packets_seen++;
    link->last_used_seq = ++r->seq;

    if (link->state == BLE_CONNECTION_CONFIRMED)
    {
        if (ble_crc_calc(pdu, pdu_bytes, link->crc_init) == (rx_crc & 0xFFFFFFu))
        {
            link->packets_accepted++;
            if (crc_init_used_out)
                *crc_init_used_out = link->crc_init;
            return 1;
        }
        return 0;
    }

    if (pdu_bytes == 2u)
    {
        link_add_candidate(link, ble_crc_reverse_init_len2(pdu, rx_crc));
        return 0;
    }

    for (unsigned int i = 0; i < link->candidate_count; i++)
    {
        if (ble_crc_calc(pdu, pdu_bytes, link->candidates[i]) == (rx_crc & 0xFFFFFFu))
        {
            uint32_t used = link->candidates[i];
            link_confirm(link, used);
            link->packets_accepted++;
            if (crc_init_used_out)
                *crc_init_used_out = used;
            return 1;
        }
    }
    return 0;
}

static void seed_candidate_locked(ble_registry_t *r, uint32_t aa, uint32_t crc_init)
{
    ble_connection_t *c = conn_acquire(r, aa, 1);
    if (!c)
        return;
    c->link.last_used_seq = ++r->seq;
    if (c->link.state != BLE_CONNECTION_CONFIRMED)
        link_add_candidate(&c->link, crc_init);
}

/* ---------------------------------------------------------------------------
 * Internal devices table (caller holds the lock)
 * ---------------------------------------------------------------------------*/

static ble_device_entry_t *device_find_or_create(ble_registry_t *r, uint64_t addr)
{
    for (size_t i = 0; i < r->devices_count; i++)
        if (r->devices[i].adv_addr == addr)
            return &r->devices[i];

    if (r->devices_count >= r->devices_cap)
    {
        size_t new_cap = r->devices_cap ? r->devices_cap * 2u : BLE_REGISTRY_DEVICES_INIT_CAP;
        ble_device_entry_t *na = (ble_device_entry_t *)realloc(
            r->devices, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + r->devices_cap, 0, (new_cap - r->devices_cap) * sizeof(*na));
        r->devices = na;
        r->devices_cap = new_cap;
    }
    ble_device_entry_t *d = &r->devices[r->devices_count++];
    memset(d, 0, sizeof(*d));
    d->adv_addr = addr;
    d->device_id = r->next_id++;
    uint64_t now = now_ms();
    d->first_seen_ms = now;
    d->last_seen_ms = now;
    return d;
}

static ble_device_entry_t *device_find(ble_registry_t *r, uint64_t addr)
{
    for (size_t i = 0; i < r->devices_count; i++)
        if (r->devices[i].adv_addr == addr)
            return &r->devices[i];
    return NULL;
}

static void device_ingest_locked(ble_registry_t *r, const ble_advertiser_event_t *ev)
{
    ble_device_entry_t *d = device_find_or_create(r, ev->adv_addr);
    if (!d)
        return;

    uint64_t now = now_ms();
    if (d->total_packets == 0u)
        d->first_seen_ms = now;
    d->last_seen_ms = now;
    d->total_packets++;

    if (ev->rssi_valid)
    {
        rx_metadata_t meta;
        memset(&meta, 0, sizeof(meta));
        meta.radio_sample_rate_hz = 1000u;
        meta.radio_start_sample_index = now;
        meta.rssi_dbr = ev->rssi_db;
        rssi_tracker_add(&d->rssi, &meta);
    }

    if (ev->addr_type && ev->addr_type[0])
        snprintf(d->addr_type, sizeof(d->addr_type), "%s", ev->addr_type);
    if (ev->name && ev->name[0])
        snprintf(d->name, sizeof(d->name), "%s", ev->name);
    if (ev->manufacturer && ev->manufacturer[0])
        snprintf(d->manufacturer, sizeof(d->manufacturer), "%s", ev->manufacturer);

    if (ev->is_connect_ind && ev->crc_ok)
    {
        /* Order lookups so pointers stay fresh across realloc growth. */
        ble_connection_t *c = conn_acquire(r, ev->conn_access_address, 1);
        ble_device_entry_t *init = device_find_or_create(r, ev->conn_initiator_addr);
        uint64_t init_id = init ? init->device_id : 0u;
        ble_device_entry_t *adv = device_find_or_create(r, ev->conn_advertiser_addr);
        /* Re-resolve after possible reallocs. */
        c = conn_find(r, ev->conn_access_address);
        init = device_find(r, ev->conn_initiator_addr);
        adv = device_find(r, ev->conn_advertiser_addr);
        if (c && adv && init)
        {
            c->device_id_central = adv->device_id;
            c->device_id_peripheral = init_id;
            if (c->first_seen_ms == 0u)
                c->first_seen_ms = now;
            seed_candidate_locked(r, ev->conn_access_address, ev->conn_crc_init);
            adv->connection_id = c->connection_id;
            adv->has_linked_aa = 1;
            adv->linked_aa = ev->conn_access_address;
        }
    }
}

/* Fill one connection snapshot. Caller holds the lock. */
static void fill_connection_snapshot(ble_registry_t *r,
                                     const ble_connection_t *c,
                                     ble_connection_snapshot_t *s)
{
    const ble_link_t *e = &c->link;
    memset(s, 0, sizeof(*s));
    s->id = c->connection_id;
    s->kind = ENTITY_BLE_CONNECTION;
    format_aa(e->access_address, s->addr_str, sizeof(s->addr_str));
    snprintf(s->label, sizeof(s->label), "connection");

    int rssi_valid = 0;
    float rssi_db = 0.0f;
    if (rssi_tracker_average(&c->rssi, &rssi_db))
        rssi_valid = 1;
    else
    {
        double sum = 0.0;
        int n = 0;
        for (size_t k = 0; k < r->devices_count; k++)
        {
            const ble_device_entry_t *a = &r->devices[k];
            if ((c->device_id_central && a->device_id == c->device_id_central) ||
                (c->device_id_peripheral && a->device_id == c->device_id_peripheral))
            {
                float adv_rssi = 0.0f;
                if (rssi_tracker_average(&a->rssi, &adv_rssi))
                {
                    sum += adv_rssi;
                    n++;
                }
            }
        }
        if (n > 0)
        {
            rssi_db = (float)(sum / (double)n);
            rssi_valid = 1;
        }
    }
    s->rssi_valid = rssi_valid;
    s->rssi_db = rssi_db;
    s->first_seen_ms = c->first_seen_ms;
    s->last_seen_ms = c->last_seen_ms;
    s->total_packets = e->packets_seen;
    s->packet_rate = 0u;
    s->access_address = e->access_address;
    s->crc_init_confirmed = (e->state == BLE_CONNECTION_CONFIRMED) ? 1 : 0;
    s->crc_init = e->crc_init;
    s->candidate_count = e->candidate_count;
    s->state = (int)e->state;
    s->device_id_central = c->device_id_central;
    s->device_id_peripheral = c->device_id_peripheral;
}

/* ---------------------------------------------------------------------------
 * Ingest
 * ---------------------------------------------------------------------------*/

int ble_registry_submit(ble_registry_t *r, const ble_event_t *event,
                        ble_connection_snapshot_t *touched_out)
{
    if (!r || !event)
        return 0;
    if (touched_out)
        memset(touched_out, 0, sizeof(*touched_out));

    if (event->frame.access_address == BLE_ADVERTISING_AA)
    {
        ble_packet_t pkt;
        if (ble_decode_frame(&event->frame, event->meta.channel_index, &pkt) != 0 ||
            !pkt.is_adv_pdu)
            return 0;

        int enforce;
        pthread_mutex_lock(&r->lock);
        enforce = r->cfg.enforce_crc;
        pthread_mutex_unlock(&r->lock);
        if (enforce && !pkt.crc_ok)
            return 0;

        const ble_adv_pdu_t *adv = &pkt.pdu.adv;
        const uint8_t *addr = ble_adv_addr_bytes(adv);
        if (addr)
        {
            char name[DEVICE_NAME_MAX];
            char manuf[DEVICE_MANUF_MAX];
            ble_advertiser_event_t ev;
            memset(&ev, 0, sizeof(ev));

            ble_addr_bytes_to_u64(addr, &ev.adv_addr);

            ble_addr_kind_t kind = (adv->pdu_type == BLE_PDU_CONNECT_IND)
                ? adv->rx_addr_kind : adv->tx_addr_kind;
            ev.addr_type = ble_adv_addr_subtype(kind, addr);

            unsigned int adv_len = 0;
            const uint8_t *adv_data = ble_adv_data_bytes(adv, &adv_len);
            ble_adv_parse_name_manuf(adv_data, adv_len, name, sizeof(name),
                                     manuf, sizeof(manuf));
            ev.name = name[0] ? name : NULL;
            ev.manufacturer = manuf[0] ? manuf : NULL;

            ev.rssi_db = event->meta.rssi_dbr;
            ev.rssi_valid = !isnan((double)event->meta.rssi_dbr);
            ev.crc_ok = pkt.crc_ok;

            if (adv->pdu_type == BLE_PDU_CONNECT_IND)
            {
                ble_connect_ind_params_t params;
                if (ble_connect_ind_parse(adv->payload.connect_ind.ll_data,
                                          &params) == 0)
                {
                    ev.is_connect_ind = 1;
                    ev.conn_access_address = params.access_address;
                    ev.conn_crc_init = params.crc_init;
                    ble_addr_bytes_to_u64(
                        adv->payload.connect_ind.init_addr.addr,
                        &ev.conn_initiator_addr);
                    ble_addr_bytes_to_u64(
                        adv->payload.connect_ind.adv_addr.addr,
                        &ev.conn_advertiser_addr);
                }
            }

            pthread_mutex_lock(&r->lock);
            device_ingest_locked(r, &ev);
            pthread_mutex_unlock(&r->lock);
        }
        return 1;
    }

    unsigned int frame_bytes = event->frame.raw_pdu_bytes;
    if (frame_bytes < (BLE_CRC_BYTES + 2u))
        return 0;

    uint8_t pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    if (frame_bytes > (unsigned int)sizeof(pdu))
        frame_bytes = (unsigned int)sizeof(pdu);
    memcpy(pdu, event->frame.raw_pdu, frame_bytes);
    ble_dewhiten(pdu, frame_bytes, event->meta.channel_index);

    unsigned int pdu_bytes = frame_bytes - BLE_CRC_BYTES;
    uint32_t rx_crc = ble_extract_crc(&pdu[pdu_bytes]);
    uint32_t crc_init_used = 0u;

    pthread_mutex_lock(&r->lock);
    int accepted = gate_data_pdu(r, event->frame.access_address,
                                 pdu, pdu_bytes, rx_crc, &crc_init_used);

    ble_connection_t *c = conn_find(r, event->frame.access_address);
    if (c)
    {
        uint64_t now = now_ms();
        if (c->first_seen_ms == 0u)
            c->first_seen_ms = now;
        c->last_seen_ms = now;
        if (!isnan(event->meta.rssi_dbr))
            rssi_tracker_add(&c->rssi, &event->meta);
        if (touched_out)
            fill_connection_snapshot(r, c, touched_out);
    }
    pthread_mutex_unlock(&r->lock);

    if (!accepted)
        return 0;

    ble_event_t *mutable = (ble_event_t *)event;
    mutable->frame.crc_ok = 1u;
    mutable->frame.crc_init = crc_init_used;
    return 1;
}

int ble_registry_add_device(ble_registry_t *r, const ble_device_obs_t *obs)
{
    if (!r || !obs)
        return -1;
    pthread_mutex_lock(&r->lock);
    ble_device_entry_t *d = device_find_or_create(r, obs->adv_addr);
    if (!d)
    {
        pthread_mutex_unlock(&r->lock);
        return -1;
    }
    uint64_t now = now_ms();
    if (d->total_packets == 0u)
        d->first_seen_ms = now;
    d->last_seen_ms = now;
    d->total_packets++;
    if (obs->rssi_valid)
    {
        rx_metadata_t meta;
        memset(&meta, 0, sizeof(meta));
        meta.radio_sample_rate_hz = 1000u;
        meta.radio_start_sample_index = now;
        meta.rssi_dbr = obs->rssi_db;
        rssi_tracker_add(&d->rssi, &meta);
    }
    if (obs->addr_type && obs->addr_type[0])
        snprintf(d->addr_type, sizeof(d->addr_type), "%s", obs->addr_type);
    if (obs->name && obs->name[0])
        snprintf(d->name, sizeof(d->name), "%s", obs->name);
    if (obs->manufacturer && obs->manufacturer[0])
        snprintf(d->manufacturer, sizeof(d->manufacturer), "%s", obs->manufacturer);
    pthread_mutex_unlock(&r->lock);
    return 0;
}

void ble_registry_confirm_for_test(ble_registry_t *r, uint32_t access_address,
                                   uint32_t crc_init)
{
    if (!r)
        return;
    pthread_mutex_lock(&r->lock);
    ble_connection_t *c = conn_acquire(r, access_address, 1);
    if (!c)
    {
        pthread_mutex_unlock(&r->lock);
        return;
    }
    c->link.last_used_seq = ++r->seq;
    link_confirm(&c->link, crc_init);
    pthread_mutex_unlock(&r->lock);
}

int ble_registry_find_link_for_test(const ble_registry_t *r,
                                            uint32_t access_address,
                                            ble_link_t *out)
{
    if (!r || !out)
        return -1;
    pthread_mutex_lock((pthread_mutex_t *)&r->lock);
    const ble_connection_t *c = NULL;
    for (size_t i = 0; i < r->conns_count; i++)
        if (r->conns[i].link.access_address == access_address)
        {
            c = &r->conns[i];
            break;
        }
    if (c)
        memcpy(out, &c->link, sizeof(*out));
    pthread_mutex_unlock((pthread_mutex_t *)&r->lock);
    return c ? 0 : -1;
}

/* ---------------------------------------------------------------------------
 * Poll (pure reads)
 * ---------------------------------------------------------------------------*/

size_t ble_registry_get_devices(const ble_registry_t *r,
                                ble_device_snapshot_t *out, size_t max)
{
    if (!r || !out || max == 0u)
        return 0u;
    pthread_mutex_lock((pthread_mutex_t *)&r->lock);
    size_t n = 0u;
    for (size_t i = 0; i < r->devices_count && n < max; i++)
    {
        const ble_device_entry_t *d = &r->devices[i];
        ble_device_snapshot_t *s = &out[n++];
        memset(s, 0, sizeof(*s));
        s->id = d->device_id;
        s->kind = ENTITY_BLE_DEVICE;
        format_addr_bytes(d->adv_addr, s->addr_str, sizeof(s->addr_str));
        snprintf(s->label, sizeof(s->label), "Advertiser");
        s->rssi_valid = rssi_tracker_average(&d->rssi, &s->rssi_db);
        s->first_seen_ms = d->first_seen_ms;
        s->last_seen_ms = d->last_seen_ms;
        s->total_packets = d->total_packets;
        s->packet_rate = 0u;
        s->adv_addr = d->adv_addr;
        snprintf(s->addr_type, sizeof(s->addr_type), "%s", d->addr_type);
        snprintf(s->name, sizeof(s->name), "%s", d->name);
        snprintf(s->manufacturer, sizeof(s->manufacturer), "%s", d->manufacturer);
    }
    pthread_mutex_unlock((pthread_mutex_t *)&r->lock);
    return n;
}

size_t ble_registry_get_connections(const ble_registry_t *r,
                                    ble_connection_snapshot_t *out, size_t max)
{
    if (!r || !out || max == 0u)
        return 0u;
    pthread_mutex_lock((pthread_mutex_t *)&r->lock);
    size_t n = 0u;
    int enforce = r->cfg.enforce_crc;
    for (size_t i = 0; i < r->conns_count && n < max; i++)
    {
        const ble_connection_t *c = &r->conns[i];
        if (enforce && c->link.state != BLE_CONNECTION_CONFIRMED)
            continue;
        fill_connection_snapshot((ble_registry_t *)r, c, &out[n++]);
    }
    pthread_mutex_unlock((pthread_mutex_t *)&r->lock);
    return n;
}
