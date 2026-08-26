/**
 * @file ble_tracker.c
 * @brief BLE tracker implementation (see ble_tracker.h).
 */

#include "ble_tracker.h"

#include <math.h>
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

/* Classify a BLE advertiser address into its device-list "type" subfield.
 * Public addresses are reported by kind; random addresses are subdivided by the
 * two most-significant bits of the address (bits 47:46), read as (bit47,bit46):
 *   11 -> STATIC         (random static)
 *   10 -> RESERVED       (reserved pattern; surfaced as-is, it denotes an anomaly)
 *   01 -> RESOLVABLE     (random private resolvable)
 *   00 -> NONRESOLVABLE  (random private non-resolvable) */
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

void ble_tracker_init(ble_tracker_t *t)
{
    if (!t)
        return;
    memset(t, 0, sizeof(*t));
    ble_piconet_store_init(&t->conn_store);
    t->adv = NULL;
    t->adv_count = 0;
    t->adv_cap = 0;
    t->conn = NULL;
    t->conn_count = 0;
    t->conn_cap = 0;
    t->next_id = 1u;
    t->enforce_crc = 0;
    pthread_mutex_init(&t->lock, NULL);
}

void ble_tracker_set_enforce_crc(ble_tracker_t *t, int on)
{
    if (!t)
        return;
    t->enforce_crc = on ? 1 : 0;
}

void ble_tracker_free(ble_tracker_t *t)
{
    if (!t)
        return;
    ble_piconet_store_free(&t->conn_store);
    free(t->adv);
    free(t->conn);
    t->adv = NULL;
    t->conn = NULL;
    t->adv_count = t->adv_cap = 0;
    t->conn_count = t->conn_cap = 0;
    pthread_mutex_destroy(&t->lock);
    memset(t, 0, sizeof(*t));
}

/* ---------------------------------------------------------------------------
 * Internal registries
 * ---------------------------------------------------------------------------*/

static ble_adv_record_t *adv_find_or_create(ble_tracker_t *t, uint64_t addr)
{
    for (size_t i = 0; i < t->adv_count; i++)
        if (t->adv[i].used && t->adv[i].adv_addr == addr)
            return &t->adv[i];

    if (t->adv_count >= t->adv_cap)
    {
        size_t new_cap = t->adv_cap ? t->adv_cap * 2u : 16u;
        ble_adv_record_t *na = (ble_adv_record_t *)realloc(
            t->adv, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + t->adv_cap, 0, (new_cap - t->adv_cap) * sizeof(*na));
        t->adv = na;
        t->adv_cap = new_cap;
    }
    ble_adv_record_t *r = &t->adv[t->adv_count++];
    memset(r, 0, sizeof(*r));
    r->used = 1;
    r->adv_addr = addr;
    r->device_id = t->next_id++;
    r->first_seen_ms = now_ms();
    return r;
}

static ble_conn_record_t *conn_find_or_create(ble_tracker_t *t, uint32_t aa)
{
    for (size_t i = 0; i < t->conn_count; i++)
        if (t->conn[i].used && t->conn[i].aa == aa)
            return &t->conn[i];

    if (t->conn_count >= t->conn_cap)
    {
        size_t new_cap = t->conn_cap ? t->conn_cap * 2u : 16u;
        ble_conn_record_t *na = (ble_conn_record_t *)realloc(
            t->conn, new_cap * sizeof(*na));
        if (!na)
            return NULL;
        memset(na + t->conn_cap, 0, (new_cap - t->conn_cap) * sizeof(*na));
        t->conn = na;
        t->conn_cap = new_cap;
    }
    ble_conn_record_t *r = &t->conn[t->conn_count++];
    memset(r, 0, sizeof(*r));
    r->used = 1;
    r->aa = aa;
    r->piconet_id = t->next_id++;
    r->first_seen_ms = now_ms();
    return r;
}

/* ---------------------------------------------------------------------------
 * Ingest
 * ---------------------------------------------------------------------------*/

void ble_tracker_seed_candidate(ble_tracker_t *t,
                                uint32_t access_address,
                                uint32_t crc_init)
{
    if (!t)
        return;
    ble_piconet_store_seed_candidate(&t->conn_store, access_address, crc_init);
}

int ble_tracker_submit_frame(ble_tracker_t *t, const ble_event_t *event)
{
    if (!t || !event)
        return 0;

    /* Advertising frame: parse and register the advertiser. */
    if (event->frame.access_address == BLE_ADVERTISING_AA)
    {
        ble_packet_t pkt;
        if (ble_decode_frame(&event->frame, event->meta.channel_index, &pkt) != 0 ||
            !pkt.is_adv_pdu)
            return 0;

        /* CRC enforcement: a failed advertising CRC means the payload is
         * garbage — do not create a device and do not surface the frame. */
        if (t->enforce_crc && !pkt.crc_ok)
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

            ble_tracker_add_advertiser(t, &ev);
        }
        return 1;   /* advertising frames are surfaced (CRC already checked) */
    }

    /* Data candidate: CRC-gate it against the piconet store. The frame's
     * raw_pdu is still whitened on the wire, so dewhiten before gating. */
    unsigned int frame_bytes = event->frame.raw_pdu_bytes;
    if (frame_bytes < (BLE_CRC_BYTES + 2u))
        return 0;

    uint8_t pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    if (frame_bytes > (unsigned int)sizeof(pdu))
        frame_bytes = (unsigned int)sizeof(pdu);
    memcpy(pdu, event->frame.raw_pdu, frame_bytes);
    ble_dewhiten(pdu, frame_bytes, event->meta.channel_index);

    unsigned int pdu_bytes = frame_bytes - BLE_CRC_BYTES;

    /* NOTE: the LL payload length is an 8-bit field (0..255) and is not
     * capped here; the decoder emits whatever the dewhitened length byte
     * dictates. Noise is filtered upstream by the decoder's AA/preamble
     * consistency check and the access-address validity rules, not by a
     * length limit. */

    uint32_t rx_crc = ble_extract_crc(&pdu[pdu_bytes]);
    uint32_t crc_init_used = 0u;
    int gate = ble_piconet_store_gate_data_pdu(&t->conn_store,
                                               event->frame.access_address,
                                               pdu, pdu_bytes, rx_crc,
                                               &crc_init_used);

    /* Update the connection record for ANY frame whose access address has
     * been promoted (recurring / CONNECT_IND-seeded), not only for
     * CRC-accepted frames. This keeps last-seen, RSSI, and packet activity
     * visible in the device list during the (sometimes lengthy) window
     * before the CRCInit confirms and frames still fail the CRC gate. */
    {
        ble_piconet_t ebuf;
        if (ble_piconet_store_find(&t->conn_store,
                                   event->frame.access_address, &ebuf) == 0)
        {
            pthread_mutex_lock(&t->lock);
            ble_conn_record_t *c = conn_find_or_create(t, event->frame.access_address);
            if (c)
            {
                uint64_t now = now_ms();
                if (c->first_seen_ms == 0u)
                    c->first_seen_ms = now;
                c->last_seen_ms = now;
                if (!isnan(event->meta.rssi_dbr))
                    rssi_tracker_add(&c->rssi_track, &event->meta);
            }
            pthread_mutex_unlock(&t->lock);
        }
    }

    if (gate != BLE_GATE_ACCEPT)
        return 0;

    /* Stamp the frame as CRC-valid so presentation layers (frame list) accept
     * it. The event is logically const, but this only annotates a verified
     * result that the decoder could no longer compute after the gate moved
     * out of the hot path. */
    ble_event_t *mutable = (ble_event_t *)event;
    mutable->frame.crc_ok = 1u;
    mutable->frame.crc_init = crc_init_used;
    return 1;
}

void ble_tracker_add_advertiser(ble_tracker_t *t,
                                const ble_advertiser_event_t *ev)
{
    if (!t || !ev)
        return;

    pthread_mutex_lock(&t->lock);

    ble_adv_record_t *r = adv_find_or_create(t, ev->adv_addr);
    if (!r)
    {
        pthread_mutex_unlock(&t->lock);
        return;
    }

    uint64_t now = now_ms();
    if (r->total_packets == 0u)
        r->first_seen_ms = now;
    r->last_seen_ms = now;
    r->total_packets++;

    if (ev->rssi_valid)
    {
        /* Advertiser events carry only a scalar RSSI, not full sample
         * metadata; synthesise a timestamp in milliseconds so the tracker's
         * 1 s window still applies (1000 Hz sample clock, index = ms). */
        rx_metadata_t meta;
        memset(&meta, 0, sizeof(meta));
        meta.radio_sample_rate_hz = 1000u;
        meta.radio_start_sample_index = now;
        meta.rssi_dbr = ev->rssi_db;
        rssi_tracker_add(&r->rssi_track, &meta);
    }

    if (ev->addr_type && ev->addr_type[0])
        snprintf(r->addr_type, sizeof(r->addr_type), "%s", ev->addr_type);
    if (ev->name && ev->name[0])
        snprintf(r->name, sizeof(r->name), "%s", ev->name);
    if (ev->manufacturer && ev->manufacturer[0])
        snprintf(r->manufacturer, sizeof(r->manufacturer), "%s",
                 ev->manufacturer);

    if (ev->is_connect_ind && ev->crc_ok)
    {
        /* Lookups may grow (realloc) the registries, freeing the previous
         * arrays and invalidating any record pointers obtained earlier.
         * Order the lookups so every pointer is dereferenced only while
         * fresh: copy what is needed from each record immediately, and do
         * the advertiser lookup (whose pointer is written through below)
         * last. */
        ble_conn_record_t *c = conn_find_or_create(t, ev->conn_access_address);
        ble_adv_record_t *init = adv_find_or_create(t, ev->conn_initiator_addr);
        uint64_t init_id = init ? init->device_id : 0u;
        ble_adv_record_t *adv = adv_find_or_create(t, ev->conn_advertiser_addr);
        if (c && adv && init)
        {
            c->device_id_master = adv->device_id;
            c->device_id_slave = init_id;
            if (c->first_seen_ms == 0u)
                c->first_seen_ms = now;
            c->crc_init = ev->conn_crc_init;
            ble_piconet_store_seed_candidate(&t->conn_store,
                                             ev->conn_access_address,
                                             ev->conn_crc_init);
            adv->has_linked_aa = 1;
            adv->linked_aa = ev->conn_access_address;
        }
    }

    pthread_mutex_unlock(&t->lock);
}

/* ---------------------------------------------------------------------------
 * Poll: devices (advertisers)
 * ---------------------------------------------------------------------------*/

size_t ble_tracker_get_devices(const ble_tracker_t *t,
                               ble_device_snapshot_t *out,
                               size_t max)
{
    if (!t || !out || max == 0u)
        return 0u;

    pthread_mutex_lock((pthread_mutex_t *)&t->lock);
    size_t n = 0u;
    for (size_t i = 0; i < t->adv_count && n < max; i++)
    {
        const ble_adv_record_t *r = &t->adv[i];
        if (!r->used)
            continue;
        ble_device_snapshot_t *s = &out[n++];
        memset(s, 0, sizeof(*s));
        s->id = r->device_id;
        s->kind = ENTITY_BLE_DEVICE;
        format_addr_bytes(r->adv_addr, s->addr_str, sizeof(s->addr_str));
        snprintf(s->label, sizeof(s->label), "Advertiser");
        s->rssi_valid = rssi_tracker_average(&r->rssi_track, &s->rssi_db);
        s->first_seen_ms = r->first_seen_ms;
        s->last_seen_ms = r->last_seen_ms;
        s->total_packets = r->total_packets;
        s->packet_rate = 0u;
        s->adv_addr = r->adv_addr;
        snprintf(s->addr_type, sizeof(s->addr_type), "%s", r->addr_type);
        snprintf(s->name, sizeof(s->name), "%s", r->name);
        snprintf(s->manufacturer, sizeof(s->manufacturer), "%s",
                 r->manufacturer);
    }
    pthread_mutex_unlock((pthread_mutex_t *)&t->lock);
    return n;
}

/* ---------------------------------------------------------------------------
 * Poll: piconets (connections)
 * ---------------------------------------------------------------------------*/

size_t ble_tracker_get_piconets(const ble_tracker_t *t,
                                ble_piconet_snapshot_t *out,
                                size_t max)
{
    if (!t || !out || max == 0u)
        return 0u;

    pthread_mutex_lock((pthread_mutex_t *)&t->lock);
    size_t n = 0u;
    unsigned int store_count = ble_piconet_store_count(&t->conn_store);
    for (unsigned int i = 0; i < store_count && n < max; i++)
    {
        ble_piconet_t e;
        memset(&e, 0, sizeof(e));
        if (ble_piconet_store_get((ble_piconet_store_t *)&t->conn_store, i, &e) != 0)
            continue;

        /* CRC enforcement: connection candidates that have not yet proven
         * their CRCInit are suppressed from the device list. */
        if (t->enforce_crc && e.state != BLE_PICONET_CONFIRMED)
            continue;

        ble_conn_record_t *c = conn_find_or_create((ble_tracker_t *)t,
                                                   e.access_address);
        if (!c)
            continue;

        if (e.state == BLE_PICONET_CONFIRMED)
        {
            c->crc_init_confirmed = 1;
            c->crc_init = e.crc_init;
            c->state = e.state;
        }

        ble_piconet_snapshot_t *s = &out[n++];
        memset(s, 0, sizeof(*s));
        s->id = c->piconet_id;
        s->kind = ENTITY_BLE_PICONET;
        format_aa(e.access_address, s->addr_str, sizeof(s->addr_str));
        snprintf(s->label, sizeof(s->label), "connection");
        /* Prefer the connection's own data-frame RSSI (every accepted data
         * packet carries signal strength); fall back to the linked
         * advertisers' RSSI when no data-frame samples have been observed
         * yet. */
        {
            int rssi_valid = 0;
            float rssi_db = 0.0f;
            if (rssi_tracker_average(&c->rssi_track, &rssi_db))
                rssi_valid = 1;
            else
            {
                double sum = 0.0;
                int n = 0;
                for (size_t k = 0; k < t->adv_count; k++)
                {
                    const ble_adv_record_t *a = &t->adv[k];
                    if (!a->used)
                        continue;
                    if ((c->device_id_master && a->device_id == c->device_id_master) ||
                        (c->device_id_slave && a->device_id == c->device_id_slave))
                    {
                        float adv_rssi = 0.0f;
                        if (rssi_tracker_average(&a->rssi_track, &adv_rssi))
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
        }
        s->first_seen_ms = c->first_seen_ms;
        s->last_seen_ms = c->last_seen_ms;
        s->total_packets = e.packets_seen; /* all frames received for this AA */
        s->packet_rate = 0u;
        s->access_address = e.access_address;
        s->crc_init_confirmed = c->crc_init_confirmed;
        s->crc_init = c->crc_init;
        s->candidate_count = e.candidate_count;
        s->state = c->state;
        s->device_id_master = c->device_id_master;
        s->device_id_slave = c->device_id_slave;
    }
    pthread_mutex_unlock((pthread_mutex_t *)&t->lock);
    return n;
}
