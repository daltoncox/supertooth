/**
 * @file backend_api.c
 * @brief C-side implementation of the GUI facade. Compiled as C so it may
 *        freely include liquid/complex/core headers.
 */
#include "backend_api.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "receiver_session.h"
#include "receive_event_models.h"
#include "ble_bitstream_decoder.h"
#include "ble_codec.h"
#include "ble_display.h"
#include "bt_assigned_numbers.h"
#include "radio_common.h"

struct backend_session
{
    receiver_session_t *session;
    backend_ble_row_fn  on_row;
    void               *user;
    unsigned long       packet_count;
};

/* ---------------------------------------------------------------------------
 * Local formatting helpers
 * ---------------------------------------------------------------------------*/

static void fmt_addr_field(char *out, size_t out_sz, const ble_address_t *addr)
{
    if (!addr)
    {
        snprintf(out, out_sz, "--");
        return;
    }
    /* ble_format_addr writes 17 chars + NUL into a char[18]. */
    char buf[18];
    ble_format_addr(buf, addr->addr);

    const char *kind = (addr->kind == BLE_ADDR_RANDOM) ? " (Random)" : " (Public)";
    snprintf(out, out_sz, "%s%s", buf, kind);
}

static void fmt_addr_plain(char *out, size_t out_sz, const ble_address_t *addr)
{
    if (!addr)
    {
        snprintf(out, out_sz, "--");
        return;
    }
    ble_format_addr(out, addr->addr); /* writes exactly 17 chars + NUL */
}

static void set_src_dst(backend_ble_row_t *row, const ble_packet_t *pkt)
{
    const uint8_t t = (uint8_t)(pkt->pdu_type & 0x0Fu);
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.adv_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.adv_direct_ind.adv_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &pkt->payload.adv_direct_ind.target_addr);
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.adv_nonconn_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        break;
    case BLE_PDU_SCAN_REQ:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.scan_req.scanner_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &pkt->payload.scan_req.adv_addr);
        break;
    case BLE_PDU_SCAN_RSP:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.scan_rsp.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "--");
        break;
    case BLE_PDU_CONNECT_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.connect_ind.init_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &pkt->payload.connect_ind.adv_addr);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &pkt->payload.adv_scan_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        break;
    default:
        snprintf(row->src, sizeof(row->src), "--");
        snprintf(row->dst, sizeof(row->dst), "--");
        break;
    }
}

static const uint8_t *adv_data_for(const ble_packet_t *pkt, const uint8_t **out, unsigned int *len)
{
    const uint8_t t = (uint8_t)(pkt->pdu_type & 0x0Fu);
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        *out = pkt->payload.adv_ind.adv_data;
        *len = pkt->payload.adv_ind.adv_data_len;
        return *out;
    case BLE_PDU_ADV_NONCONN_IND:
        *out = pkt->payload.adv_nonconn_ind.adv_data;
        *len = pkt->payload.adv_nonconn_ind.adv_data_len;
        return *out;
    case BLE_PDU_SCAN_RSP:
        *out = pkt->payload.scan_rsp.adv_data;
        *len = pkt->payload.scan_rsp.adv_data_len;
        return *out;
    case BLE_PDU_ADV_SCAN_IND:
        *out = pkt->payload.adv_scan_ind.adv_data;
        *len = pkt->payload.adv_scan_ind.adv_data_len;
        return *out;
    default:
        *out = NULL;
        *len = 0u;
        return NULL;
    }
}

static void add_detail(backend_ble_row_t *row, const char *key, const char *fmt, ...)
{
    if (row->detail_count >= BACKEND_DETAIL_MAX)
        return;
    unsigned int i = row->detail_count++;
    snprintf(row->detail_keys[i], BACKEND_DETAIL_KEY_LEN, "%s", key);

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(row->detail_vals[i], BACKEND_DETAIL_VAL_LEN, fmt, ap);
    va_end(ap);
}

static void build_detail(backend_ble_row_t *row, const ble_packet_t *pkt,
                         const rx_metadata_t *meta, const ble_frame_t *frame)
{
    const uint8_t t = (uint8_t)(pkt->pdu_type & 0x0Fu);
    char buf[64];

    add_detail(row, "PDU Type", "%s (%s)",
               ble_pdu_type_name(pkt->pdu_type), ble_pdu_type_desc(pkt->pdu_type));
    snprintf(buf, sizeof(buf), "%u", meta->channel_index);
    add_detail(row, "Channel", "%s", buf);
    snprintf(buf, sizeof(buf), "0x%08" PRIX32, frame->access_address);
    add_detail(row, "Access Address", "%s", buf);

    /* Addresses by PDU type. */
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.adv_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", pkt->payload.adv_ind.adv_data_len);
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.adv_direct_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.adv_direct_ind.target_addr);
        add_detail(row, "TargetA", "%s", buf);
        add_detail(row, "AdvData Length", "0");
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.adv_nonconn_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", pkt->payload.adv_nonconn_ind.adv_data_len);
        break;
    case BLE_PDU_SCAN_REQ:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.scan_req.scanner_addr);
        add_detail(row, "ScanA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.scan_req.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "AdvData Length", "0");
        break;
    case BLE_PDU_SCAN_RSP:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.scan_rsp.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "ScanA", "--");
        add_detail(row, "AdvData Length", "%u", pkt->payload.scan_rsp.adv_data_len);
        break;
    case BLE_PDU_CONNECT_IND:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.connect_ind.init_addr);
        add_detail(row, "InitA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.connect_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "LLData Length", "%u", pkt->payload.connect_ind.ll_data_len);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        fmt_addr_field(buf, sizeof(buf), &pkt->payload.adv_scan_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", pkt->payload.adv_scan_ind.adv_data_len);
        break;
    default:
        add_detail(row, "Payload", "(reserved/unknown)");
        break;
    }

    add_detail(row, "CRC", "0x%06" PRIX32 " (%s)", pkt->crc,
               ble_verify_crc(pkt) ? "PASS" : "FAIL");
    add_detail(row, "RSSI", "%.1f dBr", meta->rssi_dbr);

    /* AdvData TLV breakdown. */
    const uint8_t *ad = NULL;
    unsigned int ad_len = 0u;
    adv_data_for(pkt, &ad, &ad_len);
    if (ad && ad_len)
    {
        unsigned int i = 0u;
        while (i < ad_len && row->detail_count < BACKEND_DETAIL_MAX)
        {
            uint8_t ad_l = ad[i];
            if (ad_l == 0u || (i + 1u + ad_l) > ad_len)
                break;
            uint8_t ad_type = ad[i + 1u];
            unsigned int vb = i + 2u;
            unsigned int ve = i + 1u + ad_l;

            char hex[BACKEND_DETAIL_VAL_LEN];
            size_t pos = 0u;
            hex[0] = '\0';
            for (unsigned int j = vb; j < ve && pos + 4 < sizeof(hex); j++)
                pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02X ", ad[j]);
            /* trim trailing space */
            if (pos > 0u && hex[pos - 1u] == ' ')
                hex[pos - 1u] = '\0';

            add_detail(row, "AD Structure", "type=0x%02X (%s) len=%u data=%s",
                       ad_type, bt_assigned_ad_type_name(ad_type), ad_l, hex);
            i += 1u + ad_l;
        }
    }
}

static void build_raw(backend_ble_row_t *row, const ble_frame_t *frame)
{
    unsigned int pos = 0u;
    if (pos < BACKEND_RAW_MAX_BYTES)
        row->raw[pos++] = frame->preamble;
    for (unsigned int i = 0u; i < 4u && pos < BACKEND_RAW_MAX_BYTES; i++)
        row->raw[pos++] = (uint8_t)((frame->access_address >> (8u * i)) & 0xFFu);
    for (unsigned int i = 0u; i < frame->raw_pdu_bytes && pos < BACKEND_RAW_MAX_BYTES; i++)
        row->raw[pos++] = frame->raw_pdu[i];
    row->raw_len = pos;
}

/* ---------------------------------------------------------------------------
 * BLE packet trampoline (runs on the session worker thread)
 * ---------------------------------------------------------------------------*/

static void ble_packet_trampoline(const ble_event_t *event, void *user)
{
    backend_session_t *bs = (backend_session_t *)user;
    if (!bs || !event || !bs->on_row)
        return;

    ble_packet_t pkt;
    if (ble_decode_frame(&event->frame, event->meta.channel_index, &pkt) != 0)
    {
        /* Decode failed: still emit a minimal row so the capture is visible. */
        backend_ble_row_t row;
        memset(&row, 0, sizeof(row));
        row.no = ++bs->packet_count;
        const rx_metadata_t *m = &event->meta;
        double t = (m->radio_sample_rate_hz > 0u)
                       ? (double)m->radio_start_sample_index / (double)m->radio_sample_rate_hz
                       : 0.0;
        snprintf(row.time, sizeof(row.time), "%.6f", t);
        snprintf(row.rssi, sizeof(row.rssi), "%.1f", m->rssi_dbr);
        snprintf(row.proto, sizeof(row.proto), "LE");
        row.ch_idx = m->channel_index;
        snprintf(row.addr, sizeof(row.addr), "0x%08" PRIX32, event->frame.access_address);
        snprintf(row.src, sizeof(row.src), "--");
        snprintf(row.dst, sizeof(row.dst), "--");
        snprintf(row.type, sizeof(row.type), "DECODE_FAIL");
        snprintf(row.info, sizeof(row.info), "CRC %s", "FAIL");
        build_raw(&row, &event->frame);
        add_detail(&row, "Channel", "%u", m->channel_index);
        add_detail(&row, "Access Address", "0x%08" PRIX32, event->frame.access_address);
        add_detail(&row, "RSSI", "%.1f dBr", m->rssi_dbr);
        add_detail(&row, "Decode", "FAILED");
        bs->on_row(&row, bs->user);
        return;
    }

    backend_ble_row_t row;
    memset(&row, 0, sizeof(row));
    row.no = ++bs->packet_count;

    const rx_metadata_t *m = &event->meta;
    double t = (m->radio_sample_rate_hz > 0u)
                   ? (double)m->radio_start_sample_index / (double)m->radio_sample_rate_hz
                   : 0.0;
    snprintf(row.time, sizeof(row.time), "%.6f", t);
    snprintf(row.rssi, sizeof(row.rssi), "%.1f", m->rssi_dbr);
    snprintf(row.proto, sizeof(row.proto), "LE");
    row.ch_idx = m->channel_index;
    snprintf(row.addr, sizeof(row.addr), "0x%08" PRIX32, event->frame.access_address);
    snprintf(row.type, sizeof(row.type), "%s", ble_pdu_type_name(pkt.pdu_type));
    snprintf(row.info, sizeof(row.info), "%s, CRC %s",
             ble_pdu_type_desc(pkt.pdu_type),
             ble_verify_crc(&pkt) ? "PASS" : "FAIL");

    set_src_dst(&row, &pkt);
    build_detail(&row, &pkt, m, &event->frame);
    build_raw(&row, &event->frame);

    bs->on_row(&row, bs->user);
}

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

backend_session_t *backend_session_create(void)
{
    backend_session_t *bs = (backend_session_t *)calloc(1, sizeof(*bs));
    if (!bs)
        return NULL;
    bs->session = receiver_session_create();
    if (!bs->session)
    {
        free(bs);
        return NULL;
    }
    return bs;
}

void backend_session_destroy(backend_session_t *session)
{
    if (!session)
        return;
    if (session->session)
        receiver_session_destroy(session->session);
    free(session);
}

int backend_session_run_ble(backend_session_t *session,
                            uint8_t ble_channel,
                            int input_type,
                            const char *device_id,
                            backend_ble_row_fn on_row,
                            void *user)
{
    if (!session)
        return -1;

    session->on_row = on_row;
    session->user = user;
    session->packet_count = 0ul;

    uint64_t freq_hz = 0;
    switch (ble_channel)
    {
    case BACKEND_BLE_CH38:
        freq_hz = (uint64_t)BLE_CH38_FREQ_HZ;
        break;
    case BACKEND_BLE_CH39:
        freq_hz = (uint64_t)BLE_CH39_FREQ_HZ;
        break;
    case BACKEND_BLE_CH37:
    default:
        ble_channel = BACKEND_BLE_CH37;
        freq_hz = (uint64_t)BLE_CH37_FREQ_HZ;
        break;
    }

    radio_device_type_t dev_type = RADIO_DEVICE_HACKRF;
    (void)input_type; /* Only HackRF is supported by the backend today. */

    receiver_ble_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.ble_channel = ble_channel;
    cfg.lo_freq_hz = freq_hz;
    cfg.device_type = dev_type;
    cfg.device_id = device_id;
    cfg.debug = 0;

    receiver_ble_callbacks_t cb;
    memset(&cb, 0, sizeof(cb));
    cb.on_packet = ble_packet_trampoline;
    cb.user = session;

    return receiver_session_run_ble(session->session, &cfg, &cb);
}

void backend_session_request_stop(backend_session_t *session)
{
    if (!session || !session->session)
        return;
    receiver_session_request_stop(session->session);
}
