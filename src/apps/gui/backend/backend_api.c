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

#include "session.h"
#include "receive_event_models.h"
#include "ble_bitstream_decoder.h"
#include "ble_codec.h"
#include "ble_display.h"
#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"
#include "bredr_display.h"
#include "bt_assigned_numbers.h"
#include "radio_common.h"

struct backend_session
{
    session_t *session;
    backend_row_fn  on_row;
    void               *user;
    backend_stopped_fn on_stopped;
    unsigned long       packet_count;
    int                 enforce_crc;   /* drop BLE frames whose CRC fails */
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

/* Classify a BLE address into its device-list "type" subfield. Public
 * addresses are reported by kind; random addresses are subdivided by the two
 * most-significant bits of the address. The two indicator bits read in the
 * order (bit47, bit46) map to:
 *   00 -> NONRESOLVABLE
 *   01 -> RESOLVABLE
 *   10 -> RESERVED
 *   11 -> STATIC
 * (RESERVED is surfaced as-is: an address tagged random with the reserved
 * pattern indicates a real anomaly worth showing rather than hiding.) */
static const char *ble_addr_subtype(const ble_address_t *addr)
{
    if (!addr)
        return "";
    if (addr->kind == BLE_ADDR_PUBLIC)
        return "PUBLIC";
    switch ((addr->addr[5] >> 6) & 0x03u)
    {
    case 0x03u: return "STATIC";         /* random static */
    case 0x02u: return "RESERVED";       /* reserved */
    case 0x01u: return "RESOLVABLE";     /* random private resolvable */
    default:    return "NONRESOLVABLE";  /* 0x00: random private non-resolvable */
    }
}

static void set_src_dst(backend_row_t *row, const ble_adv_pdu_t *adv)
{
    const uint8_t t = (uint8_t)(adv->pdu_type & 0x0Fu);
    row->addr_type[0] = '\0';
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.adv_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.adv_ind.adv_addr));
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.adv_direct_ind.adv_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &adv->payload.adv_direct_ind.target_addr);
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.adv_direct_ind.adv_addr));
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.adv_nonconn_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.adv_nonconn_ind.adv_addr));
        break;
    case BLE_PDU_SCAN_REQ:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.scan_req.scanner_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &adv->payload.scan_req.adv_addr);
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.scan_req.scanner_addr));
        break;
    case BLE_PDU_SCAN_RSP:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.scan_rsp.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "--");
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.scan_rsp.adv_addr));
        break;
    case BLE_PDU_CONNECT_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.connect_ind.init_addr);
        fmt_addr_plain(row->dst, sizeof(row->dst), &adv->payload.connect_ind.adv_addr);
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.connect_ind.init_addr));
        break;
    case BLE_PDU_ADV_SCAN_IND:
        fmt_addr_plain(row->src, sizeof(row->src), &adv->payload.adv_scan_ind.adv_addr);
        snprintf(row->dst, sizeof(row->dst), "Broadcast");
        snprintf(row->addr_type, sizeof(row->addr_type), "%s",
                 ble_addr_subtype(&adv->payload.adv_scan_ind.adv_addr));
        break;
    default:
        snprintf(row->src, sizeof(row->src), "--");
        snprintf(row->dst, sizeof(row->dst), "--");
        break;
    }
}

static const uint8_t *adv_data_for(const ble_adv_pdu_t *adv, const uint8_t **out, unsigned int *len)
{
    const uint8_t t = (uint8_t)(adv->pdu_type & 0x0Fu);
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        *out = adv->payload.adv_ind.adv_data;
        *len = adv->payload.adv_ind.adv_data_len;
        return *out;
    case BLE_PDU_ADV_NONCONN_IND:
        *out = adv->payload.adv_nonconn_ind.adv_data;
        *len = adv->payload.adv_nonconn_ind.adv_data_len;
        return *out;
    case BLE_PDU_SCAN_RSP:
        *out = adv->payload.scan_rsp.adv_data;
        *len = adv->payload.scan_rsp.adv_data_len;
        return *out;
    case BLE_PDU_ADV_SCAN_IND:
        *out = adv->payload.adv_scan_ind.adv_data;
        *len = adv->payload.adv_scan_ind.adv_data_len;
        return *out;
    default:
        *out = NULL;
        *len = 0u;
        return NULL;
    }
}

static void add_detail(backend_row_t *row, const char *key, const char *fmt, ...)
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

static void build_detail(backend_row_t *row, const ble_packet_t *pkt,
                         const rx_metadata_t *meta, const ble_frame_t *frame)
{
    const ble_adv_pdu_t *adv = &pkt->pdu.adv;
    const uint8_t t = (uint8_t)(adv->pdu_type & 0x0Fu);
    char buf[64];

    add_detail(row, "PDU Type", "%s (%s)",
               ble_pdu_type_name(adv->pdu_type), ble_pdu_type_desc(adv->pdu_type));
    snprintf(buf, sizeof(buf), "%u", meta->channel_index);
    add_detail(row, "Channel", "%s", buf);
    snprintf(buf, sizeof(buf), "0x%08" PRIX32, frame->access_address);
    add_detail(row, "Access Address", "%s", buf);

    /* Addresses by PDU type. */
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.adv_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", adv->payload.adv_ind.adv_data_len);
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.adv_direct_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &adv->payload.adv_direct_ind.target_addr);
        add_detail(row, "TargetA", "%s", buf);
        add_detail(row, "AdvData Length", "0");
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.adv_nonconn_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", adv->payload.adv_nonconn_ind.adv_data_len);
        break;
    case BLE_PDU_SCAN_REQ:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.scan_req.scanner_addr);
        add_detail(row, "ScanA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &adv->payload.scan_req.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "AdvData Length", "0");
        break;
    case BLE_PDU_SCAN_RSP:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.scan_rsp.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "ScanA", "--");
        add_detail(row, "AdvData Length", "%u", adv->payload.scan_rsp.adv_data_len);
        break;
    case BLE_PDU_CONNECT_IND:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.connect_ind.init_addr);
        add_detail(row, "InitA", "%s", buf);
        fmt_addr_field(buf, sizeof(buf), &adv->payload.connect_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "LLData Length", "%u", adv->payload.connect_ind.ll_data_len);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        fmt_addr_field(buf, sizeof(buf), &adv->payload.adv_scan_ind.adv_addr);
        add_detail(row, "AdvA", "%s", buf);
        add_detail(row, "TargetA", "--");
        add_detail(row, "AdvData Length", "%u", adv->payload.adv_scan_ind.adv_data_len);
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
    adv_data_for(adv, &ad, &ad_len);
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

            /* AD type 0x08 (Shortened Local Name) or 0x09 (Complete Local
             * Name) carry the advertiser's local name verbatim as their
             * payload (typically ASCII / UTF-8, no NUL terminator). Surface
             * it as a dedicated "Device Name" detail so the GUI can show a
             * human-readable identifier in the device list. Non-printable
             * bytes are substituted with '.' to keep the value clean. */
            if (ad_type == 0x08u || ad_type == 0x09u)
            {
                char name[BACKEND_DETAIL_VAL_LEN];
                size_t nl = 0u;
                for (unsigned int j = vb; j < ve && nl + 1u < sizeof(name); j++)
                {
                    char c = (char)ad[j];
                    name[nl++] = (c >= 0x20 && c < 0x7f) ? c : '.';
                }
                name[nl] = '\0';
                add_detail(row, "Device Name", "%s", name);
            }

            /* AD type 0xFF (Manufacturer Specific Data) carries a 16-bit
             * little-endian Company Identifier as its leading payload,
             * assigned by the Bluetooth SIG. Surface it as a dedicated
             * "Manufacturer" detail so the GUI can show a human-readable
             * vendor in the device info tab. */
            if (ad_type == 0xFFu && (ve - vb) >= 2u)
            {
                uint16_t cid = (uint16_t)ad[vb] |
                               ((uint16_t)ad[vb + 1u] << 8u);
                add_detail(row, "Manufacturer", "%s (0x%04X)",
                           bt_assigned_company_name(cid), cid);
            }

            i += 1u + ad_l;
        }
    }
}

static void build_data_detail(backend_row_t *row, const ble_packet_t *pkt,
                              const rx_metadata_t *meta)
{
    const ble_data_pdu_t *data = &pkt->pdu.data;

    add_detail(row, "Channel", "%u", meta->channel_index);
    add_detail(row, "Access Address", "0x%08" PRIX32, pkt->access_address);
    add_detail(row, "PHY", "%s", receiver_phy_name(pkt->phy));
    add_detail(row, "CRCInit", "0x%06" PRIX32, pkt->crc_init);
    add_detail(row, "LLID", "%u [%s]", (unsigned int)data->llid,
               ble_llid_name(data->llid));
    add_detail(row, "NESN", "%u", (unsigned int)data->nesn);
    add_detail(row, "SN", "%u", (unsigned int)data->sn);
    add_detail(row, "MD", "%u", (unsigned int)data->md);
    add_detail(row, "Length", "%u", (unsigned int)data->payload_len);

    /* Payload hex, capped so it stays inside the detail value buffer. */
    if (data->payload_len == 0u)
    {
        add_detail(row, "Payload", "(empty)");
    }
    else
    {
        char hex[BACKEND_DETAIL_VAL_LEN];
        size_t pos = 0u;
        hex[0] = '\0';
        unsigned int show = data->payload_len;
        unsigned int max_bytes = (BACKEND_DETAIL_VAL_LEN - 24u) / 3u;
        if (show > max_bytes)
            show = max_bytes;
        for (unsigned int i = 0u; i < show; i++)
            pos += (size_t)snprintf(hex + pos, sizeof(hex) - pos, "%02X ",
                                    data->payload[i]);
        if (pos > 0u && hex[pos - 1u] == ' ')
            hex[pos - 1u] = '\0';
        if (show < data->payload_len)
            add_detail(row, "Payload", "%s ... (+%u bytes)", hex,
                       (unsigned int)(data->payload_len - show));
        else
            add_detail(row, "Payload", "%s", hex);
    }

    add_detail(row, "CRC", "0x%06" PRIX32 " (%s)", pkt->crc,
               ble_verify_crc(pkt) ? "PASS" : "FAIL");
    add_detail(row, "RSSI", "%.1f dBr", meta->rssi_dbr);
}

static void build_raw(backend_row_t *row, const ble_frame_t *frame)
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
        /* When CRC enforcement is on, drop decode failures silently — the
         * receiver effectively goes back to searching and emits nothing.
         * Otherwise surface a minimal "DECODE_FAIL" row so the capture is
         * still visible in the UI. */
        if (bs->enforce_crc)
            return;

        backend_row_t row;
        memset(&row, 0, sizeof(row));
        row.no = ++bs->packet_count;
        const rx_metadata_t *m = &event->meta;
        double t = (m->radio_sample_rate_hz > 0u)
                       ? (double)m->radio_start_sample_index / (double)m->radio_sample_rate_hz
                       : 0.0;
        snprintf(row.time, sizeof(row.time), "%.6f", t);
        row.rssi_db = m->rssi_dbr;
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

    /* CRC enforcement: drop frames that decoded but failed CRC verification. */
    if (bs->enforce_crc && !ble_verify_crc(&pkt))
        return;

    backend_row_t row;
    memset(&row, 0, sizeof(row));
    row.no = ++bs->packet_count;

    const rx_metadata_t *m = &event->meta;
    double t = (m->radio_sample_rate_hz > 0u)
                   ? (double)m->radio_start_sample_index / (double)m->radio_sample_rate_hz
                   : 0.0;
    snprintf(row.time, sizeof(row.time), "%.6f", t);
    row.rssi_db = m->rssi_dbr;
    snprintf(row.proto, sizeof(row.proto), "LE");
    row.ch_idx = m->channel_index;
    snprintf(row.addr, sizeof(row.addr), "0x%08" PRIX32, event->frame.access_address);

    /* Data-channel (LL) PDUs carry no device addresses: the connection is
     * identified by its random access address. The framer only emits these
     * once the per-connection CRCInit is confirmed, so every row below is a
     * recovered link-layer connection. */
    if (!pkt.is_adv_pdu)
    {
        const ble_data_pdu_t *data = &pkt.pdu.data;
        snprintf(row.type, sizeof(row.type), "LL_DATA");
        snprintf(row.info, sizeof(row.info),
                 "llid=%s sn=%u nesn=%u md=%u len=%u crc=%s",
                 ble_llid_name(data->llid),
                 (unsigned int)data->sn,
                 (unsigned int)data->nesn,
                 (unsigned int)data->md,
                 (unsigned int)data->payload_len,
                 ble_verify_crc(&pkt) ? "PASS" : "FAIL");
        snprintf(row.src, sizeof(row.src), "--");
        snprintf(row.dst, sizeof(row.dst), "--");
        build_data_detail(&row, &pkt, m);
        build_raw(&row, &event->frame);
        bs->on_row(&row, bs->user);
        return;
    }

    snprintf(row.type, sizeof(row.type), "%s", ble_pdu_type_name(pkt.pdu.adv.pdu_type));
    snprintf(row.info, sizeof(row.info), "%s, CRC %s",
             ble_pdu_type_desc(pkt.pdu.adv.pdu_type),
             ble_verify_crc(&pkt) ? "PASS" : "FAIL");

    set_src_dst(&row, &pkt.pdu.adv);
    build_detail(&row, &pkt, m, &event->frame);
    build_raw(&row, &event->frame);

    bs->on_row(&row, bs->user);
}

/* ---------------------------------------------------------------------------
 * BR/EDR packet trampoline (runs on the session worker thread)
 * ---------------------------------------------------------------------------*/

/* Replicates the small CLK1-6 derivation in bredr_display.c's
 * bredr_build_decode_inputs() so the facade can attempt a header decode
 * without depending on static CLI helpers. Returns 1 when both UAP and
 * CLK1-6 are available, 0 otherwise. */
static int bredr_gui_build_decode_inputs(const bredr_piconet_snapshot_t *pnet,
                                         const rx_metadata_t *meta,
                                         uint8_t *uap_out,
                                         uint8_t *clk1_6_out)
{
    if (!uap_out || !clk1_6_out)
        return 0;
    *uap_out = 0u;
    *clk1_6_out = 0u;
    if (!pnet)
        return 0;

    int have_uap = pnet->uap_found;
    int have_clk = pnet->clk_known && meta && meta->radio_sample_rate_hz != 0u;
    if (have_uap)
        *uap_out = pnet->uap;
    if (have_clk)
    {
        uint64_t num = meta->radio_start_sample_index * 1600u
                     + (uint64_t)(meta->radio_sample_rate_hz / 2u);
        uint32_t rx_clk_1600 = (uint32_t)(num / (uint64_t)meta->radio_sample_rate_hz);
        uint32_t delta = rx_clk_1600 - pnet->last_successful_rx_clk_1600;
        *clk1_6_out = (uint8_t)((pnet->central_clk_1_6 + delta) & 0x3Fu);
    }
    return have_uap && have_clk;
}

/* Raw on-air bytes: the 54-bit FEC header (packed LSB-first) followed by the
 * on-air payload bytes. Not dewhitened. */
static void bredr_build_raw(backend_row_t *row, const bredr_frame_t *frame)
{
    unsigned int pos = 0u;
    if (frame->has_header)
    {
        uint64_t h = frame->header_raw & 0x003FFFFFFFFFFFFFull;
        unsigned int header_bytes = (54u + 7u) / 8u;
        for (unsigned int i = 0u; i < header_bytes && pos < BACKEND_RAW_MAX_BYTES; i++)
        {
            uint8_t b = 0u;
            for (unsigned int bit = 0u; bit < 8u; bit++)
            {
                unsigned int idx = i * 8u + bit;
                if (idx >= 54u)
                    break;
                if ((h >> idx) & 0x01ull)
                    b |= (uint8_t)(1u << bit);
            }
            row->raw[pos++] = b;
        }
    }
    unsigned int n = bredr_frame_air_payload_bytes(frame);
    for (unsigned int i = 0u; i < n && pos < BACKEND_RAW_MAX_BYTES; i++)
        row->raw[pos++] = frame->air_payload[i];
    row->raw_len = pos;
}

static void bredr_packet_trampoline(const bredr_event_t *event,
                                    const bredr_piconet_snapshot_t *pnet,
                                    void *user)
{
    backend_session_t *bs = (backend_session_t *)user;
    if (!bs || !event || !bs->on_row)
        return;

    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *m = &event->meta;

    backend_row_t row;
    memset(&row, 0, sizeof(row));
    row.no = ++bs->packet_count;

    double t = (m->radio_sample_rate_hz > 0u)
                   ? (double)m->radio_start_sample_index / (double)m->radio_sample_rate_hz
                   : 0.0;
    snprintf(row.time, sizeof(row.time), "%.6f", t);
    row.rssi_db = m->rssi_dbr;
    snprintf(row.proto, sizeof(row.proto), "BR/EDR");
    row.ch_idx = m->channel_index;

    uint32_t lap = frame->lap & 0xFFFFFFu;

    /* Address: full UAP+LAP (UAP shown as "??" until recovered). */
    if (pnet && pnet->uap_found)
        snprintf(row.addr, sizeof(row.addr), "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(row.addr, sizeof(row.addr), "0x??%06" PRIX32, lap);

    /* src/dst/type stay "--" until the clock is known and the header can be
     * dewhitened (i.e. a full decode succeeds). */
    snprintf(row.src, sizeof(row.src), "--");
    snprintf(row.dst, sizeof(row.dst), "--");
    snprintf(row.type, sizeof(row.type), "--");

    /* Attempt header decode if UAP + CLK1-6 context is available. */
    uint8_t uap = 0u, clk1_6 = 0u;
    int have_ctx = bredr_gui_build_decode_inputs(pnet, m, &uap, &clk1_6);

    bredr_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    int decode_ok = 0;
    if (have_ctx)
        decode_ok = bredr_decode_frame(frame, uap, clk1_6, &pkt);
    else if (frame->has_header)
        pkt.limit = BREDR_DECODE_LIMIT_MISSING_CONTEXT;

    if (decode_ok > 0)
    {
        snprintf(row.type, sizeof(row.type), "%s",
                 bredr_packet_type_name(pkt.header.type));

        /* Direction: CLK1 (the LSB of CLK1-6) is 0 on master→slave slots and
         * 1 on slave→master slots. LT_ADDR 0 means broadcast. */
        unsigned int lt_addr = pkt.header.lt_addr & 0x07u;
        char peer[BACKEND_ADDR_TEXT_LEN];
        if (lt_addr == 0u)
            snprintf(peer, sizeof(peer), "Broadcast");
        else
            snprintf(peer, sizeof(peer), "LT_ADDR %u", lt_addr);

        if ((clk1_6 & 0x1u) == 0u)
        {
            /* Even slot: the Central is transmitting. */
            snprintf(row.src, sizeof(row.src), "Central");
            snprintf(row.dst, sizeof(row.dst), "%s", peer);
        }
        else
        {
            /* Odd slot: the addressed slave is transmitting to the Central. */
            snprintf(row.src, sizeof(row.src), "%s", peer);
            snprintf(row.dst, sizeof(row.dst), "Central");
        }
    }

    /* Info summary line (mirrors the CLI bredr_print_packet_summary_line). */
    char uap_str[8];
    char clk_str[8];
    if (pnet && pnet->uap_found)
        snprintf(uap_str, sizeof(uap_str), "%02X", pnet->uap);
    else
        snprintf(uap_str, sizeof(uap_str), "??");
    if (pnet && pnet->clk_known)
        snprintf(clk_str, sizeof(clk_str), "%02u", pnet->central_clk_1_6);
    else
        snprintf(clk_str, sizeof(clk_str), "??");
    snprintf(row.info, sizeof(row.info), "ac=%u uap=%s clk=%s track=%d",
             frame->ac_errors, uap_str, clk_str,
             pnet ? pnet->tracking_state : -1);

    bredr_build_raw(&row, frame);

    /* Detail key/value pairs for the Frame Info pane. */
    add_detail(&row, "Channel", "%u", m->channel_index);
    add_detail(&row, "LAP", "0x%06" PRIX32, lap);
    add_detail(&row, "AC Errors", "%u", frame->ac_errors);
    if (pnet)
    {
        if (pnet->uap_found)
            add_detail(&row, "UAP", "0x%02X", pnet->uap);
        else
            add_detail(&row, "UAP", "??");
        add_detail(&row, "Tracking", "%d", pnet->tracking_state);
        if (pnet->clk_known)
            add_detail(&row, "CLK1-6", "%u", pnet->central_clk_1_6);
        else
            add_detail(&row, "CLK1-6", "??");
        add_detail(&row, "Piconet Packets", "%lu", pnet->total_packets);
    }
    add_detail(&row, "RSSI", "%.1f dBr", m->rssi_dbr);
    if (frame->has_header)
    {
        add_detail(&row, "Header (FEC)", "0x%014" PRIX64,
                   frame->header_raw & 0x003FFFFFFFFFFFFFull);
        if (decode_ok > 0)
        {
            add_detail(&row, "HEC", "0x%02X (%s)", pkt.header.hec,
                       pkt.header.hec_ok ? "PASS" : "FAIL");
            add_detail(&row, "TYPE", "%u (%s)", pkt.header.type & 0x0Fu,
                       bredr_packet_type_name(pkt.header.type));
            add_detail(&row, "LT_ADDR", "%u", pkt.header.lt_addr & 0x07u);
            add_detail(&row, "FLOW", "%u", pkt.header.flow & 1u);
            add_detail(&row, "ARQN", "%u", pkt.header.arqn & 1u);
            add_detail(&row, "SEQN", "%u", pkt.header.seqn & 1u);
            add_detail(&row, "Payload Family", "%s",
                       bredr_payload_family_name(pkt.family));
        }
        else if (pkt.limit != BREDR_DECODE_LIMIT_NONE &&
                 pkt.limit != BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH)
        {
            add_detail(&row, "Decode", "%s", bredr_decode_limit_desc(pkt.limit));
        }
        else
        {
            add_detail(&row, "Decode", "raw packet only");
        }
    }
    else
    {
        add_detail(&row, "Header", "(none - shortened access code)");
    }

    /* Payload preview (first bytes). */
    unsigned int pl_bytes = bredr_frame_air_payload_bytes(frame);
    if (pl_bytes > 0u)
    {
        char hex[BACKEND_DETAIL_VAL_LEN];
        size_t hp = 0u;
        hex[0] = '\0';
        unsigned int show = pl_bytes < 16u ? pl_bytes : 16u;
        for (unsigned int i = 0u; i < show && hp + 4 < sizeof(hex); i++)
            hp += (size_t)snprintf(hex + hp, sizeof(hex) - hp, "%02X ",
                                   frame->air_payload[i]);
        if (hp > 0u && hex[hp - 1u] == ' ')
            hex[hp - 1u] = '\0';
        add_detail(&row, "Payload Preview", "%s", hex);
    }

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
    bs->session = (session_t *)calloc(1, sizeof(*bs->session));
    if (!bs->session)
    {
        free(bs);
        return NULL;
    }
    return bs;
}

void backend_session_set_stopped_callback(backend_session_t *session,
                                          backend_stopped_fn on_stopped,
                                          void *user)
{
    if (!session)
        return;
    session->on_stopped = on_stopped;
    session->user = user;
}

static void backend_session_stopped_trampoline(void *user)
{
    backend_session_t *bs = (backend_session_t *)user;
    if (bs && bs->on_stopped)
        bs->on_stopped(bs->user);
}

void backend_session_destroy(backend_session_t *session)
{
    if (!session)
        return;
    if (session->session)
        session_destroy(session->session);
    free(session->session);
    free(session);
}

int backend_session_run_ble(backend_session_t *session,
                            unsigned int bottom_le_rf,
                            unsigned int le_channel_count,
                            int input_type,
                            const char *device_id,
                            int enforce_crc,
                            backend_row_fn on_row,
                            void *user)
{
    if (!session || !session->session)
        return -1;

    session_t *s = session->session;
    session->on_row = on_row;
    session->user = user;
    session->packet_count = 0ul;
    session->enforce_crc = enforce_crc ? 1 : 0;

    radio_device_type_t dev_type = RADIO_DEVICE_HACKRF;
    (void)input_type; /* Only HackRF is supported by the backend today. */

    /* Defensive clamping of the LE window: 40 RF channels (0..39), up to
     * BLE_SESSION_MAX_CHANNELS processors. */
    if (le_channel_count < 1u)
        le_channel_count = 1u;
    if (le_channel_count > BLE_SESSION_MAX_CHANNELS)
        le_channel_count = BLE_SESSION_MAX_CHANNELS;
    if (bottom_le_rf >= BLE_RF_CHANNEL_COUNT)
        bottom_le_rf = BLE_RF_CHANNEL_COUNT - 1u;
    if (bottom_le_rf + le_channel_count > BLE_RF_CHANNEL_COUNT)
        le_channel_count = BLE_RF_CHANNEL_COUNT - bottom_le_rf;

    session_config_t cfg = {
        .device_type = dev_type,
        .device_id = device_id,
        .debug = 0,
    };
    if (session_init(s, &cfg) != 0)
        return -1;

    session_ble_config_t ble_cfg = { .enforce_crc = session->enforce_crc };
    session_enable_ble(s, &ble_cfg, ble_packet_trampoline, session);
    session_set_stopped_callback(s, backend_session_stopped_trampoline, session);

    return session_tune(s, SESSION_REF_BLE, bottom_le_rf, le_channel_count) == 0
                ? session_run(s)
                : -1;
}

int backend_session_run_bredr(backend_session_t *session,
                              unsigned int channel_count,
                              unsigned int bottom_channel,
                              int input_type,
                              const char *device_id,
                              backend_row_fn on_row,
                              void *user)
{
    if (!session)
        return -1;

    session->on_row = on_row;
    session->user = user;
    session->packet_count = 0ul;

    radio_device_type_t dev_type = RADIO_DEVICE_HACKRF;
    (void)input_type; /* Only HackRF is supported by the backend today. */

    /* Defensive clamping, mirroring supertooth-bredr validation. The on-air
     * band is channels 0..78. */
    channel_count &= ~1u;
    if (channel_count < 2u)
        channel_count = 2u;
    if (channel_count > BREDR_SESSION_MAX_CHANNELS)
        channel_count = BREDR_SESSION_MAX_CHANNELS;
    unsigned int max_bottom = 78u - (channel_count - 1u);
    if (bottom_channel > max_bottom)
        bottom_channel = max_bottom;

    session_config_t cfg = {
        .device_type = dev_type,
        .device_id = device_id,
        .debug = 0,
    };
    if (session_init(session->session, &cfg) != 0)
        return -1;

    session_bredr_config_t bredr_cfg = {
        .rssi_averaging_window = BREDR_SESSION_DEFAULT_RSSI_AVERAGING_WINDOW,
    };
    session_enable_bredr(session->session, &bredr_cfg, bredr_packet_trampoline, session);
    session_set_stopped_callback(session->session, backend_session_stopped_trampoline, session);

    return session_tune(session->session, SESSION_REF_BREDR, bottom_channel, channel_count) == 0
                ? session_run(session->session)
                : -1;
}

int backend_session_run_hybrid(backend_session_t *session,
                               unsigned int channel_count,
                               unsigned int bottom_channel,
                               int le_grid,
                               uint8_t ble_channel,
                               int input_type,
                               const char *device_id,
                               int enforce_crc,
                               backend_row_fn on_row,
                               void *user)
{
    if (!session)
        return -1;

    session->on_row = on_row;
    session->user = user;
    session->packet_count = 0ul;
    session->enforce_crc = enforce_crc ? 1 : 0;

    radio_device_type_t dev_type = RADIO_DEVICE_HACKRF;
    (void)input_type; /* Only HackRF is supported by the backend today. */

    /* Defensive validation of the channel window. On the BR/EDR grid the
     * window is channel_count MHz (even count); on the LE grid it is
     * channel_count+1 MHz (odd count, even bottom). */
    session_protocol_ref_t ref = SESSION_REF_BREDR;
    if (le_grid == BACKEND_GRID_LE)
    {
        /* The GUI passes the window in BR/EDR-style MHz units (an odd
         * channel_count spanning channel_count+1 MHz and an even
         * bottom_channel). session_tune() expects LE RF units on the LE
         * grid (one LE channel per 2 MHz), so halve both before tuning. */
        bottom_channel &= ~1u;
        if (channel_count < 1u)
            channel_count = 1u;
        if (channel_count > 2u * BLE_SESSION_MAX_CHANNELS - 1u)
            channel_count = 2u * BLE_SESSION_MAX_CHANNELS - 1u;
        if ((channel_count & 1u) == 0u)
            channel_count -= 1u;

        channel_count = (channel_count + 1u) / 2u;
        if (channel_count > BLE_SESSION_MAX_CHANNELS)
            channel_count = BLE_SESSION_MAX_CHANNELS;
        bottom_channel /= 2u;
        ref = SESSION_REF_BLE;
    }
    else
    {
        channel_count &= ~1u;
        if (channel_count < 2u)
            channel_count = 2u;
        if (channel_count > BREDR_SESSION_MAX_CHANNELS)
            channel_count = BREDR_SESSION_MAX_CHANNELS;
    }
    unsigned int max_bottom = (ref == SESSION_REF_BLE)
                                  ? BLE_RF_CHANNEL_COUNT - channel_count
                                  : 78u - (channel_count - 1u);
    if (bottom_channel > max_bottom)
        bottom_channel = max_bottom;

    session_config_t cfg = {
        .device_type = dev_type,
        .device_id = device_id,
        .debug = 0,
    };
    if (session_init(session->session, &cfg) != 0)
        return -1;

    session_bredr_config_t bredr_cfg = {
        .rssi_averaging_window = BREDR_SESSION_DEFAULT_RSSI_AVERAGING_WINDOW,
    };
    session_enable_bredr(session->session, &bredr_cfg, bredr_packet_trampoline, session);

    /* BLE is always available on the hybrid window; the picked advertising
     * channel (if any) acts as a BLE on/off for the higher layers. */
    if (ble_channel >= 37u && ble_channel <= 39u)
    {
        session_ble_config_t ble_cfg = { .enforce_crc = session->enforce_crc };
        session_enable_ble(session->session, &ble_cfg, ble_packet_trampoline, session);
    }

    session_set_stopped_callback(session->session, backend_session_stopped_trampoline, session);

    return session_tune(session->session, ref, bottom_channel, channel_count) == 0
                ? session_run(session->session)
                : -1;
}

void backend_session_request_stop(backend_session_t *session)
{
    if (!session || !session->session)
        return;
    session_request_stop(session->session);
}
