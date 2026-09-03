/**
 * @file app_summary_view.c
 * @brief Shared CLI summary table matching the GUI FrameListView columns.
 *
 * Columns: No. | Time | RSSI | Protocol | Ch | Address | Source |
 *          Destination | Type  (GUI minus the Info column).
 *
 * Field derivation mirrors src/apps/gui/backend/backend_api.c row builders
 * (ble_packet_trampoline / bredr_packet_trampoline) so the CLI and GUI agree
 * on Address / Source / Destination / Type for every frame class. The old
 * per-protocol key=value summary lines are replaced outright.
 *
 * Each row is a single printf with precision-truncated string fields, so no
 * packet can wrap onto a second line.
 */
#include "app_summary_view.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "ble_codec.h"
#include "ble_display.h"
#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"

/* Fixed column widths (data width, excluding the separating space). */
#define SUM_NO_W    6u
#define SUM_TIME_W  12u
#define SUM_RSSI_W   6u
#define SUM_PROTO_W  8u
#define SUM_CH_W     3u
#define SUM_ADDR_W  20u
#define SUM_SRC_W   17u
#define SUM_DST_W   17u
#define SUM_TYPE_W  14u

static void fmt_time(const rx_metadata_t *meta, char *out, size_t n)
{
    double t = 0.0;
    if (meta && meta->radio_sample_rate_hz > 0u)
        t = (double)meta->radio_start_sample_index /
            (double)meta->radio_sample_rate_hz;
    snprintf(out, n, "%.6f", t);
}

static void fmt_rssi(float rssi, char *out, size_t n)
{
    snprintf(out, n, "%.1f", (double)rssi);
}

static void fmt_addr_plain(char *out, size_t n, const ble_address_t *addr)
{
    if (!addr)
    {
        snprintf(out, n, "--");
        return;
    }
    char buf[18];
    ble_format_addr(buf, addr->addr);
    snprintf(out, n, "%s", buf);
}

/* Plain src/dst derivation for BLE advertising PDUs (mirrors
 * backend_api.c set_src_dst, but without the addr_type side channel since
 * the summary table has no Info column). */
static void ble_src_dst(const ble_adv_pdu_t *adv,
                        char *src, size_t src_n,
                        char *dst, size_t dst_n)
{
    const uint8_t t = (uint8_t)(adv->pdu_type & 0x0Fu);
    switch (t)
    {
    case BLE_PDU_ADV_IND:
        fmt_addr_plain(src, src_n, &adv->payload.adv_ind.adv_addr);
        snprintf(dst, dst_n, "Broadcast");
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        fmt_addr_plain(src, src_n, &adv->payload.adv_direct_ind.adv_addr);
        fmt_addr_plain(dst, dst_n, &adv->payload.adv_direct_ind.target_addr);
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        fmt_addr_plain(src, src_n, &adv->payload.adv_nonconn_ind.adv_addr);
        snprintf(dst, dst_n, "Broadcast");
        break;
    case BLE_PDU_SCAN_REQ:
        fmt_addr_plain(src, src_n, &adv->payload.scan_req.scanner_addr);
        fmt_addr_plain(dst, dst_n, &adv->payload.scan_req.adv_addr);
        break;
    case BLE_PDU_SCAN_RSP:
        fmt_addr_plain(src, src_n, &adv->payload.scan_rsp.adv_addr);
        snprintf(dst, dst_n, "--");
        break;
    case BLE_PDU_CONNECT_IND:
        fmt_addr_plain(src, src_n, &adv->payload.connect_ind.init_addr);
        fmt_addr_plain(dst, dst_n, &adv->payload.connect_ind.adv_addr);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        fmt_addr_plain(src, src_n, &adv->payload.adv_scan_ind.adv_addr);
        snprintf(dst, dst_n, "Broadcast");
        break;
    default:
        snprintf(src, src_n, "--");
        snprintf(dst, dst_n, "--");
        break;
    }
}

static void print_row(unsigned long no,
                      const char *time_s, const char *rssi_s,
                      const char *proto, unsigned int ch,
                      const char *addr, const char *src,
                      const char *dst, const char *type)
{
    /* Precision caps (%.Ns) truncate over-long fields so each packet stays
     * on exactly one line. */
    printf("%-6lu %-12.12s %-6.6s %-8.8s %-3u %-20.20s %-17.17s %-17.17s %-14.14s\n",
           no, time_s, rssi_s, proto, ch, addr, src, dst, type);
}

void app_summary_view_print_header(void)
{
    printf("%-6s %-12s %-6s %-8s %-3s %-20s %-17s %-17s %-14s\n",
           "No.", "Time", "RSSI", "Protocol", "Ch",
           "Address", "Source", "Destination", "Type");
    printf("------ ------------ ------ -------- --- -------------------- ----------------- ----------------- --------------\n");
}

void app_summary_view_print_ble_decode_fail(unsigned long packet_no,
                                            const ble_event_t *event)
{
    const rx_metadata_t *m = &event->meta;
    char time_s[32], rssi_s[16], addr[32];

    fmt_time(m, time_s, sizeof(time_s));
    fmt_rssi(m->rssi_dbr, rssi_s, sizeof(rssi_s));
    snprintf(addr, sizeof(addr), "0x%08" PRIX32, event->frame.access_address);

    print_row(packet_no, time_s, rssi_s, "LE", m->channel_index,
              addr, "--", "--", "DECODE_FAIL");
}

void app_summary_view_print_ble(unsigned long packet_no,
                                const ble_event_t *event)
{
    const rx_metadata_t *m = &event->meta;
    ble_packet_t pkt;
    char time_s[32], rssi_s[16], addr[32];
    char src[32], dst[32], type[32];

    if (ble_decode_frame(&event->frame, m->channel_index, &pkt) != 0)
    {
        app_summary_view_print_ble_decode_fail(packet_no, event);
        return;
    }

    fmt_time(m, time_s, sizeof(time_s));
    fmt_rssi(m->rssi_dbr, rssi_s, sizeof(rssi_s));
    snprintf(addr, sizeof(addr), "0x%08" PRIX32, event->frame.access_address);

    if (!pkt.is_adv_pdu)
    {
        print_row(packet_no, time_s, rssi_s, "LE", m->channel_index,
                  addr, "--", "--", "LL_DATA");
        return;
    }

    snprintf(type, sizeof(type), "%s",
             ble_pdu_type_name(pkt.pdu.adv.pdu_type));
    ble_src_dst(&pkt.pdu.adv, src, sizeof(src), dst, sizeof(dst));

    print_row(packet_no, time_s, rssi_s, "LE", m->channel_index,
              addr, src, dst, type);
}

void app_summary_view_print_bredr(unsigned long packet_no,
                                  const bredr_event_t *event,
                                  const bredr_piconet_snapshot_t *pnet)
{
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *m = &event->meta;
    char time_s[32], rssi_s[16], addr[32];
    char src[32], dst[32], type[32];
    uint32_t lap = frame->lap & 0xFFFFFFu;

    fmt_time(m, time_s, sizeof(time_s));
    fmt_rssi(m->rssi_dbr, rssi_s, sizeof(rssi_s));

    if (pnet && pnet->uap_valid)
        snprintf(addr, sizeof(addr), "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(addr, sizeof(addr), "0x??%06" PRIX32, lap);

    snprintf(src, sizeof(src), "--");
    snprintf(dst, sizeof(dst), "--");
    snprintf(type, sizeof(type), "--");

    /* Attempt a header decode when UAP + CLK context is available (mirrors
     * backend_api.c bredr_packet_trampoline). */
    uint8_t uap = 0u, clk1_6 = 0u;
    int have_ctx = 0;
    if (pnet && pnet->uap_valid)
        uap = pnet->uap;
    if (pnet && pnet->clk_known && m->radio_sample_rate_hz != 0u)
        clk1_6 = pnet->central_clk_1_6;
    have_ctx = (pnet && pnet->uap_valid && pnet->clk_known &&
                m->radio_sample_rate_hz != 0u);

    bredr_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    int decode_ok = 0;
    if (have_ctx)
        decode_ok = bredr_decode_frame(frame, uap, clk1_6, &pkt);

    if (decode_ok > 0)
    {
        snprintf(type, sizeof(type), "%s",
                 bredr_packet_type_name(pkt.header.type));
        unsigned int lt_addr = pkt.header.lt_addr & 0x07u;
        char peer[32];
        if (lt_addr == 0u)
            snprintf(peer, sizeof(peer), "Broadcast");
        else
            snprintf(peer, sizeof(peer), "LT_ADDR %u", lt_addr);

        if ((clk1_6 & 0x1u) == 0u)
        {
            snprintf(src, sizeof(src), "Central");
            snprintf(dst, sizeof(dst), "%s", peer);
        }
        else
        {
            snprintf(src, sizeof(src), "%s", peer);
            snprintf(dst, sizeof(dst), "Central");
        }
    }

    if (lap == 0x9E8B33u || lap == 0x9E8B00u)
        snprintf(type, sizeof(type), "INQUIRY");

    print_row(packet_no, time_s, rssi_s, "BR/EDR", m->channel_index,
              addr, src, dst, type);
}
