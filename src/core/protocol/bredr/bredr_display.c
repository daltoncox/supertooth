#include "bredr_display.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "bredr_codec.h"
#include "phy.h"

static const char *bredr_tracking_state_desc(int tracking_state)
{
    if (tracking_state < 0)
        return "CLK1-6 never found";
    if (tracking_state == 0)
        return "CLK1-6 reacquire required";
    if (tracking_state >= 5)
        return "strong lock";
    return "tracking";
}

static int bredr_piconet_has_active_track(const bredr_piconet_snapshot_t *pnet)
{
    return pnet && pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0;
}

static void bredr_format_piconet_id(char out[16],
                                    const bredr_frame_t *frame,
                                    const bredr_piconet_snapshot_t *pnet)
{
    uint32_t lap = frame ? (frame->lap & 0xFFFFFFu) : 0u;
    if (pnet)
        lap = pnet->lap & 0xFFFFFFu;

    if (pnet && pnet->uap_found)
        snprintf(out, 16, "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(out, 16, "0x??%06" PRIX32, lap);
}

static void bredr_format_rssi_value(char out[8], int seen, float value)
{
    if (seen)
        snprintf(out, 8, "%6.1f", value);
    else
        snprintf(out, 8, "  --.-");
}

static void bredr_print_payload_preview(const bredr_frame_t *frame)
{
    unsigned int air_payload_bytes = bredr_frame_air_payload_bytes(frame);
    if (!frame || air_payload_bytes == 0u)
    {
        printf("Payload      : (none)\n");
        return;
    }

    unsigned int show = air_payload_bytes < 32u ? air_payload_bytes : 32u;
    printf("Payload      :");
    for (unsigned int i = 0; i < show; i++)
    {
        if (i != 0u && (i % 16u) == 0u)
            printf("\n               %02X", frame->air_payload[i]);
        else
            printf(" %02X", frame->air_payload[i]);
    }
    if (air_payload_bytes > show)
        printf("...");
    printf("\n");
}

static uint32_t bredr_sample_to_rx_clk_1600(uint64_t radio_start_sample_index,
                                            unsigned int radio_sample_rate_hz)
{
    uint64_t num;

    if (radio_sample_rate_hz == 0u)
        return 0u;

    num = radio_start_sample_index * 1600u + (uint64_t)(radio_sample_rate_hz / 2u);
    return (uint32_t)(num / (uint64_t)radio_sample_rate_hz);
}

static int bredr_build_decode_inputs(const bredr_piconet_snapshot_t *pnet,
                                     const rx_metadata_t *meta,
                                     uint8_t *uap_out,
                                     uint8_t *clk1_6_out)
{
    uint32_t rx_clk_1600;
    uint32_t delta;

    if (!uap_out || !clk1_6_out)
        return 0;

    *uap_out = 0u;
    *clk1_6_out = 0u;

    if (!pnet)
        return 0;

    if (pnet->uap_found)
        *uap_out = pnet->uap;
    if (pnet->clk_known && meta && meta->radio_sample_rate_hz != 0u)
    {
        rx_clk_1600 = bredr_sample_to_rx_clk_1600(meta->radio_start_sample_index,
                                                  meta->radio_sample_rate_hz);
        delta = rx_clk_1600 - pnet->last_successful_rx_clk_1600;
        *clk1_6_out = (uint8_t)((pnet->central_clk_1_6 + delta) & 0x3Fu);
    }

    return pnet->uap_found && pnet->clk_known && meta && meta->radio_sample_rate_hz != 0u;
}

static void bredr_print_hex_line(const char *label,
                                 const uint8_t *data,
                                 unsigned int data_len,
                                 unsigned int max_show)
{
    unsigned int show = data_len < max_show ? data_len : max_show;

    printf("%-13s:", label);
    if (!data || data_len == 0u)
    {
        printf(" (none)\n");
        return;
    }

    for (unsigned int i = 0u; i < show; i++)
    {
        if (i != 0u && (i % 16u) == 0u)
            printf("\n              ");
        printf(" %02X", data[i]);
    }
    if (data_len > show)
        printf(" ...");
    printf("\n");
}

static void bredr_print_decoded_payload(const bredr_packet_t *packet, const bredr_frame_t *frame)
{
    if (!packet)
        return;

    uint8_t type_code = packet->header.type & 0x0Fu;
    unsigned int on_air_bits = bredr_on_air_payload_bits(packet->header.type);
    bredr_fec_mode_t fec_mode = bredr_fec_mode_for_type(packet->header.type);

    printf("\n[Decoded Payload Info]\n");
    printf("Family       : %s\n", bredr_payload_family_name(packet->family));

    switch (packet->family)
    {
    case BREDR_PAYLOAD_FAMILY_ACL:
    {
        const bredr_acl_payload_t *acl = &packet->payload.acl;
        unsigned int max_length = bredr_acl_max_user_payload_bytes(packet->header.type);
        uint8_t llid = (uint8_t)(acl->llid & 0x03u);
        printf("LLID         : %u [%s]%s\n",
               (unsigned int)llid,
               bredr_llid_name(llid),
               llid == 0u ? " (likely encrypted)" : "");
        printf("ACL FLOW     : %u\n", acl->flow & 1u);
        if (packet->limit == BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH && max_length != 0u)
        {
            printf("ACL Length   : %u [impossible for %s] (likely encrypted)\n",
                   acl->length,
                   bredr_packet_type_name(packet->header.type));
        }
        else
            printf("ACL Length   : %u bytes\n", acl->length);
        if (fec_mode == BREDR_FEC_MODE_2_3 && frame && on_air_bits >= 15u)
        {
            int valid = valid_fec_2_3_blocks(frame->air_payload, on_air_bits);
            printf("FEC          : 2/3 [%d Valid]\n", valid >= 0 ? valid : 0);
        }
        else
            printf("FEC          : None\n");
        if (acl->has_mic)
            printf("ACL MIC      : 0x%08X [%s]\n", acl->mic, acl->mic_ok ? "PASS" : "FAIL");
        if (acl->has_crc)
            printf("ACL CRC      : 0x%04X [%s]%s\n",
                   acl->crc,
                   acl->crc_ok ? "PASS" : "FAIL",
                   acl->crc_ok ? "" : " (likely encrypted)");
        if (packet->status == BREDR_DECODE_FULL_PAYLOAD)
            bredr_print_hex_line("ACL Payload", acl->user_payload, acl->length, 24u);
        break;
    }
    case BREDR_PAYLOAD_FAMILY_SCO:
    case BREDR_PAYLOAD_FAMILY_ESCO:
    {
        const bredr_sync_packet_t *sync = &packet->payload.sync;
        printf("Sync Length  : %u bytes\n", sync->payload_bytes);
        if (sync->has_crc)
            printf("Sync CRC     : 0x%04X [%s]\n",
                   sync->crc,
                   sync->crc_ok ? "PASS" : "FAIL");
        else if (sync->is_esco)
            printf("CRC          : [Fail] (not found, likely encrypted)\n");
        if (frame && on_air_bits > 0u)
        {
            if (fec_mode == BREDR_FEC_MODE_1_3)
            {
                int valid = valid_fec_1_3_blocks(frame->air_payload, on_air_bits);
                printf("FEC          : 1/3 [%d Valid]\n", valid >= 0 ? valid : 0);
            }
            else if (fec_mode == BREDR_FEC_MODE_2_3)
            {
                int valid = valid_fec_2_3_blocks(frame->air_payload, on_air_bits);
                printf("FEC          : 2/3 [%d Valid]\n", valid >= 0 ? valid : 0);
            }
            else
                printf("FEC          : None\n");
        }
        else
            printf("FEC          : None\n");
        if (packet->status == BREDR_DECODE_FULL_PAYLOAD)
            bredr_print_hex_line(sync->is_esco ? "eSCO Payload" : "SCO Payload",
                                 sync->payload,
                                 sync->payload_bytes,
                                 24u);
        break;
    }
    case BREDR_PAYLOAD_FAMILY_CONTROL:
        printf("Payload      : (none)\n");
        printf("FEC          : None\n");
        break;
    default:
        printf("FEC          : None\n");
        break;
    }

    if (packet->limit != BREDR_DECODE_LIMIT_NONE &&
        packet->limit != BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH)
        printf("Decode       : %s\n", bredr_decode_limit_desc(packet->limit));
}

void bredr_print_packet_details(const bredr_frame_t *frame,
                                const bredr_piconet_snapshot_t *pnet,
                                const rx_metadata_t *meta)
{
    bredr_packet_t packet;
    uint8_t decode_uap;
    uint8_t decode_clk1_6;
    int decode_ok;
    int have_decode_inputs;
    int semantic_payload_ready;

    printf("\n[%s Packet Info]\n",
           frame->has_header ? "BR/EDR Data" : "BR/EDR Inquiry");
    printf("PHY          : %s\n", receiver_phy_name(frame->phy));
    printf("LAP          : 0x%06" PRIX32 "\n", frame->lap & 0xFFFFFFu);
    if (frame->has_header)
        printf("HEADER       : 0x%014" PRIX64 "\n",
               frame->header_raw & 0x003FFFFFFFFFFFFFull);
    else
        printf("HEADER       : (none — shortened access code)\n");

    memset(&packet, 0, sizeof(packet));
    have_decode_inputs = bredr_build_decode_inputs(pnet, meta, &decode_uap, &decode_clk1_6);
    if (have_decode_inputs)
    {
        decode_ok = bredr_decode_frame(frame, decode_uap, decode_clk1_6, &packet);
    }
    else
    {
        decode_ok = 0;
        if (frame->has_header)
            packet.limit = BREDR_DECODE_LIMIT_MISSING_CONTEXT;
    }

    if (frame->has_header)
    {
        if (decode_ok > 0)
        {
            printf("\n[Decoded Header Info]\n");
            printf("HEC          : 0x%02X [PASS]\n", packet.header.hec);
            if ((packet.header.type & 0x0Fu) == 0x07u && packet.family == BREDR_PAYLOAD_FAMILY_ESCO)
                printf("TYPE         : %u [HV3/EV3]\n", packet.header.type & 0x0Fu);
            else
                printf("TYPE         : %u [%s]\n",
                       packet.header.type & 0x0Fu, bredr_packet_type_name(packet.header.type));
            printf("LT_ADDR      : %u\n", packet.header.lt_addr & 0x07u);
            printf("FLOW         : %u\n", packet.header.flow & 1u);
            printf("ARQN         : %u\n", packet.header.arqn & 1u);
            printf("SEQN         : %u\n", packet.header.seqn & 1u);
        }

        semantic_payload_ready = (packet.status == BREDR_DECODE_FULL_PAYLOAD);
        if (packet.family == BREDR_PAYLOAD_FAMILY_ACL && packet.status == BREDR_DECODE_PARTIAL_PAYLOAD)
            bredr_print_decoded_payload(&packet, frame);
        else if (semantic_payload_ready)
            bredr_print_decoded_payload(&packet, frame);

        if (!semantic_payload_ready)
        {
            bredr_print_payload_preview(frame);
            if (packet.limit != BREDR_DECODE_LIMIT_NONE &&
                packet.limit != BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH)
                printf("Decode       : %s\n", bredr_decode_limit_desc(packet.limit));
            else if (decode_ok <= 0)
                printf("Decode       : raw packet only\n");
        }
    }

    if (frame->has_header && pnet)
    {
        printf("\n[Piconet Info]\n");
        printf("Packets      : %lu\n", pnet->total_packets);
        if (pnet->uap_found)
            printf("UAP          : 0x%02X\n", pnet->uap);
        else
            printf("UAP          : 0x??\n");
        printf("Tracking     : %d [%s]\n",
               pnet->tracking_state, bredr_tracking_state_desc(pnet->tracking_state));
        if (pnet->clk_known)
            printf("CLK1-6       : %u\n", pnet->central_clk_1_6);
        else
            printf("CLK1-6       : ??\n");
    }
}

void bredr_print_packet_summary_line(unsigned long packet_no,
                                     const bredr_frame_t *frame,
                                     const bredr_piconet_snapshot_t *pnet,
                                     const rx_metadata_t *meta)
{
    if (frame->has_header)
    {
        char uap_buf[8];
        char clk_buf[8];
        if (pnet && pnet->uap_found)
            snprintf(uap_buf, sizeof(uap_buf), "%02X", pnet->uap);
        else
            snprintf(uap_buf, sizeof(uap_buf), "??");
        if (pnet && pnet->clk_known)
            snprintf(clk_buf, sizeof(clk_buf), "%02u", pnet->central_clk_1_6);
        else
            snprintf(clk_buf, sizeof(clk_buf), "??");

        printf("pkt=%-6lu lap=%06" PRIX32 " phy=%-12s uap=%s ch=%02u ac=%u clk=%s track=%d rssi=%.1f\n",
               packet_no,
               frame->lap & 0xFFFFFFu,
               receiver_phy_name(frame->phy),
               uap_buf,
               meta->channel_index,
               frame->ac_errors,
               clk_buf,
               pnet ? pnet->tracking_state : -1,
               meta->rssi_dbr);
    }
    else
    {
        printf("pkt=%-6lu lap=%06" PRIX32 " phy=%-12s uap=?? ch=%02u ac=%u clk=?? track=%d rssi=%.1f\n",
               packet_no,
               frame->lap & 0xFFFFFFu,
               receiver_phy_name(frame->phy),
               meta->channel_index,
               frame->ac_errors,
               pnet ? pnet->tracking_state : -1,
               meta->rssi_dbr);
    }
}

void bredr_print_piconet_snapshot(const bredr_piconet_snapshot_t *pnet)
{
    if (!pnet)
        return;

    printf("  LAP: 0x%06" PRIX32, pnet->lap & 0xFFFFFFu);
    if (pnet->uap_found)
        printf("  UAP: 0x%02X", pnet->uap);
    else
        printf("  UAP: ??");

    if (pnet->clk_known)
        printf("  CLK1-6: %02u [state=%d]", pnet->central_clk_1_6, pnet->tracking_state);
    else
        printf("  CLK1-6: ?? [state=%d]", pnet->tracking_state);

    printf("  Packets: %lu\n", pnet->total_packets);
}

void bredr_print_rssi_snapshot(unsigned long packet_no,
                               const bredr_frame_t *frame,
                               const rx_metadata_t *meta,
                               const bredr_piconet_snapshot_t *const *piconets,
                               size_t count,
                               unsigned int master_clock_mhz)
{
    printf("\n================ RSSI Snapshot (Packet #%lu) ================\n", packet_no);
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           meta->radio_start_sample_index, master_clock_mhz);
    printf("Piconets     : %zu\n", count);
    printf("--------------------------------------------------------------\n");

    for (size_t i = 0; i < count; i++)
    {
        const bredr_piconet_snapshot_t *cur = piconets[i];
        char piconet_id[16];
        bredr_format_piconet_id(piconet_id, frame, cur);

        if (!bredr_piconet_has_active_track(cur))
        {
            char combined_buf[8];
            bredr_format_rssi_value(combined_buf, cur->combined_rssi_seen, cur->combined_rssi);
            printf("Piconet: %-10s | Track: %2d | Combined: %s dBr\n",
                   piconet_id,
                   cur->tracking_state,
                   combined_buf);
            continue;
        }

        char central_buf[8];
        bredr_format_rssi_value(central_buf, cur->master_rssi_seen, cur->master_rssi);
        printf("Piconet: %-10s | Track: %2d | Central: %s dBr",
               piconet_id,
               cur->tracking_state,
               central_buf);

        int periph_seen = 0;
        for (int lt = 1; lt <= 7; lt++)
        {
            if (!cur->slave_rssi_seen[lt])
                continue;
            periph_seen = 1;
            char pbuf[8];
            bredr_format_rssi_value(pbuf, 1, cur->slave_rssi[lt]);
            printf(" | Periph[%d]: %s dBr", lt, pbuf);
        }
        if (!periph_seen)
            printf(" | Periph: (none yet)");
        printf("\n");
    }

    printf("==============================================================\n");
}
