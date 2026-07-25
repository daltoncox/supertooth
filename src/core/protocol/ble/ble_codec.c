/**
 * @file ble_codec.c
 * @brief BLE codec helpers extracted from the PHY/framing state machine.
 */

#include "ble_codec.h"

#include <string.h>

static void ble_copy_address(ble_address_t *out,
                             const uint8_t *src,
                             ble_addr_kind_t kind)
{
    if (!out || !src)
        return;

    memcpy(out->addr, src, BLE_ADDR_LEN);
    out->kind = kind;
}

static void ble_decode_adv_data_packet(ble_adv_data_packet_t *out,
                                       const uint8_t *payload,
                                       unsigned int payload_len,
                                       ble_addr_kind_t addr_kind)
{
    if (!payload || payload_len < BLE_ADDR_LEN)
        return;

    ble_copy_address(&out->adv_addr, payload, addr_kind);

    unsigned int adv_data_len = payload_len - BLE_ADDR_LEN;
    if (adv_data_len > sizeof(out->adv_data))
        adv_data_len = sizeof(out->adv_data);
    memcpy(out->adv_data, payload + BLE_ADDR_LEN, adv_data_len);
    out->adv_data_len = (uint8_t)adv_data_len;
}

static void ble_decode_adv_direct_packet(ble_adv_direct_packet_t *out,
                                         const uint8_t *payload,
                                         unsigned int payload_len,
                                         ble_addr_kind_t tx_kind,
                                         ble_addr_kind_t rx_kind)
{
    if (!payload || payload_len < (2u * BLE_ADDR_LEN))
        return;

    ble_copy_address(&out->adv_addr, payload, tx_kind);
    ble_copy_address(&out->target_addr, payload + BLE_ADDR_LEN, rx_kind);
}

static void ble_decode_scan_req_packet(ble_scan_req_packet_t *out,
                                       const uint8_t *payload,
                                       unsigned int payload_len,
                                       ble_addr_kind_t tx_kind,
                                       ble_addr_kind_t rx_kind)
{
    if (!payload || payload_len < (2u * BLE_ADDR_LEN))
        return;

    ble_copy_address(&out->scanner_addr, payload, tx_kind);
    ble_copy_address(&out->adv_addr, payload + BLE_ADDR_LEN, rx_kind);
}

static void ble_decode_connect_ind_packet(ble_connect_ind_packet_t *out,
                                          const uint8_t *payload,
                                          unsigned int payload_len,
                                          ble_addr_kind_t tx_kind,
                                          ble_addr_kind_t rx_kind)
{
    if (!payload || payload_len < (2u * BLE_ADDR_LEN))
        return;

    ble_copy_address(&out->init_addr, payload, tx_kind);
    ble_copy_address(&out->adv_addr, payload + BLE_ADDR_LEN, rx_kind);

    unsigned int ll_data_len = payload_len - (2u * BLE_ADDR_LEN);
    if (ll_data_len > sizeof(out->ll_data))
        ll_data_len = sizeof(out->ll_data);
    memcpy(out->ll_data, payload + (2u * BLE_ADDR_LEN), ll_data_len);
    out->ll_data_len = (uint8_t)ll_data_len;
}

static void ble_decode_unknown_packet(ble_unknown_packet_t *out,
                                      const uint8_t *payload,
                                      unsigned int payload_len)
{
    if (!payload)
        return;

    if (payload_len > sizeof(out->payload))
        payload_len = sizeof(out->payload);
    memcpy(out->payload, payload, payload_len);
    out->payload_len = (uint8_t)payload_len;
}

void ble_dewhiten(uint8_t *data, unsigned int length_bytes, uint8_t channel_index)
{
    uint8_t lfsr = (channel_index & 0x3Fu) | 0x40u;

    for (unsigned int byte_idx = 0; byte_idx < length_bytes; byte_idx++)
    {
        for (unsigned int bit_idx = 0; bit_idx < 8u; bit_idx++)
        {
            uint8_t out_bit = lfsr & 0x01u;
            data[byte_idx] ^= (uint8_t)(out_bit << bit_idx);
            lfsr ^= (uint8_t)(out_bit << 3u);
            lfsr >>= 1u;
            lfsr |= (uint8_t)(out_bit << 6u);
        }
    }
}

uint8_t ble_bit_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4u) | ((b & 0x0Fu) << 4u));
    b = (uint8_t)(((b & 0xCCu) >> 2u) | ((b & 0x33u) << 2u));
    b = (uint8_t)(((b & 0xAAu) >> 1u) | ((b & 0x55u) << 1u));
    return b;
}

uint32_t ble_crc_calc(const uint8_t *data, unsigned int len, uint32_t init)
{
    uint32_t crc = init & 0xFFFFFFu;

    for (unsigned int i = 0; i < len; i++)
    {
        for (unsigned int bit = 0; bit < 8u; bit++)
        {
            uint8_t d = (uint8_t)((data[i] >> bit) & 1u);
            uint8_t fb = (uint8_t)(((crc >> 23u) ^ d) & 1u);
            crc = (crc << 1u) & 0xFFFFFFu;
            if (fb)
                crc ^= 0x65Bu;
        }
    }

    return crc;
}

unsigned int ble_payload_length_from_header(const uint8_t header[2])
{
    unsigned int payload_len = header[1];
    if (payload_len > (BLE_PDU_MAX_BYTES - 2u))
        payload_len = BLE_PDU_MAX_BYTES - 2u;
    return payload_len;
}

uint32_t ble_extract_crc(const uint8_t *crc_bytes)
{
    return ((uint32_t)ble_bit_reverse_byte(crc_bytes[0]) << 16u) |
           ((uint32_t)ble_bit_reverse_byte(crc_bytes[1]) << 8u) |
           (uint32_t)ble_bit_reverse_byte(crc_bytes[2]);
}

int ble_decode_frame(const ble_frame_t *frame,
                     uint8_t channel_index,
                     ble_packet_t *out)
{
    if (!frame || !out || frame->raw_pdu_bytes < (2u + BLE_CRC_BYTES) ||
        frame->raw_pdu_bytes > (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES))
        return -1;

    unsigned int total_bytes = frame->raw_pdu_bytes;
    unsigned int pdu_bytes = total_bytes - BLE_CRC_BYTES;
    uint8_t dewhitened[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(dewhitened, frame->raw_pdu, total_bytes);
    ble_dewhiten(dewhitened, total_bytes, channel_index);

    memset(out, 0, sizeof(*out));
    out->phy = frame->phy;
    out->preamble = frame->preamble;
    out->access_address = frame->access_address;
    out->pdu_type = dewhitened[0] & 0x0Fu;
    out->tx_addr_kind = (dewhitened[0] & 0x40u) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    out->rx_addr_kind = (dewhitened[0] & 0x80u) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;

    unsigned int payload_len = ble_payload_length_from_header(dewhitened);
    if (payload_len > (pdu_bytes - 2u))
        payload_len = pdu_bytes - 2u;
    out->payload_len = (uint8_t)payload_len;
    out->crc = ble_extract_crc(&dewhitened[pdu_bytes]);

    uint32_t computed_crc = ble_crc_calc(dewhitened, 2u + payload_len, BLE_CRC_INIT_ADV);
    out->crc_ok = (computed_crc == (out->crc & 0xFFFFFFu)) ? 1u : 0u;

    const uint8_t *payload = &dewhitened[2];
    switch (out->pdu_type)
    {
    case BLE_PDU_ADV_IND:
        ble_decode_adv_data_packet(&out->payload.adv_ind, payload, payload_len,
                                   out->tx_addr_kind);
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        ble_decode_adv_direct_packet(&out->payload.adv_direct_ind, payload, payload_len,
                                     out->tx_addr_kind, out->rx_addr_kind);
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        ble_decode_adv_data_packet(&out->payload.adv_nonconn_ind, payload, payload_len,
                                   out->tx_addr_kind);
        break;
    case BLE_PDU_SCAN_REQ:
        ble_decode_scan_req_packet(&out->payload.scan_req, payload, payload_len,
                                   out->tx_addr_kind, out->rx_addr_kind);
        break;
    case BLE_PDU_SCAN_RSP:
        ble_decode_adv_data_packet(&out->payload.scan_rsp, payload, payload_len,
                                   out->tx_addr_kind);
        break;
    case BLE_PDU_CONNECT_IND:
        ble_decode_connect_ind_packet(&out->payload.connect_ind, payload, payload_len,
                                      out->tx_addr_kind, out->rx_addr_kind);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        ble_decode_adv_data_packet(&out->payload.adv_scan_ind, payload, payload_len,
                                   out->tx_addr_kind);
        break;
    default:
        ble_decode_unknown_packet(&out->payload.unknown, payload, payload_len);
        break;
    }

    return 0;
}

int ble_verify_crc(const ble_packet_t *pkt)
{
    return pkt ? (pkt->crc_ok ? 1 : 0) : 0;
}
