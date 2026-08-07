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

static void ble_decode_adv_data(ble_adv_data_t *out,
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

static void ble_decode_adv_direct_ind(ble_adv_direct_ind_t *out,
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

static void ble_decode_adv_scan_req(ble_adv_scan_req_t *out,
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

static void ble_decode_adv_connect_ind(ble_adv_connect_ind_t *out,
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

static void ble_decode_adv_unknown(ble_adv_unknown_t *out,
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

static void ble_decode_data_pdu(ble_data_pdu_t *out,
                                const uint8_t *dewhitened,
                                unsigned int payload_len)
{
    uint8_t b0 = dewhitened[0];
    out->llid = b0 & 0x03u;
    out->nesn = (uint8_t)((b0 >> 2u) & 0x01u);
    out->sn = (uint8_t)((b0 >> 3u) & 0x01u);
    out->md = (uint8_t)((b0 >> 4u) & 0x01u);

    if (payload_len > sizeof(out->payload))
        payload_len = sizeof(out->payload);
    if (payload_len > 0u)
        memcpy(out->payload, &dewhitened[2], payload_len);
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

/* ---------------------------------------------------------------------------
 * CRCInit recovery
 *
 * The CRC register update is linear over GF(2) in (init, data), so for a
 * fixed message length crc(M, I) = A(M) ^ T*I with A(M) = crc(M, 0) and T an
 * invertible 24x24 GF(2) matrix (a power of the LFSR companion matrix).
 * s_t16_inv_rows holds the rows of T^-1 for 16-bit (2-byte) messages, built
 * by Gauss-Jordan over GF(2) from the columns crc(zero, 2, 1<<k).
 * ---------------------------------------------------------------------------*/

static uint32_t s_t16_inv_rows[24];
static int s_t16_ready = 0;

static uint8_t ble_parity32(uint32_t v)
{
    v ^= v >> 16u;
    v ^= v >> 8u;
    v ^= v >> 4u;
    v &= 0xFu;
    return (uint8_t)((0x6996u >> v) & 1u);
}

void ble_crc_tables_init(void)
{
    if (s_t16_ready)
        return;

    static const uint8_t zero2[2] = {0u, 0u};
    uint32_t left[24];   /* rows of T (bit k of row r: coeff of init bit k
                            in crc bit r) */
    uint32_t right[24];  /* augmented identity, becomes T^-1 */

    for (unsigned int r = 0; r < 24u; r++)
    {
        left[r] = 0u;
        right[r] = 1u << r;
    }
    for (unsigned int k = 0; k < 24u; k++)
    {
        uint32_t col = ble_crc_calc(zero2, 2u, 1u << k);
        for (unsigned int r = 0; r < 24u; r++)
            if ((col >> r) & 1u)
                left[r] |= 1u << k;
    }

    for (unsigned int col = 0; col < 24u; col++)
    {
        unsigned int pivot = col;
        while (pivot < 24u && !((left[pivot] >> col) & 1u))
            pivot++;
        if (pivot >= 24u)
            return;   /* singular (cannot happen for this LFSR) */
        if (pivot != col)
        {
            uint32_t tmp;
            tmp = left[col]; left[col] = left[pivot]; left[pivot] = tmp;
            tmp = right[col]; right[col] = right[pivot]; right[pivot] = tmp;
        }
        for (unsigned int r = 0; r < 24u; r++)
        {
            if (r != col && ((left[r] >> col) & 1u))
            {
                left[r] ^= left[col];
                right[r] ^= right[col];
            }
        }
    }

    memcpy(s_t16_inv_rows, right, sizeof(s_t16_inv_rows));
    s_t16_ready = 1;
}

uint32_t ble_crc_reverse_init_len2(const uint8_t header[2], uint32_t rx_crc)
{
    if (!s_t16_ready)
        ble_crc_tables_init();
    if (!s_t16_ready || !header)
        return 0u;

    uint32_t y = (rx_crc ^ ble_crc_calc(header, 2u, 0u)) & 0xFFFFFFu;
    uint32_t init = 0u;
    for (unsigned int r = 0; r < 24u; r++)
        if (ble_parity32(s_t16_inv_rows[r] & y))
            init |= 1u << r;
    return init & 0xFFFFFFu;
}

int ble_connect_ind_parse(const uint8_t ll_data[BLE_CONNECT_IND_DATA_MAX_BYTES],
                          ble_connect_ind_params_t *out)
{
    if (!ll_data || !out)
        return -1;

    memset(out, 0, sizeof(*out));
    out->access_address =
        (uint32_t)ll_data[0] | ((uint32_t)ll_data[1] << 8u) |
        ((uint32_t)ll_data[2] << 16u) | ((uint32_t)ll_data[3] << 24u);
    out->crc_init =
        ((uint32_t)ll_data[4] | ((uint32_t)ll_data[5] << 8u) |
         ((uint32_t)ll_data[6] << 16u)) & 0xFFFFFFu;
    out->window_size = ll_data[7];
    out->window_offset = (uint16_t)((uint16_t)ll_data[8] |
                                    ((uint16_t)ll_data[9] << 8u));
    out->interval = (uint16_t)((uint16_t)ll_data[10] |
                               ((uint16_t)ll_data[11] << 8u));
    out->latency = (uint16_t)((uint16_t)ll_data[12] |
                              ((uint16_t)ll_data[13] << 8u));
    out->timeout = (uint16_t)((uint16_t)ll_data[14] |
                              ((uint16_t)ll_data[15] << 8u));
    memcpy(out->channel_map, &ll_data[16], sizeof(out->channel_map));
    out->hop_increment = ll_data[21] & 0x1Fu;
    out->sleep_clock_accuracy = (uint8_t)((ll_data[21] >> 5u) & 0x07u);
    return 0;
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
    out->crc = ble_extract_crc(&dewhitened[pdu_bytes]);

    unsigned int payload_len = ble_payload_length_from_header(dewhitened);
    if (payload_len > (pdu_bytes - 2u))
        payload_len = pdu_bytes - 2u;

    if (frame->kind == BLE_FRAME_DATA)
    {
        /* Data frames are CRC-gated by the framer: trust the stamp and the
         * confirmed per-connection CRCInit it used. */
        out->is_adv_pdu = 0u;
        out->crc_init = frame->crc_init;
        out->crc_ok = frame->crc_ok;
        ble_decode_data_pdu(&out->pdu.data, dewhitened, payload_len);
        return 0;
    }

    out->is_adv_pdu = 1u;
    out->crc_init = BLE_CRC_INIT_ADV;
    out->pdu.adv.pdu_type = dewhitened[0] & 0x0Fu;
    out->pdu.adv.tx_addr_kind =
        (dewhitened[0] & 0x40u) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    out->pdu.adv.rx_addr_kind =
        (dewhitened[0] & 0x80u) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
    out->pdu.adv.payload_len = (uint8_t)payload_len;

    uint32_t computed_crc =
        ble_crc_calc(dewhitened, 2u + payload_len, BLE_CRC_INIT_ADV);
    out->crc_ok = (computed_crc == (out->crc & 0xFFFFFFu)) ? 1u : 0u;

    const uint8_t *payload = &dewhitened[2];
    switch (out->pdu.adv.pdu_type)
    {
    case BLE_PDU_ADV_IND:
        ble_decode_adv_data(&out->pdu.adv.payload.adv_ind, payload, payload_len,
                            out->pdu.adv.tx_addr_kind);
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        ble_decode_adv_direct_ind(&out->pdu.adv.payload.adv_direct_ind, payload,
                                  payload_len, out->pdu.adv.tx_addr_kind,
                                  out->pdu.adv.rx_addr_kind);
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        ble_decode_adv_data(&out->pdu.adv.payload.adv_nonconn_ind, payload,
                            payload_len, out->pdu.adv.tx_addr_kind);
        break;
    case BLE_PDU_SCAN_REQ:
        ble_decode_adv_scan_req(&out->pdu.adv.payload.scan_req, payload,
                                payload_len, out->pdu.adv.tx_addr_kind,
                                out->pdu.adv.rx_addr_kind);
        break;
    case BLE_PDU_SCAN_RSP:
        ble_decode_adv_data(&out->pdu.adv.payload.scan_rsp, payload, payload_len,
                            out->pdu.adv.tx_addr_kind);
        break;
    case BLE_PDU_CONNECT_IND:
        ble_decode_adv_connect_ind(&out->pdu.adv.payload.connect_ind, payload,
                                   payload_len, out->pdu.adv.tx_addr_kind,
                                   out->pdu.adv.rx_addr_kind);
        break;
    case BLE_PDU_ADV_SCAN_IND:
        ble_decode_adv_data(&out->pdu.adv.payload.adv_scan_ind, payload,
                            payload_len, out->pdu.adv.tx_addr_kind);
        break;
    default:
        ble_decode_adv_unknown(&out->pdu.adv.payload.unknown, payload, payload_len);
        break;
    }

    return 0;
}

int ble_verify_crc(const ble_packet_t *pkt)
{
    return pkt ? (pkt->crc_ok ? 1 : 0) : 0;
}

uint8_t ble_rf_to_le_channel(unsigned int rf_channel_index)
{
    if (rf_channel_index == 0u)   return 37u;
    if (rf_channel_index == 12u)  return 38u;
    if (rf_channel_index >= 39u)  return 39u;
    if (rf_channel_index < 12u)   return (uint8_t)(rf_channel_index - 1u);
    return (uint8_t)(rf_channel_index - 2u);
}
