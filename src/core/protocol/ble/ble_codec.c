/**
 * @file ble_codec.c
 * @brief BLE codec helpers extracted from the PHY/framing state machine.
 */

#include "ble_codec.h"

#include <stdio.h>
#include <string.h>

#include "bt_assigned_numbers.h"

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

/* ---------------------------------------------------------------------------
 * Advertising PDU access helpers
 * ---------------------------------------------------------------------------*/

void ble_addr_bytes_to_u64(const uint8_t a[BLE_ADDR_LEN], uint64_t *out)
{
    uint64_t v = 0;
    for (int i = 0; i < (int)BLE_ADDR_LEN; i++)
        v |= (uint64_t)a[i] << (8 * i);
    *out = v;
}

const uint8_t *ble_adv_addr_bytes(const ble_adv_pdu_t *adv)
{
    if (!adv)
        return NULL;
    switch (adv->pdu_type) {
        case BLE_PDU_ADV_IND:         return adv->payload.adv_ind.adv_addr.addr;
        case BLE_PDU_ADV_NONCONN_IND: return adv->payload.adv_nonconn_ind.adv_addr.addr;
        case BLE_PDU_ADV_SCAN_IND:    return adv->payload.adv_scan_ind.adv_addr.addr;
        case BLE_PDU_SCAN_RSP:        return adv->payload.scan_rsp.adv_addr.addr;
        case BLE_PDU_ADV_DIRECT_IND:  return adv->payload.adv_direct_ind.adv_addr.addr;
        case BLE_PDU_SCAN_REQ:        return adv->payload.scan_req.adv_addr.addr;
        case BLE_PDU_CONNECT_IND:     return adv->payload.connect_ind.adv_addr.addr;
        default:                      return NULL;
    }
}

const uint8_t *ble_adv_data_bytes(const ble_adv_pdu_t *adv, unsigned int *len_out)
{
    if (!adv || !len_out)
        return NULL;
    switch (adv->pdu_type) {
        case BLE_PDU_ADV_IND:         *len_out = adv->payload.adv_ind.adv_data_len; return adv->payload.adv_ind.adv_data;
        case BLE_PDU_ADV_NONCONN_IND: *len_out = adv->payload.adv_nonconn_ind.adv_data_len; return adv->payload.adv_nonconn_ind.adv_data;
        case BLE_PDU_ADV_SCAN_IND:    *len_out = adv->payload.adv_scan_ind.adv_data_len; return adv->payload.adv_scan_ind.adv_data;
        case BLE_PDU_SCAN_RSP:        *len_out = adv->payload.scan_rsp.adv_data_len; return adv->payload.scan_rsp.adv_data;
        default:                      *len_out = 0; return NULL;
    }
}

void ble_adv_parse_name_manuf(const uint8_t *data, unsigned int len,
                              char *name_out, size_t name_cap,
                              char *manuf_out, size_t manuf_cap)
{
    if (name_cap)  name_out[0] = '\0';
    if (manuf_cap) manuf_out[0] = '\0';
    if (!data || len == 0) return;

    unsigned int i = 0;
    while (i + 1 < len) {
        uint8_t ad_len = data[i];
        if (ad_len == 0) break;
        if (i + 1 + ad_len > len) break;
        uint8_t type = data[i + 1];
        const uint8_t *ad = data + i + 2;
        uint8_t ad_dlen = (uint8_t)(ad_len - 1);
        if (type == 0x08 || type == 0x09) {
            size_t n = ad_dlen < name_cap - 1 ? ad_dlen : name_cap - 1;
            memcpy(name_out, ad, n);
            name_out[n] = '\0';
        } else if (type == 0xFF && ad_dlen >= 2) {
            uint16_t cid = (uint16_t)ad[0] | ((uint16_t)ad[1] << 8);
            const char *mn = bt_assigned_company_name(cid);
            if (mn) {
                strncpy(manuf_out, mn, manuf_cap - 1);
                manuf_out[manuf_cap - 1] = '\0';
            }
        }
        i += 1u + ad_len;
    }
}

/* ---------------------------------------------------------------------------
 * Rich advertising-data info
 * ---------------------------------------------------------------------------*/

/** Extract the 16-bit alias from a 128-bit UUID that uses the Bluetooth Base
 * UUID (0000XXXX-0000-1000-8000-00805F9B34FB). On air the 128-bit UUID is
 * little-endian, so a BT-base alias looks like
 * FB 34 9B 5F 80 00 00 80 00 10 00 00 XX XX 00 00. */
static int ble_uuid128_base_alias(const uint8_t u[16], uint16_t *alias_out)
{
    if (u[0] != 0xFB || u[1] != 0x34 || u[2] != 0x9B || u[3] != 0x5F ||
        u[4] != 0x80 || u[5] != 0x00 || u[6] != 0x00 || u[7] != 0x80 ||
        u[8] != 0x00 || u[9] != 0x10 || u[10] != 0x00 || u[11] != 0x00 ||
        u[14] != 0x00 || u[15] != 0x00)
        return 0;
    if (alias_out)
        *alias_out = (uint16_t)u[12] | ((uint16_t)u[13] << 8u);
    return 1;
}

static void ble_info_add_uuid16(ble_adv_info_t *out, uint16_t uuid)
{
    for (unsigned int i = 0; i < out->service_count; i++)
        if (out->service_uuids[i] == uuid)
            return;
    if (out->service_count < BLE_ADV_MAX_SERVICES)
        out->service_uuids[out->service_count++] = uuid;
}

static void ble_info_add_uuid32(ble_adv_info_t *out, uint32_t uuid)
{
    for (unsigned int i = 0; i < out->service32_count; i++)
        if (out->service_uuids32[i] == uuid)
            return;
    if (out->service32_count < BLE_ADV_MAX_SERVICES)
        out->service_uuids32[out->service32_count++] = uuid;
}

static void ble_info_add_uuid128(ble_adv_info_t *out, const uint8_t u[16])
{
    uint16_t alias = 0u;
    if (ble_uuid128_base_alias(u, &alias)) {
        ble_info_add_uuid16(out, alias);
        return;
    }
    out->uuid128_total++;
    for (unsigned int i = 0; i < out->uuid128_count; i++)
        if (memcmp(out->uuid128[i], u, 16) == 0)
            return;
    if (out->uuid128_count < BLE_ADV_MAX_SERVICES_128)
        memcpy(out->uuid128[out->uuid128_count++], u, 16);
}

void ble_adv_parse_info(const uint8_t *data, unsigned int len,
                        ble_adv_info_t *out)
{
    if (!out)
        return;
    memset(out, 0, sizeof(*out));
    if (!data || len == 0)
        return;

    unsigned int i = 0;
    while (i + 1 < len) {
        uint8_t ad_len = data[i];
        if (ad_len == 0)
            break;
        if (i + 1 + ad_len > len)
            break;
        uint8_t type = data[i + 1];
        const uint8_t *ad = data + i + 2;
        unsigned int ad_dlen = (unsigned int)ad_len - 1u;
        switch (type) {
        case 0x01: /* Flags */
            if (ad_dlen >= 1) {
                out->flags = ad[0];
                out->has_flags = 1;
            }
            break;
        case 0x02: /* Incomplete 16-bit UUIDs */
            out->has_incomplete_list = 1;
            for (unsigned int k = 0; k + 1 < ad_dlen; k += 2)
                ble_info_add_uuid16(out, (uint16_t)ad[k] | ((uint16_t)ad[k + 1] << 8u));
            break;
        case 0x03: /* Complete 16-bit UUIDs */
            out->has_complete_list = 1;
            for (unsigned int k = 0; k + 1 < ad_dlen; k += 2)
                ble_info_add_uuid16(out, (uint16_t)ad[k] | ((uint16_t)ad[k + 1] << 8u));
            break;
        case 0x04:
        case 0x05:
            if (type == 0x04) out->has_incomplete_list = 1;
            else out->has_complete_list = 1;
            for (unsigned int k = 0; k + 3 < ad_dlen; k += 4) {
                uint32_t u = (uint32_t)ad[k] | ((uint32_t)ad[k + 1] << 8u) |
                             ((uint32_t)ad[k + 2] << 16u) | ((uint32_t)ad[k + 3] << 24u);
                ble_info_add_uuid32(out, u);
            }
            break;
        case 0x06:
        case 0x07:
            if (type == 0x06) out->has_incomplete_list = 1;
            else out->has_complete_list = 1;
            for (unsigned int k = 0; k + 15 < ad_dlen; k += 16)
                ble_info_add_uuid128(out, ad + k);
            break;
        case 0x0A: /* Tx Power */
            if (ad_dlen >= 1) {
                out->tx_power = (int8_t)ad[0];
                out->has_tx_power = 1;
            }
            break;
        case 0x0D: /* Class of Device */
            if (ad_dlen >= 3) {
                out->cod = (uint32_t)ad[0] | ((uint32_t)ad[1] << 8u) |
                           ((uint32_t)ad[2] << 16u);
                out->has_cod = 1;
            }
            break;
        case 0x12: /* Peripheral Connection Interval Range */
            if (ad_dlen >= 4) {
                out->conn_interval_min = (uint16_t)ad[0] | ((uint16_t)ad[1] << 8u);
                out->conn_interval_max = (uint16_t)ad[2] | ((uint16_t)ad[3] << 8u);
                out->has_conn_interval = 1;
            }
            break;
        case 0x14: /* 16-bit Solicitation */
            out->has_solicitation = 1;
            for (unsigned int k = 0; k + 1 < ad_dlen; k += 2)
                ble_info_add_uuid16(out, (uint16_t)ad[k] | ((uint16_t)ad[k + 1] << 8u));
            break;
        case 0x1F: /* 32-bit Solicitation */
            out->has_solicitation = 1;
            for (unsigned int k = 0; k + 3 < ad_dlen; k += 4) {
                uint32_t u = (uint32_t)ad[k] | ((uint32_t)ad[k + 1] << 8u) |
                             ((uint32_t)ad[k + 2] << 16u) | ((uint32_t)ad[k + 3] << 24u);
                ble_info_add_uuid32(out, u);
            }
            break;
        case 0x15: /* 128-bit Solicitation */
            out->has_solicitation = 1;
            for (unsigned int k = 0; k + 15 < ad_dlen; k += 16)
                ble_info_add_uuid128(out, ad + k);
            break;
        case 0x16: /* Service Data 16-bit */
            if (ad_dlen >= 2) {
                out->has_service_data = 1;
                ble_info_add_uuid16(out, (uint16_t)ad[0] | ((uint16_t)ad[1] << 8u));
            }
            break;
        case 0x20: /* Service Data 32-bit */
            if (ad_dlen >= 4) {
                out->has_service_data = 1;
                ble_info_add_uuid32(out, (uint32_t)ad[0] | ((uint32_t)ad[1] << 8u) |
                                          ((uint32_t)ad[2] << 16u) | ((uint32_t)ad[3] << 24u));
            }
            break;
        case 0x21: /* Service Data 128-bit */
            if (ad_dlen >= 16) {
                out->has_service_data = 1;
                ble_info_add_uuid128(out, ad);
            }
            break;
        case 0x19: /* Appearance */
            if (ad_dlen >= 2) {
                out->appearance = (uint16_t)ad[0] | ((uint16_t)ad[1] << 8u);
                out->has_appearance = 1;
            }
            break;
        case 0x1A: /* Advertising Interval */
            if (ad_dlen >= 2) {
                out->adv_interval = (uint16_t)ad[0] | ((uint16_t)ad[1] << 8u);
                out->has_adv_interval = 1;
            }
            break;
        case 0x1C: /* LE Role */
            if (ad_dlen >= 1) {
                out->le_role = ad[0];
                out->has_le_role = 1;
            }
            break;
        case 0x24: /* URI */
            if (ad_dlen > 0) {
                size_t n = ad_dlen < sizeof(out->uri) - 1 ? ad_dlen : sizeof(out->uri) - 1;
                for (size_t k = 0; k < n; k++) {
                    char c = (char)ad[k];
                    out->uri[k] = (c >= 0x20 && c < 0x7f) ? c : '.';
                }
                out->uri[n] = '\0';
            }
            break;
        default:
            break;
        }
        i += 1u + ad_len;
    }
}

void ble_adv_info_merge(ble_adv_info_t *dst, const ble_adv_info_t *src)
{
    if (!dst || !src)
        return;
    for (unsigned int i = 0; i < src->service_count; i++)
        ble_info_add_uuid16(dst, src->service_uuids[i]);
    for (unsigned int i = 0; i < src->service32_count; i++)
        ble_info_add_uuid32(dst, src->service_uuids32[i]);
    for (unsigned int i = 0; i < src->uuid128_count; i++)
        ble_info_add_uuid128(dst, src->uuid128[i]);
    /* Count 128-bit UUIDs seen in packets whose first-N store already held
     * the same values: keep the max total so the "+N more" hint is stable. */
    if (src->uuid128_total > dst->uuid128_total)
        dst->uuid128_total = src->uuid128_total;
    dst->has_complete_list |= src->has_complete_list;
    dst->has_incomplete_list |= src->has_incomplete_list;
    dst->has_service_data |= src->has_service_data;
    dst->has_solicitation |= src->has_solicitation;
    if (src->has_flags && !dst->has_flags) {
        dst->flags = src->flags;
        dst->has_flags = 1;
    }
    if (src->has_tx_power && !dst->has_tx_power) {
        dst->tx_power = src->tx_power;
        dst->has_tx_power = 1;
    }
    if (src->has_appearance && !dst->has_appearance) {
        dst->appearance = src->appearance;
        dst->has_appearance = 1;
    }
    if (src->has_cod && !dst->has_cod) {
        dst->cod = src->cod;
        dst->has_cod = 1;
    }
    if (src->has_conn_interval && !dst->has_conn_interval) {
        dst->conn_interval_min = src->conn_interval_min;
        dst->conn_interval_max = src->conn_interval_max;
        dst->has_conn_interval = 1;
    }
    if (src->has_adv_interval && !dst->has_adv_interval) {
        dst->adv_interval = src->adv_interval;
        dst->has_adv_interval = 1;
    }
    if (src->has_le_role && !dst->has_le_role) {
        dst->le_role = src->le_role;
        dst->has_le_role = 1;
    }
    if (!dst->uri[0] && src->uri[0]) {
        strncpy(dst->uri, src->uri, sizeof(dst->uri) - 1);
        dst->uri[sizeof(dst->uri) - 1] = '\0';
    }
}

void ble_adv_info_format_services(const ble_adv_info_t *info,
                                  char *out, size_t cap)
{
    if (!out || cap == 0u)
        return;
    out[0] = '\0';
    if (!info)
        return;
    size_t pos = 0u;
    for (unsigned int i = 0; i < info->service_count; i++) {
        uint16_t u = info->service_uuids[i];
        const char *nm = bt_assigned_service_uuid_name(u);
        char item[96];
        if (nm && strcmp(nm, "Unknown") != 0)
            snprintf(item, sizeof(item), "%s (0x%04X)", nm, u);
        else
            snprintf(item, sizeof(item), "0x%04X", u);
        int w = snprintf(out + pos, pos < cap ? cap - pos : 0u,
                         "%s%s", pos ? ", " : "", item);
        if (w > 0)
            pos += (size_t)w;
        if (pos >= cap)
            break;
    }
    for (unsigned int i = 0; i < info->service32_count; i++) {
        char item[32];
        snprintf(item, sizeof(item), "0x%08X", info->service_uuids32[i]);
        int w = snprintf(out + pos, pos < cap ? cap - pos : 0u,
                         "%s%s", pos ? ", " : "", item);
        if (w > 0)
            pos += (size_t)w;
        if (pos >= cap)
            break;
    }
    if (info->uuid128_total > 0) {
        char item[64];
        if (info->uuid128_total > info->uuid128_count)
            snprintf(item, sizeof(item), "+%u custom 128-bit",
                     info->uuid128_total - info->uuid128_count);
        else
            snprintf(item, sizeof(item), "%u custom 128-bit",
                     info->uuid128_total);
        /* If we stored the first 128-bit UUIDs, show the first in full. */
        char first[40] = "";
        if (info->uuid128_count > 0) {
            const uint8_t *u = info->uuid128[0];
            snprintf(first, sizeof(first),
                     "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                     u[15], u[14], u[13], u[12], u[11], u[10], u[9], u[8],
                     u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);
        }
        int w;
        if (first[0])
            w = snprintf(out + pos, pos < cap ? cap - pos : 0u,
                         "%s%s [%s%s]", pos ? ", " : "", item, first,
                         info->uuid128_total > 1 ? ", ...]" : "");        else
            w = snprintf(out + pos, pos < cap ? cap - pos : 0u,
                         "%s%s", pos ? ", " : "", item);
        if (w > 0)
            pos += (size_t)w;
    }
}

void ble_adv_flags_format(uint8_t flags, char *out, size_t cap)
{
    if (!out || cap == 0u)
        return;
    out[0] = '\0';
    size_t pos = 0u;
#define FLAG_APPEND(bit, label)                                              \
    do {                                                                     \
        if ((flags >> (bit)) & 1u) {                                         \
            int w = snprintf(out + pos, pos < cap ? cap - pos : 0u,          \
                             "%s%s", pos ? ", " : "", (label));              \
            if (w > 0)                                                       \
                pos += (size_t)w;                                            \
        }                                                                    \
    } while (0)
    FLAG_APPEND(0, "LE-LimitedDisc");
    FLAG_APPEND(1, "LE-GeneralDisc");
    FLAG_APPEND(2, "BR/EDR-NotSupp");
    FLAG_APPEND(3, "Simul-LE/BR-Ctrl");
    FLAG_APPEND(4, "Simul-LE/BR-Host");
#undef FLAG_APPEND
    if (!out[0])
        snprintf(out, cap, "0x%02X", flags);
}

