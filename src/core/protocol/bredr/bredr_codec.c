/**
 * @file bredr_codec.c
 * @brief BR/EDR codec helpers extracted from the PHY state machine.
 */

#include "bredr_codec.h"

#include <string.h>

static const unsigned int s_on_air_payload_bits[16] = {
    0, 0, 240, 240, 240, 240, 240, 240,
    560, 240, 1500, 1496, 1440, 1440, 2745, 2744,
};

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"
};

static const uint8_t s_whitening_indices[64] = {
    99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2,
    37, 69, 55,  8,  20, 40,  74,114, 15,106, 30, 78, 53, 72, 28,26,
    68,  7, 39,113, 105, 77,  71, 25, 84, 49,  57, 44, 61,117, 10, 1,
   123,124, 22,125, 111, 23,  42,126,  6,112,  76, 24, 48, 43,116, 0
};

static const uint8_t s_whitening_data[127] = {
    1,1,1,0,0,0,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,0,1,1,1,1,1,0,
    1,0,1,0,1,0,0,0,0,1,0,1,1,0,1,1,1,1,0,0,1,1,1,0,0,1,0,1,
    0,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,0,1,0,1,1,1,0,1,0,0,0,
    1,1,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,1,1,0,1,0,0,1,
    1,1,1,0,1,1,1,0,0,0,0,1,1,1,1
};

static uint8_t read_packed_bit(const uint8_t *data, unsigned int bit_pos)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    return (uint8_t)((data[byte_idx] >> bit_idx) & 1u);
}

static void write_packed_bit(uint8_t *data, unsigned int bit_pos, uint8_t bit)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    if (bit & 1u)
        data[byte_idx] |= (uint8_t)(1u << bit_idx);
}

static uint32_t read_packed_field(const uint8_t *data,
                                  unsigned int bit_offset,
                                  unsigned int bit_count)
{
    uint32_t value = 0u;

    for (unsigned int i = 0u; i < bit_count; i++)
        value |= (uint32_t)read_packed_bit(data, bit_offset + i) << i;

    return value;
}

static bredr_payload_family_t bredr_classify_family(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x00u:
    case 0x01u:
        return BREDR_PAYLOAD_FAMILY_CONTROL;
    case 0x02u:
        return BREDR_PAYLOAD_FAMILY_FHS;
    case 0x03u:
    case 0x04u:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Eu:
    case 0x0Fu:
        return BREDR_PAYLOAD_FAMILY_ACL;
    case 0x05u:
    case 0x06u:
    case 0x07u:
        return BREDR_PAYLOAD_FAMILY_SCO;
    case 0x08u:
        return BREDR_PAYLOAD_FAMILY_HYBRID;
    case 0x0Cu:
    case 0x0Du:
        return BREDR_PAYLOAD_FAMILY_ESCO;
    default:
        return BREDR_PAYLOAD_FAMILY_UNKNOWN;
    }
}

static int bredr_payload_has_supported_coding(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x00u:
    case 0x01u:
    case 0x03u:
    case 0x04u:
    case 0x05u:
    case 0x06u:
    case 0x07u:
    case 0x08u:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Cu:
    case 0x0Du:
    case 0x0Eu:
    case 0x0Fu:
        return 1;
    default:
        return 0;
    }
}

static unsigned int bredr_sync_max_air_payload_bits(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x05u:
    case 0x06u:
        return 240u;
    case 0x07u:
        return 256u;
    case 0x0Cu:
        return 1470u;
    case 0x0Du:
        return 1456u;
    default:
        return 0u;
    }
}

static int bredr_acl_payload_uses_fec_2_3(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x03u:
    case 0x0Au:
    case 0x0Eu:
        return 1;
    default:
        return 0;
    }
}

static uint8_t bredr_fec_2_3_remainder(uint16_t codeword)
{
    /*
     * BT Core Spec generator g(D) = (D+1)(D^4+D+1) = D^5+D^4+D^2+1 (0x35),
     * with the first on-air bit being the highest-order coefficient. Our
     * packed codeword stores the first air bit at value bit 0, so when we
     * interpret value bit i as x^i, the effective generator is the
     * reciprocal of 0x35, namely x^5+x^3+x+1 = 0x2B.
     */
    uint16_t rem = codeword;

    for (int bit = 14; bit >= 5; bit--)
    {
        if (rem & ((uint16_t)1u << bit))
            rem ^= (uint16_t)(0x2Bu << (bit - 5));
    }

    return (uint8_t)(rem & 0x1Fu);
}

static void bredr_pack_header_raw(uint64_t header_raw, uint8_t packed[7])
{
    memset(packed, 0, 7u);
    for (unsigned int bit = 0u; bit < 54u; bit++)
    {
        if (header_raw & ((uint64_t)1u << bit))
            write_packed_bit(packed, bit, 1u);
    }
}

static int bredr_acl_payload_header_bytes(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x03u:
    case 0x04u:
    case 0x08u:
    case 0x09u:
        return 1;
    case 0x0Au:
    case 0x0Bu:
    case 0x0Eu:
    case 0x0Fu:
        return 2;
    default:
        return 0;
    }
}

unsigned int bredr_acl_max_user_payload_bytes(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x03u: // DM1
        return 17u;
    case 0x04u: // DH1
        return 27u;
    case 0x09u: // AUX1
        return 29u;
    case 0x0Au: // DM3
        return 121u;
    case 0x0Bu: // DH3
        return 183u;
    case 0x0Eu: // DM5
        return 224u;
    case 0x0Fu: // DH5
        return 339u;
    default:
        return 0u;
    }
}

static int bredr_acl_payload_supported(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x03u:
    case 0x04u:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:
    case 0x0Eu:
    case 0x0Fu:
        return 1;
    default:
        return 0;
    }
}

static unsigned int bredr_whitening_index_after_bits(uint8_t clk6,
                                                     unsigned int logical_bits)
{
    return (unsigned int)((s_whitening_indices[clk6 & 0x3Fu] + logical_bits) % 127u);
}

unsigned int bredr_dewhiten_air_payload_bytes(const uint8_t *src_air,
                                                     unsigned int src_air_bits,
                                                     uint8_t clk6,
                                                     unsigned int logical_offset_bits,
                                                     uint8_t *dst,
                                                     unsigned int dst_capacity)
{
    unsigned int bytes = (src_air_bits + 7u) / 8u;
    unsigned int index;

    if (!src_air || !dst || dst_capacity == 0u)
        return 0u;
    if (bytes > dst_capacity)
        bytes = dst_capacity;

    memset(dst, 0, bytes);
    index = bredr_whitening_index_after_bits(clk6, logical_offset_bits);

    for (unsigned int bit_pos = 0u; bit_pos < src_air_bits && (bit_pos / 8u) < bytes; bit_pos++)
    {
        uint8_t bit = read_packed_bit(src_air, bit_pos) ^ s_whitening_data[index];
        write_packed_bit(dst, bit_pos, bit);
        index = (index + 1u) % 127u;
    }

    return bytes;
}

static unsigned int bredr_decode_acl_payload_from_air(const bredr_frame_t *frame,
                                                      uint8_t type_code,
                                                      uint8_t clk6,
                                                      uint8_t *dst,
                                                      unsigned int dst_capacity)
{
    uint8_t decoded_air[BR_MAX_AIR_PAYLOAD_BYTES];
    unsigned int captured_air_bits;
    unsigned int input_air_bits;
    unsigned int decoded_air_bits = 0u;

    if (!frame || !dst || dst_capacity == 0u)
        return 0u;
    if (!bredr_acl_payload_supported(type_code))
        return 0u;

    captured_air_bits = frame->air_payload_bits;
    input_air_bits = bredr_on_air_payload_bits(type_code);
    if (input_air_bits == 0u)
        return 0u;
    if (captured_air_bits < input_air_bits)
        input_air_bits = captured_air_bits;

    switch (type_code & 0x0Fu)
    {
    case 0x03u:
    case 0x0Au:
    case 0x0Eu:
        input_air_bits -= input_air_bits % 15u;
        if (input_air_bits == 0u)
            return 0u;

        memset(decoded_air, 0, sizeof(decoded_air));
        if (bredr_fec_decode_2_3(frame->air_payload,
                                 input_air_bits,
                                 decoded_air,
                                 &decoded_air_bits) < 0)
            return 0u;

        return bredr_dewhiten_air_payload_bytes(decoded_air,
                                                decoded_air_bits,
                                                clk6,
                                                18u,
                                                dst,
                                                dst_capacity);

    case 0x04u:
    case 0x09u:
    case 0x0Bu:
    case 0x0Fu:
        return bredr_dewhiten_air_payload_bytes(frame->air_payload,
                                                input_air_bits,
                                                clk6,
                                                18u,
                                                dst,
                                                dst_capacity);

    default:
        return 0u;
    }
}

static unsigned int bredr_decode_sync_payload_from_air(const bredr_frame_t *frame,
                                                       uint8_t type_code,
                                                       uint8_t clk6,
                                                       uint8_t *dst,
                                                       unsigned int dst_capacity)
{
    uint8_t decoded_air[BR_MAX_AIR_PAYLOAD_BYTES];
    unsigned int captured_air_bits;
    unsigned int input_air_bits;
    unsigned int decoded_air_bits = 0u;

    if (!frame || !dst || dst_capacity == 0u)
        return 0u;

    captured_air_bits = frame->air_payload_bits;
    input_air_bits = bredr_sync_max_air_payload_bits(type_code);
    if (input_air_bits == 0u)
        return 0u;
    if (captured_air_bits < input_air_bits)
        input_air_bits = captured_air_bits;

    switch (type_code & 0x0Fu)
    {
    case 0x05u:
        input_air_bits -= input_air_bits % 3u;
        if (input_air_bits == 0u)
            return 0u;

        memset(decoded_air, 0, sizeof(decoded_air));
        if (bredr_fec_decode_1_3(frame->air_payload,
                                 input_air_bits,
                                 decoded_air,
                                 &decoded_air_bits) < 0)
            return 0u;

        return bredr_dewhiten_air_payload_bytes(decoded_air,
                                                decoded_air_bits,
                                                clk6,
                                                18u,
                                                dst,
                                                dst_capacity);

    case 0x06u:
    case 0x0Cu:
        input_air_bits -= input_air_bits % 15u;
        if (input_air_bits == 0u)
            return 0u;

        memset(decoded_air, 0, sizeof(decoded_air));
        if (bredr_fec_decode_2_3(frame->air_payload,
                                 input_air_bits,
                                 decoded_air,
                                 &decoded_air_bits) < 0)
            return 0u;

        return bredr_dewhiten_air_payload_bytes(decoded_air,
                                                decoded_air_bits,
                                                clk6,
                                                18u,
                                                dst,
                                                dst_capacity);

    case 0x07u:
    case 0x0Du:
        return bredr_dewhiten_air_payload_bytes(frame->air_payload,
                                                input_air_bits,
                                                clk6,
                                                18u,
                                                dst,
                                                dst_capacity);

    default:
        return 0u;
    }
}

void bredr_decode_dewhitened_header(const uint8_t dewhitened_header[18],
                                    bredr_decoded_header_t *out)
{
    if (!dewhitened_header || !out)
        return;

    out->lt_addr = (uint8_t)(dewhitened_header[0] | (dewhitened_header[1] << 1) | (dewhitened_header[2] << 2));
    out->type = (uint8_t)(dewhitened_header[3] | (dewhitened_header[4] << 1) |
                          (dewhitened_header[5] << 2) | (dewhitened_header[6] << 3));
    out->flow = dewhitened_header[7];
    out->arqn = dewhitened_header[8];
    out->seqn = dewhitened_header[9];
    out->hec = 0u;
    out->hec_ok = 0u;
    for (int i = 0; i < 8; i++)
        out->hec |= (uint8_t)(dewhitened_header[10 + i] << i);
}

static int bredr_decode_header(const bredr_frame_t *frame,
                               uint8_t uap,
                               uint8_t clk6,
                               bredr_decoded_header_t *out)
{
    uint8_t bits[18];
    uint16_t hdr_data = 0u;

    bredr_decode_header_bits(frame, clk6, bits);
    bredr_decode_dewhitened_header(bits, out);

    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    out->hec_ok = (uint8_t)(bredr_decode_uap_from_hec(hdr_data, out->hec) == uap);
    return out->hec_ok;
}

/*
 * BR/EDR payload CRC-16 per BT Core Spec Vol 2, Part B, §7.1.1.
 * Mirrors libbtbb's `crcgen`: register initialised with the bit-reversed UAP
 * in the high byte; bits are consumed in air order (LSB-first within each
 * byte of the packed buffer).
 */
uint16_t bredr_payload_crc(const uint8_t *data,
                           unsigned int bit_count,
                           uint8_t uap)
{
    uint16_t reg = (uint16_t)((uint16_t)bredr_reverse_byte(uap) << 8);

    for (unsigned int i = 0u; i < bit_count; i++)
    {
        uint8_t bit = read_packed_bit(data, i);
        reg = (uint16_t)((reg >> 1) | (((reg & 0x0001u) ^ (bit & 0x01u)) << 15));
        reg = (uint16_t)(reg ^ ((reg & 0x8000u) >> 5));
        reg = (uint16_t)(reg ^ ((reg & 0x8000u) >> 12));
    }
    return reg;
}

static int bredr_parse_acl_payload(const uint8_t *payload,
                                   unsigned int payload_len,
                                   uint8_t type_code,
                                   uint8_t uap,
                                   bredr_acl_payload_t *out,
                                   bredr_decode_limit_t *limit)
{
    int header_bytes = bredr_acl_payload_header_bytes(type_code);
    unsigned int max_user_bytes = bredr_acl_max_user_payload_bytes(type_code);

    if (!payload || !out || header_bytes == 0)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_INVALID_ACL_HEADER;
        return 0;
    }
    if (payload_len < (unsigned int)header_bytes)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    memset(out, 0, sizeof(*out));
    out->llid = (uint8_t)(payload[0] & 0x03u);
    out->flow = (uint8_t)((payload[0] >> 2u) & 0x01u);
    out->length = (uint16_t)((payload[0] >> 3u) & 0x1Fu);
    if (header_bytes == 2)
        out->length |= (uint16_t)((payload[1] & 0x1Fu) << 5u);

    if (max_user_bytes == 0u)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_INVALID_ACL_HEADER;
        return 0;
    }

    if (out->length > max_user_bytes)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH;
        return 0;
    }

    if (out->length > BREDR_ACL_MAX_USER_PAYLOAD_BYTES)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_INVALID_ACL_HEADER;
        return 0;
    }

    if (payload_len < (unsigned int)header_bytes + out->length)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    memcpy(out->user_payload, payload + header_bytes, out->length);

    // Check CRC
    unsigned int data_bytes = (unsigned int)header_bytes + out->length;
    if (payload_len >= data_bytes + 2u)
    {
        uint16_t computed = bredr_payload_crc(payload, data_bytes * 8u, uap);
        uint16_t received = (uint16_t)(payload[data_bytes]
                            | ((uint16_t)payload[data_bytes + 1u] << 8));
        out->has_crc = 1u;
        out->crc = received;
        out->crc_ok = (uint8_t)(computed == received);
    }
    

    if (limit)
        *limit = BREDR_DECODE_LIMIT_NONE;
    return 1;
}

static int bredr_parse_sync_payload(const uint8_t *payload,
                                    unsigned int payload_len,
                                    uint8_t type_code,
                                    uint8_t uap,
                                    bredr_sync_packet_t *out,
                                    bredr_payload_family_t *family,
                                    bredr_decode_limit_t *limit)
{
    unsigned int expected_bytes;
    unsigned int ev_crc_search_limit = 0u;
    unsigned int minimum_required_bytes;
    bredr_payload_family_t resolved_family;

    if (!payload || !out)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    memset(out, 0, sizeof(*out));

    switch (type_code & 0x0Fu)
    {
    case 0x0Cu:
    case 0x0Du:
        resolved_family = BREDR_PAYLOAD_FAMILY_ESCO;
        break;
    default:
        resolved_family = BREDR_PAYLOAD_FAMILY_SCO;
        break;
    }
    if (family)
        *family = resolved_family;

    switch (type_code & 0x0Fu)
    {
    case 0x05u:
        expected_bytes = 10u;
        break;
    case 0x06u:
        expected_bytes = 20u;
        break;
    case 0x07u:
        expected_bytes = 30u;
        ev_crc_search_limit = 32u;
        break;
    case 0x0Cu:
        expected_bytes = 122u;
        ev_crc_search_limit = expected_bytes;
        break;
    case 0x0Du:
        expected_bytes = 182u;
        ev_crc_search_limit = expected_bytes;
        break;
    default:
        if (limit)
            *limit = BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE;
        return 0;
    }

    minimum_required_bytes = (ev_crc_search_limit != 0u) ? 3u : expected_bytes;

    if (payload_len < minimum_required_bytes)
    {
        if (limit)
            *limit = BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD;
        return 0;
    }

    if (ev_crc_search_limit != 0u)
    {
        unsigned int search_limit = payload_len < ev_crc_search_limit ? payload_len : ev_crc_search_limit;

        for (unsigned int total_bytes = 3u; total_bytes <= search_limit; total_bytes++)
        {
            uint16_t computed = bredr_payload_crc(payload, (total_bytes - 2u) * 8u, uap);
            uint16_t received = (uint16_t)(payload[total_bytes - 2u] |
                                ((uint16_t)payload[total_bytes - 1u] << 8u));
            if (computed == received)
            {
                out->payload_bytes = (uint16_t)(total_bytes - 2u);
                memcpy(out->payload, payload, out->payload_bytes);
                out->has_crc = 1u;
                out->crc = received;
                out->crc_ok = 1u;
                out->is_esco = (uint8_t)(resolved_family == BREDR_PAYLOAD_FAMILY_ESCO ||
                                         (type_code & 0x0Fu) == 0x07u);
                if (family)
                    *family = BREDR_PAYLOAD_FAMILY_ESCO;
                if (limit)
                    *limit = BREDR_DECODE_LIMIT_NONE;
                return 1;
            }
        }

        if ((type_code & 0x0Fu) != 0x07u)
        {
            if (limit)
                *limit = BREDR_DECODE_LIMIT_ESCO_CRC_UNRESOLVED;
            return 0;
        }
    }

    if (payload_len < expected_bytes)
        expected_bytes = payload_len;

    out->payload_bytes = (uint16_t)expected_bytes;
    memcpy(out->payload, payload, expected_bytes);
    out->is_esco = (uint8_t)(resolved_family == BREDR_PAYLOAD_FAMILY_ESCO);

    if (limit)
        *limit = BREDR_DECODE_LIMIT_NONE;
    return 1;
}

int bredr_fec_decode_1_3(const uint8_t *input_bits,
                         unsigned int input_bit_count,
                         uint8_t *output_bits,
                         unsigned int *output_bit_count)
{
    unsigned int decoded_bit_count;
    int error_count = 0;

    if (!input_bits || !output_bits || !output_bit_count || (input_bit_count % 3u) != 0u)
        return -1;

    decoded_bit_count = input_bit_count / 3u;
    memset(output_bits, 0, (decoded_bit_count + 7u) / 8u);

    for (unsigned int out_bit = 0u; out_bit < decoded_bit_count; out_bit++)
    {
        unsigned int bit_base = out_bit * 3u;
        unsigned int ones = read_packed_bit(input_bits, bit_base)
                          + read_packed_bit(input_bits, bit_base + 1u)
                          + read_packed_bit(input_bits, bit_base + 2u);
        uint8_t decoded = (uint8_t)(ones >= 2u);

        error_count += decoded ? (int)(3u - ones) : (int)ones;
        write_packed_bit(output_bits, out_bit, decoded);
    }

    *output_bit_count = decoded_bit_count;
    return error_count;
}

int bredr_fec_decode_2_3(const uint8_t *input_bits,
                         unsigned int input_bit_count,
                         uint8_t *output_bits,
                         unsigned int *output_bit_count)
{
    unsigned int block_count;
    unsigned int decoded_bit_count;
    int error_count = 0;

    if (!input_bits || !output_bits || !output_bit_count || (input_bit_count % 15u) != 0u)
        return -1;

    block_count = input_bit_count / 15u;
    decoded_bit_count = block_count * 10u;
    memset(output_bits, 0, (decoded_bit_count + 7u) / 8u);

    for (unsigned int block = 0u; block < block_count; block++)
    {
        unsigned int input_offset = block * 15u;
        unsigned int output_offset = block * 10u;
        uint16_t corrected = (uint16_t)read_packed_field(input_bits, input_offset, 15u);

        if (bredr_fec_2_3_remainder(corrected) != 0u)
        {
            int corrected_single = 0;

            for (unsigned int bit = 0u; bit < 15u; bit++)
            {
                uint16_t candidate = (uint16_t)(corrected ^ ((uint16_t)1u << bit));
                if (bredr_fec_2_3_remainder(candidate) == 0u)
                {
                    corrected = candidate;
                    corrected_single = 1;
                    error_count += 1;
                    break;
                }
            }

            if (!corrected_single)
                return -1;
        }

        for (unsigned int bit = 0u; bit < 10u; bit++)
            write_packed_bit(output_bits, output_offset + bit, (uint8_t)((corrected >> bit) & 1u));
    }

    *output_bit_count = decoded_bit_count;
    return error_count;
}

bredr_fec_mode_t bredr_fec_mode_for_type(uint8_t type_code)
{
    switch (type_code & 0x0Fu)
    {
    case 0x05u: // HV1
        return BREDR_FEC_MODE_1_3;
    case 0x03u: // DM1
    case 0x06u: // HV2
    case 0x0Au: // DM3
    case 0x0Cu: // EV4
    case 0x0Eu: // DM5
        return BREDR_FEC_MODE_2_3;
    default:
        return BREDR_FEC_MODE_NONE;
    }
}

int valid_fec_1_3_blocks(const uint8_t *input_bits,
                         unsigned int input_bit_count)
{
    if (!input_bits || input_bit_count == 0u || (input_bit_count % 3u) != 0u)
        return -1;

    int valid_count = 0;

    for (unsigned int out_bit = 0u; out_bit < input_bit_count / 3u; out_bit++)
    {
        unsigned int bit_base = out_bit * 3u;
        unsigned int ones = (unsigned int)read_packed_bit(input_bits, bit_base)
                          + (unsigned int)read_packed_bit(input_bits, bit_base + 1u)
                          + (unsigned int)read_packed_bit(input_bits, bit_base + 2u);

        if (ones == 0u || ones == 3u)
            valid_count++;
    }

    return valid_count;
}

int valid_fec_2_3_blocks(const uint8_t *input_bits,
                         unsigned int input_bit_count)
{
    if (!input_bits || input_bit_count == 0u || (input_bit_count % 15u) != 0u)
        return -1;

    int valid_count = 0;

    for (unsigned int block = 0u; block < input_bit_count / 15u; block++)
    {
        unsigned int input_offset = block * 15u;
        uint16_t codeword = (uint16_t)read_packed_field(input_bits, input_offset, 15u);

        if (bredr_fec_2_3_remainder(codeword) == 0u)
            valid_count++;
    }

    return valid_count;
}

uint8_t bredr_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

/**
 * Recover the UAP from the 10-bit header value and the 8-bit HEC.
 *
 * This is the Bluetooth HEC *decode* (inverse of the hardware HEC generation).
 * It mirrors libbtbb's `uap_from_hec` semantics: the 8-bit HEC is run through a
 * 10-step LFSR (seeded by the UAP, XOR 0x65 on the MSB, consuming header bits
 * MSB-first) and the result is bit-reversed to obtain the UAP.
 *
 * NOTE: the HEC only constrains the UAP to one of (up to) 36 reachable values
 * for a given header -- it does not uniquely identify the UAP.  The final
 * disambiguation is performed by the payload CRC, exactly as libbtbb does in
 * `crc_check`.  Callers must therefore treat the returned value as a *candidate*
 * UAP and confirm it with the payload CRC before trusting it.
 */
uint8_t bredr_decode_uap_from_hec(uint16_t data, uint8_t hec)
{
    for (int i = 9; i >= 0; i--)
    {
        if (hec & 0x80u)
            hec ^= 0x65u;
        hec = (uint8_t)((hec << 1) | (((hec >> 7) ^ (data >> i)) & 0x01u));
    }
    return bredr_reverse_byte(hec);
}

void bredr_decode_header_bits(const bredr_frame_t *frame, uint8_t clk6, uint8_t bits[18])
{
    uint8_t packed_header[7];
    uint8_t decoded_header[3] = {0};
    unsigned int decoded_bits = 0u;

    bredr_pack_header_raw(frame->header_raw, packed_header);
    int be = bredr_fec_decode_1_3(packed_header, 54u, decoded_header, &decoded_bits);
    if (be < 0 || decoded_bits != 18u || be >= (int)(decoded_bits / 4u))
    {
        memset(bits, 0, 18u);
        return;
    }

    for (unsigned int i = 0u; i < 18u; i++)
        bits[i] = read_packed_bit(decoded_header, i);

    int index = (int)s_whitening_indices[clk6 & 0x3fu];
    for (int i = 0; i < 18; i++)
    {
        bits[i] ^= s_whitening_data[index];
        index = (index + 1) % 127;
    }
}

unsigned int bredr_on_air_payload_bits(uint8_t type_code)
{
    return s_on_air_payload_bits[type_code & 0x0Fu];
}

/* ---------------------------------------------------------------------------
 * Sync-word generation
 *
 * (64,30) linear block code, polynomial 0260534236651, modified for the
 * barker code.  Row i is XOR'd into the codeword when bit (23-i) of the
 * LAP is set.  Source: libbtbb bluetooth_packet.c (reference only).
 * ---------------------------------------------------------------------------*/

static const uint64_t s_sw_matrix[24] = {
    0xfe000002a0d1c014ULL, 0x01000003f0b9201fULL,
    0x008000033ae40edbULL, 0x004000035fca99b9ULL,
    0x002000036d5dd208ULL, 0x00100001b6aee904ULL,
    0x00080000db577482ULL, 0x000400006dabba41ULL,
    0x00020002f46d43f4ULL, 0x000100017a36a1faULL,
    0x00008000bd1b50fdULL, 0x000040029c3536aaULL,
    0x000020014e1a9b55ULL, 0x0000100265b5d37eULL,
    0x0000080132dae9bfULL, 0x000004025bd5ea0bULL,
    0x00000203ef526bd1ULL, 0x000001033511ab3cULL,
    0x000000819a88d59eULL, 0x00000040cd446acfULL,
    0x00000022a41aabb3ULL, 0x0000001390b5cb0dULL,
    0x0000000b0ae27b52ULL, 0x0000000585713da9ULL};

/* Default codeword (output when LAP == 0), already incorporating the PN
 * sequence and barker code modification. */
static const uint64_t SW_DEFAULT_CW = 0xb0000002c7820e7eULL;

uint64_t bredr_gen_syncword(uint32_t lap)
{
    uint64_t codeword = SW_DEFAULT_CW;
    for (int i = 0; i < 24; i++)
        if (lap & (0x800000u >> i))
            codeword ^= s_sw_matrix[i];
    return codeword;
}

/* ---------------------------------------------------------------------------
 * HEC verification
 * ---------------------------------------------------------------------------*/

int bredr_hec_ok_for_clk6(const bredr_frame_t *frame, uint8_t uap, uint8_t clk6)
{
    if (!frame || !frame->has_header)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(frame, (uint8_t)(clk6 & 0x3fu), bits);

    uint16_t hdr_data = 0;
    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    uint8_t received_hec = 0;
    for (int i = 0; i < 8; i++)
        received_hec |= (uint8_t)(bits[10 + i] << i);

    return (int)(bredr_decode_uap_from_hec(hdr_data, received_hec) == uap);
}

const char *bredr_packet_type_name(uint8_t type_code)
{
    return s_bredr_type_names[type_code & 0x0Fu];
}

const char *bredr_payload_family_name(bredr_payload_family_t family)
{
    switch (family)
    {
    case BREDR_PAYLOAD_FAMILY_NONE:
        return "None";
    case BREDR_PAYLOAD_FAMILY_CONTROL:
        return "Control";
    case BREDR_PAYLOAD_FAMILY_FHS:
        return "FHS";
    case BREDR_PAYLOAD_FAMILY_ACL:
        return "ACL";
    case BREDR_PAYLOAD_FAMILY_SCO:
        return "SCO";
    case BREDR_PAYLOAD_FAMILY_ESCO:
        return "eSCO";
    case BREDR_PAYLOAD_FAMILY_HYBRID:
        return "Hybrid";
    case BREDR_PAYLOAD_FAMILY_UNKNOWN:
    default:
        return "Unknown";
    }
}

const char *bredr_decode_limit_desc(bredr_decode_limit_t limit)
{
    switch (limit)
    {
    case BREDR_DECODE_LIMIT_NONE:
        return "none";
    case BREDR_DECODE_LIMIT_NO_HEADER:
        return "no header available";
    case BREDR_DECODE_LIMIT_MISSING_CONTEXT:
        return "missing UAP/CLK1-6 context";
    case BREDR_DECODE_LIMIT_HEC_FAILED:
        return "header HEC failed";
    case BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE:
        return "packet type not yet supported";
    case BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING:
        return "payload coding not yet supported";
    case BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD:
        return "payload shorter than indicated";
    case BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH:
        return "ACL length impossible for packet type";
    case BREDR_DECODE_LIMIT_INVALID_ACL_HEADER:
        return "invalid ACL payload header";
    case BREDR_DECODE_LIMIT_ESCO_CRC_UNRESOLVED:
        return "no valid eSCO CRC-backed payload length found";
    default:
        return "unknown decode limit";
    }
}

const char *bredr_llid_name(uint8_t llid)
{
    switch (llid)
    {
    case 0u:
        return "Reserved for Future Use";
    case 1u:
        return "Continuation";
    case 2u:
        return "L2CAP Start";
    case 3u:
        return "Control/LMP";
    default:
        return "Unknown";
    }
}

int bredr_decode_frame(const bredr_frame_t *frame,
                       uint8_t uap,
                       uint8_t clk1_6,
                       bredr_packet_t *out)
{
    uint8_t decoded_payload[BR_MAX_AIR_PAYLOAD_BYTES];
    uint8_t type_code;

    if (!frame || !out)
        return -1;

    memset(out, 0, sizeof(*out));
    out->phy = frame->phy;
    out->status = BREDR_DECODE_RAW_ONLY;
    out->limit = BREDR_DECODE_LIMIT_NONE;
    out->family = BREDR_PAYLOAD_FAMILY_NONE;
    out->air_payload_bytes = (uint16_t)bredr_frame_air_payload_bytes(frame);

    if (!frame->has_header)
    {
        out->limit = BREDR_DECODE_LIMIT_NO_HEADER;
        return 0;
    }

    if (!bredr_decode_header(frame, uap, clk1_6, &out->header))
    {
        out->limit = BREDR_DECODE_LIMIT_HEC_FAILED;
        return 0;
    }

    type_code = out->header.type;
    out->family = bredr_classify_family(type_code);
    out->status = BREDR_DECODE_HEADER_ONLY;

    if (out->family == BREDR_PAYLOAD_FAMILY_CONTROL)
    {
        out->status = BREDR_DECODE_FULL_PAYLOAD;
        return 1;
    }

    out->status = BREDR_DECODE_FAMILY_ONLY;

    if (out->family == BREDR_PAYLOAD_FAMILY_ACL)
    {
        out->decoded_payload_bytes = (uint16_t)bredr_decode_acl_payload_from_air(frame,
                                              type_code,
                                              clk1_6,
                                              decoded_payload,
                                              sizeof(decoded_payload));
        if (!bredr_parse_acl_payload(decoded_payload,
                                     out->decoded_payload_bytes,
                                     type_code,
                                     uap,
                                     &out->payload.acl,
                                     &out->limit))
        {
            out->status = BREDR_DECODE_PARTIAL_PAYLOAD;
            return 1;
        }

        out->status = BREDR_DECODE_FULL_PAYLOAD;
        return 1;
    }

    if (out->family == BREDR_PAYLOAD_FAMILY_SCO ||
        out->family == BREDR_PAYLOAD_FAMILY_ESCO)
    {
        out->decoded_payload_bytes = (uint16_t)bredr_decode_sync_payload_from_air(frame,
                                              type_code,
                                              clk1_6,
                                              decoded_payload,
                                              sizeof(decoded_payload));
        if (!bredr_parse_sync_payload(decoded_payload,
                                      out->decoded_payload_bytes,
                                      type_code,
                                      uap,
                                      &out->payload.sync,
                                      &out->family,
                                      &out->limit))
        {
            out->status = BREDR_DECODE_PARTIAL_PAYLOAD;
            return 1;
        }

        out->decoded_payload_bytes = (uint16_t)(out->payload.sync.payload_bytes +
                                     (out->payload.sync.has_crc ? 2u : 0u));

        out->status = BREDR_DECODE_FULL_PAYLOAD;
        return 1;
    }

    if (!bredr_payload_has_supported_coding(type_code))
    {
        out->limit = BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING;
        out->payload.unknown.payload_bytes = out->air_payload_bytes;
        return 1;
    }

    out->limit = BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE;
    out->payload.unknown.payload_bytes = out->air_payload_bytes;
    return 1;
}