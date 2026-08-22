/**
 * @file test_recovery_btbb_parity.c
 * @brief Differential test: native UAP recovery vs real libbtbb.
 *
 * Builds synthetic BR/EDR captures at runtime using verbatim copies of
 * libbtbb's reference primitives (HEC, CRC, whitening, FEC), feeds the exact
 * same frame sequences to:
 *
 *   1. the supertooth native backend (bredr_clock_recovery), and
 *   2. the real libbtbb btbb_uap_from_header() via btbb_test_adapter,
 *
 * and asserts that both stacks report the same outcome: recovered-or-not,
 * recovered UAP, CLK1-6 hint, and the packet index at which they converged.
 *
 * Clock model: frames are whitened with CLK1-6 = (clkn >> 1) & 0x3f and fed
 * with clkn advancing 2 ticks (one slot) per packet -- exactly the semantics
 * of the production pipeline (bredr_piconet_store passes 3200 Hz CLKN).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_clock_recovery.h"
#include "btbb_test_adapter.h"

/* libbtbb type codes (bluetooth_packet.h) */
#define PT_NULL 0
#define PT_POLL 1
#define PT_FHS 2
#define PT_DM1 3
#define PT_DH1 4
#define PT_HV1 5
#define PT_HV2 6
#define PT_HV3 7
#define PT_EV3 7 /* libbtbb: HV3 and EV3 share type code 7 */
#define PT_DV 8
#define PT_AUX1 9
#define PT_DM3 10
#define PT_DH3 11
#define PT_EV4 12
#define PT_EV5 13
#define PT_DM5 14
#define PT_DH5 15

static int g_failures = 0;

/* ------------------------------------------------------------------------
 * libbtbb reference primitives (verbatim copies, test-only)
 * ------------------------------------------------------------------------ */

static uint8_t lb_reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0) >> 4) | ((b & 0x0F) << 4));
    b = (uint8_t)(((b & 0xCC) >> 2) | ((b & 0x33) << 2));
    b = (uint8_t)(((b & 0xAA) >> 1) | ((b & 0x55) << 1));
    return b;
}

static uint8_t lb_uap_from_hec(uint16_t data, uint8_t hec)
{
    int i;
    for (i = 9; i >= 0; i--)
    {
        if (hec & 0x80)
            hec ^= 0x65;
        hec = (uint8_t)((hec << 1) | (((hec >> 7) ^ (data >> i)) & 0x01));
    }
    return lb_reverse_byte(hec);
}

static const uint8_t LB_WHITENING_DATA[127] = {
    1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0,
    1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1,
    1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0,
    1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0,
    0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1};

static const uint8_t LB_INDICES[64] = {
    99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2,
    37, 69, 55, 8, 20, 40, 74, 114, 15, 106, 30, 78, 53, 72, 28, 26,
    68, 7, 39, 113, 105, 77, 71, 25, 84, 49, 57, 44, 61, 117, 10, 1,
    123, 124, 22, 125, 111, 23, 42, 126, 6, 112, 76, 24, 48, 43, 116, 0};

static uint16_t lb_crcgen(const char *payload, int length, int UAP)
{
    char bit;
    uint16_t reg, count;
    reg = (uint16_t)((lb_reverse_byte((uint8_t)UAP) << 8) & 0xff00);
    for (count = 0; count < (uint16_t)length; count++)
    {
        bit = payload[count];
        reg = (uint16_t)((reg >> 1) | (((reg & 0x0001) ^ (bit & 0x01)) << 15));
        reg = (uint16_t)(reg ^ ((reg & 0x8000) >> 5));
        reg = (uint16_t)(reg ^ ((reg & 0x8000) >> 12));
    }
    return reg;
}

static int lb_forward_hec(uint16_t data, uint8_t uap)
{
    for (int h = 0; h < 256; h++)
        if (lb_uap_from_hec(data, (uint8_t)h) == uap)
            return h;
    return -1;
}

static const uint16_t fec23_gen_matrix[10] = {
    0x2c01, 0x5802, 0x1c04, 0x3808, 0x7010,
    0x4c20, 0x3440, 0x6880, 0x7d00, 0x5600};

static uint16_t fec23(uint16_t data)
{
    int i;
    uint16_t codeword = 0;
    for (i = 0; i < 10; i++)
        if (data & (1 << i))
            codeword ^= fec23_gen_matrix[i];
    return codeword;
}

static uint32_t g_rng = 0xBEEF1234u;
static uint32_t rng_next(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng >> 16;
}

/* ------------------------------------------------------------------------
 * Frame builders
 * ------------------------------------------------------------------------ */

typedef struct
{
    uint64_t header_raw;
    uint8_t air_payload[1024];
    unsigned int air_payload_bits;
} pframe_t;

static void bytes_to_bits(const uint8_t *bytes, unsigned int nbytes, char *bits)
{
    for (unsigned int i = 0; i < nbytes * 8u; i++)
        bits[i] = (char)((bytes[i / 8u] >> (i % 8u)) & 1u);
}

/* FEC 2/3 encode `nbits` (padded with zeros to a multiple of 10) into
 * `out`, returning the number of encoded symbols. */
static unsigned int fec23_encode_bits(const char *bits, unsigned int nbits, char *out)
{
    unsigned int blocks = (nbits + 9u) / 10u;
    for (unsigned int b = 0; b < blocks; b++)
    {
        uint16_t data = 0;
        for (unsigned int i = 0; i < 10u; i++)
        {
            unsigned int pos = b * 10u + i;
            if (pos < nbits && (bits[pos] & 1))
                data |= (uint16_t)(1u << i);
        }
        uint16_t cw = fec23(data);
        for (unsigned int i = 0; i < 15u; i++)
            out[b * 15u + i] = (char)((cw >> i) & 1u);
    }
    return blocks * 15u;
}

/* FEC 1/3 encode `nbits` into `out` (triple repetition). */
static unsigned int fec13_encode_bits(const char *bits, unsigned int nbits, char *out)
{
    for (unsigned int i = 0; i < nbits; i++)
    {
        out[3 * i] = bits[i];
        out[3 * i + 1] = bits[i];
        out[3 * i + 2] = bits[i];
    }
    return nbits * 3u;
}

/* Whiten `nbits` from `air_bits` with CLK1-6 `clk6` starting `skip` bits
 * into the whitening sequence (18 = after the packet header), packing the
 * result LSB-first into `out`. */
static void whiten_pack(const char *air_bits, unsigned int nbits, uint8_t clk6,
                        unsigned int skip, uint8_t *out)
{
    unsigned int nbytes = (nbits + 7u) / 8u;
    memset(out, 0, nbytes);
    int idx = ((int)LB_INDICES[clk6 & 0x3fu] + (int)skip) % 127;
    for (unsigned int i = 0; i < nbits; i++)
    {
        uint8_t b = (uint8_t)((air_bits[i] & 1) ^ LB_WHITENING_DATA[idx]);
        if (b)
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        idx = (idx + 1) % 127;
    }
}

static void build_header(uint16_t header_data, uint8_t uap, uint8_t clk6,
                         pframe_t *f)
{
    memset(f, 0, sizeof(*f));
    int hec = lb_forward_hec(header_data, uap);
    if (hec < 0)
        hec = 0; /* unreachable header: emit HEC 0 (still a valid frame) */

    char pre[18];
    for (int i = 0; i < 10; i++)
        pre[i] = (char)((header_data >> i) & 1u);
    for (int i = 0; i < 8; i++)
        pre[10 + i] = (char)(((uint8_t)hec >> i) & 1u);

    int idx = LB_INDICES[clk6 & 0x3fu];
    char whitened[18];
    for (int i = 0; i < 18; i++)
    {
        whitened[i] = (char)(pre[i] ^ LB_WHITENING_DATA[idx]);
        idx = (idx + 1) % 127;
    }

    f->header_raw = 0;
    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 3; j++)
            f->header_raw |= (uint64_t)(whitened[i] & 1u) << (3 * i + j);
}

/* Set the TYPE field (bits 3-6) of a 10-bit header value. */
static uint16_t header_with_type(uint16_t header_data, uint8_t type)
{
    return (uint16_t)((header_data & 0x07u) |
                      ((uint16_t)(type & 0x0Fu) << 3) |
                      (header_data & 0x380u));
}

/* ACL-style payload (DM/DH/FHS/AUX1): payload header + user + CRC, optional
 * 2/3 FEC, whitened from offset 18. */
static void build_acl_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                            int type, const uint8_t *user, unsigned int user_len,
                            int corrupt_crc, unsigned int trailing_noise_bits,
                            pframe_t *f)
{
    int header_bytes;
    int fec;
    int has_crc = (type != PT_AUX1);

    switch (type)
    {
    case PT_DM1:
    case PT_DH1:
    case PT_AUX1:
        header_bytes = 1;
        break;
    default:
        header_bytes = 2;
        break;
    }
    fec = (type == PT_DM1 || type == PT_DM3 || type == PT_DM5) ? 1 : 0;

    uint8_t bytes[512];
    unsigned int total = header_bytes + user_len + (has_crc ? 2u : 0u);
    unsigned int len_field = user_len;

    if (header_bytes == 1)
        bytes[0] = (uint8_t)(0x02u | (0x01u << 2) | ((len_field & 0x1Fu) << 3));
    else
    {
        bytes[0] = (uint8_t)(0x02u | (0x01u << 2) | ((len_field & 0x1Fu) << 3));
        bytes[1] = (uint8_t)((len_field >> 5) & 0x1Fu);
    }
    memcpy(&bytes[header_bytes], user, user_len);

    if (has_crc)
    {
        char bits[4096];
        bytes_to_bits(bytes, header_bytes + user_len, bits);
        uint16_t crc = lb_crcgen(bits, (int)((header_bytes + user_len) * 8u), uap);
        if (corrupt_crc)
            crc ^= 0xBEEFu; /* emulate encryption: CRC will not verify */
        bytes[header_bytes + user_len] = (uint8_t)(crc & 0xFFu);
        bytes[header_bytes + user_len + 1] = (uint8_t)((crc >> 8) & 0xFFu);
    }

    char data_bits[4096];
    bytes_to_bits(bytes, total, data_bits);

    char air[4608];
    unsigned int air_bits;
    if (fec)
        air_bits = fec23_encode_bits(data_bits, total * 8u, air);
    else
    {
        memcpy(air, data_bits, total * 8u);
        air_bits = total * 8u;
    }

    build_header(header_with_type(header_data, (uint8_t)type), uap, clk6, f);
    whiten_pack(air, air_bits, clk6, 18u, f->air_payload);
    f->air_payload_bits = air_bits;

    /* Append raw noise bits (models PHY over-collection past packet end). */
    for (unsigned int i = 0; i < trailing_noise_bits; i++)
    {
        uint8_t b = (uint8_t)(rng_next() & 1u);
        unsigned int pos = air_bits + i;
        if (b)
            f->air_payload[pos / 8u] |= (uint8_t)(1u << (pos % 8u));
    }
    f->air_payload_bits += trailing_noise_bits;
}

/* FHS: fixed 18-byte payload + 2-byte CRC, always 2/3 FEC. */
static void build_fhs_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                            int corrupt_crc, unsigned int trailing_noise_bits,
                            pframe_t *f)
{
    uint8_t bytes[20];
    for (unsigned int i = 0; i < 18u; i++)
        bytes[i] = (uint8_t)(0xA5u ^ (uint8_t)i);

    char bits[160];
    bytes_to_bits(bytes, 18u, bits);
    uint16_t crc = lb_crcgen(bits, 144, uap);
    if (corrupt_crc)
        crc ^= 0xDEADu;
    bytes[18] = (uint8_t)(crc & 0xFFu);
    bytes[19] = (uint8_t)((crc >> 8) & 0xFFu);

    char data_bits[160];
    bytes_to_bits(bytes, 20u, data_bits);
    char air[240];
    unsigned int air_bits = fec23_encode_bits(data_bits, 160u, air);

    build_header(header_with_type(header_data, PT_FHS), uap, clk6, f);
    whiten_pack(air, air_bits, clk6, 18u, f->air_payload);
    f->air_payload_bits = air_bits;

    for (unsigned int i = 0; i < trailing_noise_bits; i++)
    {
        uint8_t b = (uint8_t)(rng_next() & 1u);
        unsigned int pos = air_bits + i;
        if (b)
            f->air_payload[pos / 8u] |= (uint8_t)(1u << (pos % 8u));
    }
    f->air_payload_bits += trailing_noise_bits;
}

/* DV: 80 voice bits followed by a 2/3-FEC data field (1-byte payload
 * header + body + CRC). */
static void build_dv_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                           const uint8_t *user, unsigned int user_len,
                           int corrupt_crc, pframe_t *f)
{
    uint8_t bytes[64];
    unsigned int total = 1u + user_len + 2u;
    bytes[0] = (uint8_t)(0x02u | (0x01u << 2) | ((user_len & 0x1Fu) << 3));
    memcpy(&bytes[1], user, user_len);

    char bits[512];
    bytes_to_bits(bytes, 1u + user_len, bits);
    uint16_t crc = lb_crcgen(bits, (int)((1u + user_len) * 8u), uap);
    if (corrupt_crc)
        crc ^= 0x5AADu;
    bytes[1u + user_len] = (uint8_t)(crc & 0xFFu);
    bytes[2u + user_len] = (uint8_t)((crc >> 8) & 0xFFu);

    char data_bits[512];
    bytes_to_bits(bytes, total, data_bits);

    char air[1024];
    unsigned int pos = 0;
    for (unsigned int i = 0; i < 80u; i++)
        air[pos++] = (char)((i * 7u + 3u) & 1u); /* voice pattern */
    pos += fec23_encode_bits(data_bits, total * 8u, &air[pos]);

    build_header(header_with_type(header_data, PT_DV), uap, clk6, f);
    whiten_pack(air, pos, clk6, 18u, f->air_payload);
    f->air_payload_bits = pos;
}

/* eSCO-style payloads: EV3/EV5 have no FEC, EV4 uses 2/3 FEC; the payload
 * is body + CRC with no payload header (crc_check brute-forces lengths). */
static void build_ev_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                           int type, const uint8_t *body, unsigned int body_len,
                           pframe_t *f)
{
    uint8_t bytes[256];
    unsigned int total = body_len + 2u;
    memcpy(bytes, body, body_len);

    char bits[2048];
    bytes_to_bits(bytes, body_len, bits);
    uint16_t crc = lb_crcgen(bits, (int)(body_len * 8u), uap);
    bytes[body_len] = (uint8_t)(crc & 0xFFu);
    bytes[body_len + 1] = (uint8_t)((crc >> 8) & 0xFFu);

    char data_bits[2048];
    bytes_to_bits(bytes, total, data_bits);

    char air[2304];
    unsigned int air_bits;
    if (type == PT_EV4)
        air_bits = fec23_encode_bits(data_bits, total * 8u, air);
    else
    {
        memcpy(air, data_bits, total * 8u);
        air_bits = total * 8u;
    }

    build_header(header_with_type(header_data, (uint8_t)type), uap, clk6, f);
    whiten_pack(air, air_bits, clk6, 18u, f->air_payload);
    f->air_payload_bits = air_bits;
}

/* SCO payloads: HV1 (1/3 FEC, 80 bits), HV2 (2/3 FEC, 160 bits),
 * HV3 (raw, 240 bits). No CRC. `corrupt` injects uncorrectable errors. */
static void build_hv_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                           int type, int corrupt, pframe_t *f)
{
    unsigned int payload_bits = (type == PT_HV1) ? 80u
                                : (type == PT_HV2) ? 160u
                                                   : 240u;
    char data_bits[256];
    for (unsigned int i = 0; i < payload_bits; i++)
        data_bits[i] = (char)((i * 13u + 5u) & 1u);

    char air[1024];
    unsigned int air_bits;
    if (type == PT_HV1)
        air_bits = fec13_encode_bits(data_bits, payload_bits, air);
    else if (type == PT_HV2)
        air_bits = fec23_encode_bits(data_bits, payload_bits, air);
    else
    {
        memcpy(air, data_bits, payload_bits);
        air_bits = payload_bits;
    }

    if (corrupt)
    {
        /* Flip two adjacent symbols in the first FEC block (uncorrectable
         * for HV1/HV2) or raw bits for HV3. */
        air[0] ^= 1;
        air[1] ^= 1;
    }

    build_header(header_with_type(header_data, (uint8_t)type), uap, clk6, f);
    whiten_pack(air, air_bits, clk6, 18u, f->air_payload);
    f->air_payload_bits = air_bits;
}

/* ------------------------------------------------------------------------
 * Scenario runner
 * ------------------------------------------------------------------------ */

typedef struct
{
    pframe_t f;
    uint32_t clkn;
} scen_pkt_t;

static void make_bredr_frame(const pframe_t *pf, uint32_t lap, bredr_frame_t *out)
{
    memset(out, 0, sizeof(*out));
    out->lap = lap;
    out->has_header = 1;
    out->header_raw = pf->header_raw;
    out->air_payload_bits = pf->air_payload_bits;
    unsigned int nb = (pf->air_payload_bits + 7u) / 8u;
    if (nb > sizeof(out->air_payload))
        nb = sizeof(out->air_payload);
    memcpy(out->air_payload, pf->air_payload, nb);
}

static void run_scenario(const char *name, uint32_t lap, scen_pkt_t *pkts, int n)
{
    /* Native backend. */
    bredr_piconet_t *pnet = malloc(sizeof(*pnet));
    if (!pnet)
        return;
    bredr_piconet_init(pnet, lap);
    int nat_rec = 0, nat_idx = -1;
    uint8_t nat_uap = 0, nat_clk = 0;
    for (int i = 0; i < n; i++)
    {
        bredr_frame_t fr;
        make_bredr_frame(&pkts[i].f, lap, &fr);
        bredr_recovery_result_t r;
        if (bredr_recovery_process_packet(pnet, &fr, 0, pkts[i].clkn, &r))
        {
            nat_rec = 1;
            nat_idx = i;
            nat_uap = r.uap;
            nat_clk = r.clk6_hint;
            break;
        }
    }
    free(pnet);

    /* Real libbtbb. */
    btbb_piconet *pn = btbb_test_piconet_new(lap);
    int lb_rec = 0, lb_idx = -1;
    uint8_t lb_uap = 0, lb_clk = 0;
    for (int i = 0; i < n && pn; i++)
    {
        bredr_frame_t fr;
        make_bredr_frame(&pkts[i].f, lap, &fr);
        uint8_t u = 0, c = 0;
        if (btbb_test_run_uap_recovery(pn, &fr, 0, pkts[i].clkn, &u, &c))
        {
            lb_rec = 1;
            lb_idx = i;
            lb_uap = u;
            lb_clk = c;
            break;
        }
    }
    if (pn)
        btbb_piconet_unref(pn);

    int match = (nat_rec == lb_rec) &&
                (!nat_rec || (nat_uap == lb_uap && nat_clk == lb_clk &&
                              nat_idx == lb_idx));
    if (!match)
    {
        fprintf(stderr,
                "PARITY MISMATCH [%s]: native(rec=%d uap=0x%02X clk=%u idx=%d) "
                "libbtbb(rec=%d uap=0x%02X clk=%u idx=%d)\n",
                name, nat_rec, nat_uap, nat_clk, nat_idx,
                lb_rec, lb_uap, lb_clk, lb_idx);
        g_failures++;
    }
    else
    {
        printf("  [%-28s] match: rec=%d uap=0x%02X clk=%u idx=%d\n",
               name, nat_rec, nat_uap, nat_clk, nat_idx);
    }
}

/* ------------------------------------------------------------------------
 * Scenario construction
 * ------------------------------------------------------------------------ */

#define MAX_SCEN_PKTS 48

typedef struct
{
    scen_pkt_t pkts[MAX_SCEN_PKTS];
    int n;
} scen_t;

static void scen_add_header_only(scen_t *s, uint16_t data, uint8_t uap,
                                 uint32_t clkn)
{
    pframe_t f;
    build_header(data, uap, (uint8_t)((clkn >> 1) & 0x3fu), &f);
    s->pkts[s->n].f = f;
    s->pkts[s->n].clkn = clkn;
    s->n++;
}

/* Header pool: 10-bit header values whose HEC can encode `uap`. */
static int build_pool(uint16_t *pool, uint8_t uap)
{
    int n = 0;
    for (int d = 0; d < 1024; d++)
        if (lb_forward_hec((uint16_t)d, uap) >= 0)
            pool[n++] = (uint16_t)d;
    return n;
}

/* Pick a pool header with a specific TYPE field, or -1. */
static int pool_find_type(const uint16_t *pool, int pool_n, uint8_t type)
{
    for (int i = 0; i < pool_n; i++)
        if ((pool[i] & 0x78u) == ((uint16_t)type << 3))
            return i;
    return -1;
}

/* Standard shape: k header-only packets then one CRC-carrying packet. */
static void scen_header_then_payload(scen_t *s, uint8_t uap, uint32_t clkn_base,
                                     int clkn_step, int header_count,
                                     int payload_type, const uint8_t *user,
                                     unsigned int user_len, int corrupt_crc,
                                     unsigned int trailing_noise_bits)
{
    uint16_t pool[1024];
    int pool_n = build_pool(pool, uap);
    int pay_idx = pool_find_type(pool, pool_n, (uint8_t)payload_type);
    if (pool_n == 0 || pay_idx < 0)
    {
        fprintf(stderr, "scenario build failed: uap=0x%02X type=%d unreachable\n",
                uap, payload_type);
        g_failures++;
        return;
    }

    s->n = 0;
    uint32_t clkn = clkn_base;
    for (int k = 0; k < header_count; k++)
    {
        scen_add_header_only(s, pool[k % pool_n], uap, clkn);
        clkn += (uint32_t)clkn_step;
    }

    pframe_t f;
    uint8_t clk6 = (uint8_t)((clkn >> 1) & 0x3fu);
    switch (payload_type)
    {
    case PT_FHS:
        build_fhs_frame(pool[pay_idx], uap, clk6, corrupt_crc,
                        trailing_noise_bits, &f);
        break;
    case PT_DV:
        build_dv_frame(pool[pay_idx], uap, clk6, user, user_len, corrupt_crc, &f);
        break;
    case PT_EV4:
    case PT_EV5:
        build_ev_frame(pool[pay_idx], uap, clk6, payload_type, user, user_len, &f);
        break;
    case PT_HV1:
    case PT_HV2:
    case PT_HV3: /* also EV3: same type code; built as raw SCO payload */
        build_hv_frame(pool[pay_idx], uap, clk6, payload_type, 0, &f);
        break;
    default:
        build_acl_frame(pool[pay_idx], uap, clk6, payload_type, user, user_len,
                        corrupt_crc, trailing_noise_bits, &f);
        break;
    }
    s->pkts[s->n].f = f;
    s->pkts[s->n].clkn = clkn;
    s->n++;
}

int main(void)
{
    static const uint8_t user4[] = {0x11, 0x22, 0x33, 0x44};
    static const uint8_t user8[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
    static const uint8_t user20[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                       11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    scen_t s;
    char name[64];

    /* 1. Header-only + DH1 confirm across a UAP sweep. */
    static const uint8_t uaps[] = {0x00, 0x15, 0x27, 0x47, 0x9B, 0xFF, 0x80, 0x01};
    for (unsigned int i = 0; i < sizeof(uaps); i++)
    {
        snprintf(name, sizeof(name), "dh1_confirm_uap_%02X", uaps[i]);
        scen_header_then_payload(&s, uaps[i], 0x100u + 2u * i, 2, 12,
                                 PT_DH1, user4, 4, 0, 0);
        run_scenario(name, 0x001FC475u, s.pkts, s.n);
    }

    /* 2. Confirm via each ACL/FHS payload type. */
    static const int confirm_types[] = {PT_DM1, PT_DM3, PT_DM5, PT_DH3, PT_DH5,
                                        PT_FHS, PT_DV, PT_EV4};
    for (unsigned int i = 0; i < sizeof(confirm_types) / sizeof(confirm_types[0]); i++)
    {
        snprintf(name, sizeof(name), "confirm_type_%d", confirm_types[i]);
        scen_header_then_payload(&s, 0x15, 0x200u, 2, 12,
                                 confirm_types[i], user8, 8, 0, 0);
        run_scenario(name, 0x001FC475u, s.pkts, s.n);
    }

    /* 3. Encrypted (CRC-mismatch) payload must not prune candidates; the
     *    stream then converges on header consistency or a later clean packet. */
    scen_header_then_payload(&s, 0x15, 0x300u, 2, 16, PT_DH1, user4, 4, 1, 0);
    run_scenario("encrypted_dh1_headers_only", 0x001FC475u, s.pkts, s.n);

    /* Encrypted then clean: converges on the clean packet. */
    {
        scen_t s2;
        scen_header_then_payload(&s, 0x15, 0x340u, 2, 8, PT_DH1, user4, 4, 1, 0);
        s2 = s;
        scen_header_then_payload(&s, 0x15, 0x340u + 2u * 9, 2, 4, PT_DH1, user8, 8, 0, 0);
        /* merge: second sequence continues from the first's frames */
        for (int k = 9; k < s.n && s2.n < MAX_SCEN_PKTS; k++)
            s2.pkts[s2.n++] = s.pkts[k];
        run_scenario("encrypted_then_clean_dh1", 0x001FC475u, s2.pkts, s2.n);
    }

    /* 4. Trailing noise after the payload (PHY over-collects). */
    scen_header_then_payload(&s, 0x15, 0x400u, 2, 10, PT_DH1, user4, 4, 0, 137);
    run_scenario("dh1_trailing_noise_137", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x420u, 2, 10, PT_DH1, user4, 4, 0, 150);
    run_scenario("dh1_trailing_noise_150", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x440u, 2, 10, PT_DM1, user4, 4, 0, 90);
    run_scenario("dm1_trailing_noise_90", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x460u, 2, 10, PT_FHS, user4, 4, 0, 64);
    run_scenario("fhs_trailing_noise_64", 0x001FC475u, s.pkts, s.n);

    /* 5. Inconclusive types mixed in (must not disturb convergence). */
    scen_header_then_payload(&s, 0x15, 0x500u, 2, 6, PT_AUX1, user4, 4, 0, 0);
    run_scenario("aux1_then_nothing", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x520u, 2, 6, PT_EV3, user8, 8, 0, 0);
    run_scenario("ev3_brute_force", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x540u, 2, 6, PT_EV5, user8, 8, 0, 0);
    run_scenario("ev5_brute_force", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x560u, 2, 6, PT_HV1, user4, 4, 0, 0);
    run_scenario("hv1_valid", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x580u, 2, 6, PT_HV2, user4, 4, 0, 0);
    run_scenario("hv2_valid", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x5A0u, 2, 6, PT_HV3, user4, 4, 0, 0);
    run_scenario("hv3_valid", 0x001FC475u, s.pkts, s.n);

    /* 6. HV1 with uncorrectable FEC errors: hard prune on both stacks. */
    scen_header_then_payload(&s, 0x15, 0x600u, 2, 6, PT_HV1, user4, 4, 0, 0);
    {
        /* rebuild last packet as corrupted HV1 */
        uint16_t pool[1024];
        int pool_n = build_pool(pool, 0x15);
        int idx = pool_find_type(pool, pool_n, PT_HV1);
        if (idx >= 0)
        {
            uint32_t clkn = s.pkts[s.n - 1].clkn;
            build_hv_frame(pool[idx], 0x15, (uint8_t)((clkn >> 1) & 0x3fu),
                           PT_HV1, 1, &s.pkts[s.n - 1].f);
        }
    }
    run_scenario("hv1_corrupted_fec", 0x001FC475u, s.pkts, s.n);

    /* 7. Multi-slot spacing (larger clkn gaps) and odd clkn base. */
    scen_header_then_payload(&s, 0x15, 0x701u, 4, 12, PT_DH1, user4, 4, 0, 0);
    run_scenario("dh1_gap4_odd_base", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x15, 0x800u, 6, 12, PT_DM5, user20, 20, 0, 0);
    run_scenario("dm5_gap6", 0x001FC475u, s.pkts, s.n);

    /* 8. Long header-only stream (no payload at all): both stacks must
     *    agree on whether/when remaining==1 convergence happens. */
    {
        uint16_t pool[1024];
        int pool_n = build_pool(pool, 0x47);
        s.n = 0;
        for (int k = 0; k < 40 && s.n < MAX_SCEN_PKTS; k++)
            scen_add_header_only(&s, pool[k % pool_n], 0x47, 0x900u + 2u * (uint32_t)k);
        run_scenario("header_only_40", 0x001FC475u, s.pkts, s.n);
    }

    /* 9. Larger user payloads (length field spans header bytes). */
    scen_header_then_payload(&s, 0x27, 0xA00u, 2, 10, PT_DH5, user20, 20, 0, 0);
    run_scenario("dh5_user20", 0x001FC475u, s.pkts, s.n);

    scen_header_then_payload(&s, 0x27, 0xA20u, 2, 10, PT_DM3, user20, 20, 0, 0);
    run_scenario("dm3_user20", 0x001FC475u, s.pkts, s.n);

    if (g_failures != 0)
    {
        fprintf(stderr, "test_recovery_btbb_parity: %d scenario(s) diverged\n",
                g_failures);
        return 1;
    }
    printf("test_recovery_btbb_parity: native recovery matches libbtbb "
           "on all scenarios\n");
    return 0;
}
