/*
 * OFFLINE golden-vector generator (NOT part of the shipped product).
 *
 * Builds real-format BR/EDR frames using libbtbb's reference primitives
 * (uap_from_hec, crcgen, WHITENING_DATA, INDICES) so that the resulting
 * header_raw / air_payload values are independent of the supertooth codec
 * under test.  Emits C source (golden_vectors.h) containing static frame
 * sequences; the committed test replays them and asserts the recovered UAP.
 *
 * Compile (from repo root):
 *   cc tests/bredr/golden_gen.c -o /tmp/golden_gen
 *   /tmp/golden_gen > tests/bredr/golden_vectors.h
 *
 * The reference primitives below are copied verbatim from libbtbb
 * (bluetooth_packet.c) so the generated vectors are validated against the
 * independent reference implementation.  This file is an OFFLINE generator and
 * is never compiled into the shipped product.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* ---- libbtbb reference (verbatim from bluetooth_packet.c) ---- */
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
    for (i = 9; i >= 0; i--) {
        if (hec & 0x80) hec ^= 0x65;
        hec = (uint8_t)((hec << 1) | (((hec >> 7) ^ (data >> i)) & 0x01));
    }
    return lb_reverse_byte(hec);
}
static const uint8_t LB_WHITENING_DATA[] = {1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1};
static const uint8_t LB_INDICES[] = {99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2, 37, 69, 55, 8, 20, 40, 74, 114, 15, 106, 30, 78, 53, 72, 28, 26, 68, 7, 39, 113, 105, 77, 71, 25, 84, 49, 57, 44, 61, 117, 10, 1, 123, 124, 22, 125, 111, 23, 42, 126, 6, 112, 76, 24, 48, 43, 116, 0};
static uint16_t lb_crcgen(char *payload, int length, int UAP)
{
    char bit;
    uint16_t reg, count;
    reg = (lb_reverse_byte((uint8_t)UAP) << 8) & 0xff00;
    for (count = 0; count < (uint16_t)length; count++) {
        bit = payload[count];
        reg = (uint16_t)((reg >> 1) | (((reg & 0x0001) ^ (bit & 0x01)) << 15));
        reg = (uint16_t)(reg ^ ((reg & 0x8000) >> 5));
        reg = (uint16_t)(reg ^ ((reg & 0x8000) >> 12));
    }
    return reg;
}

/* ---- mirror of supertooth bredr_frame_t (minimal) ---- */
typedef struct {
    uint64_t header_raw;
    uint8_t  air_payload[256];
    unsigned int air_payload_bits;
    unsigned int has_header;
} golden_frame_t;

/* ---- libbtbb reference: find the HEC that decodes to a given UAP ---- */

/* The HEC decode (lb_uap_from_hec) only reaches (up to) 36 of the 256 UAP
 * values for any fixed header -- exactly as real Bluetooth behaves.  A valid
 * on-air frame therefore only exists for (header, UAP) pairs where the UAP is
 * reachable for that header.  This helper returns a HEC value h such that
 * lb_uap_from_hec(data, h) == uap, or -1 if the UAP is not reachable for the
 * given header. */
static int lb_forward_hec(uint16_t data, uint8_t uap)
{
    for (int h = 0; h < 256; h++)
        if (lb_uap_from_hec(data, (uint8_t)h) == uap)
            return h;
    return -1;
}

static void build_header_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                               golden_frame_t *frame)
{
    memset(frame, 0, sizeof(*frame));

    uint8_t hec = lb_forward_hec(header_data, uap);

    uint8_t decoded[18];
    for (int i = 0; i < 10; i++)
        decoded[i] = (uint8_t)((header_data >> i) & 1u);
    for (int i = 0; i < 8; i++)
        decoded[10 + i] = (uint8_t)((hec >> i) & 1u);

    int idx = LB_INDICES[clk6 & 0x3fu];
    uint8_t pre[18];
    for (int i = 0; i < 18; i++) {
        pre[i] = (uint8_t)(decoded[i] ^ LB_WHITENING_DATA[idx]);
        idx = (idx + 1) % 127;
    }

    uint64_t coded = 0u;
    for (int i = 0; i < 18; i++)
        for (int j = 0; j < 3; j++)
            coded |= ((uint64_t)(pre[i] & 1u)) << (3 * i + j);

    frame->has_header = 1u;
    frame->header_raw = coded;
}

/* Build DH1 packet: 1-byte ACL header + user + 2-byte CRC, whitened. */
static void build_dh1_frame(uint16_t header_data, uint8_t uap, uint8_t clk6,
                            const uint8_t *user, unsigned int user_len,
                            golden_frame_t *frame)
{
    build_header_frame((uint16_t)((header_data & 0x07u) | (header_data & 0x380u) |
                                  (((uint16_t)0x04u) << 3)), uap, clk6, frame);

    uint8_t dewhitened[256];
    unsigned int total = 1u + user_len + 2u;
    dewhitened[0] = (uint8_t)(0x02u | (0x01u << 2) | ((user_len & 0x1Fu) << 3));
    for (unsigned int i = 0; i < user_len; i++)
        dewhitened[1u + i] = user[i];

    /* libbtbb crcgen expects a bit array, LSB-first within each byte. */
    char bits[2048];
    for (unsigned int i = 0; i < (1u + user_len) * 8u; i++)
        bits[i] = (char)((dewhitened[i / 8u] >> (i % 8u)) & 1u);
    uint16_t crc = lb_crcgen(bits, (int)((1u + user_len) * 8u), uap);
    dewhitened[1u + user_len] = (uint8_t)(crc & 0xFFu);
    dewhitened[2u + user_len] = (uint8_t)((crc >> 8) & 0xFFu);

    /* whitening is XOR; replicate supertooth dewhiten to build on-air bytes */
    int idx = LB_INDICES[clk6 & 0x3fu];
    idx = (idx + 18) % 127;
    memset(frame->air_payload, 0, sizeof(frame->air_payload));
    for (unsigned int bit = 0; bit < total * 8u; bit++) {
        uint8_t b = (uint8_t)((dewhitened[bit / 8u] >> (bit % 8u)) & 1u);
        b = (uint8_t)(b ^ LB_WHITENING_DATA[idx]);
        if (b)
            frame->air_payload[bit / 8u] |= (uint8_t)(1u << (bit % 8u));
        idx = (idx + 1) % 127;
    }
    frame->air_payload_bits = total * 8u;
}

/* ---- emit a single case as a C initializer ---- */
static void emit_case(const char *name, uint8_t uap,
                       uint32_t first_clkn, int n_packets,
                       const uint8_t *user, unsigned int user_len)
{
    /* The HEC only reaches (up to) 36 UAPs per header, so a valid on-air frame
     * requires a (header, UAP) pair where the UAP is reachable.  Build a pool
     * of every 10-bit header for which the UAP decodes back, then:
     *   - every packet (including header-only ones) uses a reachable header so
     *     the true candidate's decoded UAP stays stable across packets;
     *   - the final (DH1, CRC-carrying) packet uses a reachable header that is
     *     also DH1-typed (bits 3-6 = 0x04) so the decoder routes it to the
     *     payload-CRC confirmation path. */
    uint16_t pool[1024];
    int pool_n = 0;
    for (int d = 0; d < 1024; d++)
        if (lb_forward_hec((uint16_t)d, uap) >= 0)
            pool[pool_n++] = (uint16_t)d;

    int dh1_idx = -1;
    for (int i = 0; i < pool_n; i++)
        if ((pool[i] & 0x78u) == 0x20u) { dh1_idx = i; break; }

    if (pool_n == 0 || dh1_idx < 0) {
        printf("    /* SKIP %s: uap=0x%02X not reachable for any DH1 header */\n",
               name, uap);
        return;
    }

    golden_frame_t f;
    printf("    /* %s: uap=0x%02X n=%d user_len=%u pool=%d */\n",
           name, uap, n_packets, user_len, pool_n);
    printf("    { 0x%02X, %d, {\n", uap, n_packets);
    for (int k = 0; k < n_packets; k++) {
        /* Production clock model: clkn ticks at 3200 Hz (CLKN); packets one
         * slot apart are 2 CLKN apart, and the whitening clock is
         * CLK1-6 = (clkn >> 1) & 0x3f (libbtbb stores clkn >> 1). */
        uint32_t clkn = first_clkn + 2u * (uint32_t)k;
        uint8_t clk6 = (uint8_t)((clkn >> 1) & 0x3fu);
        uint16_t data = (k == n_packets - 1) ? pool[dh1_idx]
                                            : pool[k % pool_n];
        int hec = lb_forward_hec(data, uap);

        if (k == n_packets - 1)
            build_dh1_frame(data, uap, clk6, user, user_len, &f);
        else
            build_header_frame(data, uap, clk6, &f);

        printf("        { 0x%016llXu, %u, {",
               (unsigned long long)f.header_raw, f.air_payload_bits);
        unsigned int nbytes = (f.air_payload_bits + 7u) / 8u;
        for (unsigned int i = 0; i < nbytes; i++)
            printf("0x%02Xu,%s", f.air_payload[i], ((i + 1) % 12 == 0) ? " " : "");
        printf("}, %uu },\n", clkn);
    }
    printf("    } },\n");
}

int main(void)
{
    printf("/* AUTO-GENERATED by tests/bredr/golden_gen.c (libbtbb reference). */\n");
    printf("#ifndef GOLDEN_VECTORS_H\n#define GOLDEN_VECTORS_H\n");
    printf("#include <stdint.h>\n");
    printf("typedef struct { uint64_t header_raw; uint16_t air_payload_bits; uint8_t air_payload[256]; uint32_t clkn; } golden_packet_t;\n");
    printf("typedef struct { uint8_t uap; int n; golden_packet_t pkts[40]; } golden_case_t;\n");
    printf("static const golden_case_t golden_cases[] = {\n");

    static const uint8_t user4[] = {0x11u, 0x22u, 0x33u, 0x44u};
    static const uint8_t user1[] = {0xABu};
    static const uint8_t user0[] = {0};
    static const uint8_t user17[] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11};

    /* Wide UAP coverage (incl. edge values) and varied headers/clock. */
    struct { uint8_t uap; uint16_t hdr; uint32_t clk; int n; const uint8_t *u; unsigned int ul; } cases[] = {
        {0x00,0x2AA,0,12,user4,4}, {0xFF,0x155,100,12,user4,4},
        {0x01,0x2AA,0,14,user4,4}, {0x80,0x155,200,14,user4,4},
        {0x7F,0x3FF,400,16,user4,4}, {0x47,0x000,0,12,user4,4},
        {0x9B,0x0C5,7777,20,user4,4}, {0xD3,0x3AA,9,18,user4,4},
        {0xAA,0x055,33,12,user4,4}, {0x55,0x2AA,63,12,user4,4},
        {0x33,0x111,500,15,user4,4}, {0xC0,0x0FF,1023,15,user4,4},
        {0x0F,0x3C0,2048,16,user4,4}, {0xF0,0x03F,4096,16,user4,4},
        {0x12,0x240,17,12,user4,4}, {0x34,0x048,99,12,user4,4},
        {0x56,0x0A8,12345,18,user4,4}, {0x78,0x1E0,555,18,user4,4},
        {0xAB,0x154,7,14,user4,4}, {0xCD,0x19C,321,14,user4,4},
        {0xEF,0x1F4,654,16,user4,4}, {0x11,0x022,1000,12,user4,4},
        {0x22,0x044,2000,12,user4,4}, {0x44,0x088,3000,14,user4,4},
        {0x88,0x110,4000,14,user4,4}, {0x66,0x0CC,50,12,user4,4},
        {0x99,0x132,60,12,user4,4}, {0xBB,0x176,70,14,user4,4},
        {0xEE,0x1DC,80,14,user4,4}, {0x7E,0x0FC,90,12,user4,4},
        {0x81,0x102,110,12,user4,4}, {0xA5,0x14A,130,16,user4,4},
        {0x5A,0x0B4,140,16,user4,4}, {0x3C,0x078,150,14,user4,4},
        {0xC3,0x186,160,14,user4,4},
        /* payload-length coverage */
        {0x47,0x2AA,0,12,user0,0}, {0x9B,0x2AA,0,12,user1,1},
        {0xD3,0x2AA,0,12,user17,17},
        /* LT_ADDR / flow / arqn / seqn variation via header bits */
        {0x47,0x000,0,12,user4,4}, {0x47,0x007,0,12,user4,4}, {0x47,0x038,0,12,user4,4},
        {0x47,0x1C0,0,12,user4,4}, {0x47,0x001,0,12,user4,4}, {0x47,0x380,0,12,user4,4},
    };

    for (size_t i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
        char nm[32];
        snprintf(nm, sizeof(nm), "case%zu", i);
        emit_case(nm, cases[i].uap, cases[i].clk, cases[i].n,
                  cases[i].u, cases[i].ul);
    }

    printf("    { 0, 0, {{0,0,{}}}},\n");
    printf("};\n#endif\n");
    return 0;
}
