/* Unified BLE bitstream decoder tests.
 *
 * Covers: advertising regression, preamble-based data detection, the CRC
 * gate (confirmed / candidate-proof / empty-packet recovery), CONNECT_IND
 * seeding across decoders sharing a store, and the rejection rule (reject
 * only bits up to the next preamble) via rescan scenarios.
 *
 * Construction notes: false candidates use an all-zero AA so their prefix
 * introduces no spurious preambles, and crafted headers are chosen (via
 * pick_false_len) so no preamble exists between the false preamble and a
 * planted real packet. Note that any real packet whose AA bit0 is 1 has an
 * overlapping preamble at bit offset 1 (0x55+1 -> 0xAA): after a reject the
 * rescan tries that overlap candidate first, so streams that chain packets
 * pad with enough zero bits to let spurious candidates complete and reject.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_test_stream.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define CH_DATA 4u
#define CH_ADV 37u
#define AA_DATA 0x9E3C5A77u
#define INIT_DATA 0x2C4A6Eu

/* Enough trailing zero bits for any spurious candidate (max 2120 bits) to
 * complete and reject before the next phase begins. */
#define FLUSH_ZEROS 2300u

/* ---------------------------------------------------------------------------
 * Stream builders
 * ---------------------------------------------------------------------------*/

/* Push a full data packet (preamble + AA + whitened PDU + CRC). */
static void stream_data_packet(ble_test_stream_t *s, uint32_t aa,
                               uint8_t hdr0, const uint8_t *payload,
                               unsigned int payload_len, uint8_t ch,
                               uint32_t crc_init)
{
    uint8_t pdu[2u + BLE_LL_PAYLOAD_MAX_BYTES];
    pdu[0] = hdr0;
    pdu[1] = (uint8_t)payload_len;
    if (payload_len > 0u)
        memcpy(&pdu[2], payload, payload_len);
    bts_push_preamble(s, bts_preamble_for_aa(aa));
    bts_push_aa(s, aa);
    bts_push_pdu(s, pdu, 2u + payload_len, ch, crc_init);
}

/* Push a full advertising packet on the advertising AA. */
static void stream_adv_packet(ble_test_stream_t *s, uint8_t pdu_type,
                              const uint8_t *payload, unsigned int payload_len,
                              uint8_t ch)
{
    uint8_t pdu[2u + BLE_RESERVED_PAYLOAD_MAX_BYTES];
    pdu[0] = pdu_type;
    pdu[1] = (uint8_t)payload_len;
    if (payload_len > 0u)
        memcpy(&pdu[2], payload, payload_len);
    bts_push_preamble(s, bts_preamble_for_aa(BLE_ADVERTISING_AA));
    bts_push_aa(s, BLE_ADVERTISING_AA);
    bts_push_pdu(s, pdu, 2u + payload_len, ch, BLE_CRC_INIT_ADV);
}

/* Push @p nbits zero bits (no preambles: deterministic filler). */
static void stream_zeros(ble_test_stream_t *s, unsigned int nbits)
{
    for (unsigned int i = 0; i < nbits; i++)
        bts_push_bit(s, 0u);
}

/* First preamble (0x55/0xAA) at any bit offset >= @p from, or -1. */
static int first_preamble_offset(const ble_test_stream_t *s, unsigned int from)
{
    for (unsigned int i = from; i + 8u <= s->count; i++)
    {
        uint8_t w = 0u;
        for (unsigned int j = 0; j < 8u; j++)
            w |= (uint8_t)(((s->bytes[(i + j) / 8u] >> ((i + j) % 8u)) & 1u)
                           << j);
        if (w == 0x55u || w == 0xAAu)
            return (int)i;
    }
    return -1;
}

/* Assert the first preamble after bit 0 is at @p expected, tolerating the
 * 1-bit overlap: 0x55 preceded by a 0 bit forms 0xAA at expected-1 (and
 * 0xAA preceded by 1 forms 0x55). The decoder rescans through the overlap
 * candidate; the assertion guards against stray preambles ANYWHERE ELSE. */
static void expect_first_preamble(const ble_test_stream_t *s,
                                  unsigned int expected,
                                  unsigned int line)
{
    int off = first_preamble_offset(s, 1u);
    if (off != (int)expected && off != (int)expected - 1)
    {
        printf("FAIL %s:%u: first preamble at %d, expected %u(+/-1)\n",
               __FILE__, line, off, expected);
        g_failures++;
    }
}
#define EXPECT_FIRST_PREAMBLE(s, expected) \
    expect_first_preamble((s), (expected), __LINE__)

/* Push 2 header bytes whitened for @p ch (for hand-crafted false
 * candidates whose header must survive the decoder's dewhitening). */
static void stream_whitened_header(ble_test_stream_t *s, uint8_t hdr0,
                                   uint8_t hdr1, uint8_t ch)
{
    uint8_t header[2] = {hdr0, hdr1};
    ble_dewhiten(header, sizeof(header), ch);
    bts_push_bytes(s, header, sizeof(header));
}

/* Append a false-candidate prefix: preamble 0x55 + all-zero AA + a whitened
 * header declaring len. Picks the smallest len >= min_len whose air bytes
 * introduce no preamble after the false preamble itself (including straddle
 * windows into following zeros). Returns 0 when none is clean. */
static unsigned int stream_false_prefix(ble_test_stream_t *s, uint8_t ch,
                                        unsigned int min_len)
{
    for (unsigned int len = min_len; len <= 255u; len++)
    {
        ble_test_stream_t trial;
        bts_reset(&trial);
        bts_push_preamble(&trial, 0x55u);
        bts_push_aa(&trial, 0u);
        uint8_t header[2] = {0x02u, (uint8_t)len};
        ble_dewhiten(header, sizeof(header), ch);
        bts_push_bytes(&trial, header, sizeof(header));
        stream_zeros(&trial, 8u);   /* cover straddle windows */
        if (first_preamble_offset(&trial, 1u) < 0)
        {
            bts_push_preamble(s, 0x55u);
            bts_push_aa(s, 0u);
            bts_push_bytes(s, header, sizeof(header));
            return len;
        }
    }
    return 0u;
}

/* ---------------------------------------------------------------------------
 * 1. Advertising detection regression.
 * ---------------------------------------------------------------------------*/
static void test_adv_regression(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_ADV, &store);

    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 0x02, 0x0A};
    ble_test_stream_t s;
    bts_reset(&s);
    stream_zeros(&s, 37u);          /* unaligned start, no false preambles */
    stream_adv_packet(&s, BLE_PDU_ADV_IND, payload, sizeof(payload), CH_ADV);
    /* The 0x55 preamble preceded by a 0 bit spawns an overlap candidate;
     * it must complete and reject before the rescan reaches the packet. */
    stream_zeros(&s, FLUSH_ZEROS);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        const ble_frame_t *f = &out.frames[0];
        TEST_ASSERT(f->kind == BLE_FRAME_ADVERTISING);
        TEST_ASSERT(f->access_address == BLE_ADVERTISING_AA);
        TEST_ASSERT(f->preamble == 0x55u);
        TEST_ASSERT(f->crc_init == BLE_CRC_INIT_ADV);
        TEST_ASSERT(f->raw_pdu_bytes == (2u + 8u + 3u));

        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(f, CH_ADV, &pkt) == 0);
        TEST_ASSERT(pkt.is_adv_pdu == 1u);
        TEST_ASSERT(pkt.crc_ok == 1u);
        TEST_ASSERT(pkt.pdu.adv.pdu_type == BLE_PDU_ADV_IND);
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 2. Data packet with a pre-confirmed store entry.
 * ---------------------------------------------------------------------------*/
static void test_data_confirmed(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);

    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[4] = {0xDEu, 0xADu, 0xBEu, 0xEFu};
    ble_test_stream_t s;
    bts_reset(&s);
    stream_zeros(&s, 19u);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    stream_zeros(&s, 23u);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        const ble_frame_t *f = &out.frames[0];
        TEST_ASSERT(f->kind == BLE_FRAME_DATA);
        TEST_ASSERT(f->access_address == AA_DATA);
        TEST_ASSERT(f->crc_init == INIT_DATA);
        TEST_ASSERT(f->crc_ok == 1u);
        TEST_ASSERT(f->raw_pdu_bytes == (2u + 4u + 3u));

        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(f, CH_DATA, &pkt) == 0);
        TEST_ASSERT(pkt.is_adv_pdu == 0u);
        TEST_ASSERT(pkt.pdu.data.llid == 0x02u);
        TEST_ASSERT(pkt.pdu.data.payload_len == 4u);
        TEST_ASSERT(memcmp(pkt.pdu.data.payload, payload, 4u) == 0);
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 3. Unknown AA, no candidates, len > 0: silently rejected.
 * ---------------------------------------------------------------------------*/
static void test_data_unknown_aa_silent(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[6] = {1, 2, 3, 4, 5, 6};
    ble_test_stream_t s;
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 0u);

    /* The store gates on promote-on-promise: a brand-new access address
     * must recur (BLE_PICONET_PROMOTE_THRESHOLD raw frames) before it
     * earns a slot, so a one-off unknown AA leaves no entry behind. */
    ble_piconet_t snap;
    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) != 0);

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 4. Empty packet -> candidate -> proof on non-empty packet -> confirmed ->
 *    subsequent empty packet accepted.
 * ---------------------------------------------------------------------------*/
static void test_recovery_lifecycle(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    ble_test_stream_t s;
    ble_test_out_t out;

    /* Phase 1: 0-length packets produce a candidate, nothing emitted. The
     * AA must first recur enough times to earn a store slot
     * (BLE_PICONET_PROMOTE_THRESHOLD); the first rounds only bump the
     * pending tally, the final one reverses the CRC into a candidate. */
    for (int round = 0; round < 3; round++)
    {
        bts_reset(&s);
        stream_data_packet(&s, AA_DATA, 0x01u, NULL, 0u, CH_DATA, INIT_DATA);
        stream_zeros(&s, FLUSH_ZEROS);
        bts_feed(&dec, &s, &out);
        TEST_ASSERT(out.count == 0u);
    }

    ble_piconet_t snap;
    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_PICONET_COLLECTING);
    TEST_ASSERT(snap.candidate_count == 1u);
    TEST_ASSERT(snap.candidates[0] == INIT_DATA);

    /* Phase 2: a non-empty packet proves the candidate -> confirmed, and
     * the proving packet itself is emitted. */
    const uint8_t payload[5] = {9, 8, 7, 6, 5};
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    stream_zeros(&s, FLUSH_ZEROS);
    bts_feed(&dec, &s, &out);
    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_DATA);
        TEST_ASSERT(out.frames[0].crc_init == INIT_DATA);
        TEST_ASSERT(out.frames[0].crc_ok == 1u);
    }

    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_PICONET_CONFIRMED);
    TEST_ASSERT(snap.crc_init == INIT_DATA);
    TEST_ASSERT(snap.candidate_count == 0u);

    /* Phase 3: now 0-length packets verify against the confirmed CRCInit. */
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x01u, NULL, 0u, CH_DATA, INIT_DATA);
    stream_zeros(&s, FLUSH_ZEROS);
    bts_feed(&dec, &s, &out);
    TEST_ASSERT(out.count == 1u);

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 5. CONNECT_IND seeding: valid seed proves on a data channel; corrupt seed
 *    is ignored. Two decoders share one store (as in the real session).
 * ---------------------------------------------------------------------------*/
static void push_connect_ind(ble_test_stream_t *s, uint8_t ch,
                             uint32_t conn_aa, uint32_t conn_crc_init)
{
    /* CONNECT_IND payload: InitA(6) + AdvA(6) + LLData(22) = 34 bytes. */
    uint8_t payload[34];
    memset(payload, 0, sizeof(payload));
    for (unsigned int i = 0; i < 6u; i++)
        payload[i] = (uint8_t)(0x11u * (i + 1u));
    for (unsigned int i = 0; i < 6u; i++)
        payload[6u + i] = (uint8_t)(0x22u * (i + 1u));
    uint8_t *ll = &payload[12];
    ll[0] = (uint8_t)(conn_aa & 0xFFu);
    ll[1] = (uint8_t)((conn_aa >> 8u) & 0xFFu);
    ll[2] = (uint8_t)((conn_aa >> 16u) & 0xFFu);
    ll[3] = (uint8_t)((conn_aa >> 24u) & 0xFFu);
    ll[4] = (uint8_t)(conn_crc_init & 0xFFu);
    ll[5] = (uint8_t)((conn_crc_init >> 8u) & 0xFFu);
    ll[6] = (uint8_t)((conn_crc_init >> 16u) & 0xFFu);
    ll[21] = 0x07u;   /* hop=7, sca=0 */

    stream_adv_packet(s, BLE_PDU_CONNECT_IND, payload, sizeof(payload), ch);
}

static void test_connect_ind_seeding(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec_adv, dec_data;
    ble_bitstream_decoder_init(&dec_adv, CH_ADV, &store);
    ble_bitstream_decoder_init(&dec_data, CH_DATA, &store);

    ble_test_stream_t s;
    ble_test_out_t out;

    /* Valid CONNECT_IND on the advertising channel: emitted as an adv
     * frame AND seeds a (still unproven) candidate. */
    bts_reset(&s);
    push_connect_ind(&s, CH_ADV, AA_DATA, INIT_DATA);
    bts_feed(&dec_adv, &s, &out);
    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_ADVERTISING);

    ble_piconet_t snap;
    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_PICONET_COLLECTING);
    TEST_ASSERT(snap.candidate_count == 1u);
    TEST_ASSERT(snap.candidates[0] == INIT_DATA);

    /* Data packet on a different decoder proves the seeded candidate. */
    const uint8_t payload[3] = {0x11u, 0x22u, 0x33u};
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    bts_feed(&dec_data, &s, &out);
    TEST_ASSERT(out.count == 1u);

    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_PICONET_CONFIRMED);
    TEST_ASSERT(snap.crc_init == INIT_DATA);

    ble_piconet_store_free(&store);
}

static void test_connect_ind_corrupt_no_seed(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec_adv, dec_data;
    ble_bitstream_decoder_init(&dec_adv, CH_ADV, &store);
    ble_bitstream_decoder_init(&dec_data, CH_DATA, &store);

    ble_test_stream_t s;
    ble_test_out_t out;

    /* Corrupt one bit inside the CONNECT_IND payload (air bit 120 is well
     * inside the whitened PDU region). */
    bts_reset(&s);
    push_connect_ind(&s, CH_ADV, AA_DATA, INIT_DATA);
    s.bytes[120u / 8u] ^= (uint8_t)(1u << (120u % 8u));
    bts_feed(&dec_adv, &s, &out);

    /* The adv frame is still emitted (advertising is not CRC-gated in the
     * framer), but its CRC fails so nothing is seeded. */
    TEST_ASSERT(out.count == 1u);
    ble_piconet_t snap;
    TEST_ASSERT(ble_piconet_store_find(&store, AA_DATA, &snap) != 0);

    /* The data packet now has no candidate to prove -> rejected. */
    const uint8_t payload[3] = {0x11u, 0x22u, 0x33u};
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    bts_feed(&dec_data, &s, &out);
    TEST_ASSERT(out.count == 0u);

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 6. Rescan: a false candidate swallows a real packet. The real packet must
 *    still be emitted (bits are only rejected up to the next preamble).
 * ---------------------------------------------------------------------------*/
static void test_rescan_swallowed_packet(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[8] = {1, 3, 5, 7, 9, 11, 13, 15};
    ble_test_stream_t real, s;
    bts_reset(&real);
    stream_data_packet(&real, AA_DATA, 0x02u, payload, sizeof(payload),
                       CH_DATA, INIT_DATA);
    TEST_ASSERT(real.count == 144u);

    /* False candidate: zero AA + header with a large clean length. Its
     * body starts with the real packet, zero-filled after. */
    bts_reset(&s);
    unsigned int len = stream_false_prefix(&s, CH_DATA, 20u);
    TEST_ASSERT(len >= 20u);
    unsigned int body_bits = (2u + len + BLE_CRC_BYTES) * 8u;
    for (unsigned int i = 0; i < real.count; i++)
        bts_push_bit(&s, (uint8_t)((real.bytes[i / 8u] >> (i % 8u)) & 1u));
    stream_zeros(&s, body_bits - 16u - real.count);

    /* Sanity: the first preamble after the false one is the real one. */
    EXPECT_FIRST_PREAMBLE(&s, 56u);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_DATA);
        TEST_ASSERT(out.frames[0].access_address == AA_DATA);
        TEST_ASSERT(out.frames[0].raw_pdu_bytes == (2u + 8u + 3u));
    }

    ble_piconet_store_free(&store);
}

/* Variant: the real packet straddles the end of the false candidate, so
 * collection must resume from backlog and finish with live bits. */
static void test_rescan_straddling_packet(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[8] = {1, 3, 5, 7, 9, 11, 13, 15};
    ble_test_stream_t real, s;
    bts_reset(&real);
    stream_data_packet(&real, AA_DATA, 0x02u, payload, sizeof(payload),
                       CH_DATA, INIT_DATA);

    /* False candidate of 40+232..312 bits; the real packet starts 200
     * bits into its body so it ends past the false candidate. */
    bts_reset(&s);
    unsigned int len = stream_false_prefix(&s, CH_DATA, 24u);
    TEST_ASSERT(len >= 24u && len <= 39u);
    stream_zeros(&s, 200u);
    for (unsigned int i = 0; i < real.count; i++)
        bts_push_bit(&s, (uint8_t)((real.bytes[i / 8u] >> (i % 8u)) & 1u));

    /* Sanity: the first preamble after the false one is the real one at
     * bit 56+200 = 256. */
    EXPECT_FIRST_PREAMBLE(&s, 256u);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_DATA);
        TEST_ASSERT(out.frames[0].access_address == AA_DATA);
        TEST_ASSERT(out.frames[0].raw_pdu_bytes == (2u + 8u + 3u));
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 7. Nested false preambles: the rescan must unwind through multiple false
 *    candidates before reaching the real packet.
 * ---------------------------------------------------------------------------*/
static void test_rescan_nested(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    /* Outer false candidate: zero AA + large clean length. Its body holds
     * an inner false candidate (zero AA, small clean length) after zeros. */
    ble_test_stream_t s;
    bts_reset(&s);
    unsigned int outer_len = stream_false_prefix(&s, CH_DATA, 60u);
    TEST_ASSERT(outer_len >= 60u);
    unsigned int outer_body = (2u + outer_len + BLE_CRC_BYTES) * 8u;
    stream_zeros(&s, 100u);

    unsigned int inner_start = s.count;
    unsigned int inner_len = stream_false_prefix(&s, CH_DATA, 0u);
    unsigned int inner_bits = 40u + (2u + inner_len + BLE_CRC_BYTES) * 8u;
    stream_zeros(&s, (2u + inner_len + BLE_CRC_BYTES) * 8u - 16u);

    stream_zeros(&s, outer_body - 16u - 100u - inner_bits);

    /* Sanity: the first preamble after the outer one is the inner one. */
    EXPECT_FIRST_PREAMBLE(&s, inner_start);

    /* Then the real packet arrives on live bits. */
    const uint8_t payload[4] = {0xCAu, 0xFEu, 0xBAu, 0xBEu};
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    stream_zeros(&s, FLUSH_ZEROS);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_DATA);
        TEST_ASSERT(out.frames[0].access_address == AA_DATA);
        TEST_ASSERT(out.frames[0].raw_pdu_bytes == (2u + 4u + 3u));
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 8. Preamble overlap at a 1-bit offset must not wedge the detector.
 * ---------------------------------------------------------------------------*/
static void test_preamble_overlap(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    ble_test_stream_t s;
    bts_reset(&s);
    /* 0xAA air bits are 0,1,0,1,0,1,0,1; appending a 0 bit forms a 0x55
     * preamble at bit offset 1. */
    bts_push_preamble(&s, 0xAAu);
    bts_push_bit(&s, 0u);
    stream_zeros(&s, 64u);

    const uint8_t payload[2] = {0x42u, 0x24u};
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    stream_zeros(&s, FLUSH_ZEROS);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
        TEST_ASSERT(out.frames[0].access_address == AA_DATA);

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 9. Maximum-length data packet (len=255).
 * ---------------------------------------------------------------------------*/
static void test_max_length(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    uint8_t payload[BLE_LL_PAYLOAD_MAX_BYTES];
    for (unsigned int i = 0; i < sizeof(payload); i++)
            payload[i] = (uint8_t)(i * 7u + 3u);

    ble_test_stream_t s;
    bts_reset(&s);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].raw_pdu_bytes ==
                    (2u + BLE_LL_PAYLOAD_MAX_BYTES + BLE_CRC_BYTES));
        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(&out.frames[0], CH_DATA, &pkt) == 0);
        TEST_ASSERT(pkt.pdu.data.payload_len == BLE_LL_PAYLOAD_MAX_BYTES);
        TEST_ASSERT(memcmp(pkt.pdu.data.payload, payload,
                           BLE_LL_PAYLOAD_MAX_BYTES) == 0);
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 10. A garbage (huge) length on an unconfirmed AA must not wedge the
 *     detector: the real packet inside it is still recovered.
 * ---------------------------------------------------------------------------*/
static void test_garbage_length_recovery(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_piconet_store_confirm(&store, AA_DATA, INIT_DATA);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[4] = {0x0Au, 0x0Bu, 0x0Cu, 0x0Du};
    ble_test_stream_t s;
    bts_reset(&s);
    /* Maximum garbage length (or the closest clean one below it). */
    unsigned int len = stream_false_prefix(&s, CH_DATA, 255u);
    TEST_ASSERT(len == 255u);
    unsigned int body_bits = (2u + len + BLE_CRC_BYTES) * 8u;
    stream_zeros(&s, 40u);
    stream_data_packet(&s, AA_DATA, 0x02u, payload, sizeof(payload), CH_DATA,
                       INIT_DATA);
    stream_zeros(&s, body_bits - 16u - 40u - 112u);

    /* Sanity: the first preamble after the false one is the real one. */
    EXPECT_FIRST_PREAMBLE(&s, 96u);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
        TEST_ASSERT(out.frames[0].access_address == AA_DATA);

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 12. Unified classification: the advertising AA takes the advertising path
 *     on ANY channel — advertising PDUs appearing on data channels stay
 *     visible (they indicate a genuine bug or a diagnosable misread).
 * ---------------------------------------------------------------------------*/
static void test_adv_aa_on_data_channel_adv_path(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 0x02, 0x0A};
    ble_test_stream_t s;
    bts_reset(&s);
    stream_adv_packet(&s, BLE_PDU_ADV_IND, payload, sizeof(payload), CH_DATA);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        const ble_frame_t *f = &out.frames[0];
        TEST_ASSERT(f->kind == BLE_FRAME_ADVERTISING);
        TEST_ASSERT(f->access_address == BLE_ADVERTISING_AA);
        TEST_ASSERT(f->crc_init == BLE_CRC_INIT_ADV);

        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(f, CH_DATA, &pkt) == 0);
        TEST_ASSERT(pkt.is_adv_pdu == 1u);
        TEST_ASSERT(pkt.crc_ok == 1u);
    }

    /* The advertising path touches no piconet state. */
    TEST_ASSERT(ble_piconet_store_count(&store) == 0u);

    ble_piconet_store_free(&store);
}

/* A rescan-spawned adv-AA candidate on a data channel surfaces as an
 * advertising frame (ungated path) — visibility over masking. */
static void test_rescan_adv_aa_visible(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_DATA, &store);

    ble_test_stream_t s;
    bts_reset(&s);
    /* Outer false candidate (zero AA, big clean length) whose body holds a
     * junk candidate (preamble + adv AA + garbage header). The junk 0x55
     * is preceded by a zero bit, so the rescan detours through the overlap
     * candidate first — generous trailing zeros let the chain resolve as
     * it would on a continuous stream. */
    unsigned int len = stream_false_prefix(&s, CH_DATA, 60u);
    TEST_ASSERT(len >= 60u);
    stream_zeros(&s, 64u);
    bts_push_preamble(&s, 0x55u);
    bts_push_aa(&s, BLE_ADVERTISING_AA);
    stream_whitened_header(&s, 0x0Eu, 30u, CH_DATA);
    stream_zeros(&s, (2u + 30u + BLE_CRC_BYTES) * 8u - 16u);
    stream_zeros(&s, FLUSH_ZEROS);

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    /* Exactly the junk frame, through the advertising path. */
    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        TEST_ASSERT(out.frames[0].kind == BLE_FRAME_ADVERTISING);
        TEST_ASSERT(out.frames[0].access_address == BLE_ADVERTISING_AA);

        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(&out.frames[0], CH_DATA, &pkt) == 0);
        TEST_ASSERT(pkt.crc_ok == 0u);   /* distinguishable as a misread */
    }

    ble_piconet_store_free(&store);
}

/* ---------------------------------------------------------------------------
 * 13. Advertising frames are NOT CRC-gated in the framer: a corrupted adv
 *     packet is still emitted (and the codec flags crc_ok=0).
 * ---------------------------------------------------------------------------*/
static void test_adv_crc_fail_still_emitted(void)
{
    ble_piconet_store_t store;
    ble_piconet_store_init(&store);
    ble_bitstream_decoder_t dec;
    ble_bitstream_decoder_init(&dec, CH_ADV, &store);

    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 0x02, 0x0A};
    ble_test_stream_t s;
    bts_reset(&s);
    stream_adv_packet(&s, BLE_PDU_ADV_IND, payload, sizeof(payload), CH_ADV);
    /* Flip a bit inside the whitened PDU region. */
    s.bytes[100u / 8u] ^= (uint8_t)(1u << (100u % 8u));

    ble_test_out_t out;
    bts_feed(&dec, &s, &out);

    TEST_ASSERT(out.count == 1u);
    if (out.count == 1u)
    {
        ble_packet_t pkt;
        TEST_ASSERT(ble_decode_frame(&out.frames[0], CH_ADV, &pkt) == 0);
        TEST_ASSERT(pkt.crc_ok == 0u);
    }

    ble_piconet_store_free(&store);
}

int main(void)
{
    test_adv_regression();
    test_data_confirmed();
    test_data_unknown_aa_silent();
    test_recovery_lifecycle();
    test_connect_ind_seeding();
    test_connect_ind_corrupt_no_seed();
    test_rescan_swallowed_packet();
    test_rescan_straddling_packet();
    test_rescan_nested();
    test_preamble_overlap();
    test_max_length();
    test_garbage_length_recovery();
    test_adv_aa_on_data_channel_adv_path();
    test_rescan_adv_aa_visible();
    test_adv_crc_fail_still_emitted();

    if (g_failures)
    {
        printf("test_ble_bitstream_decoder: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ble_bitstream_decoder: OK\n");
    return 0;
}
