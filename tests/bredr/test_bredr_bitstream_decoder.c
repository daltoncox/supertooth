/**
 * @file test_bredr_bitstream_decoder.c
 * @brief Coverage for the BR/EDR PHY bitstream decoder (previously untested --
 *        all recovery tests inject header_raw directly and bypass this layer).
 *
 * Covers: NULL guards, gen_syncword determinism, GIAC short-frame path,
 * bad-trailer short-frame path, full access-code + max-body collection,
 * and the global max_ac_errors acceptance gate.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__,  \
                    #cond);                                                   \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define TEST_LAP 0x1FC475u

/* Push `nbits` of `word` LSB-first, returning the last status seen. */
static bredr_status_t push_word(bredr_bitstream_decoder_t *proc,
                                uint64_t word, unsigned int nbits)
{
    bredr_status_t s = BREDR_SEARCHING;
    for (unsigned int i = 0u; i < nbits; i++)
        s = bredr_bitstream_decoder_push_bit(proc,
                                             (uint8_t)((word >> i) & 1u));
    return s;
}

/* Push the 4-bit preamble (decoder does not validate it) + 64-bit syncword. */
static bredr_status_t push_access_code(bredr_bitstream_decoder_t *proc,
                                       uint32_t lap)
{
    bredr_status_t s;
    s = push_word(proc, 0xAu, 4u); /* preamble 1010 */
    (void)s;
    return push_word(proc, bredr_gen_syncword(lap), 64u);
}

static void test_null_guards(void)
{
    bredr_bitstream_decoder_t proc;
    bredr_frame_t frame;

    bredr_bitstream_decoder_init(NULL); /* must not crash */
    TEST_ASSERT(bredr_bitstream_decoder_push_bit(NULL, 1u) == BREDR_ERROR);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(NULL, &frame) == -1);

    bredr_bitstream_decoder_init(&proc);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, NULL) == -1);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, &frame) == -1);
}

static void test_syncword_determinism(void)
{
    uint64_t a = bredr_gen_syncword(TEST_LAP);
    uint64_t b = bredr_gen_syncword(TEST_LAP);
    uint64_t c = bredr_gen_syncword(0x123456u);
    TEST_ASSERT(a == b);
    TEST_ASSERT(a != c);
    TEST_ASSERT(a != 0u);
}

static void test_giac_short_frame(void)
{
    bredr_bitstream_decoder_t proc;
    bredr_frame_t frame;

    bredr_bitstream_decoder_init(&proc);
    bredr_status_t s = push_access_code(&proc, BREDR_LAP_GIAC);
    TEST_ASSERT(s == BREDR_VALID_PACKET);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, &frame) == 0);
    TEST_ASSERT(frame.lap == BREDR_LAP_GIAC);
    TEST_ASSERT(frame.has_header == 0u);
    TEST_ASSERT(frame.air_payload_bits == 0u);
    /* Buffer is consumed: second get must fail. */
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, &frame) == -1);
}

static void test_bad_trailer_short_frame(void)
{
    bredr_bitstream_decoder_t proc;
    bredr_frame_t frame;
    uint64_t sw = bredr_gen_syncword(TEST_LAP);
    uint8_t sw_last = (uint8_t)((sw >> 63) & 1u);
    uint8_t good_trailer = sw_last ? 0xAu : 0x5u;
    uint8_t bad_trailer = (uint8_t)(good_trailer ^ 0xFu);

    bredr_bitstream_decoder_init(&proc);
    TEST_ASSERT(push_access_code(&proc, TEST_LAP) == BREDR_COLLECTING);
    TEST_ASSERT(push_word(&proc, bad_trailer, 4u) == BREDR_VALID_PACKET);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, &frame) == 0);
    TEST_ASSERT(frame.lap == TEST_LAP);
    TEST_ASSERT(frame.has_header == 0u);
}

static void test_full_packet_collection(void)
{
    bredr_bitstream_decoder_t proc;
    bredr_frame_t frame;
    uint64_t sw = bredr_gen_syncword(TEST_LAP);
    uint8_t sw_last = (uint8_t)((sw >> 63) & 1u);
    uint8_t trailer = sw_last ? 0xAu : 0x5u;
    uint64_t header_raw = 0x2AA555u & 0x3FFFFFu; /* 54-bit pattern */
    bredr_status_t s;

    bredr_bitstream_decoder_init(&proc);
    TEST_ASSERT(push_access_code(&proc, TEST_LAP) == BREDR_COLLECTING);
    TEST_ASSERT(push_word(&proc, trailer, 4u) == BREDR_COLLECTING);
    s = push_word(&proc, header_raw, 54u);
    TEST_ASSERT(s == BREDR_COLLECTING);
    /* Fixed max-body ceiling: exactly BR_MAX_AIR_PAYLOAD_BITS more bits. */
    for (unsigned int i = 0u; i < BR_MAX_AIR_PAYLOAD_BITS; i++)
    {
        s = bredr_bitstream_decoder_push_bit(&proc, (uint8_t)(i & 1u));
        if (i + 1u < BR_MAX_AIR_PAYLOAD_BITS)
            TEST_ASSERT(s == BREDR_COLLECTING);
    }
    TEST_ASSERT(s == BREDR_VALID_PACKET);
    TEST_ASSERT(bredr_bitstream_decoder_get_frame(&proc, &frame) == 0);
    TEST_ASSERT(frame.lap == TEST_LAP);
    TEST_ASSERT(frame.has_header == 1u);
    TEST_ASSERT(frame.header_raw == (header_raw & 0x3FFFFFFFFFFFFFu));
    TEST_ASSERT(frame.air_payload_bits == BR_MAX_AIR_PAYLOAD_BITS);
    /* Payload is packed LSB-first: bit i == (i & 1). */
    TEST_ASSERT((frame.air_payload[0] & 1u) == 0u);
    TEST_ASSERT(((frame.air_payload[0] >> 1) & 1u) == 1u);
}

static void test_ac_error_gate(void)
{
    bredr_bitstream_decoder_t proc;
    bredr_frame_t frame;
    uint64_t sw = bredr_gen_syncword(TEST_LAP);
    uint64_t corrupt = sw ^ 0x01u; /* 1-bit error outside the barker field */
    uint8_t saved = bredr_bitstream_decoder_get_global_max_ac_errors();

    /* Strict (0): single-bit error must not produce a packet. */
    bredr_bitstream_decoder_set_global_max_ac_errors(0u);
    bredr_bitstream_decoder_init(&proc);
    push_word(&proc, 0xAu, 4u);
    TEST_ASSERT(push_word(&proc, corrupt, 64u) == BREDR_SEARCHING);

    /* Tolerant (2): same stream must now enter collection. */
    bredr_bitstream_decoder_set_global_max_ac_errors(2u);
    bredr_bitstream_decoder_init(&proc);
    push_word(&proc, 0xAu, 4u);
    TEST_ASSERT(push_word(&proc, corrupt, 64u) == BREDR_COLLECTING);

    bredr_bitstream_decoder_set_global_max_ac_errors(saved);
    TEST_ASSERT(bredr_bitstream_decoder_get_global_max_ac_errors() == saved);
    (void)frame;
}

int main(void)
{
    /* Guard: this module's tolerance is process-wide; start strict. */
    bredr_bitstream_decoder_set_global_max_ac_errors(0u);

    test_null_guards();
    test_syncword_determinism();
    test_giac_short_frame();
    test_bad_trailer_short_frame();
    test_full_packet_collection();
    test_ac_error_gate();

    bredr_bitstream_decoder_set_global_max_ac_errors(0u);

    if (g_failures != 0)
    {
        fprintf(stderr, "test_bredr_bitstream_decoder: %d assertion(s) failed\n",
                g_failures);
        return 1;
    }
    printf("test_bredr_bitstream_decoder: all checks passed\n");
    return 0;
}
