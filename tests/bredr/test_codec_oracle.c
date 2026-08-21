/**
 * @file test_codec_oracle.c
 * @brief Primitive-level parity between bredr_codec and libbtbb reference.
 *
 * Compares the supertooth codec primitives against libbtbb's reference
 * implementations (verbatim copies in libbtbb_oracle.c):
 *
 *   - uap_from_hec  vs bredr_decode_uap_from_hec        (exhaustive)
 *   - crcgen        vs bredr_payload_crc                (randomized)
 *   - unfec13       vs bredr_fec_decode_1_3 + header decode (randomized)
 *   - unfec23       vs bredr_fec_decode_2_3             (randomized)
 *   - unwhiten      vs bredr_dewhiten_air_payload_bytes (all clocks)
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"
#include "libbtbb_oracle.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                            \
    do                                                                               \
    {                                                                                \
        if (!(cond))                                                                 \
        {                                                                            \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                                            \
        }                                                                            \
    } while (0)

static uint32_t g_rng = 0xC0FFEEu;
static uint32_t rng_next(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return g_rng >> 16;
}

static void air_to_packed(const char *air, uint8_t *packed, int bits)
{
    memset(packed, 0, (unsigned int)((bits + 7) / 8));
    for (int i = 0; i < bits; i++)
        if (air[i] & 1)
            packed[i / 8] |= (uint8_t)(1u << (i % 8));
}

static void packed_to_air(const uint8_t *packed, char *air, int bits)
{
    for (int i = 0; i < bits; i++)
        air[i] = (char)((packed[i / 8] >> (i % 8)) & 1u);
}

/* ------------------------------------------------------------------------ */

static void test_uap_from_hec_exhaustive(void)
{
    for (unsigned int data = 0; data < 1024; data++)
        for (unsigned int hec = 0; hec < 256; hec++)
        {
            uint8_t expect = oracle_uap_from_hec((uint16_t)data, (uint8_t)hec);
            uint8_t got = bredr_decode_uap_from_hec((uint16_t)data, (uint8_t)hec);
            TEST_ASSERT(expect == got);
            if (expect != got)
                return;
        }
    printf("  uap_from_hec: exhaustive 1024x256 parity OK\n");
}

static void test_payload_crc_random(void)
{
    enum { MAX_BITS = 2048 };
    uint8_t packed[MAX_BITS / 8];
    char air[MAX_BITS];

    for (int iter = 0; iter < 500; iter++)
    {
        int bits = (int)(rng_next() % (MAX_BITS - 8)) + 8;
        for (int i = 0; i < bits; i++)
            air[i] = (char)(rng_next() & 1u);
        uint8_t uap = (uint8_t)(rng_next() & 0xFFu);

        air_to_packed(air, packed, bits);

        uint16_t expect = oracle_crcgen(air, bits, uap);
        uint16_t got = bredr_payload_crc(packed, (unsigned int)bits, uap);
        TEST_ASSERT(expect == got);
        if (expect != got)
            return;
    }
    printf("  payload_crc: 500 randomized vectors OK\n");
}

/* Header-sized (18 bit) FEC 1/3 parity: decoded bits and the libbtbb
 * fail/success decision (unfec13: fail iff non-unanimous blocks >= 18/4). */
static void test_fec_1_3_header_parity(void)
{
    char air[54];
    char oracle_out[18];

    for (int iter = 0; iter < 2000; iter++)
    {
        for (int i = 0; i < 18; i++)
        {
            char b = (char)(rng_next() & 1u);
            air[3 * i] = b;
            air[3 * i + 1] = b;
            air[3 * i + 2] = b;
        }

        /* Inject 0..6 corrupted triples, each with 1..3 flipped symbols. */
        int corrupt = (int)(rng_next() % 7u);
        for (int c = 0; c < corrupt; c++)
        {
            int block = (int)(rng_next() % 18u);
            int flips = (int)(rng_next() % 3u) + 1;
            for (int f = 0; f < flips; f++)
                air[3 * block + f] ^= 1;
        }

        int oracle_ok = oracle_unfec13(air, oracle_out, 18);

        uint8_t air_packed[7];
        memset(air_packed, 0, sizeof(air_packed));
        for (int i = 0; i < 54; i++)
            if (air[i] & 1)
                air_packed[i / 8] |= (uint8_t)(1u << (i % 8));

        uint8_t native_decoded[18];
        unsigned int db = 0u;
        int native_be = bredr_fec_decode_1_3(air_packed, 54u, native_decoded, &db);
        int native_ok = (native_be >= 0 && db == 18u && native_be < 4);

        if (oracle_ok)
        {
            /* libbtbb decodes (raw, pre-dewhiten): native must match bit-for-bit. */
            TEST_ASSERT(native_ok);
            if (!native_ok)
                return;
            for (int i = 0; i < 18; i++)
            {
                uint8_t nb = (uint8_t)((native_decoded[i / 8] >> (i % 8)) & 1u);
                TEST_ASSERT(nb == (uint8_t)(oracle_out[i] & 1));
                if (nb != (uint8_t)(oracle_out[i] & 1))
                    return;
            }
        }
        else
        {
            /* libbtbb unfec13 fails; native must also report failure. */
            TEST_ASSERT(!native_ok);
            if (native_ok)
                return;
        }
    }
    printf("  fec_1_3 header: 2000 randomized vectors OK\n");
}

static void test_fec_2_3_parity(void)
{
    enum { MAX_BLOCKS = 40 };
    char air[MAX_BLOCKS * 15];
    uint8_t packed_in[(MAX_BLOCKS * 15 + 7) / 8];
    uint8_t packed_out[(MAX_BLOCKS * 10 + 7) / 8];

    for (int iter = 0; iter < 1000; iter++)
    {
        int blocks = (int)(rng_next() % MAX_BLOCKS) + 1;
        int data_bits = blocks * 10;

        for (int i = 0; i < data_bits; i++)
            air[i] = (char)(rng_next() & 1u);

        /* Encode with the libbtbb reference encoder (fec23 via oracle). */
        char *encoded = oracle_fec23_encode(air, data_bits);
        TEST_ASSERT(encoded != NULL);
        if (!encoded)
            return;

        /* Optionally inject one single-bit error into one codeword. */
        int inject = (int)(rng_next() % 3u);
        if (inject > 0)
        {
            int block = (int)(rng_next() % (unsigned int)blocks);
            int pos = (int)(rng_next() % 15u);
            encoded[block * 15 + pos] ^= 1;
        }

        char *oracle_out = oracle_unfec23(encoded, data_bits);

        air_to_packed(encoded, packed_in, blocks * 15);
        unsigned int out_bits = 0;
        int rc = bredr_fec_decode_2_3(packed_in, (unsigned int)(blocks * 15),
                                      packed_out, &out_bits);

        if (oracle_out)
        {
            TEST_ASSERT(rc >= 0);
            TEST_ASSERT(out_bits == (unsigned int)data_bits);
            if (rc >= 0)
            {
                char native_air[MAX_BLOCKS * 10];
                packed_to_air(packed_out, native_air, data_bits);
                for (int i = 0; i < data_bits; i++)
                {
                    TEST_ASSERT(native_air[i] == (uint8_t)(oracle_out[i] & 1));
                    if (native_air[i] != (uint8_t)(oracle_out[i] & 1))
                    {
                        free(oracle_out);
                        free(encoded);
                        return;
                    }
                }
            }
            free(oracle_out);
        }
        else
        {
            TEST_ASSERT(rc < 0);
            if (rc >= 0)
            {
                free(encoded);
                return;
            }
        }
        free(encoded);
    }
    printf("  fec_2_3: 1000 randomized codeword streams OK\n");
}

static void test_dewhiten_parity(void)
{
    enum { BITS = 300 };
    char air[BITS];
    char oracle_out[BITS];
    uint8_t packed_in[(BITS + 7) / 8];
    uint8_t packed_out[(BITS + 7) / 8];

    for (int i = 0; i < BITS; i++)
        air[i] = (char)(rng_next() & 1u);
    air_to_packed(air, packed_in, BITS);

    for (unsigned int clk6 = 0; clk6 < 64; clk6++)
    {
        unsigned int skips[2] = {0u, 18u};
        for (unsigned int s = 0; s < 2; s++)
        {
            oracle_unwhiten(air, oracle_out, (int)clk6, BITS, (int)skips[s]);

            memset(packed_out, 0, sizeof(packed_out));
            bredr_dewhiten_air_payload_bytes(packed_in, BITS, (uint8_t)clk6,
                                             skips[s], packed_out,
                                             sizeof(packed_out));

            char native_air[BITS];
            packed_to_air(packed_out, native_air, BITS);
            for (int i = 0; i < BITS; i++)
            {
                TEST_ASSERT(native_air[i] == (uint8_t)(oracle_out[i] & 1));
                if (native_air[i] != (uint8_t)(oracle_out[i] & 1))
                    return;
            }
        }
    }
    printf("  dewhiten: all 64 clocks x skip{0,18} OK\n");
}

int main(void)
{
    test_uap_from_hec_exhaustive();
    test_payload_crc_random();
    test_fec_1_3_header_parity();
    test_fec_2_3_parity();
    test_dewhiten_parity();

    if (g_failures != 0)
    {
        fprintf(stderr, "test_codec_oracle: %d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("test_codec_oracle: all libbtbb primitive parity checks passed\n");
    return 0;
}
