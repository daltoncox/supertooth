/**
 * @file test_recovery.c
 * @brief Regression test for the native BR/EDR UAP / CLK1-6 recovery backend.
 *
 * Replays golden frame sequences whose on-air bytes were generated
 * OFFLINE by build/golden_gen.c using libbtbb's reference primitives
 * (uap_from_hec, crcgen, WHITENING_DATA).  The frame bytes are therefore
 * independent of the supertooth codec under test -- this is NOT a circular
 * test.  Each sequence ends with a DH1 payload packet carrying a valid CRC,
 * which provides the definitive UAP confirmation (mirroring libbtbb's
 * crc_check).  The test asserts that the backend recovers the expected UAP.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"
#include "bredr_clock_recovery.h"
#include "golden_vectors.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                                             \
    do                                                                                                \
    {                                                                                                 \
        if (!(cond))                                                                                  \
        {                                                                                             \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                  \
            g_failures++;                                                                             \
        }                                                                                             \
    } while (0)

static void run_case(const golden_case_t *c)
{
    bredr_piconet_t *pnet = malloc(sizeof(*pnet));
    if (!pnet)
    {
        g_failures++;
        return;
    }
    bredr_piconet_init(pnet, 0x123456u);

    int recovered = 0;
    uint8_t got_uap = 0u;
    uint8_t got_clk6 = 0u;
    int converged_at = -1;

    for (int k = 0; k < c->n; k++)
    {
        const golden_packet_t *p = &c->pkts[k];

        bredr_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.has_header = 1u;
        frame.header_raw = p->header_raw;
        frame.air_payload_bits = p->air_payload_bits;
        unsigned int nb = (p->air_payload_bits + 7u) / 8u;
        if (nb > sizeof(frame.air_payload))
            nb = sizeof(frame.air_payload);
        memcpy(frame.air_payload, p->air_payload, nb);

        /* Feed packet k at its generated clkn.  The golden vectors use the
         * production clock model: clkn advances 2 ticks (one slot) per
         * packet and frames are whitened with CLK1-6 = (clkn >> 1) & 0x3f,
         * exactly as the recovery module is fed in production. */
        bredr_recovery_result_t r;
        int rc = bredr_recovery_process_packet(pnet, &frame, 0, p->clkn, &r);
        if (rc)
        {
            recovered = 1;
            got_uap = r.uap;
            got_clk6 = r.clk6_hint;
            converged_at = k;
        }
    }

    free(pnet);

    TEST_ASSERT(recovered);
    if (!recovered)
        fprintf(stderr, "  [uap=0x%02X] NEVER recovered UAP after %d packets\n", c->uap, c->n);
    else
    {
        TEST_ASSERT(got_uap == c->uap);
        if (got_uap != c->uap)
            fprintf(stderr, "  [uap=0x%02X] WRONG UAP: got 0x%02X (converged pkt %d)\n",
                    c->uap, got_uap, converged_at);
        else
            printf("  [uap=0x%02X] recovered (clk6_hint=%u) via %d packets\n",
                   got_uap, got_clk6, converged_at + 1);
    }

}

int main(void)
{
    size_t n = sizeof(golden_cases) / sizeof(golden_cases[0]);
    for (size_t i = 0; i < n; i++)
        if (golden_cases[i].n > 0)
            run_case(&golden_cases[i]);

    if (g_failures != 0)
    {
        fprintf(stderr, "test_recovery: %d assertion(s) failed\n", g_failures);
        return 1;
    }

    printf("test_recovery: native UAP recovery converges to expected UAP across %zu golden cases\n", n);
    return 0;
}
