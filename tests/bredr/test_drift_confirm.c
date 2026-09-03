/**
 * @file test_drift_confirm.c
 * @brief Unit tests for two-packet drift confirmation in clock tracking.
 *
 * A locked piconet (uap_valid && clk_known) is driven with a real captured
 * header (tests/bredr/capture_151FC475.h, UAP 0x15) via bredr_recovery_process().
 * The SAME frame is replayed at shifted radio_start_sample_index values: the
 * recovery logic cannot tell a receiver-clock shift from central-clock drift,
 * so shifting stimulates the drift paths deterministically.
 *
 * Clock model: at 3200 Hz sample rate, radio_start_sample_index == clkn and
 * rx_clk_1600 == clkn/2; the capture is whitened with CLK1-6=(clkn>>1)&63.
 * Feeding the frame at start=clkn0+2*s moves the tried base clock by s while
 * the true key stays K0, so shift s=+1 first matches at delta -1, s=-1 at
 * delta +1, and s=+3 matches nothing (total miss).
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_codec.h"
#include "bredr_clock_recovery.h"
#include "receive_event_models.h"
#include "capture_151FC475.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                             \
    do                                                                                \
    {                                                                                 \
        if (!(cond))                                                                  \
        {                                                                             \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                                             \
        }                                                                             \
    } while (0)

typedef struct
{
    bredr_frame_t frame;
    uint32_t clkn0;   /* clkn the frame was captured at */
    uint8_t k0;       /* true whitening CLK1-6 */
} clean_frame_t;

/* Find the first capture entry whose header FEC-decodes cleanly (be==0) so
 * the drift path's strict HEC gate can pass at exactly the true clock. */
static int find_clean_frame(clean_frame_t *out)
{
    for (int i = 0; i < cap_151FC475_n; i++)
    {
        const cap_pkt_t *p = &cap_151FC475[i];
        if (p->air_payload_bits == 0u && i < cap_151FC475_n - 1)
            continue; /* prefer entries that look like full packets */
        bredr_frame_t f;
        memset(&f, 0, sizeof(f));
        f.has_header = 1u;
        f.header_raw = p->header_raw;
        uint8_t true_clk = (uint8_t)((p->clkn >> 1) & 0x3fu);
        if (bredr_hec_ok_for_clk6_clean(&f, CAP_TRUE_UAP, true_clk))
        {
            out->frame = f;
            out->clkn0 = p->clkn;
            out->k0 = true_clk;
            return 0;
        }
    }
    /* Fall back to any entry (even header-only) that gates clean. */
    for (int i = 0; i < cap_151FC475_n; i++)
    {
        const cap_pkt_t *p = &cap_151FC475[i];
        bredr_frame_t f;
        memset(&f, 0, sizeof(f));
        f.has_header = 1u;
        f.header_raw = p->header_raw;
        uint8_t true_clk = (uint8_t)((p->clkn >> 1) & 0x3fu);
        if (bredr_hec_ok_for_clk6_clean(&f, CAP_TRUE_UAP, true_clk))
        {
            out->frame = f;
            out->clkn0 = p->clkn;
            out->k0 = true_clk;
            return 0;
        }
    }
    return -1;
}

/* Feed the frame at receiver shift s (rx moves by s slots vs capture). */
static int feed_shifted(bredr_piconet_t *pnet, const clean_frame_t *cf, int s)
{
    bredr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.meta.radio_sample_rate_hz = 3200u;
    ev.meta.radio_start_sample_index = cf->clkn0 + (uint64_t)(2 * s);
    ev.frame = cf->frame;
    return bredr_recovery_process(pnet, &ev);
}

static bredr_piconet_t *locked_pnet(const clean_frame_t *cf)
{
    bredr_piconet_t *pnet = (bredr_piconet_t *)malloc(sizeof(*pnet));
    if (!pnet)
    {
        g_failures++;
        return NULL;
    }
    bredr_piconet_init(pnet, CAP_LAP);
    uint32_t rx0 = cf->clkn0 >> 1;
    bredr_piconet_set_uap(pnet, CAP_TRUE_UAP, cf->k0, rx0);
    return pnet;
}

int main(void)
{
    clean_frame_t cf;
    TEST_ASSERT(find_clean_frame(&cf) == 0);
    if (g_failures)
        return 1;
    printf("using capture entry clkn=%u k0=%u\n", cf.clkn0, cf.k0);

    /* 1. Base hits build confidence, offset untouched. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1);
        TEST_ASSERT(p->tracking_state == 2);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1);
        TEST_ASSERT(p->tracking_state == 3);
        TEST_ASSERT(p->clock_offset == 0);
        TEST_ASSERT(p->drift_candidate == 0);
        free(p);
    }

    /* 2+3. Single suspect parks the candidate; repeat confirms the drift. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 2 */
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 3 */
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        TEST_ASSERT(p->clock_offset == 0);   /* held */
        TEST_ASSERT(p->tracking_state == 3); /* held */
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->clock_offset == 63); /* -1 mod 64 applied */
        TEST_ASSERT(p->drift_candidate == 0);
        TEST_ASSERT(p->tracking_state == 4); /* confirm builds confidence */
        free(p);
    }

    /* 4. A base hit clears a pending suspicion (glitch, not drift). */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 2 */
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1);
        TEST_ASSERT(p->drift_candidate == 0);
        TEST_ASSERT(p->clock_offset == 0);
        TEST_ASSERT(p->tracking_state == 3);
        free(p);
    }

    /* 5. Conflicting deltas clear the suspicion without touching tracking. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 2 */
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        TEST_ASSERT(feed_shifted(p, &cf, -1) == 1); /* suspects +1 instead */
        TEST_ASSERT(p->drift_candidate == 0);
        TEST_ASSERT(p->clock_offset == 0);
        TEST_ASSERT(p->tracking_state == 2);
        TEST_ASSERT(p->clk_known == 1);
        free(p);
    }

    /* 6. A packet matching nothing clears suspicion and decays confidence. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        TEST_ASSERT(feed_shifted(p, &cf, +3) == 0);
        TEST_ASSERT(p->drift_candidate == 0);
        TEST_ASSERT(p->tracking_state == 0);
        TEST_ASSERT(p->clk_known == 0); /* lock lost -> reacquisition next */
        free(p);
    }

    /* 7. A lone glitch no longer moves the offset (old code applied +1/-1
     * immediately); the following base hit keeps the lock healthy. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 2 */
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1); /* state 3 */
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->clock_offset == 0);
        TEST_ASSERT(feed_shifted(p, &cf, 0) == 1);
        TEST_ASSERT(p->clock_offset == 0);
        TEST_ASSERT(p->tracking_state == 4);
        free(p);
    }

    /* 8. Lifecycle clears the pending candidate. */
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        bredr_recovery_reset(p);
        TEST_ASSERT(p->drift_candidate == 0);
        free(p);
    }
    {
        bredr_piconet_t *p = locked_pnet(&cf);
        TEST_ASSERT(feed_shifted(p, &cf, +1) == 1);
        TEST_ASSERT(p->drift_candidate == -1);
        bredr_piconet_set_uap(p, CAP_TRUE_UAP, cf.k0, cf.clkn0 >> 1);
        TEST_ASSERT(p->drift_candidate == 0);
        TEST_ASSERT(p->tracking_state == 1);
        free(p);
    }

    if (g_failures != 0)
    {
        fprintf(stderr, "test_drift_confirm: %d assertion(s) failed\n", g_failures);
        return 1;
    }
    printf("test_drift_confirm: two-packet drift confirmation behaves as specified\n");
    return 0;
}
