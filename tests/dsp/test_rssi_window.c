/* RSSI measurement tests (no RF hardware required).
 *
 * Regression cover for the polyphase-channelizer RSSI work:
 *
 *  - receiver_rssi_packet_window: the decoder reports packet starts as a BIT
 *    index while the demodulated buffer is indexed in SAMPLES.  Scaling that
 *    offset the wrong way makes the averaging window grow with the packet's
 *    position inside the block, so identical packets report wildly different
 *    RSSI (measured at ~16.5 dB of spread on a production-sized block).
 *
 *  - channelizer_bank gain uniformity: reported RSSI must not depend on which
 *    bin a packet landed in, nor on the capture span, because BR/EDR hops
 *    across every channel in the window.
 */
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "channelizer_bank.h"
#include "rssi_measurements.h"

static int g_failures = 0;

#define CHECK(cond, fmt, ...)                                                 \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d " fmt "\n", __FILE__, __LINE__, __VA_ARGS__);  \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

/* Production block geometry: channelizer_thread emits CAP/M - 2 frames. */
#define BLOCK_SAMPLES 13105u
#define SPS           2u
#define PKT_BITS      68u   /* preamble + 64-bit sync word */

/* --------------------------------------------------------------------------
 * The window itself
 * -------------------------------------------------------------------------- */
static void test_window_indices(void)
{
    unsigned int start = 0u, end = 0u;

    /* Packet starting at bit 600 of this block occupies samples 1200.. */
    receiver_rssi_packet_window(0u, 600u, 1336u - SPS, SPS, BLOCK_SAMPLES,
                                &start, &end);
    CHECK(start == 600u * SPS - RECEIVER_RSSI_PRETRIGGER_SAMPLES,
          "start %u, expected %u", start,
          600u * SPS - RECEIVER_RSSI_PRETRIGGER_SAMPLES);
    CHECK(end == 1336u, "end %u, expected 1336", end);

    /* Block-relative, not absolute: same packet, block starting at bit 1000. */
    receiver_rssi_packet_window(1000u, 1600u, 1336u - SPS, SPS, BLOCK_SAMPLES,
                                &start, &end);
    CHECK(start == 600u * SPS - RECEIVER_RSSI_PRETRIGGER_SAMPLES,
          "block-relative start %u, expected %u", start,
          600u * SPS - RECEIVER_RSSI_PRETRIGGER_SAMPLES);

    /* Packet that began in an earlier block: average what is present. */
    receiver_rssi_packet_window(1000u, 900u, 200u, SPS, BLOCK_SAMPLES,
                                &start, &end);
    CHECK(start == 0u, "straddling start %u, expected 0", start);
    CHECK(end == 202u, "straddling end %u, expected 202", end);

    /* A start at or past the end must not produce an inverted window. */
    receiver_rssi_packet_window(0u, 9000u, 200u, SPS, BLOCK_SAMPLES,
                                &start, &end);
    CHECK(start < end, "degenerate window [%u,%u)", start, end);

    /* end is clamped to what the buffer actually holds. */
    receiver_rssi_packet_window(0u, 10u, BLOCK_SAMPLES + 500u, SPS,
                                BLOCK_SAMPLES, &start, &end);
    CHECK(end == BLOCK_SAMPLES, "clamped end %u, expected %u", end,
          BLOCK_SAMPLES);
}

/* --------------------------------------------------------------------------
 * The property that matters: identical packets report identical RSSI
 * regardless of where in the block they land.
 * -------------------------------------------------------------------------- */
static void test_rssi_independent_of_block_position(void)
{
    static const unsigned int starts[] = {50u, 250u, 600u, 1200u, 2400u,
                                          3600u, 4800u, 6000u, 6400u};
    const size_t nstarts = sizeof(starts) / sizeof(starts[0]);

    float complex *buf = malloc(BLOCK_SAMPLES * sizeof(float complex));
    if (!buf)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        return;
    }

    double lo = 1e300, hi = -1e300;

    for (size_t i = 0u; i < nstarts; i++)
    {
        unsigned int b0   = starts[i];
        unsigned int s_lo = b0 * SPS;
        unsigned int s_hi = (b0 + PKT_BITS) * SPS;

        /* Unit-amplitude packet, -40 dB elsewhere. */
        for (unsigned int k = 0u; k < BLOCK_SAMPLES; k++)
            buf[k] = (k >= s_lo && k < s_hi) ? 1.0f : 0.01f;

        unsigned int start = 0u, end = 0u;
        receiver_rssi_packet_window(0u, b0, s_hi - SPS, SPS, BLOCK_SAMPLES,
                                    &start, &end);

        float rssi = receiver_rssi_from_mean_power_range(buf, start, end,
                                                         RECEIVER_RSSI_INVALID);
        CHECK(!isnan(rssi), "packet at bit %u produced no RSSI", b0);
        if (rssi < lo) lo = rssi;
        if (rssi > hi) hi = rssi;
    }

    CHECK(hi - lo < 0.5, "RSSI spread %.2f dB across block positions "
          "(expected < 0.5)", hi - lo);

    free(buf);
}

/* --------------------------------------------------------------------------
 * Bank gain must not depend on bin or span
 * -------------------------------------------------------------------------- */
static void make_tone(float complex *buf, size_t n, double freq_hz, double fs)
{
    double w = 2.0 * M_PI * freq_hz / fs;
    for (size_t i = 0u; i < n; i++)
        buf[i] = cexp(I * w * (double)i);
}

static double bin_power(channelizer_bank_t *q, const float complex *in,
                        size_t n, unsigned int bin)
{
    size_t maxf = channelizer_bank_max_frames(q, n);
    float complex *out = malloc(maxf * q->M * sizeof(float complex));
    if (!out)
        return 0.0;

    channelizer_bank_reset(q);
    unsigned int frames = 0u;
    channelizer_bank_execute(q, in, n, out, &frames, NULL);

    double acc = 0.0;
    unsigned int cnt = 0u;
    for (unsigned int f = 64u; f < frames; f++) /* skip the filter transient */
    {
        float complex v = out[(size_t)f * q->M + bin];
        acc += crealf(v) * crealf(v) + cimagf(v) * cimagf(v);
        cnt++;
    }
    free(out);
    return cnt ? acc / (double)cnt : 0.0;
}

static void test_bank_gain_uniformity(void)
{
    const size_t n = 40000u;
    float complex *in = malloc(n * sizeof(float complex));
    if (!in)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        return;
    }

    /* Production BR/EDR tuning: 20 channels, LO on a half-MHz boundary so the
     * wideband pre-rotation NCO is active. */
    channelizer_bank_t q;
    CHECK(channelizer_bank_init(&q, 20000000u, 2411500000u,
                                CHANNELIZER_BANK_GRID_BR_EDR_HZ,
                                CHANNELIZER_BANK_DEFAULT_M,
                                CHANNELIZER_BANK_DEFAULT_AS) == 0,
          "channelizer_bank_init failed%s", "");

    double lo = 1e300, hi = -1e300;
    unsigned int mapped = 0u;
    for (unsigned int c = 0u; c < 79u; c++)
    {
        uint32_t center = 2402000000u + c * 1000000u;
        int bin = channelizer_bank_bin_for_center(q.M, q.lo_eff_hz, center,
                                                  CHANNELIZER_BANK_GRID_BR_EDR_HZ);
        if (bin < 0)
            continue;

        make_tone(in, n, (double)center - (double)q.lo_hz, 20000000.0);
        double db = 10.0 * log10(bin_power(&q, in, n, (unsigned int)bin) + 1e-300);
        if (db < lo) lo = db;
        if (db > hi) hi = db;
        mapped++;
    }
    CHECK(mapped == 20u, "mapped %u channels, expected 20", mapped);
    CHECK(hi - lo < 0.25, "per-bin gain spread %.4f dB (expected < 0.25)",
          hi - lo);
    channelizer_bank_destroy(&q);

    /* The bank normalises to unity for any bin count, so the fixed dBr offset
     * in CHANNELIZER_BANK_RSSI_CAL_DB keeps RSSI span-independent. */
    static const unsigned int spans_mhz[] = {4u, 8u, 12u, 20u};
    double slo = 1e300, shi = -1e300;
    for (size_t i = 0u; i < sizeof(spans_mhz) / sizeof(spans_mhz[0]); i++)
    {
        unsigned int fs = spans_mhz[i] * 1000000u;
        channelizer_bank_t b;
        if (channelizer_bank_init(&b, fs, 2412000000u,
                                  CHANNELIZER_BANK_GRID_BR_EDR_HZ,
                                  CHANNELIZER_BANK_DEFAULT_M,
                                  CHANNELIZER_BANK_DEFAULT_AS) != 0)
        {
            printf("FAIL bank init at %u MHz\n", spans_mhz[i]);
            g_failures++;
            continue;
        }
        make_tone(in, n, 0.0, (double)fs);
        double db = 10.0 * log10(bin_power(&b, in, n, 0u) + 1e-300);
        if (db < slo) slo = db;
        if (db > shi) shi = db;
        channelizer_bank_destroy(&b);
    }
    CHECK(shi - slo < 0.25, "gain varies %.4f dB across spans (expected < 0.25)",
          shi - slo);

    free(in);
}

int main(void)
{
    test_window_indices();
    test_rssi_independent_of_block_position();
    test_bank_gain_uniformity();

    if (g_failures == 0)
        printf("test_rssi_window: all checks passed\n");
    else
        printf("test_rssi_window: %d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
