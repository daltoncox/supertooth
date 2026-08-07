/* RSSI measurement tests (no RF hardware required).
 *
 * Regression cover for the polyphase-channelizer RSSI work:
 *
 *  - receiver_rssi_access_code_window: BR/EDR RSSI is measured over the
 *    access code at detection time.  The decoder cannot know a packet's true
 *    length (the header is whitened until CLK1-6 is recovered) and completes
 *    after a fixed maximum-length body, so averaging at completion dilutes
 *    every packet with up to 5 slots of post-packet channel content -- idle
 *    floor, interference, or nothing at all when the block-end clamp leaves
 *    the window entirely past the packet (~30 dB of packet-to-packet scatter
 *    measured live for one stationary device).  The access code is constant-
 *    envelope GFSK for every packet type, so a fixed window ending at
 *    detection measures the same physical quantity per packet.
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
#define AC_SPAN       (PKT_BITS * SPS)   /* samples covered by the AC window */

/* --------------------------------------------------------------------------
 * The access-code window: RSSI is measured over the fixed AC span ending at
 * detection, so every packet type reports the same physical quantity.
 * -------------------------------------------------------------------------- */
static void test_access_code_window(void)
{
    unsigned int start = 0u, end = 0u, idle = 0u;

    /* Detection completing at sample 1336 covers the preceding AC span. */
    receiver_rssi_access_code_window(1336u, AC_SPAN, BLOCK_SAMPLES,
                                     &start, &end, &idle);
    CHECK(start == 1336u - AC_SPAN, "ac start %u, expected %u", start,
          1336u - AC_SPAN);
    CHECK(end == 1336u, "ac end %u, expected 1336", end);
    CHECK(idle == start, "ac idle end %u, expected %u", idle, start);

    /* An access code that began before this block reports the part present. */
    receiver_rssi_access_code_window(100u, AC_SPAN, BLOCK_SAMPLES,
                                     &start, &end, &idle);
    CHECK(start == 0u, "straddling ac start %u, expected 0", start);
    CHECK(end == 100u, "straddling ac end %u, expected 100", end);
    CHECK(idle == 0u, "straddling ac idle %u, expected 0", idle);

    /* The end is clamped to what the buffer actually holds. */
    receiver_rssi_access_code_window(BLOCK_SAMPLES + 500u, AC_SPAN, BLOCK_SAMPLES,
                                     &start, &end, &idle);
    CHECK(end == BLOCK_SAMPLES, "clamped ac end %u, expected %u", end,
          BLOCK_SAMPLES);
    CHECK(start == BLOCK_SAMPLES - AC_SPAN, "clamped ac start %u, expected %u",
          start, BLOCK_SAMPLES - AC_SPAN);
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

        /* Unit-amplitude access code, -40 dB elsewhere. */
        for (unsigned int k = 0u; k < BLOCK_SAMPLES; k++)
            buf[k] = (k >= s_lo && k < s_hi) ? 1.0f : 0.01f;

        unsigned int start = 0u, end = 0u, idle = 0u;
        receiver_rssi_access_code_window(s_hi, AC_SPAN, BLOCK_SAMPLES,
                                         &start, &end, &idle);

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

/* --------------------------------------------------------------------------
 * The idle-prefix noise-floor estimate is still tracked (and refreshed only
 * from a long enough idle prefix, EMA-smoothed otherwise), but it is NOT
 * subtracted from the reported RSSI: doing so amplified tiny floor fluctuations
 * into tens of dB of packet-to-packet scatter for weak devices.  The reported
 * value is the raw received power over the signal window.
 * -------------------------------------------------------------------------- */
static void test_noise_floor_subtraction(void)
{
    const unsigned int N = 256u;
    float complex *buf = malloc(N * sizeof(float complex));
    if (!buf)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        return;
    }

    /* Idle prefix (64 samples) is noise/floor at amplitude 0.1 (~ -20 dB);
     * the signal window is amplitude 0.4 (~ -8 dB). */
    const float floor_amp = 0.1f;
    const float sig_amp   = 0.4f;
    for (unsigned int k = 0u; k < N; k++)
        buf[k] = (k < 64u) ? floor_amp : sig_amp;

    float floor_lin = 0.0f;
    unsigned int floor_init = 0u;

    /* Idle prefix [0,64) -> floor estimate ~ floor_amp^2. */
    float rssi = receiver_rssi_signal_dbr(buf, 64u, N, 64u,
                                          &floor_lin, &floor_init,
                                          RECEIVER_RSSI_INVALID);
    CHECK(floor_init == 1u, "floor should be initialised after a long idle", 0);
    CHECK(fabsf(floor_lin - floor_amp * floor_amp) < 1e-4f,
          "floor %.6f, expected %.6f", floor_lin, floor_amp * floor_amp);

    /* Reported power is the raw signal-window power (sig_amp^2), NOT
     * sig_amp^2 - floor_amp^2. */
    float sig_power = sig_amp * sig_amp;
    float expected = receiver_rssi_from_linear_power(sig_power,
                                                      RECEIVER_RSSI_INVALID);
    CHECK(fabsf(rssi - expected) < 0.01f,
          "signal RSSI %.3f dB, expected raw %.3f dB", rssi, expected);

    /* EMA fallback: a tiny idle prefix must not refresh the floor, so the
     * previously seeded value is reused. */
    float before = floor_lin;
    receiver_rssi_signal_dbr(buf, 64u, N, 4u, &floor_lin, &floor_init,
                             RECEIVER_RSSI_INVALID);
    CHECK(floor_lin == before, "short idle must not refresh the floor", 0);

    free(buf);
}

/* A decoded packet always carries real signal, so a measured power at or below
 * the estimated noise floor must still report a finite value (the raw received
 * power), never NaN.  Returning NaN here is exactly what produced the "--" RSSI
 * on the frontend for weak-but-present devices. */
static void test_signal_below_floor_reports_power(void)
{
    const unsigned int N = 256u;
    float complex *buf = malloc(N * sizeof(float complex));
    if (!buf)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        return;
    }

    /* Idle prefix (64 samples) is the noise/interference floor at amplitude
     * 0.4 (~ -8 dB); the signal window is weaker at amplitude 0.2 (~ -20 dB),
     * i.e. below the floor.  The old code returned NaN for this case. */
    const float floor_amp = 0.4f;
    const float sig_amp   = 0.2f;
    for (unsigned int k = 0u; k < N; k++)
        buf[k] = (k < 64u) ? floor_amp : sig_amp;

    float floor_lin = 0.0f;
    unsigned int floor_init = 0u;

    float rssi = receiver_rssi_signal_dbr(buf, 64u, N, 64u,
                                          &floor_lin, &floor_init,
                                          RECEIVER_RSSI_INVALID);
    CHECK(!isnan(rssi), "signal below floor produced no RSSI (NaN)", 0);

    /* Reported value must be the raw signal power, not the (negative) excess. */
    float expected = receiver_rssi_from_linear_power(sig_amp * sig_amp,
                                                      RECEIVER_RSSI_INVALID);
    CHECK(fabsf(rssi - expected) < 0.01f,
          "below-floor RSSI %.3f dB, expected raw signal %.3f dB",
          rssi, expected);

    free(buf);
}

/* One standard-normal sample via Box-Muller (spare cached between calls). */
static float std_normal(void)
{
    static int   have  = 0;
    static float spare = 0.0f;
    if (have)
    {
        have = 0;
        return spare;
    }
    float u1 = ((float)rand() + 0.5f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 0.5f) / ((float)RAND_MAX + 1.0f);
    if (u1 < 1e-6f) u1 = 1e-6f;
    float mag = sqrtf(-2.0f * logf(u1));
    float a   = 2.0f * (float)M_PI * u2;
    float g0  = mag * cosf(a);
    float g1  = mag * sinf(a);
    have = 1; spare = g1;
    return g0;
}

/* Fill [0,n) with complex Gaussian noise of total power `power` (E[|x|^2]=power). */
static void fill_complex_gaussian(float complex *buf, unsigned int n, float power)
{
    const float s = sqrtf(power * 0.5f);
    for (unsigned int i = 0u; i < n; i++)
    {
        float re = std_normal() * s;
        float im = std_normal() * s;
        buf[i]   = re + im * I;
    }
}

/* Measure the packet-to-packet RSSI spread for a *fixed-power* packet embedded
 * in noise, comparing the current "signal - floor" subtraction path against a
 * plain raw-signal-power path.  This isolates the variance introduced purely by
 * estimating/subtracting a noisy per-packet noise floor, which is the dominant
 * cause of the large RSSI swings reported with the polyphase channelizer. */
static void test_rssi_variance_from_floor_subtraction(void)
{
    const unsigned int N        = 256u;
    const unsigned int idle     = 64u;   /* >= RECEIVER_RSSI_FLOOR_MIN_SAMPLES */
    const unsigned int sig_start = idle + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES;
    const unsigned int trials   = 20000u;
    const float        P_n      = 0.40f; /* noise floor power */

    float complex *buf = malloc(N * sizeof(float complex));
    if (!buf)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        return;
    }

    /* (signal_power, label) pairs: one weak (near floor), one strong. */
    const float sig_powers[] = {0.05f, 10.0f};
    const char *labels[]     = {"weak (~9 dB under floor)", "strong (~14 dB over floor)"};
    const size_t np = sizeof(sig_powers) / sizeof(sig_powers[0]);

    for (size_t p = 0u; p < np; p++)
    {
        const float P_sig = sig_powers[p];
        double sub_lo = 1e300, sub_hi = -1e300;
        double raw_lo = 1e300, raw_hi = -1e300;
        double raw_sum = 0.0, raw_sumsq = 0.0;
        unsigned int raw_n = 0u, sub_nan = 0u, sub_jump = 0u;

        for (unsigned int t = 0u; t < trials; t++)
        {
            fill_complex_gaussian(buf, idle, P_n);          /* idle prefix = floor */
            const float ampl = sqrtf(P_sig);
            for (unsigned int i = sig_start; i < N; i++)     /* signal + noise */
            {
                float re = std_normal() * sqrtf(P_n * 0.5f);
                float im = std_normal() * sqrtf(P_n * 0.5f);
                buf[i]   = ampl + re + im * I;
            }

            float floor_lin = 0.0f;
            unsigned int floor_init = 0u;
            float rssi_sub = receiver_rssi_signal_dbr(buf, sig_start, N, idle,
                                                      &floor_lin, &floor_init,
                                                      RECEIVER_RSSI_INVALID);
            float rssi_raw = receiver_rssi_from_mean_power_range(buf, sig_start, N,
                                                                RECEIVER_RSSI_INVALID);

            if (isnan(rssi_sub))
                sub_nan++;
            else
            {
                if (rssi_sub < sub_lo) sub_lo = rssi_sub;
                if (rssi_sub > sub_hi) sub_hi = rssi_sub;
            }
            /* A "jump" is the new raw-signal regime: reported value far above a
             * plausible signal-only level because signal fell under the floor. */
            if (!isnan(rssi_sub) && (sub_hi - rssi_sub) > 10.0)
                sub_jump++;

            if (!isnan(rssi_raw))
            {
                if (rssi_raw < raw_lo) raw_lo = rssi_raw;
                if (rssi_raw > raw_hi) raw_hi = rssi_raw;
                raw_sum += rssi_raw;
                raw_sumsq += rssi_raw * rssi_raw;
                raw_n++;
            }
        }

        double raw_mean = raw_n ? raw_sum / (double)raw_n : 0.0;
        double raw_var  = raw_n ? (raw_sumsq / (double)raw_n - raw_mean * raw_mean) : 0.0;
        double raw_std  = raw_var > 0.0 ? sqrt(raw_var) : 0.0;
        double sub_spread = (sub_hi > sub_lo) ? (sub_hi - sub_lo) : 0.0;
        double raw_spread = (raw_hi > raw_lo) ? (raw_hi - raw_lo) : 0.0;

        printf("  [%s] sub-path spread=%.2f dB (nan=%u, jumps=%u)  "
               "raw-path spread=%.2f dB std=%.2f dB\n",
               labels[p], sub_spread, sub_nan, sub_jump, raw_spread, raw_std);

        /* The subtraction path must not be dramatically noisier than simply
         * reporting the raw received power for the same fixed packet. */
        CHECK(raw_spread < 3.0, "raw-path spread %.2f dB too high", raw_spread, 0);
        CHECK(sub_spread < 5.0 * raw_spread + 1.0,
              "floor-subtraction spread %.2f dB >> raw %.2f dB",
              sub_spread, raw_spread);
    }

    free(buf);
}

/* Fill `rf` (length `cap`) with a continuous constant-envelope MSK signal
 * (BLE-like: deviation = symrate/4) at baseband carrier (baseband_hz+offset_hz).
 * Continuous fill avoids zero-padding artefacts when sliding the RSSI window. */
static void fill_msk_stream(float complex *rf, unsigned int cap,
                            double baseband_hz, double offset_hz,
                            unsigned int sample_rate)
{
    double phase = 0.0;
    double df = 1000000.0 / 4.0;                  /* 1 Msym/s -> 250 kHz dev */
    for (unsigned int n = 0u; n < cap; n++)
    {
        unsigned int b = n / (sample_rate / 1000000u);
        int bit = (b & 1u) ? 1 : 0;
        double freq = (bit ? 1.0 : -1.0) * df + offset_hz;
        phase += 2.0 * M_PI * freq / (double)sample_rate;
        rf[n] = cosf((float)phase) + I * sinf((float)phase);
    }
}

/* Drive a BLE-like packet through the real polyphase channelizer and measure
 * the captured RSSI under three perturbations, to localise the packet-to-packet
 * RSSI scatter seen in production:
 *   (a) carrier offset from the bin centre (scalloping / LO-grid error),
 *   (b) where the packet lands inside a channelizer block (phase/alignment),
 *   (c) a packet that straddles a channelizer block boundary (window clamp). */
static void test_channelizer_rssi_stability(void)
{
    const unsigned int sample_rate = 20000000u;
    const uint32_t grid = 2000000u;
    const unsigned int M = sample_rate / grid;        /* 10 */
    const unsigned int M2 = M / 2u;                   /* 5  */
    const unsigned int stride = grid / 1000000u;      /* 2  */
    const unsigned int input_decim = M2 * stride;     /* 10 */
    const unsigned int m = CHANNELIZER_BANK_DEFAULT_M;
    const float as = CHANNELIZER_BANK_DEFAULT_AS;
    const unsigned int bin = M / 4u;                  /* mid-span bin */
    const unsigned int sps_rf = sample_rate / 1000000u;
    const unsigned int win_decim = 80u * sps_rf / input_decim; /* one packet len */

    channelizer_bank_t q;
    uint32_t lo_hz = 2402000000u;
    CHECK(channelizer_bank_init(&q, sample_rate, lo_hz, grid, m, as) == 0,
          "channelizer init failed", 0);
    uint32_t center = channelizer_bank_center_for_bin(M, q.lo_eff_hz, bin, grid);
    double baseband_hz = (double)(center - q.lo_eff_hz);

    const unsigned int block_rf = 8192u;
    const unsigned int max_frames = block_rf / M2;
    const size_t out_cap = (size_t)M * (max_frames + 2u);

    float complex *rf = malloc(block_rf * sizeof(float complex));
    float complex *out = malloc(out_cap * sizeof(float complex));
    float complex *decim = malloc((max_frames / stride + 4u) * sizeof(float complex));
    if (!rf || !out || !decim)
    {
        printf("FAIL out of memory\n");
        g_failures++;
        channelizer_bank_destroy(&q);
        return;
    }

    #define BUILD_DECIM() do {                                                  \
        unsigned int frames = 0u;                                              \
        channelizer_bank_execute(&q, rf, block_rf, out, &frames, NULL);        \
        dout = 0u;                                                             \
        for (unsigned int k = 0u; k < frames; k += stride)                    \
            decim[dout++] = out[(size_t)k * M + bin];                          \
    } while (0)

    unsigned int dout = 0u;

    /* (a) carrier offset sweep: continuous signal, fixed window.
     *
     * Realistic offsets: the BR/EDR spec allows +-75 kHz carrier error and a
     * HackRF LO adds ~+-50 kHz (20 ppm), so a fixed device sits at most
     * ~+-150 kHz off bin centre.  Within that range the measured spread must
     * be small (it is a constant per-device bias anyway, not packet-to-packet
     * variance, but it should not move the reading by much).  A full-bin sweep
     * is printed below for channelizer characterisation only: at +-1 MHz the
     * signal sits on/inside the adjacent-bin transition band, where the 60 dB
     * prototype rolls off (about -6 dB at the bin edge), so several dB of
     * spread there is expected and NOT asserted against. */
    double off_lo = 1e300, off_hi = -1e300;
    for (int oi = -6; oi <= 6; oi++)
    {
        double offset_hz = (double)oi * 25000.0;      /* -150 .. +150 kHz */
        fill_msk_stream(rf, block_rf, baseband_hz, offset_hz, sample_rate);
        channelizer_bank_reset(&q);
        BUILD_DECIM();
        float rssi = receiver_rssi_from_mean_power_range(
            decim, RECEIVER_RSSI_DEMOD_DELAY_SAMPLES,
            RECEIVER_RSSI_DEMOD_DELAY_SAMPLES + win_decim, RECEIVER_RSSI_INVALID);
        if (rssi < off_lo) off_lo = rssi;
        if (rssi > off_hi) off_hi = rssi;
    }
    printf("  [carrier offset +-150 kHz]    RSSI spread = %.2f dB\n",
           off_hi - off_lo);
    CHECK(off_hi - off_lo < 4.0,
          "scalloping spread %.2f dB too high at realistic offsets",
          off_hi - off_lo, 0);

    double full_lo = off_lo, full_hi = off_hi;
    for (int oi = -10; oi <= 10; oi++)
    {
        double offset_hz = (double)oi * 100000.0;     /* -1 .. +1 MHz */
        fill_msk_stream(rf, block_rf, baseband_hz, offset_hz, sample_rate);
        channelizer_bank_reset(&q);
        BUILD_DECIM();
        float rssi = receiver_rssi_from_mean_power_range(
            decim, RECEIVER_RSSI_DEMOD_DELAY_SAMPLES,
            RECEIVER_RSSI_DEMOD_DELAY_SAMPLES + win_decim, RECEIVER_RSSI_INVALID);
        if (rssi < full_lo) full_lo = rssi;
        if (rssi > full_hi) full_hi = rssi;
    }
    printf("  [carrier offset -1..+1 MHz]   RSSI spread = %.2f dB "
           "(characterisation, not asserted)\n", full_hi - full_lo);

    /* (b) alignment sweep: slide a fixed window over a continuous stream, at
     * offset 0 and at a realistic +0.5 MHz offset. */
    double ali0_lo = 1e300, ali0_hi = -1e300;
    double ali5_lo = 1e300, ali5_hi = -1e300;
    const unsigned int max_dout = (block_rf / M2) / stride + 2u;
    for (unsigned int start = 320u;
         start + win_decim + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES <= max_dout;
         start += 137u)
    {
        fill_msk_stream(rf, block_rf, baseband_hz, 0.0, sample_rate);
        channelizer_bank_reset(&q);
        BUILD_DECIM();
        unsigned int wstart = (start / input_decim) + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES;
        unsigned int wend = wstart + win_decim;
        if (wend > dout) wend = dout;
        float rssi = receiver_rssi_from_mean_power_range(decim, wstart, wend,
                                                         RECEIVER_RSSI_INVALID);
        if (rssi < ali0_lo) ali0_lo = rssi;
        if (rssi > ali0_hi) ali0_hi = rssi;

        fill_msk_stream(rf, block_rf, baseband_hz, 500000.0, sample_rate);
        channelizer_bank_reset(&q);
        BUILD_DECIM();
        wstart = (start / input_decim) + RECEIVER_RSSI_DEMOD_DELAY_SAMPLES;
        wend = wstart + win_decim;
        if (wend > dout) wend = dout;
        rssi = receiver_rssi_from_mean_power_range(decim, wstart, wend,
                                                   RECEIVER_RSSI_INVALID);
        if (rssi < ali5_lo) ali5_lo = rssi;
        if (rssi > ali5_hi) ali5_hi = rssi;
    }
    printf("  [alignment, offset 0]        RSSI spread = %.2f dB\n", ali0_hi - ali0_lo);
    printf("  [alignment, offset +0.5MHz]  RSSI spread = %.2f dB\n", ali5_hi - ali5_lo);
    CHECK(ali0_hi - ali0_lo < 3.0,
          "alignment spread (offset 0) %.2f dB too high", ali0_hi - ali0_lo, 0);
    CHECK(ali5_hi - ali5_lo < 3.0,
          "alignment spread (offset +0.5MHz) %.2f dB too high", ali5_hi - ali5_lo, 0);

    /* (c) straddle: a packet (preceded/followed by idle zeros) split across two
     * consecutive blocks (carry) vs the same packet wholly inside one block. */
    const unsigned int pkt_rf = 80u * sps_rf;
    const unsigned int split = 400u;
    float rssi_ctrl = 0.0f, rssi_strad = 0.0f;

    fill_msk_stream(rf, block_rf, baseband_hz, 0.0, sample_rate);
    channelizer_bank_reset(&q);
    BUILD_DECIM();
    {
        unsigned int wend = pkt_rf / input_decim;
        if (wend > dout) wend = dout;
        rssi_ctrl = receiver_rssi_from_mean_power_range(
            decim, RECEIVER_RSSI_DEMOD_DELAY_SAMPLES, wend, RECEIVER_RSSI_INVALID);
    }

    /* tail in block A, head in block B; frame emits in B with start clamped 0 */
    fill_msk_stream(rf, block_rf, baseband_hz, 0.0, sample_rate);
    for (unsigned int n = 0u; n < block_rf - split; n++) rf[n] = 0.0f + 0.0f * I; /* idle before */
    channelizer_bank_reset(&q);
    BUILD_DECIM();                                   /* block A */
    fill_msk_stream(rf, block_rf, baseband_hz, 0.0, sample_rate);
    for (unsigned int n = split; n < block_rf; n++) rf[n] = 0.0f + 0.0f * I;     /* idle after */
    BUILD_DECIM();                                   /* block B */
    {
        unsigned int wend = split / input_decim + sps_rf / input_decim;
        if (wend > dout) wend = dout;
        rssi_strad = receiver_rssi_from_mean_power_range(
            decim, RECEIVER_RSSI_DEMOD_DELAY_SAMPLES, wend, RECEIVER_RSSI_INVALID);
    }
    printf("  [straddle vs in-block]       delta = %.2f dB (ctrl=%.2f strad=%.2f)\n",
           rssi_strad - rssi_ctrl, rssi_ctrl, rssi_strad);
    CHECK(rssi_strad - rssi_ctrl > -12.0,
          "straddle dilution %.2f dB too high", rssi_strad - rssi_ctrl, 0);

    free(rf); free(out); free(decim);
    channelizer_bank_destroy(&q);
    #undef BUILD_DECIM
}

int main(void)
{
    srand((unsigned int)time(NULL));
    test_access_code_window();
    test_rssi_independent_of_block_position();
    test_bank_gain_uniformity();
    test_noise_floor_subtraction();
    test_signal_below_floor_reports_power();
    test_rssi_variance_from_floor_subtraction();
    test_channelizer_rssi_stability();

    if (g_failures == 0)
        printf("test_rssi_window: all checks passed\n");
    else
        printf("test_rssi_window: %d failure(s)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
