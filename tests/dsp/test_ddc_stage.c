/* DDC stage tests (no RF hardware required).
 *
 * Verifies:
 *  - init rejects bad decimation factors and non-divisible rates;
 *  - execute decimates by D with input-domain base mapping
 *    (out_base = in_base - previous carry) across ragged calls;
 *  - total output length over split calls equals the single-shot length
 *    (fractional carry correctness);
 *  - a tone at the subband centre emerges at DC (folded NCO correctness).
 */
#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ddc_stage.h"

static int g_failures = 0;

#define CHECK_U64(name, actual, expected)                                     \
    do {                                                                      \
        uint64_t a_ = (uint64_t)(actual), e_ = (uint64_t)(expected);          \
        if (a_ != e_) {                                                       \
            printf("FAIL %-34s expected %llu got %llu\n", name,               \
                   (unsigned long long)e_, (unsigned long long)a_);           \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define CHECK_TRUE(name, cond)                                                \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %-34s condition false\n", name);                     \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static void test_init_validation(void)
{
    ddc_stage_t q;
    memset(&q, 0, sizeof(q));
    /* decim 1 = premix-only, accepted */
    CHECK_TRUE("decim 1 accepted",
               ddc_stage_init(&q, 80000000u, 2441000000u, 2431000000u,
                              1u, 4u, 60.0f) == 0);
    CHECK_U64("premix out rate", q.sample_rate_out, 80000000u);
    CHECK_TRUE("premix no fir", q.decim_fir == NULL);
    ddc_stage_destroy(&q);
    memset(&q, 0, sizeof(q));
    /* decim > max rejected */
    CHECK_TRUE("decim 5 rejected",
               ddc_stage_init(&q, 80000000u, 2441000000u, 2431000000u,
                              5u, 4u, 60.0f) != 0);
    /* rate not divisible by decim rejected */
    CHECK_TRUE("rate/decim mismatch rejected",
               ddc_stage_init(&q, 80000001u, 2441000000u, 2431000000u,
                              4u, 4u, 60.0f) != 0);
    /* sane config accepted */
    CHECK_TRUE("decim 4 accepted",
               ddc_stage_init(&q, 80000000u, 2441000000u, 2431000000u,
                              4u, 4u, 60.0f) == 0);
    CHECK_U64("out rate", q.sample_rate_out, 20000000u);
    CHECK_U64("shift", (uint64_t)(q.shift_hz + 100000000),
              (uint64_t)((int32_t)2431000000u - (int32_t)2441000000u +
                         100000000));
    ddc_stage_destroy(&q);
}

static void test_carry_and_bases(void)
{
    ddc_stage_t q;
    memset(&q, 0, sizeof(q));
    /* Zero shift (sub centre == LO) isolates the decimation math. */
    if (ddc_stage_init(&q, 80000000u, 2441000000u, 2441000000u, 4u, 4u,
                       60.0f) != 0)
    {
        printf("FAIL ddc init\n");
        g_failures++;
        return;
    }

    const size_t N = 65536u;
    float complex *in = malloc(N * sizeof(*in));
    float complex *o1 = malloc(N * sizeof(*o1));
    float complex *o2 = malloc(N * sizeof(*o2));
    if (!in || !o1 || !o2)
    {
        printf("FAIL alloc\n");
        g_failures++;
        free(in);
        free(o1);
        free(o2);
        ddc_stage_destroy(&q);
        return;
    }
    for (size_t i = 0u; i < N; i++)
        in[i] = 1.0f + 0.0f * _Complex_I;

    /* Single shot. */
    unsigned int n_full = 0u;
    uint64_t b_full = 0u;
    ddc_stage_execute(&q, in, N, 1000000ull, o1, &n_full, &b_full);
    CHECK_U64("single-shot count", n_full, N / 4u);
    CHECK_U64("single-shot base", b_full, 1000000ull);

    /* Reset, then split into ragged halves (odd split => carry path). */
    ddc_stage_reset(&q);
    size_t half = N / 2u + 1u; /* +1 forces a fractional carry */
    unsigned int n_a = 0u, n_b = 0u;
    uint64_t b_a = 0u, b_b = 0u;
    ddc_stage_execute(&q, in, half, 1000000ull, o1, &n_a, &b_a);
    ddc_stage_execute(&q, &in[half], N - half, 1000000ull + half, o2,
                      &n_b, &b_b);
    CHECK_U64("split total", (uint64_t)n_a + n_b, n_full);
    CHECK_U64("split base a", b_a, 1000000ull);
    /* Second call's window starts at (in_base - carry): carry is
     * (half % 4) = 1 here. */
    CHECK_U64("split base b", b_b, 1000000ull + half - (half % 4u));

    free(in);
    free(o1);
    free(o2);
    ddc_stage_destroy(&q);
}

static void test_tone_to_dc(void)
{
    /* Tone exactly at the subband centre must be rotated to DC before the
     * decimator: mean phase drift of the output ~ 0. */
    const unsigned int FS = 80000000u;
    const uint32_t LO = 2441000000u, SUB = 2431000000u; /* -10 MHz */
    ddc_stage_t q;
    memset(&q, 0, sizeof(q));
    if (ddc_stage_init(&q, FS, LO, SUB, 4u, 4u, 60.0f) != 0)
    {
        printf("FAIL ddc init tone\n");
        g_failures++;
        return;
    }

    const size_t N = 65536u;
    float complex *in = malloc(N * sizeof(*in));
    float complex *out = malloc(N * sizeof(*out));
    if (!in || !out)
    {
        printf("FAIL alloc tone\n");
        g_failures++;
        free(in);
        free(out);
        ddc_stage_destroy(&q);
        return;
    }
    /* Baseband tone at SUB - LO = -10 MHz. */
    double omega = -10.0e6 / (double)FS * 2.0 * M_PI;
    for (size_t i = 0u; i < N; i++)
        in[i] = cexpf(_Complex_I * (float)(omega * (double)i));

    unsigned int n_out = 0u;
    ddc_stage_execute(&q, in, N, 0ull, out, &n_out, NULL);
    CHECK_TRUE("tone produces output", n_out > 1000u);

    /* After settling (skip filter transient), output magnitude ~1 and the
     * phase increment per sample ~0 (DC). */
    size_t skip = n_out / 4u;
    double mag_sum = 0.0;
    double ph_sum = 0.0;
    unsigned int ph_n = 0u;
    for (size_t i = skip; i < n_out; i++)
    {
        mag_sum += cabsf(out[i]);
        if (i + 1u < n_out)
        {
            float dphi = cargf(out[i + 1u] * conjf(out[i]));
            ph_sum += dphi;
            ph_n++;
        }
    }
    double mag = mag_sum / (double)(n_out - skip);
    double mean_dphi = ph_sum / (double)ph_n;
    if (fabs(mag - 1.0) > 0.05)
    {
        printf("FAIL tone mag %f (want ~1.0)\n", mag);
        g_failures++;
    }
    if (fabs(mean_dphi) > 0.01)
    {
        printf("FAIL tone drift %f rad/sample (want ~0)\n", mean_dphi);
        g_failures++;
    }

    free(in);
    free(out);
    ddc_stage_destroy(&q);
}

static void test_premix_passthrough(void)
{
    /* decim == 1: NCO straight to out, 1:1, out_base == in_base, no carry
     * ever engages. Phase must stay continuous across split calls. */
    const unsigned int FS = 20000000u;
    const uint32_t LO = 2411500000u, SUB = 2412000000u; /* +0.5 MHz */
    ddc_stage_t q;
    memset(&q, 0, sizeof(q));
    if (ddc_stage_init(&q, FS, LO, SUB, 1u, 4u, 60.0f) != 0)
    {
        printf("FAIL premix init\n");
        g_failures++;
        return;
    }

    const size_t N = 65536u;
    float complex *in = malloc(N * sizeof(*in));
    float complex *o1 = malloc(N * sizeof(*o1));
    float complex *o2 = malloc(N * sizeof(*o2));
    if (!in || !o1 || !o2)
    {
        printf("FAIL alloc premix\n");
        g_failures++;
        free(in);
        free(o1);
        free(o2);
        ddc_stage_destroy(&q);
        return;
    }
    /* Tone at the subband centre (baseband +0.5 MHz) must emerge at DC. */
    double omega = 0.5e6 / (double)FS * 2.0 * M_PI;
    for (size_t i = 0u; i < N; i++)
        in[i] = cexpf(_Complex_I * (float)(omega * (double)i));

    /* Split across two calls at an odd boundary: counts/bases stay 1:1. */
    size_t half = N / 2u + 1u;
    unsigned int n_a = 0u, n_b = 0u;
    uint64_t b_a = 0u, b_b = 0u;
    ddc_stage_execute(&q, in, half, 1000000ull, o1, &n_a, &b_a);
    ddc_stage_execute(&q, &in[half], N - half, 1000000ull + half, o2,
                      &n_b, &b_b);
    CHECK_U64("premix count a", n_a, half);
    CHECK_U64("premix count b", n_b, N - half);
    CHECK_U64("premix base a", b_a, 1000000ull);
    CHECK_U64("premix base b", b_b, 1000000ull + half);

    /* Joined output is DC at unity magnitude: phase-continuous premix. */
    double mag_sum = 0.0, ph_sum = 0.0;
    unsigned int ph_n = 0u;
    float complex prev = o1[half - 1u];
    for (size_t i = 0u; i < N - half; i++)
    {
        mag_sum += cabsf(o2[i]);
        ph_sum += cargf(o2[i] * conjf(prev));
        prev = o2[i];
        ph_n++;
    }
    for (size_t i = 0u; i < half; i++)
        mag_sum += cabsf(o1[i]);
    double mag = mag_sum / (double)N;
    if (fabs(mag - 1.0) > 0.01)
    {
        printf("FAIL premix mag %f (want ~1.0)\n", mag);
        g_failures++;
    }
    /* Boundary + interior drift: one symbol-free DC stream. */
    double mean_dphi = ph_sum / (double)ph_n;
    if (fabs(mean_dphi) > 0.01)
    {
        printf("FAIL premix drift %f rad/sample (want ~0)\n", mean_dphi);
        g_failures++;
    }

    free(in);
    free(o1);
    free(o2);
    ddc_stage_destroy(&q);
}

int main(void)
{
    test_init_validation();
    test_carry_and_bases();
    test_tone_to_dc();
    test_premix_passthrough();

    if (g_failures)
    {
        printf("test_ddc_stage: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ddc_stage: OK\n");
    return 0;
}
