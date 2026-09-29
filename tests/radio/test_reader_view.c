/**
 * @file test_reader_view.c
 * @brief Tests for the channel view on sample_reader_t.
 *
 * Covers sample_reader_configure_view() validation plus the single stream
 * function sample_reader_next():
 *  - raw mode hands out pool block pointers directly (zero copy);
 *  - view mode gathers blk[bin + k*M] stepping k by stride, bit-exact
 *    against the historical manual loop, flooring ragged tails;
 *  - held blocks auto-release on the next call and on destroy (pool
 *    quiesces); double-configure and bad args are rejected.
 */

#include <complex.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sample_dispatcher.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(cond))                                                                   \
        {                                                                              \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                              \
        }                                                                              \
    } while (0)

#define M_LANE 20u
#define STRIDE 2u

static sample_dispatcher_t *make_dispatcher(void)
{
    sample_dispatcher_t *d =
        (sample_dispatcher_t *)calloc(1, sizeof(*d));
    if (!d)
        return NULL;
    if (sample_dispatcher_init(d) != 0)
    {
        free(d);
        return NULL;
    }
    return d;
}

/* Push one block filled with f(i) = i (real) so every position is
 * identifiable; returns 0 on success. */
static int push_indexed(sample_dispatcher_t *d, unsigned int n,
                        uint64_t base)
{
    sample_block_t *b = sample_dispatcher_acquire_block(d);
    if (!b)
        return -1;
    b->num_samples       = n;
    b->block_base_sample = base;
    for (unsigned int i = 0u; i < n; i++)
        b->samples[i] = (float)i + 0.0f * _Complex_I;
    sample_dispatcher_push_block(d, b);
    sample_block_release(b);
    return 0;
}

/* Raw mode: pointer equality with the pool block (the no-copy proof). */
static void test_raw_zero_copy(void)
{
    sample_dispatcher_t *d = make_dispatcher();
    TEST_ASSERT(d != NULL);
    if (!d)
        return;

    sample_reader_t r;
    memset(&r, 0, sizeof(r));
    TEST_ASSERT(sample_reader_init(&r, d) == 0);

    const unsigned int N = 1024u;
    TEST_ASSERT(push_indexed(d, N, 777ull) == 0);

    _Atomic unsigned int stop = 0u;
    const float complex *s = NULL;
    unsigned int count = 0u;
    uint64_t base = 0u;
    TEST_ASSERT(sample_reader_next(&r, &stop, &s, &count, &base) == 0);
    TEST_ASSERT(count == N);
    TEST_ASSERT(base == 777ull);
    TEST_ASSERT(s == d->blocks[0].samples);
    TEST_ASSERT(crealf(s[100]) == 100.0f);

    /* Second call with an empty queue would block: shut down instead and
     * confirm the held block flushes through destroy (pool quiesces). */
    atomic_store_explicit(&stop, 1u, memory_order_release);
    sample_reader_signal(&r);
    TEST_ASSERT(sample_reader_next(&r, &stop, &s, &count, &base) != 0);
    sample_reader_destroy(&r);
    TEST_ASSERT(sample_dispatcher_all_free(d));
    sample_dispatcher_destroy(d);
    free(d);
}

/* View mode: gathered stream matches the historical manual loop exactly,
 * including the ragged-tail floor. */
static void test_view_gather(void)
{
    sample_dispatcher_t *d = make_dispatcher();
    TEST_ASSERT(d != NULL);
    if (!d)
        return;

    sample_reader_t r;
    memset(&r, 0, sizeof(r));
    TEST_ASSERT(sample_reader_init(&r, d) == 0);

    const unsigned int BIN = 7u;
    TEST_ASSERT(sample_reader_configure_view(&r, BIN, M_LANE, STRIDE,
                                             10u * M_LANE / 2u, 2000000u,
                                             2402000000u, 20.0f) == 0);
    /* Double-configure is rejected. */
    TEST_ASSERT(sample_reader_configure_view(&r, BIN, M_LANE, STRIDE, 1u,
                                             2000000u, 2402000000u,
                                             20.0f) != 0);
    TEST_ASSERT(r.view_bin == BIN && r.view_M == M_LANE &&
                r.view_stride == STRIDE);
    TEST_ASSERT(r.view_decimation == 10u * M_LANE / 2u);
    TEST_ASSERT(r.view_rate_hz == 2000000u);
    TEST_ASSERT(r.view_center_hz == 2402000000u);
    TEST_ASSERT(r.view_rssi_cal_db == 20.0f);

    /* Ragged block: not a multiple of M (plus a strided remainder). */
    const unsigned int N = 3u * M_LANE + 5u;
    TEST_ASSERT(push_indexed(d, N, 1000000ull) == 0);

    _Atomic unsigned int stop = 0u;
    const float complex *s = NULL;
    unsigned int count = 0u;
    uint64_t base = 0u;
    TEST_ASSERT(sample_reader_next(&r, &stop, &s, &count, &base) == 0);
    TEST_ASSERT(base == 1000000ull);
    /* frames = floor(65/20) = 3; k = 0, 2 → 2 samples. */
    TEST_ASSERT(count == 2u);
    TEST_ASSERT(s != d->blocks[0].samples); /* the one (owned) copy */
    TEST_ASSERT(crealf(s[0]) == (float)(BIN + 0u * M_LANE));
    TEST_ASSERT(crealf(s[1]) == (float)(BIN + 2u * M_LANE));

    /* Reference: the exact historical manual loop. */
    {
        sample_block_t *b = &d->blocks[0];
        unsigned int frames = b->num_samples / M_LANE;
        unsigned int k = 0u;
        for (unsigned int f = 0u; f < frames; f += STRIDE)
        {
            TEST_ASSERT(k < count);
            TEST_ASSERT(crealf(s[k]) ==
                        crealf(b->samples[BIN + (size_t)f * M_LANE]));
            k++;
        }
        TEST_ASSERT(k == count);
    }

    sample_reader_destroy(&r);
    TEST_ASSERT(sample_dispatcher_all_free(d));
    sample_dispatcher_destroy(d);
    free(d);
}

/* Held blocks release across calls: the pool never leaks held references. */
static void test_held_auto_release(void)
{
    sample_dispatcher_t *d = make_dispatcher();
    TEST_ASSERT(d != NULL);
    if (!d)
        return;

    sample_reader_t r;
    memset(&r, 0, sizeof(r));
    TEST_ASSERT(sample_reader_init(&r, d) == 0);
    TEST_ASSERT(sample_reader_configure_view(&r, 3u, 10u, 1u, 5u, 2000000u,
                                             2402000000u, 0.0f) == 0);

    _Atomic unsigned int stop = 0u;
    const float complex *s = NULL;
    unsigned int count = 0u;
    uint64_t base = 0u;
    for (unsigned int b = 0u; b < 4u; b++)
    {
        TEST_ASSERT(push_indexed(d, 100u, (uint64_t)b * 100u) == 0);
        TEST_ASSERT(sample_reader_next(&r, &stop, &s, &count, &base) == 0);
        TEST_ASSERT(count == 10u && base == (uint64_t)b * 100u);
    }
    /* Three earlier blocks must have been released by subsequent calls;
     * only the latest is still held. */
    TEST_ASSERT(!sample_dispatcher_all_free(d));
    sample_reader_destroy(&r);
    TEST_ASSERT(sample_dispatcher_all_free(d));
    sample_dispatcher_destroy(d);
    free(d);
}

/* Bad arguments are rejected without side effects. */
static void test_bad_args(void)
{
    sample_dispatcher_t *d = make_dispatcher();
    TEST_ASSERT(d != NULL);
    if (!d)
        return;

    sample_reader_t r;
    memset(&r, 0, sizeof(r));
    TEST_ASSERT(sample_reader_init(&r, d) == 0);

    TEST_ASSERT(sample_reader_configure_view(NULL, 0u, 10u, 1u, 1u, 1u, 1u,
                                             0.0f) != 0);
    TEST_ASSERT(sample_reader_configure_view(&r, 0u, 0u, 1u, 1u, 1u, 1u,
                                             0.0f) != 0); /* M == 0 */
    TEST_ASSERT(sample_reader_configure_view(&r, 0u, 10u, 0u, 1u, 1u, 1u,
                                             0.0f) != 0); /* stride == 0 */
    TEST_ASSERT(sample_reader_configure_view(&r, 10u, 10u, 1u, 1u, 1u, 1u,
                                             0.0f) != 0); /* bin >= M */
    TEST_ASSERT(sample_reader_configure_view(&r, 0u, 10u, 1u, 0u, 1u, 1u,
                                             0.0f) != 0); /* decim == 0 */
    TEST_ASSERT(sample_reader_configure_view(&r, 0u, 10u, 1u, 1u, 0u, 1u,
                                             0.0f) != 0); /* rate == 0 */
    /* Still raw after all rejections. */
    TEST_ASSERT(r.view_stride == 0u && r.view_scratch == NULL);

    _Atomic unsigned int stop = 0u;
    TEST_ASSERT(sample_reader_next(NULL, &stop, NULL, NULL, NULL) != 0);
    TEST_ASSERT(sample_reader_next(&r, NULL, NULL, NULL, NULL) != 0);

    sample_reader_destroy(&r);
    sample_dispatcher_destroy(d);
    free(d);
}

int main(void)
{
    test_raw_zero_copy();
    test_view_gather();
    test_held_auto_release();
    test_bad_args();

    if (g_failures)
    {
        fprintf(stderr, "test_reader_view: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_reader_view: ok\n");
    return 0;
}
