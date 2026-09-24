/**
 * @file test_dispatcher_backpressure.c
 * @brief Tests for sample_dispatcher_can_push().
 *
 * Verifies the query used by exhaustive file replay for backpressure: true
 * when every reader has room, false as soon as any reader's queue is full,
 * true again after a pop drains it.
 */

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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

typedef struct
{
    sample_dispatcher_t *dispatcher;
    const _Atomic unsigned int *stop;
    sample_block_t *block;
    unsigned int delivered;
} blocking_arg_t;

static void *blocking_pusher(void *arg)
{
    blocking_arg_t *a = (blocking_arg_t *)arg;
    a->delivered =
        sample_dispatcher_push_blocking(a->dispatcher, a->block, a->stop);
    return NULL;
}

static void *blocking_acquirer(void *arg)
{
    blocking_arg_t *a = (blocking_arg_t *)arg;
    a->block =
        sample_dispatcher_acquire_blocking(a->dispatcher, a->stop);
    return NULL;
}

static unsigned int reader_count(sample_reader_t *r)
{
    unsigned int n;
    pthread_mutex_lock(&r->mutex);
    n = r->count;
    pthread_mutex_unlock(&r->mutex);
    return n;
}

static void sleep_ms(long ms)
{
    struct timespec ts = {.tv_sec = ms / 1000L,
                          .tv_nsec = (ms % 1000L) * 1000000L};
    nanosleep(&ts, NULL);
}

/* Multi-reader fan-out on a fresh dispatcher: one push reaches every
 * reader; can_push is false while ANY reader is full; per-reader drops
 * are accounted without affecting the other reader. */
static void test_multi_reader_fanout(void)
{
    sample_dispatcher_t *d =
        (sample_dispatcher_t *)calloc(1, sizeof(*d));
    sample_reader_t a, b;
    _Atomic unsigned int stop = 0u;
    TEST_ASSERT(d != NULL);
    TEST_ASSERT(sample_dispatcher_init(d) == 0);
    TEST_ASSERT(sample_reader_init(&a, d) == 0);
    TEST_ASSERT(sample_reader_init(&b, d) == 0);

    /* One push reaches both readers. */
    {
        sample_block_t *blk = sample_dispatcher_acquire_block(d);
        TEST_ASSERT(blk != NULL);
        blk->num_samples = 1u;
        blk->block_base_sample = 42u;
        TEST_ASSERT(sample_dispatcher_push_block(d, blk) == 2u);
        sample_block_release(blk);
        sample_block_t *pa = NULL, *pb = NULL;
        TEST_ASSERT(sample_reader_wait_pop(&a, &stop, &pa) == 0);
        TEST_ASSERT(sample_reader_wait_pop(&b, &stop, &pb) == 0);
        TEST_ASSERT(pa != NULL && pb != NULL);
        TEST_ASSERT(pa->block_base_sample == 42u);
        TEST_ASSERT(pb->block_base_sample == 42u);
        sample_block_release(pa);
        sample_block_release(pb);
    }

    /* Fill A only (drain B after each push): A full gates can_push. */
    for (unsigned int i = 0u; i < SAMPLE_READER_QUEUE_CAPACITY; i++)
    {
        sample_block_t *blk = sample_dispatcher_acquire_block(d);
        TEST_ASSERT(blk != NULL);
        blk->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(d, blk) == 2u);
        sample_block_release(blk);
        {
            sample_block_t *q = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&b, &stop, &q) == 0);
            sample_block_release(q);
        }
    }
    TEST_ASSERT(sample_dispatcher_can_push(d) == 0);
    {
        sample_block_t *q = NULL;
        TEST_ASSERT(sample_reader_wait_pop(&a, &stop, &q) == 0);
        sample_block_release(q);
    }
    TEST_ASSERT(sample_dispatcher_can_push(d) != 0);

    /* A full again while B drains: push delivers to B only, A drops. */
    {
        sample_block_t *blk = sample_dispatcher_acquire_block(d);
        TEST_ASSERT(blk != NULL);
        blk->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(d, blk) == 2u);
        sample_block_release(blk);
        {
            sample_block_t *q = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&b, &stop, &q) == 0);
            sample_block_release(q);
        }
    }
    TEST_ASSERT(sample_dispatcher_can_push(d) == 0);
    {
        unsigned long before = sample_dispatcher_total_dropped(d);
        sample_block_t *blk = sample_dispatcher_acquire_block(d);
        TEST_ASSERT(blk != NULL);
        blk->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(d, blk) == 1u);
        sample_block_release(blk);
        TEST_ASSERT(sample_dispatcher_total_dropped(d) == before + 1u);
        {
            sample_block_t *q = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&b, &stop, &q) == 0);
            sample_block_release(q);
        }
    }

    /* Drop-counter helpers are NULL-safe; note_drop bumps the total. */
    {
        unsigned long before = sample_dispatcher_total_dropped(d);
        sample_dispatcher_note_drop(NULL, 0);
        sample_dispatcher_note_drop(d, 0);
        TEST_ASSERT(sample_dispatcher_total_dropped(d) == before + 1u);
        TEST_ASSERT(sample_dispatcher_total_dropped(NULL) == 0ul);
    }

    sample_reader_destroy(&a);
    sample_reader_destroy(&b);
    sample_dispatcher_destroy(d);
    free(d);
}

/* Exhausted pool: non-blocking acquire returns NULL; guards hold. */
static void test_pool_exhausted_null(void)
{
    sample_dispatcher_t *d =
        (sample_dispatcher_t *)calloc(1, sizeof(*d));
    _Atomic unsigned int stop = 0u;
    TEST_ASSERT(d != NULL);
    TEST_ASSERT(sample_dispatcher_init(d) == 0);

    static sample_block_t *held[SAMPLE_DISPATCHER_BLOCK_CAPACITY];
    unsigned int nheld = 0u;
    for (unsigned int i = 0u; i < SAMPLE_DISPATCHER_BLOCK_CAPACITY; i++)
    {
        held[i] = sample_dispatcher_acquire_block(d);
        if (!held[i])
            break;
        nheld++;
    }
    TEST_ASSERT(nheld == SAMPLE_DISPATCHER_BLOCK_CAPACITY);
    TEST_ASSERT(sample_dispatcher_acquire_block(d) == NULL);
    TEST_ASSERT(sample_dispatcher_acquire_block(NULL) == NULL);
    TEST_ASSERT(sample_dispatcher_acquire_blocking(NULL, &stop) == NULL);
    TEST_ASSERT(sample_dispatcher_push_block(NULL, held[0]) == 0u);
    TEST_ASSERT(sample_dispatcher_push_block(d, NULL) == 0u);
    TEST_ASSERT(sample_dispatcher_push_blocking(NULL, held[0], &stop) == 0u);
    TEST_ASSERT(sample_dispatcher_push_blocking(d, NULL, &stop) == 0u);
    TEST_ASSERT(sample_reader_init(NULL, d) == -1);
    for (unsigned int i = 0u; i < nheld; i++)
        sample_block_release(held[i]);
    TEST_ASSERT(sample_dispatcher_all_free(d) != 0);
    TEST_ASSERT(sample_dispatcher_all_free(NULL) == 0);

    /* Shutdown-set pop on an empty reader escapes without a block. */
    {
        sample_reader_t r;
        sample_block_t *blk = (sample_block_t *)0x1;
        TEST_ASSERT(sample_reader_init(&r, d) == 0);
        atomic_store_explicit(&stop, 1u, memory_order_release);
        TEST_ASSERT(sample_reader_wait_pop(&r, &stop, &blk) != 0);
        atomic_store_explicit(&stop, 0u, memory_order_release);
        sample_reader_destroy(&r);
    }

    sample_dispatcher_destroy(d);
    free(d);
}

int main(void)
{
    /* NB: the dispatcher owns ~128 MB of blocks; heap-allocate it. */
    sample_dispatcher_t *dispatcher =
        (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
    sample_reader_t reader;
    _Atomic unsigned int stop = 0u;

    TEST_ASSERT(dispatcher != NULL);
    TEST_ASSERT(sample_dispatcher_init(dispatcher) == 0);

    /* No readers: vacuously true, and NULL-safe false. */
    TEST_ASSERT(sample_dispatcher_can_push(dispatcher) != 0);
    TEST_ASSERT(sample_dispatcher_can_push(NULL) == 0);

    TEST_ASSERT(sample_reader_init(&reader, dispatcher) == 0);
    TEST_ASSERT(sample_dispatcher_can_push(dispatcher) != 0);

    /* Fill the reader queue exactly to capacity. */
    for (unsigned int i = 0u; i < SAMPLE_READER_QUEUE_CAPACITY; i++)
    {
        sample_block_t *block =
            sample_dispatcher_acquire_block(dispatcher);
        TEST_ASSERT(block != NULL);
        block->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(dispatcher, block) == 1u);
        sample_block_release(block);
        if (i + 1u < SAMPLE_READER_QUEUE_CAPACITY)
            TEST_ASSERT(sample_dispatcher_can_push(dispatcher) != 0);
    }
    TEST_ASSERT(sample_dispatcher_can_push(dispatcher) == 0);

    /* Draining one block re-arms the query. */
    {
        sample_block_t *block = NULL;
        TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
        TEST_ASSERT(block != NULL);
        sample_block_release(block);
    }
    TEST_ASSERT(sample_dispatcher_can_push(dispatcher) != 0);

    /* Drain the rest; a fresh push must succeed without drops. */
    for (unsigned int i = 0u; i + 1u < SAMPLE_READER_QUEUE_CAPACITY; i++)
    {
        sample_block_t *block = NULL;
        TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
        sample_block_release(block);
    }
    {
        sample_block_t *block =
            sample_dispatcher_acquire_block(dispatcher);
        TEST_ASSERT(block != NULL);
        block->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(dispatcher, block) == 1u);
        sample_block_release(block);
        TEST_ASSERT(sample_dispatcher_total_dropped(dispatcher) == 0ul);
    }

    /* Blocking push waits for room instead of dropping: drain first, refill
     * the queue, then push from a thread (it must stall), pop once (it
     * completes). */
    while (reader_count(&reader) > 0u)
    {
        sample_block_t *block = NULL;
        TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
        sample_block_release(block);
    }
    TEST_ASSERT(sample_dispatcher_all_free(dispatcher) != 0);
    for (unsigned int i = 0u; i < SAMPLE_READER_QUEUE_CAPACITY; i++)
    {
        sample_block_t *block =
            sample_dispatcher_acquire_block(dispatcher);
        TEST_ASSERT(block != NULL);
        block->num_samples = 1u;
        TEST_ASSERT(sample_dispatcher_push_block(dispatcher, block) == 1u);
        sample_block_release(block);
    }
    {
        blocking_arg_t arg = {.dispatcher = dispatcher,
                              .stop = &stop,
                              .delivered = 999u};
        pthread_t th;
        sample_block_t *extra =
            sample_dispatcher_acquire_blocking(dispatcher, &stop);
        TEST_ASSERT(extra != NULL);
        extra->num_samples = 1u;
        arg.block = extra;
        TEST_ASSERT(pthread_create(&th, NULL, blocking_pusher, &arg) == 0);
        sleep_ms(50);
        /* Still stalled: queue untouched, nothing dropped. */
        TEST_ASSERT(sample_dispatcher_total_dropped(dispatcher) == 0ul);
        {
            sample_block_t *block = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
            sample_block_release(block);
        }
        pthread_join(th, NULL);
        TEST_ASSERT(arg.delivered == 1u);
        sample_block_release(extra);
        TEST_ASSERT(sample_dispatcher_total_dropped(dispatcher) == 0ul);
    }

    /* Blocking acquire escapes on shutdown once the pool is exhausted. */
    {
        static sample_block_t *held[SAMPLE_DISPATCHER_BLOCK_CAPACITY];
        unsigned int nheld = 0u;
        pthread_t th;
        blocking_arg_t arg;
        for (unsigned int i = 0u;
             i < SAMPLE_DISPATCHER_BLOCK_CAPACITY; i++)
        {
            /* Drain the reader first so pushes below never drop. */
            sample_block_t *q = NULL;
            while (reader_count(&reader) > 0u)
            {
                TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &q) == 0);
                sample_block_release(q);
            }
            held[i] = sample_dispatcher_acquire_block(dispatcher);
            if (!held[i])
                break;
            nheld++;
        }
        TEST_ASSERT(nheld == SAMPLE_DISPATCHER_BLOCK_CAPACITY);
        TEST_ASSERT(sample_dispatcher_all_free(dispatcher) == 0);
        arg.dispatcher = dispatcher;
        arg.stop = &stop;
        arg.block = NULL;
        arg.delivered = 999u;
        atomic_store_explicit(&stop, 0u, memory_order_release);
        TEST_ASSERT(pthread_create(&th, NULL, blocking_acquirer, &arg) == 0);
        sleep_ms(50);
        atomic_store_explicit(&stop, 1u, memory_order_release);
        pthread_join(th, NULL);
        TEST_ASSERT(arg.block == NULL); /* escaped, no block */
        atomic_store_explicit(&stop, 0u, memory_order_release);
        for (unsigned int i = 0u; i < nheld; i++)
            sample_block_release(held[i]);
        TEST_ASSERT(sample_dispatcher_all_free(dispatcher) != 0);
    }

    sample_reader_destroy(&reader);
    sample_dispatcher_destroy(dispatcher);
    free(dispatcher);

    test_multi_reader_fanout();
    test_pool_exhausted_null();

    if (g_failures)
    {
        fprintf(stderr, "test_dispatcher_backpressure: %d failure(s)\n",
                g_failures);
        return 1;
    }
    printf("test_dispatcher_backpressure: ok\n");
    return 0;
}
