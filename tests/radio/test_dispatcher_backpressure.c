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

    if (g_failures)
    {
        fprintf(stderr, "test_dispatcher_backpressure: %d failure(s)\n",
                g_failures);
        return 1;
    }
    printf("test_dispatcher_backpressure: ok\n");
    return 0;
}
