/**
 * @file test_dispatcher_backpressure.c
 * @brief Tests for sample_dispatcher_can_push().
 *
 * Verifies the query used by exhaustive file replay for backpressure: true
 * when every reader has room, false as soon as any reader's queue is full,
 * true again after a pop drains it.
 */

#include <stdio.h>
#include <stdlib.h>

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
