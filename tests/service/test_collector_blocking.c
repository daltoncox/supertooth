/**
 * @file test_collector_blocking.c
 * @brief Tests for collector blocking submit (exhaustive replay).
 *
 * Verifies: overwrite-oldest still drops in default mode; blocking mode
 * waits for room instead of dropping; a blocked submit escapes on shutdown.
 */

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "collector.h"

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
    collector_t *c;
    int value;
    int result;
} submit_arg_t;

static void *submitter(void *arg)
{
    submit_arg_t *a = (submit_arg_t *)arg;
    a->result = collector_submit(a->c, &a->value);
    return NULL;
}

static void sleep_ms(long ms)
{
    struct timespec ts = {.tv_sec = ms / 1000L,
                          .tv_nsec = (ms % 1000L) * 1000000L};
    nanosleep(&ts, NULL);
}

int main(void)
{
    _Atomic unsigned int shutdown = 0u;

    /* Default mode: overfill overwrites oldest and counts drops. */
    {
        collector_t c;
        int v;
        TEST_ASSERT(collector_init(&c, sizeof(int), 4u, &shutdown) == 0);
        for (int i = 0; i < 4; i++)
            TEST_ASSERT(collector_submit(&c, &i) == 0);
        v = 99;
        TEST_ASSERT(collector_submit(&c, &v) == 0);
        TEST_ASSERT(collector_dropped(&c) == 1ul);
        TEST_ASSERT(collector_count(&c) == 4u);
        /* Oldest (0) was discarded; head now holds 1,2,3,99. */
        TEST_ASSERT(collector_pop(&c, &v) == 0 && v == 1);
        collector_destroy(&c);
    }

    /* Blocking mode: a submit into a full queue waits; a pop releases it
     * with nothing dropped. */
    {
        collector_t c;
        pthread_t th;
        submit_arg_t arg;
        int v = -1;
        TEST_ASSERT(collector_init(&c, sizeof(int), 4u, &shutdown) == 0);
        collector_set_blocking(&c, 1);
        for (int i = 0; i < 4; i++)
            TEST_ASSERT(collector_submit(&c, &i) == 0);
        arg.c = &c;
        arg.value = 99;
        arg.result = -99;
        TEST_ASSERT(pthread_create(&th, NULL, submitter, &arg) == 0);
        sleep_ms(50);
        /* Still blocked: nothing consumed, nothing dropped. */
        TEST_ASSERT(collector_count(&c) == 4u);
        TEST_ASSERT(collector_dropped(&c) == 0ul);
        TEST_ASSERT(collector_pop(&c, &v) == 0 && v == 0);
        pthread_join(th, NULL);
        TEST_ASSERT(arg.result == 0);
        TEST_ASSERT(collector_dropped(&c) == 0ul);
        TEST_ASSERT(collector_count(&c) == 4u);
        collector_destroy(&c);
    }

    /* A blocked submit escapes on shutdown. */
    {
        collector_t c;
        pthread_t th;
        submit_arg_t arg;
        TEST_ASSERT(collector_init(&c, sizeof(int), 2u, &shutdown) == 0);
        collector_set_blocking(&c, 1);
        {
            int i;
            for (i = 0; i < 2; i++)
                TEST_ASSERT(collector_submit(&c, &i) == 0);
        }
        atomic_store_explicit(&shutdown, 0u, memory_order_release);
        arg.c = &c;
        arg.value = 7;
        arg.result = -99;
        TEST_ASSERT(pthread_create(&th, NULL, submitter, &arg) == 0);
        sleep_ms(50);
        atomic_store_explicit(&shutdown, 1u, memory_order_release);
        collector_wake(&c);
        pthread_join(th, NULL);
        TEST_ASSERT(arg.result == -1);
        atomic_store_explicit(&shutdown, 0u, memory_order_release);
        collector_destroy(&c);
    }

    if (g_failures)
    {
        fprintf(stderr, "test_collector_blocking: %d failure(s)\n",
                g_failures);
        return 1;
    }
    printf("test_collector_blocking: ok\n");
    return 0;
}
