#include "sample_dispatcher.h"

#include <stddef.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void sample_block_acquire(sample_block_t *block)
{
    if (!block)
        return;

    __atomic_add_fetch(&block->refcount, 1u, __ATOMIC_ACQ_REL);
}

void sample_block_release(sample_block_t *block)
{
    if (!block)
        return;

    __atomic_sub_fetch(&block->refcount, 1u, __ATOMIC_ACQ_REL);
}

static int sample_block_is_free(const sample_block_t *block)
{
    if (!block)
        return 0;

    return __atomic_load_n(&block->refcount, __ATOMIC_ACQUIRE) == 0u;
}

static int sample_reader_queue_init(sample_reader_t *reader)
{
    if (!reader)
        return -1;

    memset(reader, 0, sizeof(*reader));
    if (pthread_mutex_init(&reader->mutex, NULL) != 0)
        return -1;
    if (pthread_cond_init(&reader->cv, NULL) != 0)
    {
        pthread_mutex_destroy(&reader->mutex);
        return -1;
    }
    return 0;
}

void sample_reader_destroy(sample_reader_t *reader)
{
    if (!reader)
        return;

    sample_block_release(reader->view_held);
    reader->view_held = NULL;
    free(reader->view_scratch);
    reader->view_scratch = NULL;
    reader->view_cap = 0u;

    pthread_cond_destroy(&reader->cv);
    pthread_mutex_destroy(&reader->mutex);
    memset(reader, 0, sizeof(*reader));
}

void sample_reader_signal(sample_reader_t *reader)
{
    if (!reader)
        return;

    pthread_mutex_lock(&reader->mutex);
    pthread_cond_signal(&reader->cv);
    pthread_mutex_unlock(&reader->mutex);
}

int sample_reader_wait_pop(sample_reader_t *reader,
                            const _Atomic unsigned int *shutdown_requested,
                            sample_block_t **block)
{
    if (!reader || !shutdown_requested || !block)
        return -1;

    pthread_mutex_lock(&reader->mutex);
    while (atomic_load_explicit(shutdown_requested, memory_order_acquire) == 0u &&
           reader->count == 0u)
        pthread_cond_wait(&reader->cv, &reader->mutex);
    if (atomic_load_explicit(shutdown_requested, memory_order_acquire) != 0u)
    {
        pthread_mutex_unlock(&reader->mutex);
        return -1;
    }

    *block = reader->ring[reader->read_idx];
    reader->ring[reader->read_idx] = NULL;
    reader->read_idx = (reader->read_idx + 1u) % SAMPLE_READER_QUEUE_CAPACITY;
    reader->count--;
    pthread_mutex_unlock(&reader->mutex);
    return 0;
}

static int sample_reader_queue_try_push(sample_reader_t *reader, sample_block_t *block)
{
    int result = 0;

    if (!reader || !block)
        return -1;

    pthread_mutex_lock(&reader->mutex);
    if (reader->count == SAMPLE_READER_QUEUE_CAPACITY)
        result = 1;
    else
    {
        reader->ring[reader->write_idx] = block;
        reader->write_idx = (reader->write_idx + 1u) % SAMPLE_READER_QUEUE_CAPACITY;
        reader->count++;
        pthread_cond_signal(&reader->cv);
    }
    pthread_mutex_unlock(&reader->mutex);
    return result;
}

static int sample_dispatcher_add_reader(sample_dispatcher_t *dispatcher,
                                        sample_reader_t *reader)
{
    if (!dispatcher || !reader || dispatcher->reader_count == SAMPLE_DISPATCHER_READER_CAPACITY)
        return -1;

    dispatcher->readers[dispatcher->reader_count++] = reader;
    return 0;
}

int sample_dispatcher_init(sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return -1;

    sample_dispatcher_reset(dispatcher);
    return 0;
}

void sample_dispatcher_destroy(sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return;

    sample_dispatcher_reset(dispatcher);
}

int sample_reader_init(sample_reader_t *reader,
                       sample_dispatcher_t *dispatcher)
{
    if (!reader || !dispatcher)
        return -1;

    if (sample_reader_queue_init(reader) != 0)
        return -1;

    if (sample_dispatcher_add_reader(dispatcher, reader) != 0)
    {
        sample_reader_destroy(reader);
        return -1;
    }

    return 0;
}

int sample_reader_configure_view(sample_reader_t *reader,
                                 unsigned int bin, unsigned int M,
                                 unsigned int stride,
                                 unsigned int decimation,
                                 unsigned int rate_hz,
                                 uint32_t center_hz,
                                 float rssi_cal_db)
{
    float complex *scratch;
    size_t cap;

    if (!reader || M == 0u || stride == 0u || bin >= M ||
        decimation == 0u || rate_hz == 0u)
        return -1;
    if (reader->view_stride != 0u || reader->view_scratch != NULL)
        return -1; /* single configuration per reader lifetime */

    cap = SAMPLE_BLOCK_SAMPLE_CAPACITY / (size_t)M / (size_t)stride + 16u;
    scratch = (float complex *)malloc(cap * sizeof(*scratch));
    if (!scratch)
        return -1;

    reader->view_bin        = bin;
    reader->view_M          = M;
    reader->view_stride     = stride;
    reader->view_decimation = decimation;
    reader->view_rate_hz    = rate_hz;
    reader->view_center_hz  = center_hz;
    reader->view_rssi_cal_db = rssi_cal_db;
    reader->view_scratch    = scratch;
    reader->view_cap        = cap;
    return 0;
}

int sample_reader_next(sample_reader_t *reader,
                       const _Atomic unsigned int *shutdown,
                       const float complex **out_samples,
                       unsigned int *out_count,
                       uint64_t *out_base_radio)
{
    sample_block_t *block = NULL;
    unsigned int frames, out = 0u;

    if (!reader || !shutdown || !out_samples || !out_count || !out_base_radio)
        return -1;

    sample_block_release(reader->view_held);
    reader->view_held = NULL;

    if (sample_reader_wait_pop(reader, shutdown, &block) != 0 || !block)
        return -1;
    reader->view_held = block;

    *out_base_radio = block->block_base_sample;
    if (reader->view_stride == 0u)
    {
        /* Raw mode: pool block handed out directly, zero copy. */
        *out_samples = block->samples;
        *out_count   = block->num_samples;
        return 0;
    }

    frames = block->num_samples / reader->view_M;
    for (unsigned int k = 0u;
         k < frames && out < reader->view_cap;
         k += reader->view_stride)
        reader->view_scratch[out++] =
            block->samples[reader->view_bin + (size_t)k * reader->view_M];
    *out_samples = reader->view_scratch;
    *out_count   = out;
    return 0;
}

void sample_dispatcher_reset(sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return;

    dispatcher->next_block_idx = 0u;
    dispatcher->reader_count = 0u;
    dispatcher->dropped_blocks = 0ul;
    memset(dispatcher->blocks, 0, sizeof(dispatcher->blocks));
    memset(dispatcher->readers, 0, sizeof(dispatcher->readers));
}

void sample_dispatcher_note_drop(sample_dispatcher_t *dispatcher, int debug_enabled)
{
    if (!dispatcher)
        return;

    dispatcher->dropped_blocks++;
    if (debug_enabled)
        fprintf(stderr, "[debug] dropped callback block: block pool exhausted (%u)\n",
                SAMPLE_DISPATCHER_BLOCK_CAPACITY);
}

sample_block_t *sample_dispatcher_acquire_block(sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return NULL;

    for (unsigned int i = 0; i < SAMPLE_DISPATCHER_BLOCK_CAPACITY; i++)
    {
        unsigned int idx = (dispatcher->next_block_idx + i) % SAMPLE_DISPATCHER_BLOCK_CAPACITY;
        sample_block_t *block = &dispatcher->blocks[idx];
        if (sample_block_is_free(block))
        {
            dispatcher->next_block_idx = (idx + 1u) % SAMPLE_DISPATCHER_BLOCK_CAPACITY;
            sample_block_acquire(block);
            return block;
        }
    }

    return NULL;
}

unsigned int sample_dispatcher_push_block(sample_dispatcher_t *dispatcher,
                                            sample_block_t *block)
{
    unsigned int delivered = 0u;

    if (!dispatcher || !block)
        return 0u;

    static unsigned long push_dbg = 0;
    if (dispatcher->reader_count == 0u && (push_dbg++ % 500u) == 0u)
        fprintf(stderr,
                "[dispatcher] push with reader_count=0 (block leaked!)\n");

    for (unsigned int i = 0; i < dispatcher->reader_count; i++)
    {
        sample_reader_t *reader = dispatcher->readers[i];
        sample_block_acquire(block);
        if (sample_reader_queue_try_push(reader, block) == 0)
        {
            delivered++;
            continue;
        }

        sample_block_release(block);
        reader->dropped_blocks++;
    }

    return delivered;
}

int sample_dispatcher_can_push(sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return 0;

    for (unsigned int i = 0u; i < dispatcher->reader_count; i++)
    {
        sample_reader_t *reader = dispatcher->readers[i];
        int full;

        pthread_mutex_lock(&reader->mutex);
        full = (reader->count == SAMPLE_READER_QUEUE_CAPACITY);
        pthread_mutex_unlock(&reader->mutex);

        if (full)
            return 0;
    }

    return 1;
}

/* Poll quantum for the blocking backpressure helpers: short enough to react
 * promptly to freed blocks / drained queues and shutdown, long enough to
 * stay out of the way. Timed with nanosleep (a duration, not an absolute
 * timestamp). */
#define SAMPLE_DISPATCHER_BACKPRESSURE_NS 1000000L

static void sample_dispatcher_wait_quantum(const _Atomic unsigned int *shutdown)
{
    struct timespec ts = {.tv_sec = 0,
                          .tv_nsec = SAMPLE_DISPATCHER_BACKPRESSURE_NS};
    (void)shutdown;
    nanosleep(&ts, NULL);
}

static int sample_dispatcher_shutdown_set(const _Atomic unsigned int *shutdown)
{
    if (!shutdown)
        return 0;
    return atomic_load_explicit(shutdown, memory_order_acquire) != 0u;
}

sample_block_t *sample_dispatcher_acquire_blocking(
    sample_dispatcher_t *dispatcher,
    const _Atomic unsigned int *shutdown)
{
    if (!dispatcher)
        return NULL;

    for (;;)
    {
        sample_block_t *block = sample_dispatcher_acquire_block(dispatcher);
        if (block)
            return block;
        if (sample_dispatcher_shutdown_set(shutdown))
            return NULL;
        sample_dispatcher_wait_quantum(shutdown);
    }
}

unsigned int sample_dispatcher_push_blocking(
    sample_dispatcher_t *dispatcher,
    sample_block_t *block,
    const _Atomic unsigned int *shutdown)
{
    if (!dispatcher || !block)
        return 0u;

    for (;;)
    {
        if (sample_dispatcher_can_push(dispatcher))
            return sample_dispatcher_push_block(dispatcher, block);
        if (sample_dispatcher_shutdown_set(shutdown))
            return 0u;
        sample_dispatcher_wait_quantum(shutdown);
    }
}

int sample_dispatcher_all_free(const sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return 0;

    for (unsigned int i = 0u; i < SAMPLE_DISPATCHER_BLOCK_CAPACITY; i++)
    {
        if (!sample_block_is_free(&dispatcher->blocks[i]))
            return 0;
    }
    return 1;
}

unsigned long sample_dispatcher_total_dropped(const sample_dispatcher_t *dispatcher)
{
    if (!dispatcher)
        return 0ul;

    unsigned long total = dispatcher->dropped_blocks;
    for (unsigned int i = 0u; i < dispatcher->reader_count; i++)
        total += dispatcher->readers[i]->dropped_blocks;
    return total;
}