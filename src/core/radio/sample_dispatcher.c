#include "sample_dispatcher.h"

#include <stddef.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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