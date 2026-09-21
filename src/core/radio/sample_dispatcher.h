#ifndef SAMPLE_DISPATCHER_H
#define SAMPLE_DISPATCHER_H

#include <complex.h>
#include <pthread.h>
#include <stdint.h>

#define SAMPLE_BLOCK_SAMPLE_CAPACITY 262144u
/* RF-producer chunk size: every radio backend (HackRF, file replay, and any
 * future radio) must push RF input in blocks of at most this many samples.
 * Under overload the RF queues stay full and loss degenerates into
 * alternating survive/hole runs of one RF block; 131072-sample (6.5 ms)
 * holes swallow whole multi-slot packets *and* their ARQ retries together
 * while 65536-sample (3.3 ms) holes give retries an independent chance in
 * the next survivor run (measured ~3.5x better BR/EDR header decode on
 * identical samples). Keep producers uniform so no backend degrades worse
 * than the others. Must stay <= SAMPLE_BLOCK_SAMPLE_CAPACITY. */
#define SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES 65536u
#define SAMPLE_READER_QUEUE_CAPACITY 8u
/* Worst-case fan-out: 20 BR/EDR channel workers + 10 BLE channel workers
 * sharing one output dispatcher in a hybrid session. Headroom above the
 * 30-reader maximum so a future channel never fails init silently. */
#define SAMPLE_DISPATCHER_READER_CAPACITY 32u
#define SAMPLE_DISPATCHER_BLOCK_CAPACITY 64u

typedef struct sample_block
{
    float complex samples[SAMPLE_BLOCK_SAMPLE_CAPACITY];
    unsigned int num_samples;
    uint64_t block_base_sample;
    unsigned int refcount;
} sample_block_t;

typedef struct
{
    sample_block_t *ring[SAMPLE_READER_QUEUE_CAPACITY];
    unsigned int write_idx;
    unsigned int read_idx;
    unsigned int count;
    pthread_mutex_t mutex;
    pthread_cond_t cv;
    unsigned long dropped_blocks;
} sample_reader_t;

typedef struct
{
    sample_block_t blocks[SAMPLE_DISPATCHER_BLOCK_CAPACITY];
    unsigned int next_block_idx;
    sample_reader_t *readers[SAMPLE_DISPATCHER_READER_CAPACITY];
    unsigned int reader_count;
    unsigned long dropped_blocks;
} sample_dispatcher_t;

void sample_block_release(sample_block_t *block);

int sample_dispatcher_init(sample_dispatcher_t *dispatcher);
void sample_dispatcher_destroy(sample_dispatcher_t *dispatcher);
void sample_dispatcher_reset(sample_dispatcher_t *dispatcher);
void sample_dispatcher_note_drop(sample_dispatcher_t *dispatcher, int debug_enabled);
sample_block_t *sample_dispatcher_acquire_block(sample_dispatcher_t *dispatcher);
unsigned int sample_dispatcher_push_block(sample_dispatcher_t *dispatcher,
                                           sample_block_t *block);

/**
 * 1 when every reader's queue currently has room for another block, 0 when
 * any reader is full (a push now would drop for that reader) or @p
 * dispatcher is NULL. Sole-producer sources (file replay) use this to apply
 * backpressure instead of outrunning consumers; with a single producer,
 * check-then-push is race-free because only our own push can fill a queue
 * observed non-full.
 */
int sample_dispatcher_can_push(sample_dispatcher_t *dispatcher);

/**
 * Blocking acquire for backpressure paths (exhaustive replay): waits until a
 * pool block is free or @p shutdown is set. Returns the block (with one ref
 * held, as acquire_block) or NULL on shutdown (also NULL on bad arguments).
 * Poll-based so the lock-free fast path is untouched.
 */
sample_block_t *sample_dispatcher_acquire_blocking(
    sample_dispatcher_t *dispatcher,
    const _Atomic unsigned int *shutdown);

/**
 * Blocking push for backpressure paths: waits until every reader's queue has
 * room (or @p shutdown is set), then delivers to all readers. Returns the
 * number of readers delivered to, or 0 on shutdown/bad arguments. With a
 * single producer the wait-then-push is race-free.
 */
unsigned int sample_dispatcher_push_blocking(
    sample_dispatcher_t *dispatcher,
    sample_block_t *block,
    const _Atomic unsigned int *shutdown);

/**
 * 1 when every pool block is unreferenced (and therefore no reader queue can
 * hold a block either), 0 otherwise. Used to detect pipeline quiescence at
 * end of exhaustive replay.
 */
int sample_dispatcher_all_free(const sample_dispatcher_t *dispatcher);

/**
 * Total dropped blocks across this dispatcher: blocks the pool could not
 * acquire plus blocks every reader's queue rejected because it was full.
 * Both are real drops the capture loop could not keep up with.
 */
unsigned long sample_dispatcher_total_dropped(const sample_dispatcher_t *dispatcher);

int sample_reader_init(sample_reader_t *reader,
                       sample_dispatcher_t *dispatcher);
void sample_reader_destroy(sample_reader_t *reader);
void sample_reader_signal(sample_reader_t *reader);
int sample_reader_wait_pop(sample_reader_t *reader,
                            const _Atomic unsigned int *shutdown_requested,
                            sample_block_t **block);

#endif