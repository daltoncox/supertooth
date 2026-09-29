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
/* Worst-case fan-out: 79 BR/EDR channel workers + 40 BLE channel workers
 * sharing partitioned output dispatchers in a full-span session, plus up to
 * 4 DDC readers broadcasting on the RF dispatcher. Headroom above the
 * maximum so a future channel never fails init silently. */
#define SAMPLE_DISPATCHER_READER_CAPACITY 64u
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

    /* Optional channel view. view_stride == 0 means raw mode: next()
     * hands out pool blocks directly (zero copy). Otherwise next() gathers
     * blk[view_bin + k*view_M], stepping k by view_stride, into
     * view_scratch (exactly one copy, owned here). */
    unsigned int  view_bin;        /* element offset of this stream in a frame */
    unsigned int  view_M;          /* elements per frame */
    unsigned int  view_stride;     /* frames skipped per output sample; 0 = raw */
    unsigned int  view_decimation; /* end-to-end RF->demod decimation */
    unsigned int  view_rate_hz;    /* post-gather stream rate (2 Msps) */
    uint32_t      view_center_hz;  /* RF centre of this stream */
    float         view_rssi_cal_db;
    float complex *view_scratch;   /* gather buffer (view mode only) */
    size_t         view_cap;
    sample_block_t *view_held;     /* checked-out block, auto-released */
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

/**
 * Describe an already-registered reader as one strided stream (plain
 * scalars, so no service types cross into the radio layer). Copies the
 * view fields and sizes view_scratch = BLOCK_CAP/M/stride + 16.
 * Fails (-1, reader left raw) on bad args (NULL, M==0, stride==0,
 * bin>=M, decimation==0, rate==0) or double-configure.
 */
int sample_reader_configure_view(sample_reader_t *reader,
                                 unsigned int bin, unsigned int M,
                                 unsigned int stride,
                                 unsigned int decimation,
                                 unsigned int rate_hz,
                                 uint32_t center_hz,
                                 float rssi_cal_db);

/**
 * The single stream function every dispatcher consumer calls. Releases the
 * previously held block, wait_pops the next one and holds it (nonzero
 * return on shutdown/bad args; any held block is kept for destroy).
 *
 * Raw mode returns pool pointers directly (zero copy); view mode gathers
 * into the reader-owned scratch (one copy). Empty blocks are returned
 * as-is (count 0, caller continues). Output is valid until the next call
 * or destroy; single-thread owner only.
 */
int sample_reader_next(sample_reader_t *reader,
                       const _Atomic unsigned int *shutdown,
                       const float complex **out_samples,
                       unsigned int *out_count,
                       uint64_t *out_base_radio);

#endif