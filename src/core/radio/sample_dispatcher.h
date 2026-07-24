#ifndef SAMPLE_DISPATCHER_H
#define SAMPLE_DISPATCHER_H

#include <complex.h>
#include <pthread.h>
#include <stdint.h>

#define SAMPLE_BLOCK_SAMPLE_CAPACITY 262144u
#define SAMPLE_READER_QUEUE_CAPACITY 8u
/* Worst-case fan-out: 20 BR/EDR channel workers + 10 BLE channel workers
 * in a hybrid session. */
#define SAMPLE_DISPATCHER_READER_CAPACITY 30u
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

int sample_reader_init(sample_reader_t *reader,
                       sample_dispatcher_t *dispatcher);
void sample_reader_destroy(sample_reader_t *reader);
void sample_reader_signal(sample_reader_t *reader);
int sample_reader_wait_pop(sample_reader_t *reader,
                           const unsigned int *shutdown_requested,
                           sample_block_t **block);

#endif