#ifndef COLLECTOR_H
#define COLLECTOR_H

#include <stddef.h>
#include <stdatomic.h>
#include <pthread.h>

/**
 * @file collector.h
 * @brief Bounded multi-producer / single-consumer event queue.
 *
 * Channel-processor workers (many producers) copy a decoded event into the
 * queue and return immediately, so the heavy tracking + presentation work is
 * performed by exactly one consumer thread per protocol. This decouples the
 * per-channel workers from the shared block pool (they release their sample
 * block right after submitting) and removes cross-protocol contention on the
 * tracker, which previously collapsed N parallel workers onto a single mutex.
 *
 * The queue is bounded and uses an overwrite-oldest policy on overflow so the
 * newest events (the ones the tracker prefers) survive; a `dropped` counter
 * records how many oldest entries were discarded.
 */
typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    size_t item_size;
    size_t capacity;
    unsigned char *buf;   /* capacity * item_size ring buffer */
    size_t head;          /* index of next write */
    size_t tail;          /* index of oldest live item */
    size_t count;
    const _Atomic unsigned int *shutdown;
    unsigned long dropped;
} collector_t;

int  collector_init(collector_t *c, size_t item_size, size_t capacity,
                   const _Atomic unsigned int *shutdown);
void collector_destroy(collector_t *c);

/* Multi-producer. Copies @p item (item_size bytes) into the ring. On overflow
 * the oldest entry is discarded (overwrite-oldest) and dropped is incremented.
 * Returns 0 on success, -1 on bad arguments. */
int  collector_submit(collector_t *c, const void *item);

/* Single-consumer. Blocks until an item is available or @p shutdown is set.
 * Returns 0 and fills @p out_item on success, -1 when shutdown is requested and
 * the queue is empty (caller should exit its loop). */
int  collector_pop(collector_t *c, void *out_item);

/* Wake a consumer blocked in collector_pop (e.g. on shutdown). */
void collector_wake(collector_t *c);

#endif /* COLLECTOR_H */
