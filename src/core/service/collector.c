/**
 * @file service/collector.c
 * @brief See collector.h.
 */

#include "collector.h"

#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>

int collector_init(collector_t *c, size_t item_size, size_t capacity,
                   const _Atomic unsigned int *shutdown)
{
    if (!c || item_size == 0u || capacity == 0u || !shutdown)
        return -1;
    memset(c, 0, sizeof(*c));

    c->buf = (unsigned char *)malloc(item_size * capacity);
    if (!c->buf)
        return -1;

    if (pthread_mutex_init(&c->mutex, NULL) != 0)
    {
        free(c->buf);
        c->buf = NULL;
        return -1;
    }
    if (pthread_cond_init(&c->cond, NULL) != 0)
    {
        pthread_mutex_destroy(&c->mutex);
        free(c->buf);
        c->buf = NULL;
        return -1;
    }

    c->item_size = item_size;
    c->capacity  = capacity;
    c->shutdown  = shutdown;
    c->head = 0u;
    c->tail = 0u;
    c->count = 0u;
    c->dropped = 0u;
    return 0;
}

void collector_destroy(collector_t *c)
{
    if (!c)
        return;
    pthread_mutex_destroy(&c->mutex);
    pthread_cond_destroy(&c->cond);
    free(c->buf);
    c->buf = NULL;
}

int collector_submit(collector_t *c, const void *item)
{
    if (!c || !item)
        return -1;

    pthread_mutex_lock(&c->mutex);
    if (c->count == c->capacity)
    {
        /* Overwrite-oldest: drop the tail entry to make room. */
        c->tail = (c->tail + 1u) % c->capacity;
        c->count--;
        c->dropped++;
    }
    memcpy(c->buf + c->head * c->item_size, item, c->item_size);
    c->head = (c->head + 1u) % c->capacity;
    c->count++;
    pthread_cond_signal(&c->cond);
    pthread_mutex_unlock(&c->mutex);
    return 0;
}

int collector_pop(collector_t *c, void *out_item)
{
    if (!c || !out_item)
        return -1;

    pthread_mutex_lock(&c->mutex);
    while (c->count == 0u)
    {
        if (atomic_load_explicit(c->shutdown, memory_order_acquire) != 0u)
        {
            pthread_mutex_unlock(&c->mutex);
            return -1;
        }
        pthread_cond_wait(&c->cond, &c->mutex);
    }
    memcpy(out_item, c->buf + c->tail * c->item_size, c->item_size);
    c->tail = (c->tail + 1u) % c->capacity;
    c->count--;
    pthread_mutex_unlock(&c->mutex);
    return 0;
}

void collector_wake(collector_t *c)
{
    if (!c)
        return;
    pthread_mutex_lock(&c->mutex);
    pthread_cond_broadcast(&c->cond);
    pthread_mutex_unlock(&c->mutex);
}
