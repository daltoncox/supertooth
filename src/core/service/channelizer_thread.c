/**
 * @file service/channelizer_thread.c
 * @brief See channelizer_thread.h.
 */

#include "channelizer_thread.h"

#include <string.h>

int channelizer_init(channelizer_t *c,
                     sample_dispatcher_t *rf,
                     sample_dispatcher_t *out,
                     unsigned int sample_rate_hz,
                     uint32_t lo_hz,
                     uint32_t grid_hz,
                     int debug)
{
    if (!c || !rf || !out)
        return -1;
    memset(c, 0, sizeof(*c));

    c->out   = out;
    c->debug = debug;

    if (channelizer_bank_init(&c->bank, sample_rate_hz, lo_hz, grid_hz,
                              CHANNELIZER_BANK_DEFAULT_M,
                              CHANNELIZER_BANK_DEFAULT_AS) != 0)
        return -1;

    if (sample_reader_init(&c->rf_reader, rf) != 0)
    {
        channelizer_bank_destroy(&c->bank);
        return -1;
    }

    c->active = 1;
    return 0;
}

void channelizer_destroy(channelizer_t *c)
{
    if (!c)
        return;
    sample_reader_destroy(&c->rf_reader);
    channelizer_bank_destroy(&c->bank);
    memset(c, 0, sizeof(*c));
}

void *channelizer_worker(void *arg)
{
    channelizer_t *c = (channelizer_t *)arg;
    if (!c || !c->active)
        return NULL;

    const unsigned int M  = c->bank.M;
    const unsigned int M2 = c->bank.M2;
    const size_t CAP = SAMPLE_BLOCK_SAMPLE_CAPACITY;

    size_t max_frames = (CAP / (size_t)M);
    if (max_frames > 2u)
        max_frames -= 2u;
    else
        max_frames = 1u;
    size_t max_rf = max_frames * (size_t)M2;

    sample_block_t *rf = NULL;

    for (;;)
    {
        if (sample_reader_wait_pop(&c->rf_reader, c->shutdown, &rf) != 0)
            break;
        if (!rf)
            continue;

        size_t off = 0u;
        while (off < rf->num_samples)
        {
            size_t n = rf->num_samples - off;
            if (n > max_rf)
                n = max_rf;

            sample_block_t *fm = sample_dispatcher_acquire_block(c->out);
            if (!fm)
            {
                sample_dispatcher_note_drop(c->out, c->debug);
                break; /* output pool exhausted: drop the rest of this RF block */
            }

            unsigned int frames_out = 0u;
            channelizer_bank_execute(&c->bank, &rf->samples[off], n,
                                     fm->samples, &frames_out, NULL);
            fm->num_samples        = (unsigned int)((size_t)M * frames_out);
            fm->block_base_sample  = rf->block_base_sample + (uint64_t)off;

            sample_dispatcher_push_block(c->out, fm);
            sample_block_release(fm);
            fm = NULL;

            off += n;
        }

        sample_block_release(rf);
        rf = NULL;
    }

    return NULL;
}
