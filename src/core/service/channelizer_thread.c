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

    const unsigned int M  = c->bank.M;
    const size_t CAP = SAMPLE_BLOCK_SAMPLE_CAPACITY;
    size_t max_frames = (CAP / (size_t)M);
    if (max_frames > 2u)
        max_frames -= 2u;
    else
        max_frames = 1u;
    max_frames = (max_frames / 4u) * 4u;
    if (max_frames < 4u)
        max_frames = 4u;
    c->max_in = max_frames * (size_t)c->bank.M2;

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

    const unsigned int M = c->bank.M;
    const size_t max_in = c->max_in;

    sample_block_t *rf = NULL;

    for (;;)
    {
        if (sample_reader_wait_pop(&c->rf_reader, c->shutdown, &rf) != 0)
            break;
        if (!rf)
            continue;

        /* Feed the raw RF block to the bank in place, in max_in-sized sub
         * chunks.  No worker-side copy: rf->samples is read directly and the
         * bank's own internal carry bridges sub-chunk boundaries so frame
         * alignment is preserved. */
        uint64_t base = rf->block_base_sample;
        size_t done = 0u;
        while (done < (size_t)rf->num_samples)
        {
            size_t n = (size_t)rf->num_samples - done;
            if (n > max_in)
                n = max_in;

            sample_block_t *fm = sample_dispatcher_acquire_block(c->out);
            if (!fm)
            {
                sample_dispatcher_note_drop(c->out, c->debug);
                break; /* backpressure: drop the rest of this RF block */
            }

            unsigned int frames_out = 0u;
            channelizer_bank_execute(&c->bank, &rf->samples[done], n,
                                     fm->samples, &frames_out, NULL);
            fm->num_samples        = (unsigned int)((size_t)M * frames_out);
            fm->block_base_sample  = base + (uint64_t)done;

            sample_dispatcher_push_block(c->out, fm);
            sample_block_release(fm);
            fm = NULL;

            done += n;
        }

        sample_block_release(rf);
        rf = NULL;
    }

    return NULL;
}
