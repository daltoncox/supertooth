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
    c->max_rf = max_frames * (size_t)c->bank.M2;

    c->rf_carry = malloc(c->max_rf * sizeof(float complex));
    c->scratch  = malloc((CAP + c->max_rf) * sizeof(float complex));
    if (!c->rf_carry || !c->scratch)
    {
        channelizer_destroy(c);
        return -1;
    }
    c->rf_carry_len  = 0u;
    c->rf_carry_base = 0u;

    c->active = 1;
    return 0;
}

void channelizer_destroy(channelizer_t *c)
{
    if (!c)
        return;
    sample_reader_destroy(&c->rf_reader);
    channelizer_bank_destroy(&c->bank);
    free(c->rf_carry);
    free(c->scratch);
    memset(c, 0, sizeof(*c));
}

void *channelizer_worker(void *arg)
{
    channelizer_t *c = (channelizer_t *)arg;
    if (!c || !c->active)
        return NULL;

    const unsigned int M = c->bank.M;
    const size_t max_rf = c->max_rf;

    sample_block_t *rf = NULL;

    for (;;)
    {
        if (sample_reader_wait_pop(&c->rf_reader, c->shutdown, &rf) != 0)
            break;
        if (!rf)
            continue;

        size_t combined_len = c->rf_carry_len + (size_t)rf->num_samples;
        uint64_t combined_base = c->rf_carry_len ? c->rf_carry_base
                                                 : rf->block_base_sample;

        if (c->rf_carry_len)
            memcpy(c->scratch, c->rf_carry,
                   c->rf_carry_len * sizeof(float complex));
        memcpy(c->scratch + c->rf_carry_len, rf->samples,
               (size_t)rf->num_samples * sizeof(float complex));

        size_t off = 0u;
        int dropped = 0;
        while (combined_len - off >= max_rf)
        {
            sample_block_t *fm = sample_dispatcher_acquire_block(c->out);
            if (!fm)
            {
                sample_dispatcher_note_drop(c->out, c->debug);
                dropped = 1; /* backpressure: the loop below discards the
                                unconsumed tail instead of letting rf_carry
                                grow past max_rf and overflow the heap. */
                break;
            }

            unsigned int frames_out = 0u;
            channelizer_bank_execute(&c->bank, &c->scratch[off], max_rf,
                                     fm->samples, &frames_out, NULL);
            fm->num_samples        = (unsigned int)((size_t)M * frames_out);
            fm->block_base_sample  = combined_base + (uint64_t)off;

            sample_dispatcher_push_block(c->out, fm);
            sample_block_release(fm);
            fm = NULL;

            off += max_rf;
        }

        if (dropped)
        {
            /* Backpressure: discard the entire unconsumed input rather than
               let rf_carry grow past max_rf and overflow. */
            c->rf_carry_len = 0u;
        }
        else
        {
            size_t remainder = combined_len - off; /* always < max_rf here */
            if (remainder > 0u)
            {
                memcpy(c->rf_carry, &c->scratch[off],
                       remainder * sizeof(float complex));
                c->rf_carry_len  = remainder;
                c->rf_carry_base = combined_base + (uint64_t)off;
            }
            else
            {
                c->rf_carry_len = 0u;
            }
        }

        sample_block_release(rf);
        rf = NULL;
    }

    return NULL;
}
