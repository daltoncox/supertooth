/**
 * @file dsp/ddc_stage.c
 * @brief See ddc_stage.h.
 */

#include "ddc_stage.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

int ddc_stage_init(ddc_stage_t *q,
                    unsigned int sample_rate_in,
                    uint32_t lo_hz,
                    uint32_t sub_center_hz,
                    unsigned int decim,
                    unsigned int m,
                    float as)
{
    if (!q || sample_rate_in == 0u || decim < 1u ||
        decim > DDC_STAGE_MAX_DECIM)
        return -1;
    if (sample_rate_in % decim != 0u)
        return -1;
    memset(q, 0, sizeof(*q));

    q->decim            = decim;
    q->sample_rate_in   = sample_rate_in;
    q->sample_rate_out  = sample_rate_in / decim;
    q->sub_center_hz    = sub_center_hz;
    q->lo_hz            = lo_hz;
    q->shift_hz         = (int32_t)sub_center_hz - (int32_t)lo_hz;

    if (q->shift_hz != 0)
    {
        q->nco = nco_crcf_create(LIQUID_NCO);
        if (!q->nco)
        {
            ddc_stage_destroy(q);
            return -1;
        }
        /* mix_block_down multiplies by exp(-j*omega*n), moving a component
         * at baseband f to f - shift. A component at the subband centre
         * (RF sub_center, i.e. baseband sub_center - lo) lands on DC. */
        double omega =
            ((double)q->shift_hz / (double)sample_rate_in) * 2.0 * M_PI;
        nco_crcf_set_frequency(q->nco, (float)omega);
    }

    /* decim == 1 is premix-only: no FIR, no scale, no fractional carry. */
    if (decim == 1u)
    {
        q->samples_out = 0u;
        return 0;
    }

    q->decim_fir = firdecim_crcf_create_kaiser(decim, m, as);
    if (!q->decim_fir)
    {
        ddc_stage_destroy(q);
        return -1;
    }
    /* Unity passband gain: the raw decimator prototype carries gain D (like
     * the legacy per-channel firdecim chain), while the downstream PFB is
     * unity-gain. Scaling by 1/D keeps wideband RSSI on the same scale as
     * the narrowband path (see CHANNELIZER_BANK_RSSI_CAL_DB). */
    firdecim_crcf_set_scale(q->decim_fir, 1.0f / (float)decim);

    /* Carry holds at most D-1 fractional samples between calls. */
    q->carry = (float complex *)calloc(decim, sizeof(float complex));
    if (!q->carry)
    {
        ddc_stage_destroy(q);
        return -1;
    }
    q->carry_len  = 0u;
    q->samples_out = 0u;
    return 0;
}

void ddc_stage_destroy(ddc_stage_t *q)
{
    if (!q)
        return;
    if (q->decim_fir)
        firdecim_crcf_destroy(q->decim_fir);
    if (q->nco)
        nco_crcf_destroy(q->nco);
    free(q->carry);
    free(q->mix);
    memset(q, 0, sizeof(*q));
}

void ddc_stage_reset(ddc_stage_t *q)
{
    if (!q)
        return;
    if (q->decim_fir)
        firdecim_crcf_reset(q->decim_fir);
    if (q->nco)
        nco_crcf_reset(q->nco);
    q->carry_len   = 0u;
    q->samples_out = 0u;
}

static int ddc_stage_reserve_mix(ddc_stage_t *q, size_t n)
{
    if (q->mix_cap >= n)
        return 0;
    float complex *p =
        (float complex *)realloc(q->mix, n * sizeof(float complex));
    if (!p)
        return -1;
    q->mix     = p;
    q->mix_cap = n;
    return 0;
}

int ddc_stage_execute(ddc_stage_t *q,
                       const float complex *in,
                       size_t n,
                       uint64_t in_base,
                       float complex *out,
                       unsigned int *n_out,
                       uint64_t *out_base)
{
    if (!q || !in || !out || !n_out)
        return -1;

    /* Premix-only: NCO straight into the caller's buffer, 1:1. No scratch
     * (nothing to stage), no carry, no second copy. The shared input is
     * never mutated in place. */
    if (q->decim == 1u)
    {
        if (out_base)
            *out_base = in_base;
        if (n == 0u)
        {
            *n_out = 0u;
            return 0;
        }
        if (q->nco)
            nco_crcf_mix_block_down(q->nco, (float complex *)in, out,
                                    (unsigned int)n);
        else if (out != in)
            memcpy(out, in, n * sizeof(float complex));
        q->samples_out += (uint64_t)n;
        *n_out = (unsigned int)n;
        return 0;
    }

    unsigned int prev_carry = q->carry_len;
    if (out_base)
        *out_base = in_base - (uint64_t)prev_carry;
    *n_out = 0u;
    if (n == 0u && prev_carry == 0u)
        return 0;

    /* Build one contiguous post-rotation window: [carry | rotated in].
     * The carry already holds rotated samples from the previous call. */
    size_t total = (size_t)prev_carry + n;
    if (ddc_stage_reserve_mix(q, total) != 0)
        return -1;
    if (prev_carry > 0u)
        memcpy(q->mix, q->carry, (size_t)prev_carry * sizeof(float complex));
    if (q->nco)
        nco_crcf_mix_block_down(q->nco, (float complex *)in,
                                &q->mix[prev_carry], (unsigned int)n);
    else if (n > 0u)
        memcpy(&q->mix[prev_carry], in, n * sizeof(float complex));

    unsigned int full = (unsigned int)(total / (size_t)q->decim);
    unsigned int rem  = (unsigned int)(total % (size_t)q->decim);

    if (full > 0u)
        firdecim_crcf_execute_block(q->decim_fir, q->mix, full, out);

    /* Stash the fractional tail (the last rem window samples) for the next
     * call. It overlaps the filter state correctly because the filter
     * consumed exactly full*D leading samples. */
    if (rem > 0u)
        memcpy(q->carry, &q->mix[(size_t)full * q->decim],
               (size_t)rem * sizeof(float complex));
    q->carry_len = rem;

    q->samples_out += full;
    *n_out = full;
    return 0;
}
