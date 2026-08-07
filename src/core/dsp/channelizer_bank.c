/**
 * @file dsp/channelizer_bank.c
 * @brief See channelizer_bank.h.
 */

#include "channelizer_bank.h"

#include <math.h>
#include <string.h>

unsigned int channelizer_bank_bins_for_rate(unsigned int sample_rate_hz,
                                            uint32_t grid_hz)
{
    if (grid_hz == 0u || sample_rate_hz < 2u * grid_hz)
        return 0u;
    unsigned int M = sample_rate_hz / grid_hz;
    /* firpfbch2 requires an even channel count. */
    if (M & 1u)
        return 0u;
    return M;
}

uint32_t channelizer_bank_grid_align(uint32_t lo_hz, uint32_t grid_hz)
{
    if (grid_hz == 0u)
        return lo_hz;
    uint32_t half = grid_hz / 2u;
    return ((lo_hz + half) / grid_hz) * grid_hz;
}

uint32_t channelizer_bank_grid_align_ble(uint32_t lo_hz)
{
    const uint32_t ble_base = 2402000000u;
    const uint32_t grid = 2000000u;
    const uint32_t half = grid / 2u;
    if (lo_hz < ble_base)
        return ble_base;
    return ((lo_hz - ble_base + half) / grid) * grid + ble_base;
}

int32_t channelizer_bank_grid_shift(uint32_t lo_hz, uint32_t grid_hz)
{
    return (int32_t)channelizer_bank_grid_align(lo_hz, grid_hz) - (int32_t)lo_hz;
}

int channelizer_bank_bin_for_center(unsigned int M, uint32_t lo_eff_hz,
                                    uint32_t center_hz, uint32_t grid_hz)
{
    if (M == 0u || grid_hz == 0u)
        return -1;

    int64_t delta = (int64_t)center_hz - (int64_t)lo_eff_hz;
    if (delta % (int64_t)grid_hz != 0)
        return -1; /* not on the raster: caller mis-aligned the LO */

    int64_t k = delta / (int64_t)grid_hz;

    /* Reject anything outside the Nyquist span. The bank covers bins
     * -M/2 .. +M/2-1; bin -M/2 (== +M/2) is the usable wrap-around bin. */
    if (k < -(int64_t)(M / 2u) || k > (int64_t)(M / 2u) - 1)
        return -1;

    int64_t bin = k % (int64_t)M;
    if (bin < 0)
        bin += (int64_t)M;
    return (int)bin;
}

int channelizer_bank_bin_for_center_ble(unsigned int M, uint32_t lo_eff_hz,
                                        uint32_t center_hz)
{
    if (M == 0u)
        return -1;

    const uint32_t grid = 2000000u;
    int64_t delta = (int64_t)center_hz - (int64_t)lo_eff_hz;
    if (delta % (int64_t)grid != 0)
        return -1;

    int64_t k = delta / (int64_t)grid;

    if (k < -(int64_t)(M / 2u) || k > (int64_t)(M / 2u) - 1)
        return -1;

    int64_t bin = k % (int64_t)M;
    if (bin < 0)
        bin += (int64_t)M;
    return (int)bin;
}

uint32_t channelizer_bank_center_for_bin(unsigned int M, uint32_t lo_eff_hz,
                                         unsigned int bin, uint32_t grid_hz)
{
    int k = (int)bin;
    if (k >= (int)(M / 2u))
        k -= (int)M; /* upper half represents negative frequencies */
    return (uint32_t)((int64_t)lo_eff_hz + (int64_t)k * (int64_t)grid_hz);
}

int channelizer_bank_init(channelizer_bank_t *q,
                          unsigned int sample_rate_hz,
                          uint32_t lo_hz,
                          uint32_t grid_hz,
                          unsigned int m,
                          float as)
{
    if (!q || grid_hz == 0u)
        return -1;
    memset(q, 0, sizeof(*q));

    unsigned int M = channelizer_bank_bins_for_rate(sample_rate_hz, grid_hz);
    if (M == 0u)
        return -1;

    q->M              = M;
    q->M2             = M / 2u;
    q->sample_rate_hz = sample_rate_hz;
    q->grid_hz        = grid_hz;
    q->lo_hz          = lo_hz;
    q->lo_eff_hz      = (grid_hz == CHANNELIZER_BANK_GRID_BLE_HZ)
                        ? channelizer_bank_grid_align_ble(lo_hz)
                        : channelizer_bank_grid_align(lo_hz, grid_hz);
    q->shift_hz       = (int32_t)q->lo_eff_hz - (int32_t)lo_hz;

    if (q->shift_hz != 0)
    {
        q->nco = nco_crcf_create(LIQUID_NCO);
        if (!q->nco)
        {
            channelizer_bank_destroy(q);
            return -1;
        }
        /* mix_block_down multiplies by exp(-j*omega*n), moving a component at
         * baseband f to f - shift. We want the grid to move by -shift so that
         * (center - lo) becomes (center - lo_eff). */
        double omega = ((double)q->shift_hz / (double)sample_rate_hz) * 2.0 * M_PI;
        nco_crcf_set_frequency(q->nco, (float)omega);
    }

    q->pfb = firpfbch2_crcf_create_kaiser(LIQUID_ANALYZER, M, m, as);
    if (!q->pfb)
    {
        channelizer_bank_destroy(q);
        return -1;
    }

    q->carry = (float complex *)calloc(q->M2, sizeof(float complex));
    if (!q->carry)
    {
        channelizer_bank_destroy(q);
        return -1;
    }
    q->carry_len = 0u;
    q->frames_out = 0u;
    q->phase_fix = 0;
    return 0;
}

void channelizer_bank_destroy(channelizer_bank_t *q)
{
    if (!q)
        return;
    if (q->pfb)
        firpfbch2_crcf_destroy(q->pfb);
    if (q->nco)
        nco_crcf_destroy(q->nco);
    free(q->carry);
    free(q->mix);
    memset(q, 0, sizeof(*q));
}

void channelizer_bank_reset(channelizer_bank_t *q)
{
    if (!q)
        return;
    if (q->pfb)
        firpfbch2_crcf_reset(q->pfb);
    if (q->nco)
        nco_crcf_reset(q->nco);
    q->carry_len  = 0u;
    q->frames_out = 0u;
}

static int channelizer_bank_reserve_mix(channelizer_bank_t *q, size_t n)
{
    if (q->mix_cap >= n)
        return 0;
    float complex *p = (float complex *)realloc(q->mix, n * sizeof(float complex));
    if (!p)
        return -1;
    q->mix     = p;
    q->mix_cap = n;
    return 0;
}

int channelizer_bank_execute(channelizer_bank_t *q,
                             const float complex *in,
                             size_t n,
                             float complex *out,
                             unsigned int *frames_out,
                             uint64_t *base_frame)
{
    if (!q || !in || !out || !frames_out)
        return -1;

    if (base_frame)
        *base_frame = q->frames_out;
    *frames_out = 0u;
    if (n == 0u)
        return 0;

    /* ---- 1. half-bin pre-rotation so the channel grid lands on bin centres */
    const float complex *src;
    if (q->nco)
    {
        if (channelizer_bank_reserve_mix(q, n) != 0)
            return -1;
        nco_crcf_mix_block_down(q->nco, (float complex *)in, q->mix, (unsigned int)n);
        src = q->mix;
    }
    else
    {
        src = in;
    }

    const unsigned int M  = q->M;
    const unsigned int M2 = q->M2;
    unsigned int frames = 0u;
    size_t consumed = 0u;

    /* ---- 2. finish the frame left over from the previous call */
    if (q->carry_len > 0u)
    {
        unsigned int need = M2 - q->carry_len;
        if (n < need)
        {
            memcpy(&q->carry[q->carry_len], src, n * sizeof(float complex));
            q->carry_len += (unsigned int)n;
            return 0;
        }
        memcpy(&q->carry[q->carry_len], src, need * sizeof(float complex));
        firpfbch2_crcf_execute(q->pfb, q->carry, &out[0]);
        frames   = 1u;
        consumed = need;
        q->carry_len = 0u;
    }

    /* ---- 3. whole frames straight from the input, written frame-major */
    while (consumed + M2 <= n)
    {
        firpfbch2_crcf_execute(q->pfb,
                               (float complex *)&src[consumed],
                               &out[(size_t)frames * M]);
        frames++;
        consumed += M2;
    }

    /* ---- 4. stash the ragged tail for the next call */
    if (consumed < n)
    {
        q->carry_len = (unsigned int)(n - consumed);
        memcpy(q->carry, &src[consumed], q->carry_len * sizeof(float complex));
    }

    /* ---- 5. optional per-frame de-rotation of odd bins */
    if (q->phase_fix)
    {
        for (unsigned int f = 0u; f < frames; f++)
        {
            if (((q->frames_out + f) & 1u) == 0u)
                continue;
            float complex *row = &out[(size_t)f * M];
            for (unsigned int k = 1u; k < M; k += 2u)
                row[k] = -row[k];
        }
    }

    q->frames_out += frames;
    *frames_out = frames;
    return 0;
}
