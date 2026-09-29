/**
 * @file dsp/ddc_stage.h
 * @brief Thread-free digital down-converter (DDC) stage: NCO + FIR decimator.
 *
 * This is the DSP half of every channelizer lane's first stage (thread-free,
 * like dsp/channelizer_bank.h). The threading half lives in
 * service/channelizer_service.h, which owns one of these per lane.
 *
 * Each stage translates one subband to baseband and, when decim > 1,
 * decimates by D (D == K, the lane count) to the per-lane rate. The NCO
 * folds two shifts into one so the downstream PFB needs no NCO of its own:
 *
 *   shift_hz = (sub_center_hz - lo_hz)
 *            = (sub_center_hz - lo_eff_hz) + (lo_eff_hz - lo_hz)
 *              \_____________/   \_________________________/
 *              lane offset        grid residual (half-channel premix)
 *
 * decim == 1 is premix-only mode (the C <= 20 path): no FIR, no fractional
 * carry. The NCO mixes straight into the caller's output buffer, so the
 * full-rate stream is still read once and written once -- no extra copy
 * versus the legacy in-bank premix, just on its own thread.
 *
 * Timestamp convention: sample clocks stay in wideband (RF input) units end
 * to end. Output blocks carry an input-domain base:
 *   out_base = in_base - carry_len_prev   (== in_base when decim == 1)
 * i.e. the input time of the first sample of the decimation window. The
 * PFB stage converts sub-domain offsets back with out_base + done*D, so
 * channel processors keep working with a single end-to-end
 * input_decimation = D * M2_lane * stride. Filter group delay is NOT
 * compensated.
 */

#ifndef DDC_STAGE_H
#define DDC_STAGE_H

#include <complex.h>
#include <stddef.h>
#include <stdint.h>

#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Default prototype semi-length (symbols), matching the PFB default. */
#define DDC_STAGE_DEFAULT_M 4u

/** Default prototype stop-band attenuation (dB), matching the PFB default. */
#define DDC_STAGE_DEFAULT_AS 60.0f

/** Maximum decimation factor (= max lane count). */
#define DDC_STAGE_MAX_DECIM 4u

typedef struct
{
    unsigned int decim;           /**< D: input samples consumed per output */
    unsigned int sample_rate_in;  /**< wideband input rate (Hz) */
    unsigned int sample_rate_out; /**< per-lane output rate = in / D (Hz) */

    uint32_t sub_center_hz; /**< this lane's subband centre (RF, grid-aligned) */
    uint32_t lo_hz;         /**< physical radio LO */
    int32_t  shift_hz;      /**< mix-down applied: sub_center - lo */

    nco_crcf      nco;       /**< NULL when shift_hz == 0 */
    firdecim_crcf decim_fir; /**< NULL when decim == 1 (premix-only) */

    /** Scratch for the pre-rotated stream (decim > 1 only). */
    float complex *mix;
    size_t         mix_cap;

    /** Fractional-decimation carry (< D samples, post-rotation;
     *  unused when decim == 1). */
    float complex *carry;
    unsigned int   carry_len;

    /** Monotonic count of output samples emitted since reset. */
    uint64_t samples_out;
} ddc_stage_t;

/**
 * @param decim  decimation factor (== lane count K, 1..DDC_STAGE_MAX_DECIM;
 *               1 = premix-only, no FIR)
 * @param m      prototype semi-length (symbols), DDC_STAGE_DEFAULT_M
 *               (ignored when decim == 1)
 * @param as     stop-band attenuation (dB), DDC_STAGE_DEFAULT_AS
 *               (ignored when decim == 1)
 * @return 0 on success, -1 on failure.
 */
int  ddc_stage_init(ddc_stage_t *q,
                     unsigned int sample_rate_in,
                     uint32_t lo_hz,
                     uint32_t sub_center_hz,
                     unsigned int decim,
                     unsigned int m,
                     float as);

void ddc_stage_destroy(ddc_stage_t *q);
void ddc_stage_reset(ddc_stage_t *q);

/** Upper bound on outputs produced by an execute() of @p n input samples. */
static inline size_t ddc_stage_max_out(const ddc_stage_t *q, size_t n)
{
    return (n + q->carry_len + q->decim - 1u) / q->decim;
}

/**
 * Run @p n input samples through the stage.
 *
 * decim == 1 mixes straight into @p out (1:1, no scratch, no carry):
 * @p n_out == @p n and @p out_base == @p in_base. Otherwise decimates by D
 * (see ddc_stage_max_out for the bound).
 *
 * @param in_base   input-domain index of in[0] (rf block_base_sample + offset)
 * @param out       destination, must hold ddc_stage_max_out(q,n) samples
 * @param n_out     [out] samples actually written
 * @param out_base  [out, optional] input-domain index of the first sample of
 *                  the decimation window (in_base - previous carry_len)
 * @return 0 on success
 */
int ddc_stage_execute(ddc_stage_t *q,
                       const float complex *in,
                       size_t n,
                       uint64_t in_base,
                       float complex *out,
                       unsigned int *n_out,
                       uint64_t *out_base);

#ifdef __cplusplus
}
#endif

#endif /* DDC_STAGE_H */
