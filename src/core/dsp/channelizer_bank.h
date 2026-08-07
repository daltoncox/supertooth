/**
 * @file dsp/channelizer_bank.h
 * @brief Polyphase channelizer front end for the supertooth DSP overhaul.
 *
 * Replaces the per-channel `nco_crcf` + `firdecim_crcf` chain with a single
 * `firpfbch2_crcf` analysis filterbank shared by every channel in the capture
 * span.
 *
 * Design
 * ------
 *  - Bins:        M = sample_rate_hz / grid_hz.  For BR/EDR grid_hz = 1 MHz so
 *                 an N-channel window (N MHz span) yields N bins.
 *  - Output rate: firpfbch2 is natively 2x oversampled (M/2 in -> M out), so
 *                 every bin emerges at exactly 2.000 Msps, which is what
 *                 `cpfskdem(k=2)` already expects.  No trailing resampler.
 *  - Alignment:   `session_tune()` may put the LO on a half-MHz boundary for
 *                 DC-spur avoidance, which is off-grid from the channel raster.
 *                 A single wideband NCO rotates the stream by the residual so
 *                 the channel grid lands on bin centres.
 *  - Nyquist bin: because the bank is 2x oversampled, bin M/2 is an ordinary
 *                 complex bin whose +/-grid window wraps circularly.  It is
 *                 fully usable, so an N MHz span yields N usable channels.
 *
 * The object is stateful and carries up to M/2-1 input samples between calls
 * so that ragged block sizes (the HackRF delivers 131072 samples, which is not
 * a multiple of M/2) do not disturb frame alignment.
 *
 * Output layout
 * -------------
 * `execute()` writes output frame-major: `out[frame * M + bin]`.  Every frame
 * holds exactly M samples (one per channel), so channel `bin` is always at the
 * same offset within a frame and is read with a uniform stride of M:
 *   `sample_k_of_channel_bin = out[bin + k * M]`
 * This is exactly the layout `firpfbch2_crcf_execute` produces, so the inner
 * loop is copy-free and every downstream worker shares the same strided read.
 */

#ifndef CHANNELIZER_BANK_H
#define CHANNELIZER_BANK_H

#include <complex.h>
#include <stdint.h>
#include <stdlib.h>

#include <liquid/liquid.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Channel raster (1 MHz). BR/EDR channels sit at 2402+k MHz. */
#define CHANNELIZER_BANK_GRID_BR_EDR_HZ 1000000u

/** BLE channel raster (2 MHz). BLE channels sit at 2402+2k MHz. A 2 MHz bank
 *  yields one bin per BLE channel; the 4 Msps per-bin output is decimated to
 *  2 Msps by striding the readers by 2 (see ble_channel_processor.c). */
#define CHANNELIZER_BANK_GRID_BLE_HZ 2000000u

/** Per-bin output rate produced by the 2x-oversampled bank. */
#define CHANNELIZER_BANK_OUTPUT_RATE_HZ 2000000u

/** Default prototype semi-length (h_len = 2*M*m taps at the input rate). */
#define CHANNELIZER_BANK_DEFAULT_M 4u

/** Default prototype stop-band attenuation, matching the legacy firdecim. */
#define CHANNELIZER_BANK_DEFAULT_AS 60.0f

/**
 * Fixed dBr reference offset added to every channelized RSSI measurement.
 *
 * `firpfbch2` normalises its analysis bank to unity passband gain for ANY bin
 * count, which 05_rssi_stability confirms: the raw per-bin power is 0.000 dB
 * at spans from 4 to 20 MHz, and identical across all 20 bins to within
 * 0.0000 dB.  Reported RSSI is therefore already independent of the capture
 * span and of which channel a packet landed on -- the property we want.
 *
 * The legacy per-channel `firdecim` chain did NOT have that property: its
 * passband gain was the decimation factor D = sample_rate / 2 Msps, so its
 * RSSI moved by 20*log10(D) as the span changed.  This constant is
 * 20*log10(10), i.e. the legacy gain at the default 20 Msps / 20-channel
 * capture, so previously recorded values still line up for that configuration.
 *
 * It is deliberately NOT recomputed per span: doing so would re-introduce the
 * legacy chain's span dependence.  Treat it purely as the zero point of the
 * "dBr" scale.
 */
#define CHANNELIZER_BANK_RSSI_CAL_DB 20.0036f

typedef struct
{
    unsigned int M;          /**< number of bins (even) */
    unsigned int M2;         /**< M/2: input samples consumed per frame */
    unsigned int sample_rate_hz;
    uint32_t     grid_hz;    /**< channel spacing (Hz) */

    uint32_t lo_hz;          /**< physical radio LO */
    uint32_t lo_eff_hz;      /**< grid-aligned LO the bins are centred on */
    int32_t  shift_hz;       /**< mix-down applied to the input: lo_eff - lo */

    nco_crcf       nco;      /**< NULL when shift_hz == 0 */
    firpfbch2_crcf pfb;

    /** Partial frame carried across execute() calls. */
    float complex *carry;
    unsigned int   carry_len;

    /** Scratch for the pre-rotated stream. */
    float complex *mix;
    size_t         mix_cap;

    /** Monotonic count of frames emitted since reset. */
    uint64_t frames_out;

    /**
     * When non-zero, negate odd-indexed bins on odd frames.  liquid 1.8.0 does
     * not need this (measured in 01_probe_semantics); the flag exists so the
     * correction is a single predicate rather than a fork of the pipeline.
     */
    int phase_fix;
} channelizer_bank_t;

/* --------------------------------------------------------------------------
 * Static grid helpers (no state)
 * -------------------------------------------------------------------------- */

/** Number of bins for a given wideband rate + grid. Returns 0 if unusable
 *  (rate too small, or M would be odd). */
unsigned int channelizer_bank_bins_for_rate(unsigned int sample_rate_hz,
                                            uint32_t grid_hz);

/** Round an LO onto the channel raster (ties round up). */
uint32_t channelizer_bank_grid_align(uint32_t lo_hz, uint32_t grid_hz);

/** Round an LO onto the BLE 2 MHz raster (channels at 2402 + 2k MHz), nearest. */
uint32_t channelizer_bank_grid_align_ble(uint32_t lo_hz);

/** Mix-down applied to the input so the channel grid lands on bin centres. */
int32_t channelizer_bank_grid_shift(uint32_t lo_hz, uint32_t grid_hz);

/**
 * Bin index carrying @p center_hz, or -1 when the channel is outside the span.
 * @p lo_eff_hz must be grid-aligned (see channelizer_bank_grid_align).
 */
int channelizer_bank_bin_for_center(unsigned int M, uint32_t lo_eff_hz,
                                    uint32_t center_hz, uint32_t grid_hz);

/** BLE bin (2 MHz raster) carrying @p center_hz, or -1 when outside the span. */
int channelizer_bank_bin_for_center_ble(unsigned int M, uint32_t lo_eff_hz,
                                        uint32_t center_hz);

/** Centre frequency of bin @p bin. */
uint32_t channelizer_bank_center_for_bin(unsigned int M, uint32_t lo_eff_hz,
                                         unsigned int bin, uint32_t grid_hz);

/* --------------------------------------------------------------------------
 * Object API
 * -------------------------------------------------------------------------- */

int  channelizer_bank_init(channelizer_bank_t *q,
                           unsigned int sample_rate_hz,
                           uint32_t lo_hz,
                           uint32_t grid_hz,
                           unsigned int m,
                           float as);

void channelizer_bank_destroy(channelizer_bank_t *q);
void channelizer_bank_reset(channelizer_bank_t *q);

/** Upper bound on frames produced by an execute() of @p n input samples. */
static inline size_t channelizer_bank_max_frames(const channelizer_bank_t *q, size_t n)
{
    return (n + q->M2) / q->M2;
}

/**
 * Channelize @p n input samples.
 *
 * Output is written frame-major: `out[frame * M + bin]`, which is exactly the
 * layout `firpfbch2_crcf_execute` produces, so the inner loop is copy-free.
 *
 * @param out          destination, must hold channelizer_bank_max_frames(q,n)*M samples
 * @param frames_out   [out] frames actually written
 * @param base_frame   [out, optional] absolute index of the first frame written
 * @return 0 on success
 */
int channelizer_bank_execute(channelizer_bank_t *q,
                            const float complex *in,
                            size_t n,
                            float complex *out,
                            unsigned int *frames_out,
                            uint64_t *base_frame);

#ifdef __cplusplus
}
#endif

#endif /* CHANNELIZER_BANK_H */
