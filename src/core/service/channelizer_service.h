/**
 * @file service/channelizer_service.h
 * @brief Two-stage channelization service: owns DDC + PFB DSP objects,
 *        intermediate/output dispatchers, and worker threads.
 *
 * Every lane is uniformly DDC -> PFB on its own threads, so no single
 * thread carries more than one stage:
 *
 *   K = 1 (Fs <= 20 Msps): one premix-only DDC (NCO, decim == 1, no FIR)
 *           feeds one PFB (M <= 20) through a private sub dispatcher.
 *   K > 1 (Fs up to 80 Msps): K DDC lanes (nco + firdecim, decim = K)
 *           broadcast-read the RF dispatcher into K private sub
 *           dispatchers; K PFB lanes (each M_lane <= 20) publish
 *           frame-major blocks to K partitioned output dispatchers.
 *
 * The PFB itself never mixes: the DDC's single folded NCO covers lane
 * translation + grid premix on every path.
 *
 * Lane plan (no tables): K = ceil(Fs / 20 MHz), capped at 4, with Fs % K
 * == 0 and M_lane = (Fs/K)/grid even. This yields exactly the supported
 * BR/EDR counts 2..20 (even) + 24,28,32,36,40,42,48,54,60,64,72.
 *
 * The service hands the session plain channel descriptors; the session
 * builds channel processors from them and never touches M/M2/bin/stride.
 * 2 MHz bins are BLE-only (power saver): a 2 MHz request with an odd bin
 * count transparently falls back to the 1 MHz grid.
 */

#ifndef CHANNELIZER_SERVICE_H
#define CHANNELIZER_SERVICE_H

#include <complex.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>

#include "channelizer_bank.h"
#include "ddc_stage.h"
#include "sample_dispatcher.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum lanes (80 Msps / 20 Msps). */
#define CHANNELIZER_SERVICE_MAX_LANES 4u

/** Per-lane output rate ceiling: no thread carries more than this. */
#define CHANNELIZER_SERVICE_MAX_LANE_RATE_HZ 20000000u

/** BR/EDR channel count ceiling (band limit, 0..78). */
#define CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS 79u

/** BLE RF channel count ceiling (0..39). */
#define CHANNELIZER_SERVICE_MAX_BLE_CHANNELS 40u

/**
 * One deliverable channel. Everything a channel processor needs to attach
 * to its lane's output dispatcher and read its bin.
 */
typedef struct
{
    sample_dispatcher_t *dispatcher; /**< owning out-dispatcher (service-owned) */
    unsigned int  bin;               /**< bin index within that dispatcher's PFB */
    unsigned int  M;                 /**< that PFB's bin count (M_lane) */
    unsigned int  stride;            /**< frames skipped per sample (grid/1MHz) */
    unsigned int  input_decimation;  /**< end-to-end RF->demod decimation (D*M2*stride) */
    uint32_t      center_hz;         /**< RF centre of this channel */
    float         rssi_cal_db;       /**< CHANNELIZER_BANK_RSSI_CAL_DB */
} channelizer_channel_t;

typedef struct
{
    unsigned int sample_rate_hz; /**< wideband input rate (Fs) */
    uint32_t     lo_hz;          /**< physical radio LO */
    uint32_t     grid_hz;        /**< 1 MHz, or 2 MHz (BLE-only; falls back if odd) */
    unsigned int m;              /**< prototype semi-length (0 = default) */
    float        as;             /**< stop-band attenuation (0 = default) */
    int          debug;
    int          exhaustive;     /**< backpressure instead of drops (file replay) */
    const _Atomic unsigned int *shutdown; /**< stop flag (e.g. session's) */
} channelizer_service_config_t;

typedef struct channelizer_service channelizer_service_t;

struct channelizer_service
{
    channelizer_service_config_t cfg;

    unsigned int K;            /**< lane count (1 narrowband, 2..4 wideband) */
    unsigned int D;            /**< DDC decimation (== K; 1 = premix-only) */
    unsigned int sub_rate_hz;  /**< per-lane rate = Fs / K */
    unsigned int M_lane;       /**< per-lane PFB bins */
    unsigned int M2_lane;      /**< M_lane / 2 */
    uint32_t     grid_actual_hz; /**< grid after 2 MHz -> 1 MHz fallback */
    uint32_t     lo_eff_hz;    /**< grid-aligned wideband LO */
    uint32_t     span_lo_hz;   /**< lo_eff - Fs/2 */
    uint32_t     sub_centers_hz[CHANNELIZER_SERVICE_MAX_LANES];

    sample_dispatcher_t *rf; /**< borrowed RF dispatcher (not owned) */
    ddc_stage_t          ddc[CHANNELIZER_SERVICE_MAX_LANES]; /**< one per lane */
    channelizer_bank_t   pfb[CHANNELIZER_SERVICE_MAX_LANES];
    sample_dispatcher_t *sub[CHANNELIZER_SERVICE_MAX_LANES]; /**< owned, one per lane */
    sample_dispatcher_t *out[CHANNELIZER_SERVICE_MAX_LANES]; /**< owned */
    sample_reader_t      ddc_readers[CHANNELIZER_SERVICE_MAX_LANES];
    sample_reader_t      pfb_readers[CHANNELIZER_SERVICE_MAX_LANES];
    size_t               max_in[CHANNELIZER_SERVICE_MAX_LANES];

    /* Precomputed descriptors for every channel inside the span. */
    channelizer_channel_t bredr_desc[CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS];
    size_t                bredr_count;
    channelizer_channel_t ble_desc[CHANNELIZER_SERVICE_MAX_BLE_CHANNELS];
    size_t                ble_count;

    /* Worker threads (service-owned): K DDC + K PFB. */
    pthread_t worker_threads[2u * CHANNELIZER_SERVICE_MAX_LANES];
    struct {
        channelizer_service_t *svc;
        unsigned int lane;
    } worker_ctx[2u * CHANNELIZER_SERVICE_MAX_LANES];
    size_t    worker_count;
    int       running;
    int       readers_live;
    int       dsps_live;
};

/* --------------------------------------------------------------------------
 * Static planning helpers (no state)
 * -------------------------------------------------------------------------- */

/**
 * Lane plan for a wideband rate + grid: K = ceil(Fs / 20 MHz), capped at 4,
 * with Fs % K == 0 and M_lane = (Fs/K)/grid even. Fewest lanes that fit the
 * per-lane budget (minimum thread count).
 * @return 0 on success (K/M_lane written), -1 when unsupported.
 */
int channelizer_service_plan(unsigned int sample_rate_hz,
                              uint32_t grid_hz,
                              unsigned int *K_out,
                              unsigned int *M_lane_out);

/** Nonzero when @p C is a supported BR/EDR channel count. */
int channelizer_service_valid_bredr_count(unsigned int C);

/**
 * Largest supported BR/EDR count <= @p C (floored, min 0 when none).
 * Used by CLIs/GUI to snap a requested count down to something tuneable.
 */
unsigned int channelizer_service_snap_bredr_count(unsigned int C);

/**
 * Nonzero when a @p Fs sample rate is channelizable at @p grid (applies the
 * 2 MHz -> 1 MHz fallback rule: true if either grid works).
 */
int channelizer_service_valid_sample_rate(unsigned int sample_rate_hz,
                                           uint32_t grid_hz);

/* --------------------------------------------------------------------------
 * Object API
 * -------------------------------------------------------------------------- */

/**
 * Build the service: plans lanes, allocates owned dispatchers, inits DDC +
 * PFB banks, registers readers, precomputes descriptors. Spawns no threads
 * (see start). @p rf is borrowed. @return 0 on success, -1 on failure.
 */
int  channelizer_service_init(channelizer_service_t *s,
                               sample_dispatcher_t *rf,
                               const channelizer_service_config_t *cfg);
void channelizer_service_destroy(channelizer_service_t *s);

/** Spawn DDC + PFB worker threads. @return 0 on success, -1 on failure. */
int channelizer_service_start(channelizer_service_t *s);

/** Stop (signal + join) worker threads. Idempotent. */
void channelizer_service_stop(channelizer_service_t *s);

/** Wake internal readers (called by the owner's stop path). */
void channelizer_service_signal(channelizer_service_t *s);

/** Total owned dispatchers (sub + out). */
size_t channelizer_service_dispatcher_count(const channelizer_service_t *s);

/** Owned dispatcher by index [0, count): sub lanes first, then out lanes. */
sample_dispatcher_t *channelizer_service_dispatcher_at(
    const channelizer_service_t *s, size_t i);

/** Copy BR/EDR descriptors (up to @p cap). @return total available. */
size_t channelizer_service_get_bredr_channels(
    const channelizer_service_t *s,
    channelizer_channel_t *out, size_t cap);

/** Copy BLE descriptors (up to @p cap). @return total available. */
size_t channelizer_service_get_ble_channels(
    const channelizer_service_t *s,
    channelizer_channel_t *out, size_t cap);

#ifdef __cplusplus
}
#endif

#endif /* CHANNELIZER_SERVICE_H */
