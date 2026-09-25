/**
 * @file service/channelizer_service.c
 * @brief See channelizer_service.h.
 */

#include "channelizer_service.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RF centres (Hz). BLE geometry mirrors ble_bitstream_decoder.h without
 * pulling the protocol headers into the service. */
#define SVC_BREDR_BASE_HZ 2402000000u
#define SVC_BREDR_STEP_HZ 1000000u
#define SVC_BLE_BASE_HZ 2402000000u
#define SVC_BLE_STEP_HZ 2000000u

int channelizer_service_plan(unsigned int sample_rate_hz,
                              uint32_t grid_hz,
                              unsigned int *K_out,
                              unsigned int *M_lane_out)
{
    if (sample_rate_hz == 0u || grid_hz == 0u)
        return -1;
    if (grid_hz != CHANNELIZER_BANK_GRID_BR_EDR_HZ &&
        grid_hz != CHANNELIZER_BANK_GRID_BLE_HZ)
        return -1;
    if (sample_rate_hz % 1000000u != 0u)
        return -1;

    /* Fewest lanes that fit the per-lane budget: K = ceil(Fs / 20 MHz).
     * Lanes are equal (Fs % K == 0) with an even bin count each. Larger K
     * is never tried: staging uses the minimum thread count, so counts
     * like 30 (3x10) or 56 stay unsupported by design. Supported BR/EDR
     * counts are exactly 2..20 (even) + 24,28,32,36,40,42,48,54,60,64,72. */
    unsigned int K =
        (sample_rate_hz + CHANNELIZER_SERVICE_MAX_LANE_RATE_HZ - 1u) /
        CHANNELIZER_SERVICE_MAX_LANE_RATE_HZ;
    if (K < 1u || K > CHANNELIZER_SERVICE_MAX_LANES)
        return -1;
    if (sample_rate_hz % K != 0u)
        return -1;
    unsigned int lane_hz = sample_rate_hz / K;
    if (lane_hz % grid_hz != 0u)
        return -1;
    unsigned int M = lane_hz / grid_hz;
    if (M < 2u || (M & 1u) != 0u)
        return -1; /* firpfbch2 requires an even bin count */
    if (K_out)
        *K_out = K;
    if (M_lane_out)
        *M_lane_out = M;
    return 0;
}

int channelizer_service_valid_bredr_count(unsigned int C)
{
    if (C < 2u || C > CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS)
        return 0;
    if ((C & 1u) != 0u)
        return 0;
    return channelizer_service_plan(C * 1000000u,
                                     CHANNELIZER_BANK_GRID_BR_EDR_HZ,
                                     NULL, NULL) == 0;
}

unsigned int channelizer_service_snap_bredr_count(unsigned int C)
{
    if (C > CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS)
        C = CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS;
    C &= ~1u;
    while (C >= 2u)
    {
        if (channelizer_service_valid_bredr_count(C))
            return C;
        C -= 2u;
    }
    return 0u;
}

int channelizer_service_valid_sample_rate(unsigned int sample_rate_hz,
                                           uint32_t grid_hz)
{
    if (channelizer_service_plan(sample_rate_hz, grid_hz, NULL, NULL) == 0)
        return 1;
    /* 2 MHz -> 1 MHz fallback (BLE-only power saver). */
    if (grid_hz == CHANNELIZER_BANK_GRID_BLE_HZ &&
        channelizer_service_plan(sample_rate_hz,
                                  CHANNELIZER_BANK_GRID_BR_EDR_HZ,
                                  NULL, NULL) == 0)
        return 1;
    return 0;
}

/* max RF samples fed per PFB call so one bank call never exceeds the output
 * block capacity (same bound as the legacy channelizer_thread). */
static size_t svc_max_in(unsigned int M, unsigned int M2)
{
    const size_t CAP = SAMPLE_BLOCK_SAMPLE_CAPACITY;
    size_t max_frames = CAP / (size_t)M;
    if (max_frames > 2u)
        max_frames -= 2u;
    else
        max_frames = 1u;
    max_frames = (max_frames / 4u) * 4u;
    if (max_frames < 4u)
        max_frames = 4u;
    return max_frames * (size_t)M2;
}

static int svc_alloc_dispatcher(sample_dispatcher_t **out)
{
    *out = (sample_dispatcher_t *)calloc(1, sizeof(**out));
    if (!*out)
        return -1;
    if (sample_dispatcher_init(*out) != 0)
    {
        free(*out);
        *out = NULL;
        return -1;
    }
    return 0;
}

static void svc_free_dispatcher(sample_dispatcher_t **d)
{
    if (!d || !*d)
        return;
    sample_dispatcher_destroy(*d);
    free(*d);
    *d = NULL;
}

/* Lane index for an in-span centre (integer-division tie-break: a centre
 * exactly on a lane edge belongs to the upper lane). */
static unsigned int svc_lane_for_center(const channelizer_service_t *s,
                                        uint32_t center_hz)
{
    return (center_hz - s->span_lo_hz) / s->sub_rate_hz;
}

/* Effective LO of a lane's PFB: the bank aligns the (1 MHz grid) sub-band
 * centre onto the service grid itself, enabling its own residual NCO when
 * they differ (2 MHz grid on a half-grid LO). Descriptors must map bins
 * against this, not the raw sub-centre. */
static uint32_t svc_lane_lo_eff(const channelizer_service_t *s,
                                unsigned int lane)
{
    return s->pfb[lane].lo_eff_hz;
}

static void svc_build_descriptors(channelizer_service_t *s)
{
    const unsigned int Fs = s->cfg.sample_rate_hz;
    const unsigned int stride = s->grid_actual_hz / 1000000u;
    const unsigned int decim =
        s->D * s->M2_lane * stride;

    s->bredr_count = 0u;
    for (unsigned int c = 0u;
         c < CHANNELIZER_SERVICE_MAX_BREDR_CHANNELS; c++)
    {
        uint32_t center =
            SVC_BREDR_BASE_HZ + (uint64_t)c * SVC_BREDR_STEP_HZ;
        int32_t offset = (int32_t)center - (int32_t)s->cfg.lo_hz;
        if (labs((long)offset) >= (int32_t)(Fs / 2u))
            continue;
        unsigned int lane = svc_lane_for_center(s, center);
        if (lane >= s->K)
            continue;
        int bin = channelizer_bank_bin_for_center(
            s->M_lane, svc_lane_lo_eff(s, lane), center,
            s->grid_actual_hz);
        if (bin < 0)
            continue;
        channelizer_channel_t *d =
            &s->bredr_desc[s->bredr_count++];
        d->dispatcher       = s->out[lane];
        d->bin              = (unsigned int)bin;
        d->M                = s->M_lane;
        d->stride           = 1u; /* BR/EDR always reads every frame */
        d->input_decimation = s->D * s->M2_lane;
        d->center_hz        = center;
        d->rssi_cal_db      = CHANNELIZER_BANK_RSSI_CAL_DB;
    }

    /* BLE descriptors use the service grid (stride 2 at 2 MHz): only the
     * BLE-only service requests 2 MHz; hybrid BLE fans out over the shared
     * 1 MHz service with stride 1. */
    s->ble_count = 0u;
    for (unsigned int rf = 0u;
         rf < CHANNELIZER_SERVICE_MAX_BLE_CHANNELS; rf++)
    {
        uint32_t center =
            SVC_BLE_BASE_HZ + (uint64_t)rf * SVC_BLE_STEP_HZ;
        int32_t offset = (int32_t)center - (int32_t)s->cfg.lo_hz;
        if (labs((long)offset) >= (int32_t)(Fs / 2u))
            continue;
        unsigned int lane = svc_lane_for_center(s, center);
        if (lane >= s->K)
            continue;
        int bin = channelizer_bank_bin_for_center(
            s->M_lane, svc_lane_lo_eff(s, lane), center,
            s->grid_actual_hz);
        if (bin < 0)
            continue;
        channelizer_channel_t *d = &s->ble_desc[s->ble_count++];
        d->dispatcher       = s->out[lane];
        d->bin              = (unsigned int)bin;
        d->M                = s->M_lane;
        d->stride           = stride;
        d->input_decimation = decim;
        d->center_hz        = center;
        d->rssi_cal_db      = CHANNELIZER_BANK_RSSI_CAL_DB;
    }
}

int channelizer_service_init(channelizer_service_t *s,
                              sample_dispatcher_t *rf,
                              const channelizer_service_config_t *cfg)
{
    if (!s || !rf || !cfg || cfg->sample_rate_hz == 0u)
        return -1;
    if (cfg->grid_hz != CHANNELIZER_BANK_GRID_BR_EDR_HZ &&
        cfg->grid_hz != CHANNELIZER_BANK_GRID_BLE_HZ)
        return -1;
    memset(s, 0, sizeof(*s));
    s->cfg = *cfg;
    s->rf  = rf;

    /* 2 MHz -> 1 MHz fallback (BLE-only power saver). */
    uint32_t grid = cfg->grid_hz;
    unsigned int K = 0u, M_lane = 0u;
    if (channelizer_service_plan(cfg->sample_rate_hz, grid, &K,
                                  &M_lane) != 0)
    {
        if (grid != CHANNELIZER_BANK_GRID_BLE_HZ ||
            channelizer_service_plan(cfg->sample_rate_hz,
                                      CHANNELIZER_BANK_GRID_BR_EDR_HZ, &K,
                                      &M_lane) != 0)
            return -1;
        grid = CHANNELIZER_BANK_GRID_BR_EDR_HZ;
    }
    s->grid_actual_hz = grid;
    s->K              = K;
    s->D              = K; /* Fs/sub_rate; 1 when K == 1 */
    s->sub_rate_hz    = cfg->sample_rate_hz / K;
    s->M_lane         = M_lane;
    s->M2_lane        = M_lane / 2u;
    /* Wideband grid anchor: sub-centres inherit this alignment, so every
     * staged (K > 1) lane is grid-aligned and its PFB runs NCO-free (the
     * DDC's single folded NCO covers translation + premix). */
    s->lo_eff_hz      = channelizer_bank_grid_align(cfg->lo_hz, grid);
    s->span_lo_hz = (uint32_t)((int64_t)s->lo_eff_hz -
                               (int64_t)(cfg->sample_rate_hz / 2u));
    for (unsigned int k = 0u; k < K; k++)
    {
        s->sub_centers_hz[k] = (uint32_t)(
            (int64_t)s->span_lo_hz +
            (int64_t)(s->sub_rate_hz / 2u) +
            (int64_t)k * (int64_t)s->sub_rate_hz);
    }

    unsigned int m  = cfg->m != 0u ? cfg->m : CHANNELIZER_BANK_DEFAULT_M;
    float as = cfg->as != 0.0f ? cfg->as : CHANNELIZER_BANK_DEFAULT_AS;

    /* Progress counters for the fail path (destroy() assumes full init). */
    unsigned int n_out = 0u, n_sub = 0u, n_ddc = 0u, n_pfb = 0u;
    unsigned int n_rd_ddc = 0u, n_rd_pfb = 0u;

    /* Owned dispatchers: K outputs always; K intermediates when staged. */
    for (unsigned int k = 0u; k < K; k++)
    {
        if (svc_alloc_dispatcher(&s->out[k]) != 0)
            goto fail;
        n_out++;
    }
    if (K > 1u)
    {
        for (unsigned int k = 0u; k < K; k++)
        {
            if (svc_alloc_dispatcher(&s->sub[k]) != 0)
                goto fail;
            n_sub++;
        }
        for (unsigned int k = 0u; k < K; k++)
        {
            /* Single folded NCO per lane: lane offset + grid residual. */
            if (ddc_stage_init(&s->ddc[k], cfg->sample_rate_hz,
                                cfg->lo_hz, s->sub_centers_hz[k], K,
                                DDC_STAGE_DEFAULT_M,
                                DDC_STAGE_DEFAULT_AS) != 0)
                goto fail;
            n_ddc++;
        }
        s->dsps_live = 1;
    }
    for (unsigned int k = 0u; k < K; k++)
    {
        /* K > 1: the DDC already translated + premixed, so the lane PFB is
         * fed its grid-aligned sub-centre and runs NCO-free. K == 1 has no
         * DDC: feed the raw LO exactly like the legacy single bank so the
         * PFB keeps its own residual (half-channel premix) NCO. */
        uint32_t pfb_lo = (K > 1u) ? s->sub_centers_hz[k] : cfg->lo_hz;
        if (channelizer_bank_init(&s->pfb[k], s->sub_rate_hz,
                                   pfb_lo, grid, m,
                                   as) != 0)
            goto fail;
        n_pfb++;
        s->max_in[k] = svc_max_in(s->M_lane, s->M2_lane);
    }
    s->dsps_live = 1;

    /* Readers: DDC lanes broadcast-read RF; PFB lanes read their sub lane
     * (or RF directly when K == 1). */
    if (K > 1u)
    {
        for (unsigned int k = 0u; k < K; k++)
        {
            if (sample_reader_init(&s->ddc_readers[k], rf) != 0)
                goto fail;
            n_rd_ddc++;
        }
        for (unsigned int k = 0u; k < K; k++)
        {
            if (sample_reader_init(&s->pfb_readers[k], s->sub[k]) != 0)
                goto fail;
            n_rd_pfb++;
        }
    }
    else
    {
        if (sample_reader_init(&s->pfb_readers[0], rf) != 0)
            goto fail;
        n_rd_pfb++;
    }
    s->readers_live = 1;

    svc_build_descriptors(s);
    if (s->bredr_count == 0u && s->ble_count == 0u)
        goto fail;
    if (s->cfg.debug)
    {
        fprintf(stderr,
                "[chan_svc] lo=%u eff=%u Fs=%u grid=%u K=%u M_lane=%u D=%u : "
                "%zu BLE + %zu BR/EDR descriptors\n",
                s->cfg.lo_hz, s->lo_eff_hz, s->cfg.sample_rate_hz,
                s->grid_actual_hz, s->K, s->M_lane, s->D, s->ble_count,
                s->bredr_count);
    }
    return 0;

fail:
    for (unsigned int k = 0u; k < n_rd_ddc; k++)
        sample_reader_destroy(&s->ddc_readers[k]);
    for (unsigned int k = 0u; k < n_rd_pfb; k++)
        sample_reader_destroy(&s->pfb_readers[k]);
    for (unsigned int k = 0u; k < n_ddc; k++)
        ddc_stage_destroy(&s->ddc[k]);
    for (unsigned int k = 0u; k < n_pfb; k++)
        channelizer_bank_destroy(&s->pfb[k]);
    for (unsigned int k = 0u; k < n_sub; k++)
        svc_free_dispatcher(&s->sub[k]);
    for (unsigned int k = 0u; k < n_out; k++)
        svc_free_dispatcher(&s->out[k]);
    memset(s, 0, sizeof(*s));
    return -1;
}

static void *svc_ddc_worker(void *arg)
{
    channelizer_service_t *s;
    unsigned int lane;
    {
        typeof(s->worker_ctx[0]) *ctx = arg;
        s    = ctx->svc;
        lane = ctx->lane;
    }
    ddc_stage_t *stage       = &s->ddc[lane];
    sample_reader_t *reader  = &s->ddc_readers[lane];
    sample_dispatcher_t *dst = s->sub[lane];
    const _Atomic unsigned int *shutdown = s->cfg.shutdown;
    int exhaustive = s->cfg.exhaustive;
    int debug      = s->cfg.debug;

    sample_block_t *rfb = NULL;
    for (;;)
    {
        if (sample_reader_wait_pop(reader, shutdown, &rfb) != 0)
            break;
        if (!rfb)
            continue;

        sample_block_t *db = exhaustive
            ? sample_dispatcher_acquire_blocking(dst, shutdown)
            : sample_dispatcher_acquire_block(dst);
        if (!db)
        {
            if (!exhaustive)
                sample_dispatcher_note_drop(dst, debug);
            sample_block_release(rfb);
            rfb = NULL;
            if (exhaustive)
                break; /* shutdown requested */
            continue;
        }

        unsigned int n_out = 0u;
        uint64_t out_base  = 0u;
        ddc_stage_execute(stage, rfb->samples, rfb->num_samples,
                          rfb->block_base_sample, db->samples, &n_out,
                          &out_base);
        db->num_samples       = n_out;
        db->block_base_sample = out_base;

        if (exhaustive)
        {
            sample_dispatcher_push_blocking(dst, db, shutdown);
            sample_block_release(db);
            if (shutdown &&
                atomic_load_explicit(shutdown,
                                      memory_order_acquire) != 0u)
            {
                sample_block_release(rfb);
                rfb = NULL;
                break;
            }
        }
        else
        {
            sample_dispatcher_push_block(dst, db);
            sample_block_release(db);
        }
        sample_block_release(rfb);
        rfb = NULL;
    }
    return NULL;
}

static void *svc_pfb_worker(void *arg)
{
    channelizer_service_t *s;
    unsigned int lane;
    {
        typeof(s->worker_ctx[0]) *ctx = arg;
        s    = ctx->svc;
        lane = ctx->lane;
    }
    channelizer_bank_t *bank = &s->pfb[lane];
    sample_reader_t *reader  = &s->pfb_readers[lane];
    sample_dispatcher_t *dst = s->out[lane];
    const unsigned int M     = s->M_lane;
    const size_t max_in      = s->max_in[lane];
    /* Sub-domain offsets convert back to input-domain samples via D. */
    const uint64_t lane_scale = (uint64_t)s->D;
    const _Atomic unsigned int *shutdown = s->cfg.shutdown;
    int exhaustive = s->cfg.exhaustive;
    int debug      = s->cfg.debug;

    sample_block_t *sb = NULL;
    for (;;)
    {
        if (sample_reader_wait_pop(reader, shutdown, &sb) != 0)
            break;
        if (!sb)
            continue;

        /* Bank keeps its own carry across calls; feed in place in
         * max_in-sized sub-chunks. Bases stay in input-domain units. */
        uint64_t base = sb->block_base_sample;
        size_t done   = 0u;
        while (done < (size_t)sb->num_samples)
        {
            size_t n = (size_t)sb->num_samples - done;
            if (n > max_in)
                n = max_in;

            sample_block_t *fm = exhaustive
                ? sample_dispatcher_acquire_blocking(dst, shutdown)
                : sample_dispatcher_acquire_block(dst);
            if (!fm)
            {
                if (!exhaustive)
                    sample_dispatcher_note_drop(dst, debug);
                break;
            }

            unsigned int frames_out = 0u;
            channelizer_bank_execute(bank, &sb->samples[done], n,
                                     fm->samples, &frames_out, NULL);
            fm->num_samples       = (unsigned int)((size_t)M * frames_out);
            fm->block_base_sample = base + (uint64_t)done * lane_scale;

            if (exhaustive)
            {
                sample_dispatcher_push_blocking(dst, fm, shutdown);
                sample_block_release(fm);
                if (shutdown &&
                    atomic_load_explicit(shutdown,
                                          memory_order_acquire) != 0u)
                    break;
            }
            else
            {
                sample_dispatcher_push_block(dst, fm);
                sample_block_release(fm);
            }
            done += n;
        }

        sample_block_release(sb);
        sb = NULL;
    }
    return NULL;
}

int channelizer_service_start(channelizer_service_t *s)
{
    if (!s || s->running || s->K == 0u)
        return -1;
    size_t started = 0u;

    if (s->K > 1u)
    {
        for (unsigned int k = 0u; k < s->K; k++)
        {
            s->worker_ctx[started].svc    = s;
            s->worker_ctx[started].lane   = k;
            s->worker_ctx[started].is_ddc = 1;
            if (pthread_create(&s->worker_threads[started], NULL,
                               svc_ddc_worker,
                               &s->worker_ctx[started]) != 0)
            {
                channelizer_service_stop(s);
                return -1;
            }
            started++;
        }
    }
    for (unsigned int k = 0u; k < s->K; k++)
    {
        s->worker_ctx[started].svc    = s;
        s->worker_ctx[started].lane   = k;
        s->worker_ctx[started].is_ddc = 0;
        if (pthread_create(&s->worker_threads[started], NULL,
                           svc_pfb_worker,
                           &s->worker_ctx[started]) != 0)
        {
            channelizer_service_stop(s);
            return -1;
        }
        started++;
    }
    s->worker_count = started;
    s->running      = 1;
    return 0;
}

void channelizer_service_stop(channelizer_service_t *s)
{
    if (!s || s->worker_count == 0u)
    {
        if (s)
            s->running = 0;
        return;
    }
    /* Request stop so wait_pop unblocks, then wake and join. Setting the
     * owner's flag mirrors session_request_stop; the session is tearing
     * down whenever this runs. */
    if (s->cfg.shutdown)
        atomic_store_explicit(( _Atomic unsigned int *)s->cfg.shutdown, 1u,
                              memory_order_release);
    channelizer_service_signal(s);
    for (size_t w = 0u; w < s->worker_count; w++)
        pthread_join(s->worker_threads[w], NULL);
    s->worker_count = 0u;
    s->running      = 0;
}

void channelizer_service_signal(channelizer_service_t *s)
{
    if (!s || !s->readers_live)
        return;
    if (s->K > 1u)
    {
        for (unsigned int k = 0u; k < s->K; k++)
            sample_reader_signal(&s->ddc_readers[k]);
    }
    for (unsigned int k = 0u; k < s->K; k++)
        sample_reader_signal(&s->pfb_readers[k]);
}

size_t channelizer_service_dispatcher_count(
    const channelizer_service_t *s)
{
    if (!s || s->K == 0u)
        return 0u;
    return s->K > 1u ? 2u * (size_t)s->K : 1u;
}

sample_dispatcher_t *channelizer_service_dispatcher_at(
    const channelizer_service_t *s, size_t i)
{
    if (!s || s->K == 0u)
        return NULL;
    if (s->K == 1u)
        return i == 0u ? s->out[0] : NULL;
    if (i < (size_t)s->K)
        return s->sub[i];
    i -= (size_t)s->K;
    return i < (size_t)s->K ? s->out[i] : NULL;
}

size_t channelizer_service_get_bredr_channels(
    const channelizer_service_t *s,
    channelizer_channel_t *out, size_t cap)
{
    if (!s)
        return 0u;
    if (out && cap > 0u)
    {
        size_t n = s->bredr_count < cap ? s->bredr_count : cap;
        memcpy(out, s->bredr_desc, n * sizeof(*out));
    }
    return s->bredr_count;
}

size_t channelizer_service_get_ble_channels(
    const channelizer_service_t *s,
    channelizer_channel_t *out, size_t cap)
{
    if (!s)
        return 0u;
    if (out && cap > 0u)
    {
        size_t n = s->ble_count < cap ? s->ble_count : cap;
        memcpy(out, s->ble_desc, n * sizeof(*out));
    }
    return s->ble_count;
}

void channelizer_service_destroy(channelizer_service_t *s)
{
    if (!s)
        return;
    if (s->running)
        channelizer_service_stop(s);
    if (s->readers_live)
    {
        if (s->K > 1u)
        {
            for (unsigned int k = 0u; k < s->K; k++)
                sample_reader_destroy(&s->ddc_readers[k]);
        }
        for (unsigned int k = 0u; k < s->K; k++)
            sample_reader_destroy(&s->pfb_readers[k]);
        s->readers_live = 0;
    }
    if (s->dsps_live)
    {
        if (s->K > 1u)
        {
            for (unsigned int k = 0u; k < s->K; k++)
                ddc_stage_destroy(&s->ddc[k]);
        }
        for (unsigned int k = 0u; k < s->K; k++)
            channelizer_bank_destroy(&s->pfb[k]);
        s->dsps_live = 0;
    }
    for (unsigned int k = 0u; k < CHANNELIZER_SERVICE_MAX_LANES; k++)
    {
        svc_free_dispatcher(&s->sub[k]);
        svc_free_dispatcher(&s->out[k]);
    }
    memset(s, 0, sizeof(*s));
}
