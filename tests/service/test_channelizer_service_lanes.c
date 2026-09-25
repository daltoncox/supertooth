/* Channelizer-service lane tests (no RF hardware required).
 *
 * Verifies the static planner plus service init without starting threads:
 *  - plan() yields the documented K/M_lane for every supported count and
 *    rejects the gaps (22, 26, 30, 44, ...);
 *  - snap() floors to the nearest supported count;
 *  - init() builds partitioned output dispatchers with sane descriptors
 *    (bin range, decimation, dispatcher ownership) for K=1 and K=4;
 *  - the 2 MHz -> 1 MHz fallback triggers on odd bin counts.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "channelizer_service.h"

static int g_failures = 0;

#define CHECK_U64(name, actual, expected)                                     \
    do {                                                                      \
        uint64_t a_ = (uint64_t)(actual), e_ = (uint64_t)(expected);          \
        if (a_ != e_) {                                                       \
            printf("FAIL %-34s expected %llu got %llu\n", name,               \
                   (unsigned long long)e_, (unsigned long long)a_);           \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define CHECK_TRUE(name, cond)                                                \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %-34s condition false\n", name);                     \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static void test_plan(void)
{
    unsigned int K = 0u, M = 0u;

    CHECK_U64("plan 20M/1M", channelizer_service_plan(20000000u, 1000000u,
                                                      &K, &M),
              0);
    CHECK_U64("plan 20M K", K, 1u);
    CHECK_U64("plan 20M M", M, 20u);

    CHECK_U64("plan 40M/1M", channelizer_service_plan(40000000u, 1000000u,
                                                      &K, &M),
              0);
    CHECK_U64("plan 40M K", K, 2u);
    CHECK_U64("plan 40M M", M, 20u);

    CHECK_U64("plan 72M/1M", channelizer_service_plan(72000000u, 1000000u,
                                                      &K, &M),
              0);
    CHECK_U64("plan 72M K", K, 4u);
    CHECK_U64("plan 72M M", M, 18u);

    CHECK_U64("plan 42M/1M", channelizer_service_plan(42000000u, 1000000u,
                                                      &K, &M),
              0);
    CHECK_U64("plan 42M K", K, 3u);
    CHECK_U64("plan 42M M", M, 14u);

    CHECK_U64("plan 22M rejected",
              channelizer_service_plan(22000000u, 1000000u, NULL, NULL),
              -1);
    CHECK_U64("plan 26M rejected",
              channelizer_service_plan(26000000u, 1000000u, NULL, NULL),
              -1);
    CHECK_U64("plan 78M rejected",
              channelizer_service_plan(78000000u, 1000000u, NULL, NULL),
              -1);
    CHECK_U64("plan 100M rejected",
              channelizer_service_plan(100000000u, 1000000u, NULL, NULL),
              -1);

    /* 2 MHz grid: 20 Msps -> M=10 even. */
    CHECK_U64("plan 20M/2M", channelizer_service_plan(20000000u, 2000000u,
                                                      &K, &M),
              0);
    CHECK_U64("plan 20M/2M M", M, 10u);
    /* 10 Msps at 2 MHz -> M=5 odd -> rejected (service falls back). */
    CHECK_U64("plan 10M/2M rejected",
              channelizer_service_plan(10000000u, 2000000u, NULL, NULL),
              -1);
    CHECK_TRUE("10M/2M valid via fallback",
               channelizer_service_valid_sample_rate(10000000u, 2000000u));

    CHECK_U64("snap 22 -> 20", channelizer_service_snap_bredr_count(22u),
              20u);
    CHECK_U64("snap 78 -> 72", channelizer_service_snap_bredr_count(78u),
              72u);
    /* 79 ("all") is the full-band sentinel at 80 Msps: requests >= 79 snap
     * to it rather than flooring to the even lane-split table. */
    CHECK_U64("valid 79 (all)", channelizer_service_valid_bredr_count(79u),
              1u);
    CHECK_U64("snap 79 -> 79", channelizer_service_snap_bredr_count(79u),
              79u);
    CHECK_U64("snap 80 -> 79", channelizer_service_snap_bredr_count(80u),
              79u);
    CHECK_U64("snap 20 -> 20", channelizer_service_snap_bredr_count(20u),
              20u);
}

static sample_dispatcher_t *make_rf(void)
{
    sample_dispatcher_t *rf =
        (sample_dispatcher_t *)calloc(1, sizeof(*rf));
    if (!rf)
        return NULL;
    if (sample_dispatcher_init(rf) != 0)
    {
        free(rf);
        return NULL;
    }
    return rf;
}

static void check_readers(channelizer_service_t *s, const char *tag)
{
    size_t nb = channelizer_service_get_bredr_count(s);
    size_t nl = channelizer_service_get_ble_count(s);
    CHECK_TRUE(tag, nb > 0u && nl > 0u);
    /* Bad indexes are rejected. */
    {
        sample_reader_t r;
        memset(&r, 0, sizeof(r));
        CHECK_TRUE(tag, channelizer_service_bredr_reader_init(s, nb, &r) != 0);
        CHECK_TRUE(tag, channelizer_service_ble_reader_init(s, nl, &r) != 0);
        CHECK_TRUE(tag, channelizer_service_bredr_center(s, nb) == 0u);
        CHECK_TRUE(tag, channelizer_service_ble_center(s, nl) == 0u);
    }
    for (size_t i = 0u; i < nb; i++)
    {
        sample_reader_t r;
        memset(&r, 0, sizeof(r));
        uint32_t center = channelizer_service_bredr_center(s, i);
        if (channelizer_service_bredr_reader_init(s, i, &r) != 0 ||
            center == 0u)
        {
            printf("FAIL %s bredr[%zu] reader init\n", tag, i);
            g_failures++;
            break;
        }
        if (r.view_bin >= r.view_M || r.view_M != s->M_lane ||
            r.view_stride != 1u ||
            r.view_decimation != s->D * s->M2_lane ||
            r.view_center_hz != center ||
            r.view_rate_hz != CHANNELIZER_BANK_OUTPUT_RATE_HZ)
        {
            printf("FAIL %s bredr[%zu] view\n", tag, i);
            g_failures++;
            sample_reader_destroy(&r);
            break;
        }
        sample_reader_destroy(&r);
    }
    /* Lane outputs exist for the partitioned dispatchers. */
    CHECK_TRUE(tag, channelizer_service_dispatcher_at(s, s->K) != NULL);
}

static void test_init_narrowband(void)
{
    sample_dispatcher_t *rf = make_rf();
    CHECK_TRUE("rf alloc", rf != NULL);
    if (!rf)
        return;

    channelizer_service_t s;
    memset(&s, 0, sizeof(s));
    channelizer_service_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sample_rate_hz = 20000000u;
    cfg.lo_hz          = 2411500000u;
    cfg.grid_hz        = 1000000u;
    CHECK_TRUE("narrowband init",
               channelizer_service_init(&s, rf, &cfg) == 0);
    CHECK_U64("narrowband K", s.K, 1u);
    CHECK_U64("narrowband D", s.D, 1u);
    CHECK_U64("narrowband dispatchers",
              channelizer_service_dispatcher_count(&s), 2u);
    CHECK_U64("narrowband bredr", s.bredr_count, 20u);
    CHECK_U64("narrowband ble", s.ble_count, 10u);
    /* Premix-only first stage: decim 1, shift is the grid residual, no FIR. */
    CHECK_U64("narrowband ddc decim", s.ddc[0].decim, 1u);
    CHECK_U64("narrowband ddc rate", s.ddc[0].sample_rate_out, 20000000u);
    CHECK_TRUE("narrowband ddc no fir", s.ddc[0].decim_fir == NULL);
    CHECK_U64("narrowband ddc shift",
              (uint64_t)(s.ddc[0].shift_hz + 1000000),
              (uint64_t)((int32_t)2412000000u - (int32_t)2411500000u +
                         1000000));
    CHECK_U64("narrowband rf readers", rf->reader_count, 1u);
    check_readers(&s, "narrowband");
    channelizer_service_destroy(&s);
    sample_dispatcher_destroy(rf);
    free(rf);
}

static void test_init_wideband(void)
{
    sample_dispatcher_t *rf = make_rf();
    CHECK_TRUE("rf alloc", rf != NULL);
    if (!rf)
        return;

    channelizer_service_t s;
    memset(&s, 0, sizeof(s));
    channelizer_service_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sample_rate_hz = 80000000u;
    /* "all" mode: full 0..78 band, LO on the exact band centre (2441 MHz,
     * on the 1 MHz raster, so no half-channel grid premix). */
    cfg.lo_hz   = 2441000000u;
    cfg.grid_hz = 1000000u;
    CHECK_TRUE("wideband init",
               channelizer_service_init(&s, rf, &cfg) == 0);
    CHECK_U64("wideband K", s.K, 4u);
    CHECK_U64("wideband M_lane", s.M_lane, 20u);
    CHECK_U64("wideband D", s.D, 4u);
    CHECK_U64("wideband dispatchers",
              channelizer_service_dispatcher_count(&s), 8u);
    CHECK_U64("wideband bredr (all 79)", s.bredr_count, 79u);
    CHECK_U64("wideband ble (all 40)", s.ble_count, 40u);
    /* No half-channel mix: LO already grid-aligned, every DDC shift is a
     * whole-MHz lane offset. */
    CHECK_U64("wideband lo_eff == lo", s.lo_eff_hz, 2441000000u);
    for (unsigned int k = 0u; k < s.K; k++)
        CHECK_U64("wideband ddc shift whole-MHz",
                  (uint64_t)((s.ddc[k].shift_hz % 1000000) + 1000000),
                  1000000u);
    check_readers(&s, "wideband");
    channelizer_service_destroy(&s);
    sample_dispatcher_destroy(rf);
    free(rf);
}

static void test_init_fallback(void)
{
    sample_dispatcher_t *rf = make_rf();
    CHECK_TRUE("rf alloc", rf != NULL);
    if (!rf)
        return;

    channelizer_service_t s;
    memset(&s, 0, sizeof(s));
    channelizer_service_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.sample_rate_hz = 10000000u; /* M=5 at 2 MHz -> odd */
    cfg.lo_hz          = 2407000000u;
    cfg.grid_hz        = 2000000u;
    CHECK_TRUE("fallback init",
               channelizer_service_init(&s, rf, &cfg) == 0);
    CHECK_U64("fallback grid", s.grid_actual_hz, 1000000u);
    channelizer_service_destroy(&s);
    sample_dispatcher_destroy(rf);
    free(rf);
}

int main(void)
{
    test_plan();
    test_init_narrowband();
    test_init_wideband();
    test_init_fallback();

    if (g_failures)
    {
        printf("test_channelizer_service_lanes: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_channelizer_service_lanes: OK\n");
    return 0;
}
