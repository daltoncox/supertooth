/* Channel-layout tests for the unified session (no RF hardware required).
 *
 * Verifies:
 *  - session_tune: LO / sample-rate derivation for the BR/EDR grid
 *    (even N, half-MHz LO) and the BLE grid (2N MHz, whole-MHz LO,
 *    BLE-only sessions).
 *  - session_create_channels_for_test: processor counts when only BLE is
 *    enabled, only BR/EDR enabled, and hybrid (shared 1 MHz bank: BLE fans
 *    out over the BR/EDR window; BLE-ref hybrid tunes are rejected).
 *  - RF <-> LE channel mapping helpers used by the fan-out math.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "session.h"

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

static int make_session(session_t *s, unsigned int bottom, unsigned int count,
                        session_protocol_ref_t ref, int enable_ble, int enable_bredr)
{
    session_config_t cfg = { .device_type = RADIO_DEVICE_HACKRF, .device_id = NULL, .debug = 0 };
    if (session_init(s, &cfg) != 0)
        return -1;
    if (enable_ble)
    {
        session_ble_config_t bc = { .enforce_crc = 1u };
        session_enable_ble(s, &bc, NULL, NULL);
    }
    if (enable_bredr)
    {
        session_bredr_config_t bc = { 0 };
        session_enable_bredr(s, &bc, NULL, NULL);
    }
    return session_tune(s, ref, bottom, count);
}

/* File-replay sessions see the generic 80 Msps ceiling (wideband tunes). */
static int make_file_session(session_t *s, unsigned int bottom,
                              unsigned int count, session_protocol_ref_t ref,
                              int enable_ble, int enable_bredr)
{
    session_config_t cfg = { .device_type = RADIO_DEVICE_FILE, .device_id = NULL, .debug = 0 };
    if (session_init(s, &cfg) != 0)
        return -1;
    if (enable_ble)
    {
        session_ble_config_t bc = { .enforce_crc = 1u };
        session_enable_ble(s, &bc, NULL, NULL);
    }
    if (enable_bredr)
    {
        session_bredr_config_t bc = { 0 };
        session_enable_bredr(s, &bc, NULL, NULL);
    }
    return session_tune(s, ref, bottom, count);
}

static void test_tune_layout(void)
{
    /* BR/EDR grid: even N, LO at a half-MHz, rate = N MHz. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("bredr N=20 b=0 tune", make_session(&s, 0, 20, SESSION_REF_BREDR, 0, 1), 0);
        CHECK_U64("bredr N=20 b=0 LO", s.lo_frequency_hz, 2411500000ULL);
        CHECK_U64("bredr N=20 b=0 rate", s.sample_rate_hz, 20000000u);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 40, 4, SESSION_REF_BREDR, 0, 1);
        CHECK_U64("bredr N=4 b=40 LO", s.lo_frequency_hz, 2443500000ULL);
        CHECK_U64("bredr N=4 b=40 rate", s.sample_rate_hz, 4000000u);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 2, SESSION_REF_BREDR, 0, 1);
        CHECK_U64("bredr N=2 b=0 rate (4 Msps)", s.sample_rate_hz, 4000000u);
        session_destroy(&s);
    }

    /* "all" mode (c=79): full 0..78 band at 80 Msps, LO on the exact band
     * centre (2441 MHz, whole MHz, so no half-channel premix). */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("all b=0 c=79 tune",
                  make_file_session(&s, 0, 79, SESSION_REF_BREDR, 0, 1), 0);
        CHECK_U64("all b=0 c=79 LO", s.lo_frequency_hz, 2441000000ULL);
        CHECK_U64("all b=0 c=79 rate", s.sample_rate_hz, 80000000u);
        session_destroy(&s);
    }

    /* BLE grid: 2N MHz window, whole-MHz LO. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 10, SESSION_REF_BLE, 1, 0);
        CHECK_U64("ble k0 n10 LO", s.lo_frequency_hz, 2411000000ULL);
        CHECK_U64("ble k0 n10 rate", s.sample_rate_hz, 20000000u);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 18, 2, SESSION_REF_BLE, 1, 0);
        CHECK_U64("ble k18 n2 LO", s.lo_frequency_hz, 2439000000ULL);
        CHECK_U64("ble k18 n2 rate", s.sample_rate_hz, 4000000u);
        session_destroy(&s);
    }
}

static void test_processor_counts(void)
{
    /* BLE-only: every LE RF channel in the window gets a processor. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 10, SESSION_REF_BLE, 1, 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("ble-only setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("ble-only ble count", ble_n, 10u);
        CHECK_U64("ble-only bredr count", bredr_n, 0u);
        session_destroy(&s);
    }

    /* BR/EDR-only: every BR/EDR channel inside the capture span. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 20, SESSION_REF_BREDR, 0, 1);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("bredr-only setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("bredr-only bredr count", bredr_n, 20u);
        CHECK_U64("bredr-only ble count", ble_n, 0u);
        session_destroy(&s);
    }

    /* Hybrid BR/EDR-ref: BLE fans out over the shared 1 MHz service (no
     * second bank). */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 20, SESSION_REF_BREDR, 1, 1);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("hybrid bredr-ref setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("hybrid bredr-ref bredr count", bredr_n, 20u);
        CHECK_U64("hybrid bredr-ref ble count", ble_n, 10u);
        /* Shared topology: single-lane 1 MHz service, one RF reader total,
         * BLE workers stride the 1 MHz lanes directly. */
        CHECK_U64("hybrid shared K==1", s.chan_svc.K, 1u);
        CHECK_U64("hybrid shared grid 1MHz",
                  s.chan_svc.grid_actual_hz, 1000000u);
        CHECK_U64("hybrid shared single rf reader",
                  s.dispatcher->reader_count, 1u);
        for (size_t i = 0u; i < ble_n; i++)
            CHECK_U64("hybrid shared ble stride 1",
                      s.ble_channels[i].reader.view_stride, 1u);
        session_destroy(&s);
    }

    /* Hybrid BLE-ref no longer exists: BLE fans out from the shared 1 MHz
     * bank, so a BLE-grid tune with both protocols enabled is rejected. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("hybrid ble-ref rejected",
                  make_session(&s, 0, 10, SESSION_REF_BLE, 1, 1), -1);
        session_destroy(&s);
    }

    /* Invalid configs are rejected. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("ble count=0 rejected", make_session(&s, 0, 0, SESSION_REF_BLE, 1, 0), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("ble window overflow rejected",
                  make_session(&s, 39, 2, SESSION_REF_BLE, 1, 0), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("ble count>40 rejected",
                  make_session(&s, 0, 41, SESSION_REF_BLE, 1, 0), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("ble 80Msps over ceiling rejected",
                  make_session(&s, 0, 40, SESSION_REF_BLE, 1, 0), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("bredr window overflow rejected",
                  make_session(&s, 78, 2, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("bredr count>79 rejected",
                  make_session(&s, 0, 80, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("all on hackrf over ceiling rejected",
                  make_session(&s, 0, 79, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("all with bottom!=0 rejected",
                  make_file_session(&s, 1, 79, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }
    CHECK_U64("tune NULL session rejected",
              session_tune(NULL, SESSION_REF_BLE, 0u, 2u), -1);
}

static void test_wideband_lanes(void)
{
    /* Lane-split allowlist: even C<=20, else K=ceil(C/20) lanes with
     * C%K==0 and even C/K<=20, plus 79 ("all": full 0..78 band at 80 Msps,
     * LO 2441 MHz). 80 as a count is still rejected (band holds 79). */
    static const unsigned int valid[] = {
        2u, 4u, 6u, 8u, 10u, 12u, 14u, 16u, 18u, 20u,
        24u, 28u, 32u, 36u, 40u, 42u, 48u, 54u, 60u, 64u, 72u,
        79u,
    };
    for (unsigned int c = 2u; c <= 80u; c += 2u)
    {
        int expect = 0;
        for (size_t i = 0u; i < sizeof(valid) / sizeof(valid[0]); i++)
            if (valid[i] == c)
                expect = 1;
        char name[64];
        snprintf(name, sizeof(name), "bredr C=%u plan", c);
        CHECK_U64(name, channelizer_service_valid_bredr_count(c), expect);
    }
    CHECK_U64("bredr C=79 plan (all)",
              channelizer_service_valid_bredr_count(79u), 1u);
    CHECK_U64("bredr C=80 rejected",
              channelizer_service_valid_bredr_count(80u), 0u);

    /* 40ch hybrid on file replay: K=2, M_lane=20, 40 BR/EDR + 20 BLE. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("wideband 40ch tune",
                  make_file_session(&s, 0, 40, SESSION_REF_BREDR, 1, 1), 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("wideband 40ch setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("wideband 40ch K", s.chan_svc.K, 2u);
        CHECK_U64("wideband 40ch M_lane", s.chan_svc.M_lane, 20u);
        CHECK_U64("wideband 40ch bredr count", bredr_n, 40u);
        CHECK_U64("wideband 40ch ble count", ble_n, 20u);
        CHECK_U64("wideband 40ch rf readers", s.dispatcher->reader_count, 2u);
        for (size_t i = 0u; i < bredr_n; i++)
        {
            if (s.bredr_channels[i].reader.view_M != 20u ||
                s.bredr_channels[i].reader.view_decimation != 20u)
            {
                printf("FAIL wideband 40ch bredr[%zu] M/decim\n", i);
                g_failures++;
                break;
            }
            if (s.bredr_channels[i].reader.view_bin >= 20u)
            {
                printf("FAIL wideband 40ch bredr[%zu] bin range\n", i);
                g_failures++;
                break;
            }
        }
        session_destroy(&s);
    }

    /* 72ch hybrid on file replay: K=4, M_lane=18. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("wideband 72ch tune",
                  make_file_session(&s, 0, 72, SESSION_REF_BREDR, 1, 1), 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("wideband 72ch setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("wideband 72ch K", s.chan_svc.K, 4u);
        CHECK_U64("wideband 72ch M_lane", s.chan_svc.M_lane, 18u);
        CHECK_U64("wideband 72ch bredr count", bredr_n, 72u);
        session_destroy(&s);
    }

    /* "all" hybrid on file replay: 80 Msps, K=4, M_lane=20, LO 2441 MHz
     * (whole MHz: no half-channel premix), 79 BR/EDR + 40 BLE. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("all 79ch tune",
                  make_file_session(&s, 0, 79, SESSION_REF_BREDR, 1, 1), 0);
        CHECK_U64("all 79ch LO", s.lo_frequency_hz, 2441000000ULL);
        CHECK_U64("all 79ch rate", s.sample_rate_hz, 80000000u);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("all 79ch setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("all 79ch K", s.chan_svc.K, 4u);
        CHECK_U64("all 79ch M_lane", s.chan_svc.M_lane, 20u);
        CHECK_U64("all 79ch grid", s.chan_svc.grid_actual_hz, 1000000u);
        CHECK_U64("all 79ch bredr count", bredr_n, 79u);
        CHECK_U64("all 79ch ble count", ble_n, 40u);
        CHECK_U64("all 79ch lo_eff == lo", s.chan_svc.lo_eff_hz,
                  s.lo_frequency_hz);
        for (unsigned int k = 0u; k < s.chan_svc.K; k++)
            CHECK_U64("all 79ch ddc shift whole-MHz",
                      (uint64_t)((s.chan_svc.ddc[k].shift_hz % 1000000) +
                                 1000000),
                      1000000u);
        session_destroy(&s);
    }

    /* "all" bredr-only: 79 BR/EDR processors, no BLE fan-out. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("all 79ch bredr-only tune",
                  make_file_session(&s, 0, 79, SESSION_REF_BREDR, 0, 1), 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("all 79ch bredr-only setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("all 79ch bredr-only bredr count", bredr_n, 79u);
        CHECK_U64("all 79ch bredr-only ble count", ble_n, 0u);
        session_destroy(&s);
    }

    /* Unsupported wideband counts are rejected even on file replay. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("wideband C=22 rejected",
                  make_file_session(&s, 0, 22, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("wideband C=78 rejected",
                  make_file_session(&s, 0, 78, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }

    /* Wideband is gated by the device ceiling: HackRF rejects 40ch. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("hackrf 40ch over ceiling rejected",
                  make_session(&s, 0, 40, SESSION_REF_BREDR, 0, 1), -1);
        session_destroy(&s);
    }

    /* BLE-only wideband: 40 LE RF channels at 80 Msps, K=4, 2 MHz bins. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        CHECK_U64("ble wideband 40ch tune",
                  make_file_session(&s, 0, 40, SESSION_REF_BLE, 1, 0), 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("ble wideband 40ch setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("ble wideband 40ch K", s.chan_svc.K, 4u);
        CHECK_U64("ble wideband 40ch grid 2MHz",
                  s.chan_svc.grid_actual_hz, 2000000u);
        CHECK_U64("ble wideband 40ch count", ble_n, 40u);
        for (size_t i = 0u; i < ble_n; i++)
            CHECK_U64("ble wideband stride 2",
                      s.ble_channels[i].reader.view_stride, 2u);
        session_destroy(&s);
    }

    /* BLE-only narrowband keeps the 2 MHz power-saver grid. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 10, SESSION_REF_BLE, 1, 0);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("ble narrow stride setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("ble narrow grid 2MHz",
                  s.chan_svc.grid_actual_hz, 2000000u);
        for (size_t i = 0u; i < ble_n; i++)
            CHECK_U64("ble narrow stride 2",
                      s.ble_channels[i].reader.view_stride, 2u);
        session_destroy(&s);
    }
}

static void test_validate_layout(void)
{
    /* Valid combos across devices and modes. */
    CHECK_U64("valid hackrf bredr 20",
              session_validate_layout(RADIO_DEVICE_HACKRF, 0, 1,
                                      SESSION_REF_BREDR, 0u, 20u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file bredr 40",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 40u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file bredr 72",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 72u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file bredr all (79)",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 79u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file hybrid 40",
              session_validate_layout(RADIO_DEVICE_FILE, 1, 1,
                                      SESSION_REF_BREDR, 0u, 40u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file hybrid all (79)",
              session_validate_layout(RADIO_DEVICE_FILE, 1, 1,
                                      SESSION_REF_BREDR, 0u, 79u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid hackrf ble 10",
              session_validate_layout(RADIO_DEVICE_HACKRF, 1, 0,
                                      SESSION_REF_BLE, 0u, 10u),
              SESSION_LAYOUT_OK);
    CHECK_U64("valid file ble 40",
              session_validate_layout(RADIO_DEVICE_FILE, 1, 0,
                                      SESSION_REF_BLE, 0u, 40u),
              SESSION_LAYOUT_OK);

    /* Range failures: empty, band overflow, hybrid BLE-ref. */
    CHECK_U64("count 0 rejected",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 0u),
              SESSION_LAYOUT_BAD_RANGE);
    CHECK_U64("bredr overflow rejected",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 78u, 2u),
              SESSION_LAYOUT_BAD_RANGE);
    CHECK_U64("all with bottom!=0 rejected",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 1u, 79u),
              SESSION_LAYOUT_BAD_RANGE);
    CHECK_U64("ble overflow rejected",
              session_validate_layout(RADIO_DEVICE_FILE, 1, 0,
                                      SESSION_REF_BLE, 39u, 2u),
              SESSION_LAYOUT_BAD_RANGE);
    CHECK_U64("hybrid ble-ref rejected",
              session_validate_layout(RADIO_DEVICE_FILE, 1, 1,
                                      SESSION_REF_BLE, 0u, 10u),
              SESSION_LAYOUT_BAD_RANGE);

    /* Device ceiling gates wideband on narrowband radios. */
    CHECK_U64("hackrf 40ch over ceiling",
              session_validate_layout(RADIO_DEVICE_HACKRF, 0, 1,
                                      SESSION_REF_BREDR, 0u, 40u),
              SESSION_LAYOUT_RATE_EXCEEDED);
    CHECK_U64("hackrf all (79) over ceiling",
              session_validate_layout(RADIO_DEVICE_HACKRF, 0, 1,
                                      SESSION_REF_BREDR, 0u, 79u),
              SESSION_LAYOUT_RATE_EXCEEDED);
    CHECK_U64("hackrf ble 20ch over ceiling",
              session_validate_layout(RADIO_DEVICE_HACKRF, 1, 0,
                                      SESSION_REF_BLE, 0u, 20u),
              SESSION_LAYOUT_RATE_EXCEEDED);

    /* Lane-split gaps fail even where the ceiling allows. */
    CHECK_U64("file 22ch no split",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 22u),
              SESSION_LAYOUT_NO_LANE_SPLIT);
    CHECK_U64("file 78ch no split",
              session_validate_layout(RADIO_DEVICE_FILE, 0, 1,
                                      SESSION_REF_BREDR, 0u, 78u),
              SESSION_LAYOUT_NO_LANE_SPLIT);

    /* Wrappers. */
    CHECK_U64("default hackrf", session_default_bredr_count(RADIO_DEVICE_HACKRF),
              20u);
    CHECK_U64("default file", session_default_bredr_count(RADIO_DEVICE_FILE),
              79u);
    CHECK_U64("snap 22", session_snap_bredr_count(22u), 20u);
    CHECK_U64("snap 78", session_snap_bredr_count(78u), 72u);
    CHECK_U64("snap 79", session_snap_bredr_count(79u), 79u);
    CHECK_U64("snap 80", session_snap_bredr_count(80u), 79u);
    CHECK_U64("hackrf max rate", session_device_max_rate_hz(RADIO_DEVICE_HACKRF),
              20000000u);
    CHECK_U64("hackrf type name non-null",
              session_device_type_name(RADIO_DEVICE_HACKRF) != NULL, 1u);

    /* No-drift property: tune agrees with the validator on every case. */
    {
        static const struct {
            int file;
            int ble_en, bredr_en;
            session_protocol_ref_t ref;
            unsigned int bottom, count;
        } cases[] = {
            { 0, 0, 1, SESSION_REF_BREDR, 0u, 20u },
            { 0, 0, 1, SESSION_REF_BREDR, 0u, 40u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 40u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 72u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 79u },
            { 1, 1, 1, SESSION_REF_BREDR, 0u, 79u },
            { 0, 0, 1, SESSION_REF_BREDR, 0u, 79u },
            { 1, 1, 1, SESSION_REF_BREDR, 0u, 40u },
            { 0, 1, 0, SESSION_REF_BLE, 0u, 10u },
            { 1, 1, 0, SESSION_REF_BLE, 0u, 40u },
            { 1, 1, 0, SESSION_REF_BLE, 0u, 20u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 22u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 78u },
            { 1, 0, 1, SESSION_REF_BREDR, 78u, 2u },
            { 1, 1, 1, SESSION_REF_BLE, 0u, 10u },
            { 1, 0, 1, SESSION_REF_BREDR, 0u, 0u },
        };
        for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); i++)
        {
            radio_device_type_t dev = cases[i].file ? RADIO_DEVICE_FILE
                                                    : RADIO_DEVICE_HACKRF;
            session_layout_status_t want = session_validate_layout(
                dev, cases[i].ble_en, cases[i].bredr_en, cases[i].ref,
                cases[i].bottom, cases[i].count);
            session_t s;
            int tune_rc;
            memset(&s, 0, sizeof(s));
            if (cases[i].file)
                tune_rc = make_file_session(&s, cases[i].bottom,
                                            cases[i].count, cases[i].ref,
                                            cases[i].ble_en,
                                            cases[i].bredr_en);
            else
                tune_rc = make_session(&s, cases[i].bottom, cases[i].count,
                                       cases[i].ref, cases[i].ble_en,
                                       cases[i].bredr_en);
            {
                char name[64];
                snprintf(name, sizeof(name), "tune==validate case %zu", i);
                CHECK_U64(name, tune_rc == 0,
                          want == SESSION_LAYOUT_OK);
            }
            session_destroy(&s);
        }
    }
}

static void test_rf_mapping(void)
{
    CHECK_U64("rf0 -> LE37", ble_channel_number_for_rf(0), 37u);
    CHECK_U64("rf1 -> LE0", ble_channel_number_for_rf(1), 0u);
    CHECK_U64("rf12 -> LE38", ble_channel_number_for_rf(12), 38u);
    CHECK_U64("rf13 -> LE11", ble_channel_number_for_rf(13), 11u);
    CHECK_U64("rf39 -> LE39", ble_channel_number_for_rf(39), 39u);
    CHECK_U64("rf0 is adv", ble_rf_is_advertising(0), 1);
    CHECK_U64("rf12 is adv", ble_rf_is_advertising(12), 1);
    CHECK_U64("rf39 is adv", ble_rf_is_advertising(39), 1);
    CHECK_U64("rf24 not adv", ble_rf_is_advertising(24), 0);
}

int main(void)
{
    test_tune_layout();
    test_processor_counts();
    test_wideband_lanes();
    test_validate_layout();
    test_rf_mapping();

    if (g_failures)
    {
        printf("test_channel_layout: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_channel_layout: OK\n");
    return 0;
}
