/* Channel-layout tests for the BR/EDR and BLE channelizers (no RF
 * hardware required).
 *
 * Verifies:
 *  - receiver_bredr_update_layout: LO/sample-rate/decimation on the
 *    BR/EDR grid (even N, half-MHz LO) and the LE grid (odd N, whole-MHz
 *    LO, span widened by 1 MHz).
 *  - receiver_ble_channel_processor_setup (SESSION pipeline): LE window
 *    layout, per-channel active flags (every channel in the window is
 *    decoded, advertising and data alike), NCO/decimation wiring, and
 *    config validation.
 *  - receiver_ble_channel_processor_setup (HYBRID pipeline): BLE fan-out
 *    over every LE channel fully inside the bredr capture window.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "receiver_session.h"
#include "ble_channel_processor.h"
#include "bredr_channel_processor.h"

static int g_failures = 0;

#define CHECK_U64(name, actual, expected)                                     \
    do {                                                                      \
        uint64_t a_ = (uint64_t)(actual), e_ = (uint64_t)(expected);          \
        if (a_ != e_) {                                                       \
            printf("FAIL %-26s expected %llu got %llu\n", name,               \
                   (unsigned long long)e_, (unsigned long long)a_);           \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static void bredr_layout(unsigned int count, unsigned int bottom,
                         unsigned int le_grid,
                         uint64_t *lo_hz, unsigned int *rate)
{
    receiver_session_t *s = receiver_session_create();
    receiver_bredr_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.channel_count = count;
    cfg.bottom_channel = bottom;
    cfg.le_grid = le_grid;
    cfg.rssi_averaging_window = RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW;

    receiver_bredr_callbacks_t cb;
    memset(&cb, 0, sizeof(cb));
    receiver_bredr_session_init(s, &cfg, &cb);   /* runs update_layout */

    *lo_hz = s->bredr_lo_freq_hz;
    *rate = s->bredr_sample_rate;
    receiver_session_destroy(s);
}

static void test_bredr_grid_layout(void)
{
    uint64_t lo;
    unsigned int rate;

    /* BR/EDR grid regressions (pre-le_grid behavior). */
    bredr_layout(20, 0, RECEIVER_BREDR_GRID_BREDR, &lo, &rate);
    CHECK_U64("bredr N=20 b=0 LO", lo, 2411500000ULL);
    CHECK_U64("bredr N=20 b=0 rate", rate, 20000000u);

    bredr_layout(4, 40, RECEIVER_BREDR_GRID_BREDR, &lo, &rate);
    CHECK_U64("bredr N=4 b=40 LO", lo, 2443500000ULL);
    CHECK_U64("bredr N=4 b=40 rate", rate, 4000000u);

    bredr_layout(2, 0, RECEIVER_BREDR_GRID_BREDR, &lo, &rate);
    CHECK_U64("bredr N=2 b=0 rate (4 Msps)", rate, 4000000u);

    /* LE grid: N = c-1, window c MHz wide, LO at a whole MHz. */
    bredr_layout(19, 0, RECEIVER_BREDR_GRID_LE, &lo, &rate);
    CHECK_U64("le N=19 b=0 LO (whole MHz)", lo, 2411000000ULL);
    CHECK_U64("le N=19 b=0 rate", rate, 20000000u);

    bredr_layout(19, 48, RECEIVER_BREDR_GRID_LE, &lo, &rate);
    CHECK_U64("le N=19 b=48 LO", lo, 2459000000ULL);

    bredr_layout(1, 0, RECEIVER_BREDR_GRID_LE, &lo, &rate);
    CHECK_U64("le N=1 b=0 LO", lo, 2402000000ULL);
    CHECK_U64("le N=1 b=0 rate (4 Msps)", rate, 4000000u);
}

static receiver_session_t *make_ble_session(unsigned int bottom, unsigned int count)
{
    receiver_session_t *s = receiver_session_create();
    memset(&s->ble_config, 0, sizeof(s->ble_config));
    s->ble_config.bottom_le_channel = bottom;
    s->ble_config.le_channel_count = count;
    return s;
}

static unsigned int count_active(const receiver_session_t *s)
{
    unsigned int n = 0;
    for (unsigned int i = 0; i < s->ble_ctx_count; i++)
        n += s->ble_ctx[i].active ? 1u : 0u;
    return n;
}

static void test_ble_session_layout(void)
{
    /* RF <-> LE channel mapping. */
    CHECK_U64("rf0 -> LE37", ble_channel_number_for_rf(0), 37u);
    CHECK_U64("rf1 -> LE0", ble_channel_number_for_rf(1), 0u);
    CHECK_U64("rf12 -> LE38", ble_channel_number_for_rf(12), 38u);
    CHECK_U64("rf13 -> LE11", ble_channel_number_for_rf(13), 11u);
    CHECK_U64("rf39 -> LE39", ble_channel_number_for_rf(39), 39u);
    CHECK_U64("rf0 is adv", ble_rf_is_advertising(0), 1);
    CHECK_U64("rf12 is adv", ble_rf_is_advertising(12), 1);
    CHECK_U64("rf39 is adv", ble_rf_is_advertising(39), 1);
    CHECK_U64("rf24 not adv", ble_rf_is_advertising(24), 0);

    /* Full window: bottom=0, count=10 -> LO 2411, 20 Msps, decim 10;
     * every channel active (RF0 = LE37 advertising, rest data). */
    {
        receiver_session_t *s = make_ble_session(0, 10);
        CHECK_U64("setup k=0 n=10 ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), 0);
        CHECK_U64("k0n10 LO", s->ble_lo_freq_hz, 2411000000ULL);
        CHECK_U64("k0n10 rate", s->ble_sample_rate, 20000000u);
        CHECK_U64("k0n10 decim", s->ble_decim_factor, 10u);
        CHECK_U64("k0n10 ctx count", s->ble_ctx_count, 10u);
        CHECK_U64("k0n10 ctx0 active", s->ble_ctx[0].active, 1u);
        CHECK_U64("k0n10 ctx0 channel", s->ble_ctx[0].channel_index, 37u);
        CHECK_U64("k0n10 ctx0 decim", s->ble_ctx[0].input_decimation, 10u);
        CHECK_U64("k0n10 ctx1 (data) active", s->ble_ctx[1].active, 1u);
        CHECK_U64("k0n10 active count", count_active(s), 10u);
        receiver_ble_channel_processor_destroy(s);
        CHECK_U64("destroy resets ctx count", s->ble_ctx_count, 0u);
        receiver_session_destroy(s);
    }

    /* Window RF10..13: ch38 (RF12) at +1 MHz offset, LO 2425; all active. */
    {
        receiver_session_t *s = make_ble_session(10, 4);
        CHECK_U64("setup k=10 n=4 ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), 0);
        CHECK_U64("k10n4 LO", s->ble_lo_freq_hz, 2425000000ULL);
        CHECK_U64("k10n4 rate", s->ble_sample_rate, 8000000u);
        CHECK_U64("k10n4 ctx2 ch38 active", s->ble_ctx[2].active, 1u);
        CHECK_U64("k10n4 ctx2 channel", s->ble_ctx[2].channel_index, 38u);
        CHECK_U64("k10n4 active count", count_active(s), 4u);
        receiver_ble_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }

    /* Single-channel windows (CLI parity): LO = channel center, 4 Msps. */
    {
        receiver_session_t *s = make_ble_session(0, 1);
        receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION);
        CHECK_U64("k0n1 LO", s->ble_lo_freq_hz, 2402000000ULL);
        CHECK_U64("k0n1 rate", s->ble_sample_rate, 4000000u);
        CHECK_U64("k0n1 decim", s->ble_decim_factor, 2u);
        receiver_ble_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }
    {
        receiver_session_t *s = make_ble_session(39, 1);
        receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION);
        CHECK_U64("k39n1 LO", s->ble_lo_freq_hz, 2480000000ULL);
        CHECK_U64("k39n1 ctx0 ch39", s->ble_ctx[0].channel_index, 39u);
        receiver_ble_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }

    /* Data-only window (RF2..3): every channel is decoded now. */
    {
        receiver_session_t *s = make_ble_session(2, 2);
        CHECK_U64("setup data-only window ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), 0);
        CHECK_U64("data-only window active count", count_active(s), 2u);
        CHECK_U64("data-only ctx0 channel", s->ble_ctx[0].channel_index, 1u);
        receiver_ble_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }

    /* Invalid configs are rejected. */
    {
        receiver_session_t *s = make_ble_session(0, 0);
        CHECK_U64("count=0 rejected",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), -1);
        receiver_session_destroy(s);
    }
    {
        receiver_session_t *s = make_ble_session(39, 2);
        CHECK_U64("window overflow rejected",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), -1);
        receiver_session_destroy(s);
    }
    {
        receiver_session_t *s = make_ble_session(0, 11);
        CHECK_U64("count>max rejected",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_SESSION), -1);
        receiver_session_destroy(s);
    }
}

static receiver_session_t *make_hybrid_session(unsigned int bottom,
                                               unsigned int count,
                                               unsigned int le_grid)
{
    receiver_session_t *s = receiver_session_create();
    receiver_bredr_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.channel_count = count;
    cfg.bottom_channel = bottom;
    cfg.le_grid = le_grid;
    cfg.rssi_averaging_window = RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW;

    receiver_bredr_callbacks_t cb;
    memset(&cb, 0, sizeof(cb));
    receiver_bredr_session_init(s, &cfg, &cb);
    if (receiver_bredr_channel_processor_setup(s) != 0)
        printf("WARN: bredr processor setup failed in test harness\n");
    s->hybrid_ble_enabled = 1u;
    return s;
}

static void test_ble_hybrid_fanout(void)
{
    /* 20 MHz BR/EDR window from ch0: LE RF 0..9 fit (ch37 + data 0-8). */
    {
        receiver_session_t *s = make_hybrid_session(0, 20, RECEIVER_BREDR_GRID_BREDR);
        CHECK_U64("hybrid b0 n20 setup ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_HYBRID), 0);
        CHECK_U64("hybrid b0n20 ble ctx count", s->ble_ctx_count, 10u);
        CHECK_U64("hybrid b0n20 ctx0 ch", s->ble_ctx[0].channel_index, 37u);
        CHECK_U64("hybrid b0n20 ctx1 ch", s->ble_ctx[1].channel_index, 0u);
        CHECK_U64("hybrid b0n20 ctx9 ch", s->ble_ctx[9].channel_index, 8u);
        CHECK_U64("hybrid b0n20 ctx0 active", s->ble_ctx[0].active, 1u);
        CHECK_U64("hybrid b0n20 ctx9 active", s->ble_ctx[9].active, 1u);
        receiver_ble_channel_processor_destroy(s);
        receiver_bredr_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }

    /* 8 MHz window from ch2: LE RF 1..4 fit (data ch 0-3, no advertising). */
    {
        receiver_session_t *s = make_hybrid_session(2, 8, RECEIVER_BREDR_GRID_BREDR);
        CHECK_U64("hybrid b2 n8 setup ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_HYBRID), 0);
        CHECK_U64("hybrid b2n8 ble ctx count", s->ble_ctx_count, 4u);
        CHECK_U64("hybrid b2n8 ctx0 ch", s->ble_ctx[0].channel_index, 0u);
        CHECK_U64("hybrid b2n8 ctx3 ch", s->ble_ctx[3].channel_index, 3u);
        receiver_ble_channel_processor_destroy(s);
        receiver_bredr_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }

    /* LE grid (window widened by 1 MHz, whole-MHz LO): same LE set as the
     * BR/EDR grid for the equivalent span. */
    {
        receiver_session_t *s = make_hybrid_session(0, 19, RECEIVER_BREDR_GRID_LE);
        CHECK_U64("hybrid LE-grid b0 n19 setup ok",
                  receiver_ble_channel_processor_setup(s, RECEIVER_BLE_PIPELINE_HYBRID), 0);
        CHECK_U64("hybrid LE b0n19 ble ctx count", s->ble_ctx_count, 10u);
        CHECK_U64("hybrid LE b0n19 ctx0 ch", s->ble_ctx[0].channel_index, 37u);
        receiver_ble_channel_processor_destroy(s);
        receiver_bredr_channel_processor_destroy(s);
        receiver_session_destroy(s);
    }
}

int main(void)
{
    test_bredr_grid_layout();
    test_ble_session_layout();
    test_ble_hybrid_fanout();

    if (g_failures)
    {
        printf("test_channel_layout: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_channel_layout: OK\n");
    return 0;
}
