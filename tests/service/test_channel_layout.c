/* Channel-layout tests for the unified session (no RF hardware required).
 *
 * Verifies:
 *  - session_tune: LO / sample-rate derivation for the BR/EDR grid
 *    (even N, half-MHz LO) and the BLE grid (2N MHz, whole-MHz LO).
 *  - session_create_channels_for_test: processor counts when only BLE is
 *    enabled, only BR/EDR enabled, and when the non-reference protocol fans
 *    out over the reference window (hybrid fan-out).
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

    /* Hybrid BR/EDR-ref: BLE fans out over the BR/EDR capture window. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 20, SESSION_REF_BREDR, 1, 1);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("hybrid bredr-ref setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("hybrid bredr-ref bredr count", bredr_n, 20u);
        CHECK_U64("hybrid bredr-ref ble count", ble_n, 10u);
        session_destroy(&s);
    }

    /* Hybrid BLE-ref: BR/EDR fans out inside the BLE capture window. */
    {
        session_t s;
        memset(&s, 0, sizeof(s));
        make_session(&s, 0, 10, SESSION_REF_BLE, 1, 1);
        size_t ble_n = 0, bredr_n = 0;
        CHECK_U64("hybrid ble-ref setup",
                  session_create_channels_for_test(&s, &ble_n, &bredr_n), 0);
        CHECK_U64("hybrid ble-ref ble count", ble_n, 10u);
        /* The 20 MHz BLE window spans 2402..2420 MHz; BR/EDR channels with
         * centers strictly inside that (ch0..ch18) are 19 processors.
         * Channel 19 at 2421 MHz is at the Nyquist edge and excluded by the
         * channelizer's asymmetric bin range [-M/2, +M/2-1]. */
        CHECK_U64("hybrid ble-ref bredr count", bredr_n, 19u);
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
    test_rf_mapping();

    if (g_failures)
    {
        printf("test_channel_layout: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_channel_layout: OK\n");
    return 0;
}
