/**
 * @file test_exhaustive_replay.c
 * @brief End-to-end true-exhaustive replay regression test.
 *
 * Writes a small synthetic WAV capture, replays it through a full session
 * (channelizers + workers + collectors) in exhaustive mode, and asserts
 * every pipeline drop counter reads zero: all three block dispatchers and
 * both event collectors. This is the test the RF-stage-only mode failed
 * (downstream consumer_full drops) and true exhaustive mode must pass.
 */

#include <complex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "session.h"
#include "wav.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(cond))                                                                   \
        {                                                                              \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                              \
        }                                                                              \
    } while (0)

#define FIXTURE_RATE_HZ 4000000u
#define FIXTURE_FRAMES 2000000u /* 0.5 s at 4 Msps */

int main(void)
{
    char path[256];
    session_t session;
    session_drop_breakdown_t drops;
    unsigned long ble_dropped = 999ul, bredr_dropped = 999ul;

    snprintf(path, sizeof(path), "/tmp/test_exhaustive_replay_%d.wav",
             (int)getpid());

    /* Deterministic noise fixture: unlikely to decode, exercises the full
     * sample path (RF -> channelizer -> workers) without depending on
     * protocol content. */
    {
        static float complex tx[8192];
        wav_writer_t w;
        uint32_t left = FIXTURE_FRAMES;
        uint32_t lcg = 0x12345678u;
        TEST_ASSERT(wav_write_open(path, FIXTURE_RATE_HZ, WAV_SAMP_S16,
                                   2403500000ull, &w) == 0);
        while (left > 0u)
        {
            size_t n = left > 8192u ? 8192u : left;
            for (size_t i = 0u; i < n; i++)
            {
                lcg = lcg * 1664525u + 1013904223u;
                float v = (float)(lcg >> 8) / 8388608.0f - 1.0f;
                tx[i] = 0.25f * v + 0.25f * v * _Complex_I;
            }
            TEST_ASSERT(wav_write_frames(&w, tx, n) == 0);
            left -= (uint32_t)n;
        }
        TEST_ASSERT(wav_write_close(&w) == 0);
    }

    memset(&session, 0, sizeof(session));
    {
        session_config_t cfg = {
            .device_type = RADIO_DEVICE_FILE,
            .device_id = path,
            .debug = 0,
            .file_exhaustive = 1,
        };
        session_bredr_config_t bcfg = {0};
        TEST_ASSERT(session_init(&session, &cfg) == 0);
        session_enable_bredr(&session, &bcfg, NULL, NULL);
        TEST_ASSERT(session_tune(&session, SESSION_REF_BREDR, 0u, 4u) == 0);
        TEST_ASSERT(session_run(&session) == 0);
    }

    TEST_ASSERT(session_dropped_blocks(&session) == 0ul);
    memset(&drops, 0xA5, sizeof(drops));
    session_dropped_blocks_breakdown(&session, &drops);
    TEST_ASSERT(drops.rf_pool_exhausted == 0ul);
    TEST_ASSERT(drops.rf_consumer_full == 0ul);
    TEST_ASSERT(drops.bredr_out_pool_exhausted == 0ul);
    TEST_ASSERT(drops.bredr_out_consumer_full == 0ul);
    TEST_ASSERT(drops.ble_out_pool_exhausted == 0ul);
    TEST_ASSERT(drops.ble_out_consumer_full == 0ul);
    session_collector_dropped(&session, &ble_dropped, &bredr_dropped);
    TEST_ASSERT(ble_dropped == 0ul);
    TEST_ASSERT(bredr_dropped == 0ul);

    session_destroy(&session);
    unlink(path);

    if (g_failures)
    {
        fprintf(stderr, "test_exhaustive_replay: %d failure(s)\n",
                g_failures);
        return 1;
    }
    printf("test_exhaustive_replay: ok\n");
    return 0;
}
