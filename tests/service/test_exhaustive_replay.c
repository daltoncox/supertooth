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
#define FIXTURE_LO_HZ 2403500000ull

static void check_zero_drops(session_t *session, const char *tag)
{
    session_drop_breakdown_t drops;
    unsigned long ble_dropped = 999ul, bredr_dropped = 999ul;

    if (session_dropped_blocks(session) != 0ul)
    {
        fprintf(stderr, "ASSERT FAILED %s: session_dropped_blocks != 0\n",
                tag);
        g_failures++;
    }
    memset(&drops, 0xA5, sizeof(drops));
    session_dropped_blocks_breakdown(session, &drops);
    if (drops.rf_pool_exhausted != 0ul ||
        drops.rf_consumer_full != 0ul ||
        drops.bredr_out_pool_exhausted != 0ul ||
        drops.bredr_out_consumer_full != 0ul ||
        drops.ble_out_pool_exhausted != 0ul ||
        drops.ble_out_consumer_full != 0ul)
    {
        fprintf(stderr, "ASSERT FAILED %s: drop breakdown nonzero\n", tag);
        g_failures++;
    }
    session_collector_dropped(session, &ble_dropped, &bredr_dropped);
    if (ble_dropped != 0ul || bredr_dropped != 0ul)
    {
        fprintf(stderr, "ASSERT FAILED %s: collector drops nonzero\n", tag);
        g_failures++;
    }
}

/* Replay the fixture in exhaustive mode under one protocol configuration.
 * The fixture rate/center match the BREDR N=4 tune; the BLE-only tune uses
 * RF channels 0-1 at 4 Msps so the same file stays rate-compatible. */
static void run_once(const char *path, int enable_ble, int enable_bredr,
                     session_protocol_ref_t ref, unsigned int bottom,
                     unsigned int count, const char *tag)
{
    session_t session;
    memset(&session, 0, sizeof(session));
    {
        session_config_t cfg = {
            .device_type = RADIO_DEVICE_FILE,
            .device_id = path,
            .debug = 0,
            .file_exhaustive = 1,
        };
        session_ble_config_t lcfg = {.enforce_crc = 1u};
        session_bredr_config_t bcfg = {0};
        if (session_init(&session, &cfg) != 0)
        {
            fprintf(stderr, "ASSERT FAILED %s: session_init\n", tag);
            g_failures++;
            return;
        }
        if (enable_ble)
            session_enable_ble(&session, &lcfg, NULL, NULL);
        if (enable_bredr)
            session_enable_bredr(&session, &bcfg, NULL, NULL);
        if (session_tune(&session, ref, bottom, count) != 0)
        {
            fprintf(stderr, "ASSERT FAILED %s: session_tune\n", tag);
            g_failures++;
            session_destroy(&session);
            return;
        }
        if (session_run(&session) != 0)
        {
            fprintf(stderr, "ASSERT FAILED %s: session_run\n", tag);
            g_failures++;
        }
    }

    check_zero_drops(&session, tag);
    session_destroy(&session);
}

int main(void)
{
    char path[256];

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
                                   FIXTURE_LO_HZ, &w) == 0);
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

    /* BREDR-only, BLE-only (RF channels 0-1, 4 Msps) and hybrid (shared
     * BREDR-ref window) must all replay drop-free. */
    run_once(path, 0, 1, SESSION_REF_BREDR, 0u, 4u, "bredr-only");
    run_once(path, 1, 0, SESSION_REF_BLE, 0u, 2u, "ble-only");
    run_once(path, 1, 1, SESSION_REF_BREDR, 0u, 4u, "hybrid");

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
