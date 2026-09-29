/**
 * @file test_file_device.c
 * @brief Tests for the file-backed fake radio device.
 *
 * Writes a small WAV capture with the shared writer, then verifies the
 * backend: header facts, compatibility checking (match + mismatch), block
 * streaming through a dispatcher with continuous sample indices, and
 * single-pass EOF signaling.
 */

#include <complex.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "file.h"
#include "sample_dispatcher.h"
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

#define TEST_FRAMES 100000u
#define TEST_RATE 4000000u
#define TEST_LO 2406500000ull

int main(void)
{
    char path[256];
    snprintf(path, sizeof(path), "/tmp/test_file_device_%d.wav",
             (int)getpid());

    /* Build a fixture capture: constant 0.5+0.25j tone. */
    {
        static float complex tx[8192];
        wav_writer_t w;
        uint32_t left = TEST_FRAMES;
        for (size_t i = 0u; i < 8192u; i++)
            tx[i] = 0.5f + 0.25f * _Complex_I;
        TEST_ASSERT(wav_write_open(path, TEST_RATE, WAV_SAMP_S16, TEST_LO,
                                   &w) == 0);
        while (left > 0u)
        {
            size_t n = left > 8192u ? 8192u : left;
            TEST_ASSERT(wav_write_frames(&w, tx, n) == 0);
            left -= (uint32_t)n;
        }
        TEST_ASSERT(wav_write_close(&w) == 0);
    }

    /* Compatibility pre-check: match ok, mismatch + missing rejected. */
    {
        uint32_t rate = 0u;
        uint64_t center = 0u;
        TEST_ASSERT(file_radio_check_compatible(path, TEST_RATE, &rate,
                                                &center) == 0);
        TEST_ASSERT(rate == TEST_RATE);
        TEST_ASSERT(center == TEST_LO);
        TEST_ASSERT(file_radio_check_compatible(path, TEST_RATE * 2u, NULL,
                                                NULL) != 0);
        TEST_ASSERT(file_radio_check_compatible("/nonexistent/x.wav",
                                                TEST_RATE, NULL,
                                                NULL) != 0);
    }

    /* Stream it through a dispatcher like session_run() would. Note: the
     * dispatcher owns ~128 MB of blocks, so it must live on the heap. */
    {
        sample_dispatcher_t *dispatcher =
            (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
        sample_reader_t reader;
        _Atomic unsigned int stop = 0u;
        void *dev = NULL;
        radio_stream_config_t cfg = {
            .lo_freq_hz = (uint64_t)TEST_LO,
            .sample_rate = TEST_RATE,
            .gain = {0},
        };
        uint64_t total = 0u, expect_base = 0u;
        int pops = 0;

        TEST_ASSERT(dispatcher != NULL);
        TEST_ASSERT(sample_dispatcher_init(dispatcher) == 0);
        TEST_ASSERT(sample_reader_init(&reader, dispatcher) == 0);
        TEST_ASSERT(file_radio_open(&dev, path, dispatcher, 0) == 0);
        TEST_ASSERT(file_radio_configure(dev, &cfg) == 0);
        TEST_ASSERT(file_radio_get_file_sample_rate(dev) == TEST_RATE);
        TEST_ASSERT(file_radio_get_file_center_hz(dev) == TEST_LO);
        TEST_ASSERT(file_radio_is_finished(dev) == 0);
        TEST_ASSERT(file_radio_start_rx(dev) == 0);

        /* Pull every block; indices must be gapless from 0..FRAMES. */
        while (total < TEST_FRAMES && pops < 64)
        {
            sample_block_t *block = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
            if (!block)
                break;
            TEST_ASSERT(block->block_base_sample == expect_base);
            TEST_ASSERT(block->num_samples > 0u);
            /* Sample-value fidelity: the fixture tone is 0.5+0.25j. */
            if (pops == 0)
            {
                TEST_ASSERT(fabsf(crealf(block->samples[0]) - 0.5f) < 0.01f);
                TEST_ASSERT(fabsf(cimagf(block->samples[0]) - 0.25f) < 0.01f);
            }
            total += block->num_samples;
            expect_base += block->num_samples;
            sample_block_release(block);
            pops++;
        }
        TEST_ASSERT(total == TEST_FRAMES);

        /* EOF must be signaled shortly after the last sample. */
        {
            int finished = 0;
            for (int i = 0; i < 200 && !finished; i++)
            {
                struct timespec ts = {.tv_sec = 0, .tv_nsec = 10000000L};
                nanosleep(&ts, NULL);
                finished = file_radio_is_finished(dev);
            }
            TEST_ASSERT(finished != 0);
        }

        TEST_ASSERT(file_radio_stop_rx(dev) == 0);
        file_radio_close(dev);
        sample_reader_destroy(&reader);
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
    }

    /* Exhaustive replay of the same fixture: identical samples, and the
     * backpressure path must report zero drops anywhere. */
    {
        sample_dispatcher_t *dispatcher =
            (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
        sample_reader_t reader;
        _Atomic unsigned int stop = 0u;
        void *dev = NULL;
        radio_stream_config_t cfg = {
            .lo_freq_hz = (uint64_t)TEST_LO,
            .sample_rate = TEST_RATE,
            .gain = {0},
        };
        uint64_t total = 0u, expect_base = 0u;

        TEST_ASSERT(dispatcher != NULL);
        TEST_ASSERT(sample_dispatcher_init(dispatcher) == 0);
        TEST_ASSERT(sample_reader_init(&reader, dispatcher) == 0);
        TEST_ASSERT(file_radio_open(&dev, path, dispatcher, 0) == 0);
        TEST_ASSERT(file_radio_configure(dev, &cfg) == 0);
        file_radio_set_exhaustive(dev, 1);
        TEST_ASSERT(file_radio_start_rx(dev) == 0);

        while (total < TEST_FRAMES)
        {
            sample_block_t *block = NULL;
            TEST_ASSERT(sample_reader_wait_pop(&reader, &stop, &block) == 0);
            if (!block)
                break;
            TEST_ASSERT(block->block_base_sample == expect_base);
            total += block->num_samples;
            expect_base += block->num_samples;
            sample_block_release(block);
        }
        TEST_ASSERT(total == TEST_FRAMES);
        TEST_ASSERT(sample_dispatcher_total_dropped(dispatcher) == 0ul);

        TEST_ASSERT(file_radio_stop_rx(dev) == 0);
        file_radio_close(dev);
        sample_reader_destroy(&reader);
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
    }

    /* Lifecycle edges: rate mismatch, double start, idle stop. */
    {
        sample_dispatcher_t *dispatcher =
            (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
        void *dev = NULL;
        radio_stream_config_t cfg = {
            .lo_freq_hz = (uint64_t)TEST_LO, .sample_rate = TEST_RATE,
        };
        radio_stream_config_t bad = {
            .lo_freq_hz = (uint64_t)TEST_LO, .sample_rate = TEST_RATE * 2u,
        };
        TEST_ASSERT(sample_dispatcher_init(dispatcher) == 0);
        TEST_ASSERT(file_radio_open(&dev, path, dispatcher, 0) == 0);
        /* Rate mismatch is rejected at configure time. */
        TEST_ASSERT(file_radio_configure(dev, &bad) != 0);
        TEST_ASSERT(file_radio_configure(dev, &cfg) == 0);
        /* Not finished before the first start. */
        TEST_ASSERT(file_radio_is_finished(dev) == 0);
        TEST_ASSERT(file_radio_start_rx(dev) == 0);
        /* Second start while running is rejected. */
        TEST_ASSERT(file_radio_start_rx(dev) != 0);
        TEST_ASSERT(file_radio_stop_rx(dev) == 0);
        /* Stopping an idle device succeeds. */
        TEST_ASSERT(file_radio_stop_rx(dev) == 0);
        file_radio_close(dev);
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
    }

    /* Missing path fails cleanly at configure time. */
    {
        sample_dispatcher_t *dispatcher =
            (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
        void *dev = NULL;
        radio_stream_config_t cfg = {
            .lo_freq_hz = 2402000000ull, .sample_rate = TEST_RATE,
        };
        TEST_ASSERT(sample_dispatcher_init(dispatcher) == 0);
        TEST_ASSERT(file_radio_open(&dev, "/nonexistent/x.wav", dispatcher,
                                    0) == 0);
        TEST_ASSERT(file_radio_configure(dev, &cfg) != 0);
        file_radio_close(dev);
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
    }

    unlink(path);

    if (g_failures)
    {
        fprintf(stderr, "test_file_device: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_file_device: ok\n");
    return 0;
}
