#include "file.h"

#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "wav.h"

/* Frames decoded per reader iteration; split into dispatcher blocks of at
 * most SAMPLE_BLOCK_SAMPLE_CAPACITY. Sized to SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES
 * (256 KiB scratch at the widest depth, well under one block) so file replay
 * pushes the same RF granularity as every other radio backend. */
#define FILE_RADIO_READ_FRAMES SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES

typedef struct
{
    char *path;
    sample_dispatcher_t *dispatcher;
    int debug_enabled;

    wav_info_t info;
    int header_valid;

    pthread_t thread;
    int thread_running;
    _Atomic unsigned int stop_requested;
    _Atomic unsigned int finished;

    /* 1 = exhaustive replay (backpressure, never drop for pace reasons);
     * 0 = realtime replay (wall-clock paced, live-equivalent drops). Set via
     * file_radio_set_exhaustive() before start_rx; read once by the reader
     * thread at startup. */
    _Atomic unsigned int exhaustive;

    uint64_t samples_sent;
} file_radio_t;

/* Monotonic nanoseconds for replay pacing. */
static uint64_t file_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Sleep until (absolute) @p deadline_ns, in chunks that honor @p stop so
 * Ctrl+C stays responsive. stop is a pointer to the radio's stop flag. */
static void file_sleep_until(uint64_t deadline_ns,
                             const _Atomic unsigned int *stop)
{
    for (;;)
    {
        uint64_t now = file_now_ns();
        uint64_t remain;
        struct timespec ts;

        if (now >= deadline_ns)
            return;
        if (atomic_load_explicit(stop, memory_order_acquire) != 0u)
            return;
        remain = deadline_ns - now;
        if (remain > 50000000ull)
            remain = 50000000ull;
        ts.tv_sec = (time_t)(remain / 1000000000ull);
        ts.tv_nsec = (long)(remain % 1000000000ull);
        nanosleep(&ts, NULL);
    }
}

static void *file_radio_reader_thread(void *arg)
{
    file_radio_t *radio = (file_radio_t *)arg;
    FILE *fp = NULL;
    float complex *scratch = NULL;

    if (!radio)
        return NULL;

    if (wav_read_open(radio->path, &radio->info, &fp) != 0)
    {
        fprintf(stderr, "[file] cannot open '%s' for replay\n", radio->path);
        atomic_store_explicit(&radio->finished, 1u, memory_order_release);
        return NULL;
    }
    radio->header_valid = 1;

    int exhaustive =
        atomic_load_explicit(&radio->exhaustive, memory_order_acquire) != 0u;

    scratch = (float complex *)malloc(sizeof(*scratch) * FILE_RADIO_READ_FRAMES);
    if (!scratch)
    {
        wav_read_close(fp);
        atomic_store_explicit(&radio->finished, 1u, memory_order_release);
        return NULL;
    }

    if (radio->debug_enabled)
        fprintf(stderr, "[file] replaying '%s': %u Hz, %llu frames%s (%s)\n",
                radio->path, radio->info.sample_rate_hz,
                (unsigned long long)radio->info.total_frames,
                radio->info.center_freq_hz ? "" : " (no auxi/filename center)",
                exhaustive ? "exhaustive" : "realtime");

    /* Realtime pacing anchor: monotonic time of the first push. The session
     * starts all consumers before radio_start_rx, so the deadline math below
     * reproduces live arrival timing from the first block on. */
    uint64_t pace_origin_ns = 0u;
    uint32_t pace_rate_hz = radio->info.sample_rate_hz;

    for (;;)
    {
        size_t want, got, off;
        if (atomic_load_explicit(&radio->stop_requested,
                                 memory_order_acquire) != 0u)
            break;

        want = FILE_RADIO_READ_FRAMES;
        got = want;
        if (wav_read_frames(fp, &radio->info, scratch, &got) != 0)
            break;
        if (got == 0u)
            break; /* EOF: single pass, then finish */

        /* Fan out in dispatcher-sized blocks, exactly like hackrf_rx_cb. */
        off = 0u;
        while (off < got)
        {
            size_t n = got - off;
            sample_block_t *block;
            if (n > SAMPLE_BLOCK_SAMPLE_CAPACITY)
                n = SAMPLE_BLOCK_SAMPLE_CAPACITY;

            if (exhaustive)
            {
                /* Backpressure instead of drops via the shared blocking
                 * helpers (this thread is the sole producer, so the
                 * wait-then-push is race-free and always delivers). */
                block = sample_dispatcher_acquire_blocking(
                    radio->dispatcher, &radio->stop_requested);
                if (!block)
                    break; /* stop requested */
            }
            else
            {
                /* Realtime: reproduce live arrival timing. The deadline for
                 * this block is anchored to the first push, so per-block
                 * jitter never accumulates into drift. */
                if (pace_origin_ns == 0u)
                    pace_origin_ns = file_now_ns();
                if (pace_rate_hz > 0u && radio->samples_sent > 0u)
                    file_sleep_until(pace_origin_ns +
                                         radio->samples_sent * 1000000000ull /
                                             pace_rate_hz,
                                     &radio->stop_requested);
                if (atomic_load_explicit(&radio->stop_requested,
                                         memory_order_acquire) != 0u)
                    break;

                block = sample_dispatcher_acquire_block(radio->dispatcher);
                if (!block)
                {
                    sample_dispatcher_note_drop(radio->dispatcher,
                                                radio->debug_enabled);
                    /* Drop this slice but keep the stream timelines advancing
                     * on both axes: the file position (off) and the radio
                     * sample clock (samples_sent, which drives all downstream
                     * rx_clk_1600/CLKN timestamps). Advancing only the file
                     * position compresses the radio timeline, making the
                     * receiver slot clock run slow and breaking BR/EDR
                     * UAP/clock tracking under load. */
                    radio->samples_sent += (uint64_t)n;
                    off += n;
                    continue;
                }
            }

            block->num_samples = (unsigned int)n;
            block->block_base_sample = radio->samples_sent;
            radio->samples_sent += (uint64_t)n;
            memcpy(block->samples, scratch + off, n * sizeof(float complex));

            __atomic_thread_fence(__ATOMIC_RELEASE);
            if (exhaustive)
            {
                sample_dispatcher_push_blocking(radio->dispatcher, block,
                                                &radio->stop_requested);
                sample_block_release(block);
                if (atomic_load_explicit(&radio->stop_requested,
                                         memory_order_acquire) != 0u)
                    break;
            }
            else
            {
                sample_dispatcher_push_block(radio->dispatcher, block);
                sample_block_release(block);
            }
            off += n;
        }
    }

    free(scratch);
    wav_read_close(fp);

    if (radio->debug_enabled)
        fprintf(stderr, "[file] replay finished: %llu samples sent\n",
                (unsigned long long)radio->samples_sent);

    atomic_store_explicit(&radio->finished, 1u, memory_order_release);
    return NULL;
}

int file_radio_open(void **out_device,
                    const char *path,
                    sample_dispatcher_t *dispatcher,
                    int debug_enabled)
{
    file_radio_t *radio = NULL;

    if (!out_device || !dispatcher || !path || path[0] == '\0')
        return -1;

    *out_device = NULL;
    radio = (file_radio_t *)calloc(1, sizeof(*radio));
    if (!radio)
        return -1;

    radio->path = strdup(path);
    if (!radio->path)
    {
        free(radio);
        return -1;
    }
    radio->dispatcher = dispatcher;
    radio->debug_enabled = debug_enabled;
    atomic_store_explicit(&radio->stop_requested, 0u, memory_order_release);
    atomic_store_explicit(&radio->finished, 0u, memory_order_release);
    atomic_store_explicit(&radio->exhaustive, 0u, memory_order_release);

    *out_device = radio;
    return RADIO_SUCCESS;
}

int file_radio_configure(void *device, const radio_stream_config_t *config)
{
    file_radio_t *radio = (file_radio_t *)device;
    wav_info_t info;
    FILE *fp = NULL;

    if (!radio || !config)
        return -1;

    /* Gains/LO are meaningless for a file; the sample rate must match the
     * tuned session rate or the channelizers would misinterpret the data. */
    if (wav_read_open(radio->path, &info, &fp) != 0)
    {
        fprintf(stderr, "[file] cannot read '%s' (not a 2ch IQ WAV?)\n",
                radio->path);
        return -1;
    }
    wav_read_close(fp);

    if (info.sample_rate_hz != config->sample_rate)
    {
        fprintf(stderr,
                "[file] sample-rate mismatch: file is %u Hz but the session "
                "is tuned for %u Hz. Re-run with the channel window used at "
                "record time.\n",
                info.sample_rate_hz, config->sample_rate);
        return -1;
    }

    radio->info = info;
    radio->header_valid = 1;
    return RADIO_SUCCESS;
}

int file_radio_start_rx(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;

    if (!radio || !radio->dispatcher)
        return -1;
    if (radio->thread_running)
        return -1;

    radio->samples_sent = 0u;
    atomic_store_explicit(&radio->stop_requested, 0u, memory_order_release);
    atomic_store_explicit(&radio->finished, 0u, memory_order_release);

    if (pthread_create(&radio->thread, NULL, file_radio_reader_thread,
                       radio) != 0)
        return -1;
    radio->thread_running = 1;
    return RADIO_SUCCESS;
}

int file_radio_stop_rx(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;

    if (!radio)
        return -1;
    atomic_store_explicit(&radio->stop_requested, 1u, memory_order_release);
    if (radio->thread_running)
    {
        pthread_join(radio->thread, NULL);
        radio->thread_running = 0;
    }
    return RADIO_SUCCESS;
}

void file_radio_close(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;
    if (!radio)
        return;
    file_radio_stop_rx(radio);
    free(radio->path);
    free(radio);
}

int file_radio_is_finished(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;
    if (!radio)
        return 1;
    return atomic_load_explicit(&radio->finished,
                                memory_order_acquire) != 0u;
}

void file_radio_set_exhaustive(void *device, int exhaustive)
{
    file_radio_t *radio = (file_radio_t *)device;
    if (!radio)
        return;
    atomic_store_explicit(&radio->exhaustive, exhaustive ? 1u : 0u,
                          memory_order_release);
}

uint32_t file_radio_get_file_sample_rate(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;
    if (!radio || !radio->header_valid)
        return 0u;
    return radio->info.sample_rate_hz;
}

uint64_t file_radio_get_file_center_hz(void *device)
{
    file_radio_t *radio = (file_radio_t *)device;
    uint64_t lo = 0u;
    uint32_t rate = 0u;
    if (!radio)
        return 0u;
    if (radio->header_valid && radio->info.center_freq_hz != 0u)
        return radio->info.center_freq_hz;
    /* Fall back to the self-describing filename tokens. */
    wav_parse_recording_filename(radio->path, &lo, &rate);
    (void)rate;
    return lo;
}

int file_radio_check_compatible(const char *path, uint32_t expected_rate_hz,
                                uint32_t *out_file_rate_hz,
                                uint64_t *out_file_center_hz)
{
    wav_info_t info;
    FILE *fp = NULL;
    uint64_t name_lo = 0u;
    uint32_t name_rate = 0u;

    if (!path || path[0] == '\0')
    {
        fprintf(stderr, "[file] empty replay path\n");
        return -1;
    }
    if (wav_read_open(path, &info, &fp) != 0)
    {
        fprintf(stderr, "[file] cannot read '%s' (not a 2-channel IQ WAV?)\n",
                path);
        return -1;
    }
    wav_read_close(fp);

    if (expected_rate_hz != 0u && info.sample_rate_hz != expected_rate_hz)
    {
        fprintf(stderr,
                "[file] sample-rate mismatch: '%s' is %u Hz but the session "
                "is tuned for %u Hz. Re-run with the channel window used at "
                "record time.\n",
                path, info.sample_rate_hz, expected_rate_hz);
        return -1;
    }

    if (out_file_rate_hz)
        *out_file_rate_hz = info.sample_rate_hz;
    if (out_file_center_hz)
    {
        *out_file_center_hz = info.center_freq_hz;
        if (*out_file_center_hz == 0u)
        {
            wav_parse_recording_filename(path, &name_lo, &name_rate);
            *out_file_center_hz = name_lo;
        }
    }
    return RADIO_SUCCESS;
}
