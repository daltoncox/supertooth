#include "app_record.h"

#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "wav.h"

static _Atomic unsigned int g_record_stop = 0u;
static sample_reader_t *g_record_reader = NULL;

static void app_record_handle_sigint(int sig)
{
    (void)sig;
    atomic_store_explicit(&g_record_stop, 1u, memory_order_release);
    /* Same pattern as session_request_stop(): wake the blocked pop so the
     * loop observes the flag promptly. */
    sample_reader_signal(g_record_reader);
}

int app_record_run(const app_record_config_t *cfg, const char *out_path)
{
    /* NB: the dispatcher owns 64 x 256K-sample blocks (~128 MB) and must
     * live on the heap, same as in session_init(). */
    sample_dispatcher_t *dispatcher = NULL;
    sample_reader_t reader;
    radio_device_t *device = NULL;
    wav_writer_t writer;
    char resolved[1024];
    int reader_ready = 0;
    int radio_opened = 0;
    int writer_opened = 0;
    uint64_t samples_written = 0u;
    unsigned long blocks = 0ul;
    int result = 1;

    if (!cfg || !out_path || out_path[0] == '\0')
    {
        fprintf(stderr, "--record: missing output path\n");
        return 1;
    }
    if (cfg->device_type == RADIO_DEVICE_FILE)
    {
        fprintf(stderr, "--record: cannot record from a file replay\n");
        return 1;
    }
    if (cfg->sample_rate_hz == 0u || cfg->lo_freq_hz == 0u)
    {
        fprintf(stderr, "--record: invalid tune (rate/LO)\n");
        return 1;
    }

    /* Resolve directory → self-describing filename. */
    {
        size_t len = strlen(out_path);
        int is_dir = (len > 0u && out_path[len - 1u] == '/');
        struct stat st;
        if (!is_dir && stat(out_path, &st) == 0 && S_ISDIR(st.st_mode))
            is_dir = 1;
        if (is_dir)
        {
            char name[256];
            if (wav_build_recording_filename(cfg->lo_freq_hz,
                                             cfg->sample_rate_hz,
                                             name, sizeof(name)) != 0)
            {
                fprintf(stderr, "--record: cannot build filename\n");
                return 1;
            }
            int needed;
            if (len > 0u && out_path[len - 1u] == '/')
                needed = snprintf(resolved, sizeof(resolved), "%s%s",
                                  out_path, name);
            else
                needed = snprintf(resolved, sizeof(resolved), "%s/%s",
                                  out_path, name);
            if (needed < 0 || (size_t)needed >= sizeof(resolved))
            {
                fprintf(stderr, "--record: output path too long\n");
                return 1;
            }
        }
        else
        {
            if (len >= sizeof(resolved))
            {
                fprintf(stderr, "--record: output path too long\n");
                return 1;
            }
            memcpy(resolved, out_path, len + 1u);
        }
    }

    dispatcher = (sample_dispatcher_t *)calloc(1, sizeof(*dispatcher));
    if (!dispatcher)
    {
        fprintf(stderr, "--record: out of memory\n");
        return 1;
    }
    if (sample_dispatcher_init(dispatcher) != 0)
    {
        fprintf(stderr, "--record: dispatcher init failed\n");
        free(dispatcher);
        return 1;
    }
    if (sample_reader_init(&reader, dispatcher) != 0)
    {
        fprintf(stderr, "--record: reader init failed\n");
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
        return 1;
    }
    reader_ready = 1;

    if (radio_open(&device, cfg->device_type, cfg->device_id, dispatcher,
                   cfg->debug) != RADIO_SUCCESS)
    {
        fprintf(stderr, "--record: cannot open radio\n");
        goto done;
    }
    radio_opened = 1;

    {
        radio_stream_config_t rcfg = {
            .lo_freq_hz = cfg->lo_freq_hz,
            .sample_rate = cfg->sample_rate_hz,
            /* Record at the session's usual gains. */
            .lna_gain = 24u,
            .vga_gain = 18u,
        };
        if (radio_configure(device, &rcfg) != RADIO_SUCCESS)
        {
            fprintf(stderr, "--record: radio configure failed\n");
            goto done;
        }
    }

    if (wav_write_open(resolved, cfg->sample_rate_hz, WAV_SAMP_S16,
                       cfg->lo_freq_hz, &writer) != 0)
    {
        fprintf(stderr, "--record: cannot create '%s'\n", resolved);
        goto done;
    }
    writer_opened = 1;

    if (radio_start_rx(device) != RADIO_SUCCESS)
    {
        fprintf(stderr, "--record: radio start failed\n");
        goto done;
    }

    atomic_store_explicit(&g_record_stop, 0u, memory_order_release);
    g_record_reader = &reader;
    signal(SIGINT, app_record_handle_sigint);
    signal(SIGTERM, app_record_handle_sigint);

    printf("Recording raw IQ (no decoding)\n");
    printf("  Output : %s\n", resolved);
    printf("  LO     : %u Hz\n", cfg->lo_freq_hz);
    printf("  Rate   : %u Hz (16-bit stereo WAV + auxi)\n",
           cfg->sample_rate_hz);
    printf("Press Ctrl+C to stop.\n");
    fflush(stdout);

    while (atomic_load_explicit(&g_record_stop,
                                memory_order_acquire) == 0u)
    {
        sample_block_t *block = NULL;
        if (sample_reader_wait_pop(&reader, &g_record_stop, &block) != 0)
            break;
        if (!block || block->num_samples == 0u)
        {
            if (block)
                sample_block_release(block);
            continue;
        }
        if (wav_write_frames(&writer, block->samples,
                             block->num_samples) != 0)
        {
            fprintf(stderr, "--record: write failed\n");
            sample_block_release(block);
            break;
        }
        samples_written += block->num_samples;
        sample_block_release(block);
        blocks++;
        if (cfg->debug && (blocks % 500ul) == 0ul)
            fprintf(stderr, "[record] blocks=%lu samples=%llu\n", blocks,
                    (unsigned long long)samples_written);
    }

    signal(SIGINT, SIG_DFL);
    signal(SIGTERM, SIG_DFL);
    g_record_reader = NULL;
    result = 0;

done:
    if (radio_opened)
    {
        radio_stop_rx(device);
        radio_close(device);
    }
    if (writer_opened)
    {
        uint64_t frames = writer.frames_written;
        if (wav_write_close(&writer) != 0)
        {
            fprintf(stderr, "--record: finalize failed\n");
            result = 1;
        }
        (void)frames;
    }
    if (reader_ready)
        sample_reader_destroy(&reader);
    if (dispatcher)
    {
        sample_dispatcher_destroy(dispatcher);
        free(dispatcher);
    }

    if (result == 0)
    {
        struct stat st;
        double seconds = cfg->sample_rate_hz > 0u
                             ? (double)samples_written /
                                   (double)cfg->sample_rate_hz
                             : 0.0;
        printf("\n=== Record Summary ===\n");
        printf("  File    : %s\n", resolved);
        printf("  Samples : %llu (%.1f s @ %u Hz)\n",
               (unsigned long long)samples_written, seconds,
               cfg->sample_rate_hz);
        if (stat(resolved, &st) == 0)
            printf("  Size    : %lld bytes\n", (long long)st.st_size);
    }
    return result;
}
