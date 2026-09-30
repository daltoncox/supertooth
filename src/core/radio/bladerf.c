#include "bladerf.h"

#if HAVE_BLADERF

#include <libbladeRF.h>

#include <complex.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Sync-stream tuning: 16 internal buffers x 8192 samples keeps USB3 fed
 * while each bladerf_sync_rx() call below asks for an RF-chunk-sized gulp
 * (see SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES). Timeouts are generous; a wedged
 * device must surface as an error, not a hang. */
#define BLADERF_NUM_BUFFERS 16u
#define BLADERF_BUFFER_SIZE 8192u
#define BLADERF_NUM_TRANSFERS 8u
#define BLADERF_STREAM_TIMEOUT_MS 4000u
#define BLADERF_RX_TIMEOUT_MS 4000u

/* AD9361 native ceiling in 16-bit mode (61.44 Msps). Anything above this
 * (in practice the 80 Msps full-band capture) autoswitches to 8-bit
 * SC8_Q7 samples with the AD9361 oversample feature (libbladeRF >= 2.5.0,
 * FW >= 2.4.0, FPGA >= 0.15.0; up to 122.88 Msps). At 80 Msps the 16-bit
 * stream would need ~320 MB/s over USB3; the 8-bit stream needs ~160 MB/s,
 * which fits. Bit depth halves (same as HackRF's native 8 bits), but the
 * full 79-channel band becomes reachable. */
#define BLADERF_SC16_MAX_SAMPLE_RATE_HZ 61440000u

/* The 8-bit oversample path needs libbladeRF >= 2.5.0 (SC8_Q7 formats +
 * bladerf_enable_feature); enforced at configure time (see
 * src/core/CMakeLists.txt) with this compile-time backstop for
 * hand-rolled include paths. */
#if !defined(LIBBLADERF_API_VERSION) || (LIBBLADERF_API_VERSION < 0x02050000)
#error "libbladeRF >= 2.5.0 required (8-bit SC8_Q7 + oversample support)"
#endif

typedef struct
{
    struct bladerf *device;
    sample_dispatcher_t *dispatcher;
    int debug_enabled;
    uint64_t samples_received;

    pthread_t thread;
    int thread_running;
    _Atomic unsigned int stop_requested;

    /* Latched at configure time (the RX thread only reads). */
    uint64_t lo_freq_hz;
    uint32_t sample_rate;
    int gain_db;
    int use_sc8;
} bladerf_radio_t;

static inline float complex bladerf_iq16_to_complex(const int16_t *samples,
                                                    unsigned int sample_index)
{
    return samples[2u * sample_index] / 2048.0f +
           (samples[2u * sample_index + 1u] / 2048.0f) * _Complex_I;
}

/* 8-bit native DAC format: [-128, 127] maps to [-1.0, 1.0), same scaling
 * as the HackRF backend. */
static inline float complex bladerf_iq8_to_complex(const int8_t *samples,
                                                   unsigned int sample_index)
{
    return samples[2u * sample_index] / 128.0f +
           (samples[2u * sample_index + 1u] / 128.0f) * _Complex_I;
}

/* Push @p num_samples (interleaved int16 I/Q) into the dispatcher in
 * SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES pieces, exactly like hackrf_rx_cb, so
 * every backend degrades identically under overload. Returns 0 on success,
 * -1 when stop was requested mid-push. */
static int bladerf_push_samples16(bladerf_radio_t *radio,
                                  const int16_t *samples,
                                  unsigned int num_samples)
{
    unsigned int off = 0u;

    while (off < num_samples)
    {
        unsigned int n = num_samples - off;
        sample_block_t *block;
        if (n > SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES)
            n = SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES;
        if (atomic_load_explicit(&radio->stop_requested,
                                 memory_order_acquire) != 0u)
            return -1;
        block = sample_dispatcher_acquire_block(radio->dispatcher);
        if (!block)
        {
            sample_dispatcher_note_drop(radio->dispatcher,
                                        radio->debug_enabled);
            /* Keep the sample timeline truthful across the drop (same
             * reasoning as hackrf_rx_cb): the dropped chunk's duration must
             * still advance the clock downstream rx_clk_1600/CLKN derive. */
            radio->samples_received += (uint64_t)(num_samples - off);
            return 0;
        }
        block->num_samples = n;
        block->block_base_sample = radio->samples_received;
        radio->samples_received += n;
        for (unsigned int i = 0u; i < n; i++)
            block->samples[i] = bladerf_iq16_to_complex(samples, off + i);
        __atomic_thread_fence(__ATOMIC_RELEASE);
        sample_dispatcher_push_block(radio->dispatcher, block);
        sample_block_release(block);
        off += n;
    }
    return 0;
}

/* 8-bit variant of the above (interleaved int8 I/Q from SC8_Q7 streams).
 * Kept as a separate loop so the per-sample hot path has no format
 * branch at 80 Msps. */
static int bladerf_push_samples8(bladerf_radio_t *radio,
                                 const int8_t *samples,
                                 unsigned int num_samples)
{
    unsigned int off = 0u;

    while (off < num_samples)
    {
        unsigned int n = num_samples - off;
        sample_block_t *block;
        if (n > SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES)
            n = SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES;
        if (atomic_load_explicit(&radio->stop_requested,
                                 memory_order_acquire) != 0u)
            return -1;
        block = sample_dispatcher_acquire_block(radio->dispatcher);
        if (!block)
        {
            sample_dispatcher_note_drop(radio->dispatcher,
                                        radio->debug_enabled);
            /* Keep the sample timeline truthful across the drop (same
             * reasoning as hackrf_rx_cb): the dropped chunk's duration must
             * still advance the clock downstream rx_clk_1600/CLKN derive. */
            radio->samples_received += (uint64_t)(num_samples - off);
            return 0;
        }
        block->num_samples = n;
        block->block_base_sample = radio->samples_received;
        radio->samples_received += n;
        for (unsigned int i = 0u; i < n; i++)
            block->samples[i] = bladerf_iq8_to_complex(samples, off + i);
        __atomic_thread_fence(__ATOMIC_RELEASE);
        sample_dispatcher_push_block(radio->dispatcher, block);
        sample_block_release(block);
        off += n;
    }
    return 0;
}

static void *bladerf_rx_thread(void *arg)
{
    bladerf_radio_t *radio = (bladerf_radio_t *)arg;
    void *buf = NULL;
    /* One gulp per sync call: matches the dispatcher chunk size so each
     * transfer fans out to exactly one RF block in the common case. */
    const unsigned int gulp = SAMPLE_BLOCK_RADIO_CHUNK_SAMPLES;
    unsigned long iters = 0u;
    int use_sc8;

    if (!radio)
        return NULL;

    /* Latched at configure time; copy locally so the loop has no shared
     * read per iteration. Allocate the SC16 size (4 bytes/sample); the
     * SC8 path (2 bytes/sample) reuses the same buffer. */
    use_sc8 = radio->use_sc8;
    buf = malloc(sizeof(int16_t) * 2u * gulp);
    if (!buf)
    {
        fprintf(stderr, "[bladerf] RX thread: out of memory\n");
        return NULL;
    }

    while (atomic_load_explicit(&radio->stop_requested,
                                memory_order_acquire) == 0u)
    {
        int status = bladerf_sync_rx(radio->device, buf, gulp, NULL,
                                     BLADERF_RX_TIMEOUT_MS);
        if (atomic_load_explicit(&radio->stop_requested,
                                 memory_order_acquire) != 0u)
            break;
        if (status != 0)
        {
            /* Timeouts happen on idle bands; log sparingly and keep going.
             * Anything else is likely fatal (unplugged, FPGA wedged). */
            if (status == BLADERF_ERR_TIMEOUT)
            {
                if (radio->debug_enabled && (iters++ % 200u) == 0u)
                    fprintf(stderr, "[bladerf] sync_rx timeout (%lu)\n",
                            iters);
                continue;
            }
            fprintf(stderr, "[bladerf] sync_rx failed: %s\n",
                    bladerf_strerror(status));
            break;
        }
        if (use_sc8)
        {
            if (bladerf_push_samples8(radio, (const int8_t *)buf, gulp) != 0)
                break;
        }
        else
        {
            if (bladerf_push_samples16(radio, (const int16_t *)buf, gulp) != 0)
                break;
        }

        iters++;
        if (radio->debug_enabled && (iters % 500u) == 0u)
            fprintf(stderr,
                    "[bladerf] blocks=%lu samples_received=%llu\n",
                    iters, (unsigned long long)radio->samples_received);
    }

    free(buf);
    return NULL;
}

int bladerf_radio_open(void **out_device,
                       const char *device_id,
                       sample_dispatcher_t *dispatcher,
                       int debug_enabled)
{
    bladerf_radio_t *radio = NULL;
    int status;

    if (!out_device || !dispatcher)
        return BLADERF_ERR_INVAL;

    *out_device = NULL;
    radio = (bladerf_radio_t *)calloc(1, sizeof(*radio));
    if (!radio)
        return BLADERF_ERR_MEM;

    radio->dispatcher = dispatcher;
    radio->debug_enabled = debug_enabled;
    atomic_store_explicit(&radio->stop_requested, 0u, memory_order_release);

    if (device_id && device_id[0] != '\0')
    {
        /* Open by serial: the identifier is "<serial>" as reported by
         * bladerf_list_devices(). The backend string "*:serial=<id>"
         * selects it without walking indices. */
        char selector[BLADERF_ID_LEN + 16u];
        snprintf(selector, sizeof(selector), "*:serial=%s", device_id);
        status = bladerf_open(&radio->device, selector);
    }
    else
    {
        status = bladerf_open(&radio->device, NULL);
    }
    if (status != 0)
    {
        free(radio);
        return status;
    }

    /* Firmware/libbladeRF/FPGA compatibility gate. FX3 firmware v2.6.0
     * switched the FX3<->FPGA streaming interface to 4x-larger GPIF buffers
     * (2048 samples/message, up from 512), which requires libbladeRF
     * >= 2.6.0 and FPGA >= 0.16.0 on the host side. Upstream handles every
     * mismatch EXCEPT one by falling back to legacy 512-sample framing
     * (new lib adapts to old firmware; old lib still frames correctly
     * while the FPGA stays < 0.16). The single un-adaptable combo is old
     * lib + new FW + new FPGA: the old host stack silently mis-frames the
     * stream and delivers 4x the configured sample rate (DSP saturates
     * every core, drops nearly every block, decodes nothing). Fail fast
     * only there; warn-and-continue everywhere else so users can still
     * open the device to reflash it. */
    {
        struct bladerf_version libver, fwver, fpgaver;

        memset(&libver, 0, sizeof(libver));
        memset(&fwver, 0, sizeof(fwver));
        memset(&fpgaver, 0, sizeof(fpgaver));
        bladerf_version(&libver);
        if (bladerf_fw_version(radio->device, &fwver) == 0 &&
            (fwver.major > 2 ||
             (fwver.major == 2 && fwver.minor >= 6u)))
        {
            int lib_ok = (libver.major > 2 ||
                          (libver.major == 2 && libver.minor >= 6u));
            int fpga_known = (bladerf_fpga_version(radio->device, &fpgaver) == 0);
            int fpga_configured = bladerf_is_fpga_configured(radio->device) == 1;
            int fpga_loaded = fpga_known && fpga_configured &&
                (fpgaver.major != 0u || fpgaver.minor != 0u ||
                 fpgaver.patch != 0u);
            int fpga_ok = !fpga_loaded ||
                (fpgaver.major > 0 ||
                 (fpgaver.major == 0 && fpgaver.minor >= 16u));
            /* New FPGA means the 2048-sample framing is actually on the
             * wire; an old lib cannot parse it. Old/absent FPGA means
             * legacy 512-sample framing, which the old lib still reads. */
            int fpga_new = fpga_loaded && fpga_ok;

            fprintf(stderr,
                    "[bladerf] versions: lib %u.%u.%u, fw %u.%u.%u, fpga %u.%u.%u\n",
                    (unsigned)libver.major, (unsigned)libver.minor,
                    (unsigned)libver.patch,
                    (unsigned)fwver.major, (unsigned)fwver.minor,
                    (unsigned)fwver.patch,
                    (unsigned)fpgaver.major, (unsigned)fpgaver.minor,
                    (unsigned)fpgaver.patch);
            if (!lib_ok && fpga_new)
            {
                fprintf(stderr,
                        "[bladerf] FX3 firmware %u.%u.%u with FPGA %u.%u.%u "
                        "uses 2048-sample framing, which requires "
                        "libbladeRF >= 2.6.0, but have libbladeRF %u.%u.%u.\n",
                        (unsigned)fwver.major, (unsigned)fwver.minor,
                        (unsigned)fwver.patch,
                        (unsigned)fpgaver.major, (unsigned)fpgaver.minor,
                        (unsigned)fpgaver.patch,
                        (unsigned)libver.major, (unsigned)libver.minor,
                        (unsigned)libver.patch);
                fprintf(stderr,
                        "[bladerf] With this mismatch the device streams at "
                        "4x the configured sample rate (all cores saturated, "
                        "no packets decoded). Upgrade libbladeRF (see "
                        "packaging/build-bladerf-linux.sh) and retry.\n");
                bladerf_close(radio->device);
                free(radio);
                return BLADERF_ERR_UPDATE_FW;
            }
            if (!lib_ok)
            {
                /* Old lib, but legacy framing is on the wire (old or not
                 * yet loaded FPGA): streaming works. Warn anyway, since
                 * loading an FPGA >= 0.16.0 under this lib breaks. */
                fprintf(stderr,
                        "[bladerf] WARNING: FX3 firmware %u.%u.%u with "
                        "libbladeRF %u.%u.%u. Legacy framing is in use, so "
                        "this works, but loading an FPGA >= 0.16.0 requires "
                        "libbladeRF >= 2.6.0 -- upgrade before reflashing "
                        "(see https://www.nuand.com/fpga_images/).\n",
                        (unsigned)fwver.major, (unsigned)fwver.minor,
                        (unsigned)fwver.patch,
                        (unsigned)libver.major, (unsigned)libver.minor,
                        (unsigned)libver.patch);
            }
            else if (!fpga_ok)
            {
                /* New lib, old FPGA: upstream falls back to legacy message
                 * size and streams fine, at reduced throughput. Nudge the
                 * upgrade without blocking the reflash path. */
                fprintf(stderr,
                        "[bladerf] WARNING: FPGA %u.%u.%u predates 0.16.0; "
                        "using legacy message size at reduced throughput. "
                        "Load a current FPGA image "
                        "(see https://www.nuand.com/fpga_images/).\n",
                        (unsigned)fpgaver.major, (unsigned)fpgaver.minor,
                        (unsigned)fpgaver.patch);
            }
        }
        else if (radio->debug_enabled)
        {
            /* Best-effort version line for older firmware; FPGA query may
             * fail when no image is loaded, in which case fpgaver stays 0.0.0. */
            (void)bladerf_fpga_version(radio->device, &fpgaver);
            fprintf(stderr,
                    "[bladerf] versions: lib %u.%u.%u, fw %u.%u.%u, fpga %u.%u.%u\n",
                    (unsigned)libver.major, (unsigned)libver.minor,
                    (unsigned)libver.patch,
                    (unsigned)fwver.major, (unsigned)fwver.minor,
                    (unsigned)fwver.patch,
                    (unsigned)fpgaver.major, (unsigned)fpgaver.minor,
                    (unsigned)fpgaver.patch);
        }
    }

    /* Manual gain control: -g sets the overall gain directly (see
     * bladerf_radio_configure). AGC modes are intentionally left off so
     * captures are reproducible for decoding. */
    status = bladerf_set_gain_mode(radio->device, BLADERF_CHANNEL_RX(0),
                                   BLADERF_GAIN_MGC);
    if (status != 0 && status != BLADERF_ERR_UNSUPPORTED)
    {
        fprintf(stderr, "[bladerf] cannot set MGC mode: %s\n",
                bladerf_strerror(status));
        bladerf_close(radio->device);
        free(radio);
        return status;
    }

    *out_device = radio;
    return 0;
}

int bladerf_radio_configure(void *device, const radio_stream_config_t *config)
{
    bladerf_radio_t *radio = (bladerf_radio_t *)device;
    bladerf_channel ch = BLADERF_CHANNEL_RX(0);
    bladerf_sample_rate actual_rate = 0u;
    bladerf_bandwidth actual_bw = 0u;
    int status;
    /* Autoswitch: rates above the SC16 ceiling (i.e. the 80 Msps
     * full-band capture) use 8-bit SC8_Q7 + the oversample feature at
     * half the bit depth. Everything at or below 61.44 Msps stays in
     * 16-bit SC16_Q11 for full fidelity. No flag; the requested rate
     * alone selects the path. */
    int want_sc8 = 0;

    if (!radio || !radio->device || !config)
        return BLADERF_ERR_INVAL;

    want_sc8 = (config->sample_rate > BLADERF_SC16_MAX_SAMPLE_RATE_HZ) ? 1 : 0;

    /* The oversample feature must be set BEFORE bladerf_set_sample_rate
     * (it switches the AD9361 into overclock + selects the valid rate
     * range). Set explicitly both ways so a previous high-rate tune
     * cannot leak into a later narrow capture (and vice versa). */
    status = bladerf_enable_feature(radio->device,
                                    BLADERF_FEATURE_OVERSAMPLE,
                                    want_sc8 ? true : false);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] enable oversample=%d failed: %s\n",
                want_sc8, bladerf_strerror(status));
        return status;
    }

    status = bladerf_set_frequency(radio->device, ch, config->lo_freq_hz);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] set_frequency %llu failed: %s\n",
                (unsigned long long)config->lo_freq_hz,
                bladerf_strerror(status));
        return status;
    }

    status = bladerf_set_sample_rate(radio->device, ch, config->sample_rate,
                                     &actual_rate);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] set_sample_rate %u failed: %s\n",
                config->sample_rate, bladerf_strerror(status));
        if (want_sc8)
            fprintf(stderr,
                    "[bladerf] 8-bit rates above %u need FW >= 2.4.0 and "
                    "FPGA >= 0.15.0 with the oversample feature "
                    "(see https://www.nuand.com/fpga_images/).\n",
                    BLADERF_SC16_MAX_SAMPLE_RATE_HZ);
        return status;
    }
    if (actual_rate != config->sample_rate && radio->debug_enabled)
        fprintf(stderr, "[bladerf] sample rate %u -> actual %u\n",
                config->sample_rate, actual_rate);

    /* LPF at the sample rate keeps aliases out of the window; the part
     * snaps to its discrete options and reports back via actual_bw. */
    {
        bladerf_bandwidth want_bw = config->sample_rate;
        if (want_bw > 56000000u)
            want_bw = 56000000u;
        status = bladerf_set_bandwidth(radio->device, ch, want_bw,
                                       &actual_bw);
        if (status != 0)
        {
            fprintf(stderr, "[bladerf] set_bandwidth %u failed: %s\n",
                    want_bw, bladerf_strerror(status));
            return status;
        }
        if (radio->debug_enabled)
            fprintf(stderr, "[bladerf] bandwidth %u -> actual %u\n",
                    want_bw, actual_bw);
    }

    status = bladerf_set_gain(radio->device, ch, config->gain.bladerf_gain_db);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] set_gain %d failed: %s\n",
                config->gain.bladerf_gain_db,
                bladerf_strerror(status));
        return status;
    }

    radio->lo_freq_hz = config->lo_freq_hz;
    radio->sample_rate = actual_rate != 0u ? actual_rate : config->sample_rate;
    radio->gain_db = config->gain.bladerf_gain_db;
    radio->use_sc8 = want_sc8;
    if (radio->debug_enabled)
        fprintf(stderr, "[bladerf] tune lo=%llu rate=%u (%s, oversample %s)\n",
                (unsigned long long)radio->lo_freq_hz, radio->sample_rate,
                radio->use_sc8 ? "SC8_Q7 8-bit" : "SC16_Q11 16-bit",
                radio->use_sc8 ? "on" : "off");
    return 0;
}

int bladerf_radio_start_rx(void *device)
{
    bladerf_radio_t *radio = (bladerf_radio_t *)device;
    int status;

    if (!radio || !radio->device || !radio->dispatcher)
        return BLADERF_ERR_INVAL;
    if (radio->thread_running)
        return BLADERF_ERR_INVAL;

    status = bladerf_sync_config(radio->device, BLADERF_RX_X1,
                                 radio->use_sc8 ? BLADERF_FORMAT_SC8_Q7
                                                : BLADERF_FORMAT_SC16_Q11,
                                 BLADERF_NUM_BUFFERS, BLADERF_BUFFER_SIZE,
                                 BLADERF_NUM_TRANSFERS,
                                 BLADERF_STREAM_TIMEOUT_MS);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] sync_config failed: %s\n",
                bladerf_strerror(status));
        return status;
    }

    status = bladerf_enable_module(radio->device, BLADERF_CHANNEL_RX(0),
                                   true);
    if (status != 0)
    {
        fprintf(stderr, "[bladerf] enable RX failed: %s\n",
                bladerf_strerror(status));
        return status;
    }

    radio->samples_received = 0ULL;
    atomic_store_explicit(&radio->stop_requested, 0u, memory_order_release);
    if (pthread_create(&radio->thread, NULL, bladerf_rx_thread, radio) != 0)
    {
        bladerf_enable_module(radio->device, BLADERF_CHANNEL_RX(0), false);
        return BLADERF_ERR_IO;
    }
    radio->thread_running = 1;
    return 0;
}

int bladerf_radio_stop_rx(void *device)
{
    bladerf_radio_t *radio = (bladerf_radio_t *)device;

    if (!radio)
        return BLADERF_ERR_INVAL;
    atomic_store_explicit(&radio->stop_requested, 1u, memory_order_release);
    if (radio->thread_running)
    {
        pthread_join(radio->thread, NULL);
        radio->thread_running = 0;
    }
    if (radio->device)
        bladerf_enable_module(radio->device, BLADERF_CHANNEL_RX(0), false);
    return 0;
}

void bladerf_radio_close(void *device)
{
    bladerf_radio_t *radio = (bladerf_radio_t *)device;
    if (!radio)
        return;

    bladerf_radio_stop_rx(radio);
    if (radio->device)
        bladerf_close(radio->device);
    free(radio);
}

int bladerf_radio_get_max_sample_rate(void *device, uint32_t *out_rate_hz)
{
    (void)device;
    if (!out_rate_hz)
        return -1;

    /* 8-bit oversample path reaches 122.88 Msps on the part, but the
     * channelizer stages at most 80 Msps, so the session ceiling wins. */
    *out_rate_hz = RADIO_MAX_SAMPLE_RATE_HZ;
    return 0;
}

int bladerf_list_devices(char ***out_identifiers, size_t *out_count)
{
    char **identifiers = NULL;
    struct bladerf_devinfo *devs = NULL;
    int count = 0;
    int backend_count;
    size_t copied = 0u;

    if (!out_identifiers || !out_count)
        return -1;

    *out_identifiers = NULL;
    *out_count = 0u;

    backend_count = bladerf_get_device_list(&devs);
    if (backend_count < 0)
        return backend_count;
    if (backend_count == 0)
        return 0;

    identifiers = (char **)calloc((size_t)backend_count, sizeof(char *));
    if (!identifiers)
    {
        bladerf_free_device_list(devs);
        return BLADERF_ERR_MEM;
    }

    for (count = 0; count < backend_count; count++)
    {
        /* Serial is the stable per-board id (32 hex chars on bladeRF 2.0).
         * Skip entries with an empty serial; they cannot be reopened by
         * the "*:serial=" selector used in bladerf_radio_open(). */
        if (devs[count].serial[0] == '\0')
            continue;
        identifiers[copied] = strdup(devs[count].serial);
        if (!identifiers[copied])
        {
            for (size_t j = 0u; j < copied; j++)
                free(identifiers[j]);
            free(identifiers);
            bladerf_free_device_list(devs);
            return BLADERF_ERR_MEM;
        }
        copied++;
    }
    bladerf_free_device_list(devs);

    /* Tolerate zero usable serials (all skipped): report an empty
     * success rather than leaking an empty array. */
    if (copied == 0u)
    {
        free(identifiers);
        return 0;
    }

    *out_identifiers = identifiers;
    *out_count = copied;
    return 0;
}

#endif /* HAVE_BLADERF */
