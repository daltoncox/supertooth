/* Multiplexed `supertooth` CLI: one file holding every operating mode.
 *
 * Only one mode runs per process, so decode modes share plain globals
 * (output mode, debug flag, packet counter, CRC/AC tolerances, the
 * BR/EDR window and the heap session). LE keeps its own stack session
 * and LE window; BR/EDR keeps its LAP filter. Shared option parsing,
 * device setup, tune validation and banner/debug helpers live in the
 * "Shared helpers" section; per-mode getopt/session flows stay explicit
 * under their own section. The record backend (moved from app_record.c,
 * its sole caller is record_main) sits at the end of the shared part.
 *
 * Entry points (declared extern in supertooth.c, which strips the mode
 * word before dispatching). The canonical mode names are `le` and
 * `bredr`; `ble` and `classic` are hidden aliases handled by the
 * dispatcher and intentionally omitted from help output:
 *   int ble_main(int argc, char *argv[]);     // runs as `supertooth le`
 *   int bredr_main(int argc, char *argv[]);   // also runs as `supertooth classic`
 *   int hybrid_main(int argc, char *argv[]);
 *   int record_main(int argc, char *argv[]);
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <getopt.h>
#include <strings.h>
#include <signal.h>
#include <stdatomic.h>
#include <sys/stat.h>

#include "app_common.h"
#include "app_device_view.h"
#include "app_summary_view.h"
#include "file.h"
#include "version.h"
#include "ble_display.h"
#include "ble_bitstream_decoder.h"
#include "bredr_display.h"
#include "bredr_bitstream_decoder.h"
#include "session.h"
#include "wav.h"

#define BREDR_MAX_CHANNEL 79u

/* Record-only raw IQ capture config (record mode only). */
typedef struct
{
    radio_device_type_t device_type; /* must be a live radio, not FILE */
    const char *device_id;           /* optional, NULL for default */
    uint32_t lo_freq_hz;
    uint32_t sample_rate_hz;
    radio_gain_spec_t gain;
    int debug;
} app_record_config_t;

/* -------------------------------------------------------------------------
 * Shared decode-mode state (exactly one mode runs per process)
 * -------------------------------------------------------------------------*/

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_SUMMARY;
static int g_debug = 0;
static unsigned long g_packet_count = 0;
static int g_enforce_crc = 1;   /* drop BLE frames whose CRC fails; default on */
/* Maximum access-code bit errors accepted by the BR/EDR bitstream decoder.
 * Defaults to 0 (strict, byte-perfect access-code match). */
static unsigned int g_ac_errors = 0u;
/* BR/EDR channel window shared by bredr/hybrid/record. The default count
 * is derived from the radio's max sample rate at startup (see each mode);
 * the initial value here is overwritten before use. */
static unsigned int g_num_bredr_channels = BREDR_SESSION_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;
static int g_channels_explicit = 0;
/* Heap session shared by bredr/hybrid (BLE keeps its own stack session). */
static session_t *g_session = NULL;
/* Live device/connection table view (started/stopped around session_run). */
static app_device_view_t *g_device_view = NULL;

static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
    {APP_OUTPUT_MODE_DEVICES, "devices"},
};

typedef void (*cli_usage_fn)(const char *prog);

/* -------------------------------------------------------------------------
 * Shared option-parsing helpers
 * -------------------------------------------------------------------------*/

static int parse_bredr_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    /* "all" is the full 0..78 band (c=79): 80 Msps, LO 2441 MHz. */
    if (strcasecmp(arg, "all") == 0)
    {
        *out_channels = BREDR_SESSION_MAX_CHANNELS;
        return 0;
    }

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 2ul || value > (unsigned long)BREDR_SESSION_MAX_CHANNELS)
        return -1;
    /* 79 ("all") is the odd exception; every other count must be even. */
    if (value != (unsigned long)BREDR_SESSION_MAX_CHANNELS &&
        (value & 1ul) != 0ul)
        return -1;

    *out_channels = (unsigned int)value;
    return 0;
}

static int parse_bredr_bottom_channel(const char *arg, unsigned int *out_bottom_channel)
{
    if (!arg || !out_bottom_channel)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > (unsigned long)BREDR_MAX_CHANNEL)
        return -1;

    *out_bottom_channel = (unsigned int)value;
    return 0;
}

static int parse_le_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 1ul || value > (unsigned long)BLE_SESSION_MAX_CHANNELS)
        return -1;

    *out_channels = (unsigned int)value;
    return 0;
}

static int parse_le_bottom_channel(const char *arg, unsigned int *out_bottom_channel)
{
    if (!arg || !out_bottom_channel)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 39ul)
        return -1;

    *out_bottom_channel = (unsigned int)value;
    return 0;
}

static int parse_lap_filter(const char *arg, uint32_t *out_lap)
{
    if (!arg || !out_lap)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 0xFFFFFFul)
        return -1;

    *out_lap = (uint32_t)value;
    return 0;
}

static int parse_ac_errors_value(const char *arg, unsigned int *out)
{
    char *end = NULL;
    unsigned long value;

    if (!arg || !out)
        return -1;
    value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 64ul)
        return -1;

    *out = (unsigned int)value;
    return 0;
}

static int parse_enforce_crc_value(const char *arg, int *out)
{
    const char *v = arg ? arg : "on";

    if (!out)
        return -1;
    if (strcmp(v, "on") == 0 || strcmp(v, "1") == 0)
        *out = 1;
    else if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0)
        *out = 0;
    else
        return -1;
    return 0;
}

static int parse_view_mode(const char *prog, const char *arg, cli_usage_fn print_usage)
{
    if (app_parse_output_mode(arg, s_output_modes,
                              sizeof(s_output_modes) / sizeof(s_output_modes[0]),
                              &g_output_mode) != 0)
    {
        fprintf(stderr, "Invalid view mode: %s\n", arg);
        print_usage(prog);
        return -1;
    }
    return 0;
}

/* Consume the `-d` optional argument: getopt's optional_argument doesn't
 * attach a spaced arg to a short option, so take the next token manually
 * when it doesn't look like another option. */
static void consume_device_arg(const char *optarg, int argc, char *argv[],
                               int *list_devices, const char **device_spec)
{
    *list_devices = 1;
    *device_spec = optarg;
    if (!*device_spec && optind < argc && argv[optind][0] != '-')
        *device_spec = argv[optind++];
}

/* Shared `-d` handling: bare `-d` lists devices, `-d <n>|<type>|<type>:<id>`
 * selects one, and no `-d` auto-selects the default device (so the session
 * always opens hardware that is actually present).
 * Returns 0 to continue into capture setup, 1 if the mode is done (the
 * caller returns EXIT_SUCCESS), -1 on error (caller returns EXIT_FAILURE).
 */
static int resolve_device(const char *prog, int list_devices, const char *device_spec,
                          app_device_spec_t *parsed, int *selected,
                          cli_usage_fn print_usage)
{
    if (list_devices)
    {
        if (device_spec)
        {
            if (app_resolve_device_arg(prog, device_spec, parsed) != 0)
            {
                print_usage(prog);
                return -1;
            }
            *selected = 1;
        }
        else
        {
            return app_print_available_devices(prog) == EXIT_SUCCESS ? 1 : -1;
        }
    }

    if (!*selected)
    {
        if (app_pick_default_device(prog, parsed) != 0)
            return -1;
        *selected = 1;
    }
    return 0;
}

/* Default the BR/EDR window to what the selected radio can sustain (the
 * pre-parse default assumed the build default), then validate the
 * bottom/range/layout. @p ble_fanout selects the session layout kind:
 * hybrid enables the BLE fan-out alongside BR/EDR, bredr/record do not.
 * Returns 0 on success, -1 on error (diagnostic already printed).
 */
static int resolve_bredr_window(const char *prog, int device_selected,
                                radio_device_type_t selected_type,
                                int ble_fanout)
{
    radio_device_type_t dtype;
    (void)prog;

    if (!g_channels_explicit)
    {
        dtype = device_selected ? selected_type : app_default_device_type();
        g_num_bredr_channels = session_default_bredr_count(dtype);
    }

    if (g_bottom_channel_explicit)
    {
        /* "all" mode covers the whole band: bottom must be 0. */
        if (g_num_bredr_channels == BREDR_SESSION_MAX_CHANNELS &&
            g_bottom_bredr_channel != 0u)
        {
            fprintf(stderr,
                    "Invalid --bottom %u for --channels all: "
                    "full-band capture always starts at 0.\n",
                    g_bottom_bredr_channel);
            return -1;
        }
        {
            unsigned int max_bottom_channel =
                BREDR_MAX_CHANNEL - (g_num_bredr_channels - 1u);
            if (g_bottom_bredr_channel > max_bottom_channel)
            {
                fprintf(stderr,
                        "Invalid --bottom %u for --channels %u: out of BR/EDR band (0-%u).\n"
                        "For %u channels, the highest bottom channel would be %u.\n",
                        g_bottom_bredr_channel, g_num_bredr_channels, BREDR_MAX_CHANNEL,
                        g_num_bredr_channels, max_bottom_channel);
                return -1;
            }
        }
    }

    /* The session owns layout validity: device bandwidth ceiling plus the
     * lane-split table. */
    dtype = device_selected ? selected_type : app_default_device_type();
    switch (session_validate_layout(dtype, ble_fanout, 1, SESSION_REF_BREDR,
                                    g_bottom_bredr_channel,
                                    g_num_bredr_channels))
    {
    case SESSION_LAYOUT_OK:
        break;
    case SESSION_LAYOUT_RATE_EXCEEDED:
    {
        uint32_t max_rate = session_device_max_rate_hz(dtype);
        fprintf(stderr,
                "Invalid --channels %u for %s (max %u MHz): choose <= %u channels.\n",
                g_num_bredr_channels,
                session_device_type_name(dtype),
                max_rate / 1000000u, max_rate / 1000000u);
        return -1;
    }
    case SESSION_LAYOUT_NO_LANE_SPLIT:
        fprintf(stderr,
                "Invalid --channels %u: no even <=20-channel lane split "
                "(nearest supported: %u).\n",
                g_num_bredr_channels,
                session_snap_bredr_count(g_num_bredr_channels));
        return -1;
    case SESSION_LAYOUT_BAD_RANGE:
    default:
        fprintf(stderr,
                "Invalid --channels %u (bottom %u): outside the BR/EDR band.\n",
                g_num_bredr_channels, g_bottom_bredr_channel);
        return -1;
    }
    return 0;
}

/* "all" (c=79) captures the full 0..78 band at 80 Msps with the LO on
 * the exact band centre (2441 MHz, no half-channel premix). */
static void compute_bredr_tune(unsigned int num_channels, unsigned int bottom_channel,
                               unsigned int *sample_rate_hz, double *lo_mhz,
                               uint64_t *tune_lo_hz)
{
    unsigned int sample_rate = (num_channels == BREDR_SESSION_MAX_CHANNELS)
        ? 80000000u
        : ((num_channels == 2u) ? 4000000u : num_channels * 1000000u);
    double lo = 2402.0 + (double)bottom_channel +
                ((double)num_channels - 1.0) / 2.0;

    *sample_rate_hz = sample_rate;
    *lo_mhz = lo;
    *tune_lo_hz = (uint64_t)(lo * 1e6);
}

static int check_exhaustive(int exhaustive, int is_file_input)
{
    if (exhaustive && !is_file_input)
    {
        fprintf(stderr, "--exhaustive is only meaningful with file replay input (-d file:...).\n");
        return -1;
    }
    return 0;
}

static int check_file_compatible(const char *device_id, uint32_t tune_rate_hz,
                                 uint32_t *file_rate_hz, uint64_t *file_center_hz)
{
    if (file_radio_check_compatible(device_id, tune_rate_hz,
                                    file_rate_hz, file_center_hz) != 0)
        return -1;
    return 0;
}

static int resolve_gain(const char *prog, int device_selected,
                        radio_device_type_t selected_type,
                        const char *gain_raw, radio_gain_spec_t *out)
{
    radio_device_type_t type =
        device_selected ? selected_type : app_default_device_type();
    if (app_resolve_gain_spec(prog, type, gain_raw, out) != 0)
        return -1;
    return 0;
}

static void print_device_and_gain(int device_selected,
                                  radio_device_type_t selected_type,
                                  const char *device_id,
                                  const radio_gain_spec_t *gain)
{
    /* Device selection is always resolved by now (explicit -d or default
     * pick), so the banner names the hardware that will be opened. */
    if (device_selected)
        printf("Device      : %s:%s\n",
               session_device_type_name(selected_type),
               device_id);
    else
        printf("Device      : (default)\n");
    app_print_gain_summary(device_selected ? selected_type
                                           : app_default_device_type(),
                           gain);
}

static void print_replay_banner(int exhaustive, uint32_t file_rate_hz,
                                uint64_t file_center_hz, uint64_t tune_lo_hz,
                                double lo_mhz)
{
    printf("Replay      : %s, single pass, %u Hz",
           exhaustive ? "exhaustive" : "realtime", file_rate_hz);
    if (file_center_hz != 0u)
        printf(", capture LO %llu Hz", (unsigned long long)file_center_hz);
    else
        printf(" (capture LO unknown: no auxi/filename tag)");
    printf("\n");
    if (file_center_hz != 0u && file_center_hz != tune_lo_hz)
        printf("Warning     : tuned LO %.1f MHz differs from capture LO %.3f MHz\n",
               lo_mhz, (double)file_center_hz / 1e6);
}

/* Debug tail of the session summary. @p with_header reproduces the
 * historical difference that bredr omits the "=== Debug Summary ==="
 * header line while ble/hybrid print it. */
static void print_debug_summary(session_t *session, int with_header,
                                int want_ble, int want_bredr)
{
    session_drop_breakdown_t drops;

    if (with_header)
        printf("\n=== Debug Summary ===\n");
    session_dropped_blocks_breakdown(session, &drops);
    app_print_drop_breakdown(&drops);
    if (want_ble)
    {
        unsigned long emitted = 0ul, confirmed = 0ul;
        session_ble_frame_counts(session, &emitted, &confirmed);
        printf("  BLE frames emitted   : %lu\n", emitted);
        printf("  BLE frames confirmed : %lu\n", confirmed);
    }
    if (want_bredr)
        printf("  BR/EDR frames emitted: %lu\n", session_bredr_frame_count(session));
    if (want_ble || want_bredr)
    {
        unsigned long ble_dropped = 0ul, bredr_dropped = 0ul;
        session_collector_dropped(session,
                                  want_ble ? &ble_dropped : NULL,
                                  want_bredr ? &bredr_dropped : NULL);
        if (want_ble)
            printf("  BLE collector drops  : %lu\n", ble_dropped);
        if (want_bredr)
            printf("  BR/EDR collector drops: %lu\n", bredr_dropped);
    }
}

/* -------------------------------------------------------------------------
 * Shared packet printers (single copy; used by ble+hybrid and bredr+hybrid)
 * -------------------------------------------------------------------------*/

static void print_ble_packet_full(unsigned long packet_no,
                                  const ble_event_t *event)
{
    const rx_metadata_t *meta = &event->meta;
    ble_packet_t packet;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
        printf("Radio Sample : %" PRIu64 " (%u Msps input)\n",
            meta->radio_start_sample_index,
            (unsigned int)(meta->radio_sample_rate_hz / 1000000u));
    printf("Type         : BLE\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n", meta->rssi_dbr);
    printf("\n");
    if (ble_decode_frame(&event->frame, meta->channel_index, &packet) == 0)
        ble_print_packet(&packet);
    else
        printf("[BLE Decode Error]\n");
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const ble_event_t *event)
{
    app_summary_view_print_ble(packet_no, event);
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                       const bredr_event_t *event,
                                       const bredr_connection_snapshot_t *connection)
{
    app_summary_view_print_bredr(packet_no, event, connection);
}

/* BR/EDR full view for the bredr mode (blank line after the details). */
static void print_bredr_packet_full(unsigned long packet_no,
                                    const bredr_event_t *event,
                                    const bredr_connection_snapshot_t *connection)
{
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
        printf("Radio Sample : %" PRIu64 " (%u Msps input)\n",
            meta->radio_start_sample_index,
            (unsigned int)(meta->radio_sample_rate_hz / 1000000u));
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n", meta->rssi_dbr);
    bredr_print_packet_details(frame, connection, meta);

    printf("--------------------------------------------------\n");
}

/* BR/EDR full view for the hybrid mode (blank line after the RSSI line,
 * none after the details). */
static void print_hybrid_bredr_packet_full(unsigned long packet_no,
                                           const bredr_event_t *event,
                                           const bredr_connection_snapshot_t *connection)
{
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
        printf("Radio Sample : %" PRIu64 " (%u Msps input)\n",
            meta->radio_start_sample_index,
            (unsigned int)(meta->radio_sample_rate_hz / 1000000u));
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(meta->center_frequency_hz / 1000000u), meta->channel_index);
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    bredr_print_packet_details(frame, connection, meta);
    printf("--------------------------------------------------\n");
}

/* Shared BLE packet path (ble + hybrid): devices mode is owned by the
 * live table thread, and CRC enforcement drops undecodable/bad-CRC
 * frames before anything is emitted. */
static void handle_ble_packet(const ble_event_t *event, void *user)
{
    (void)user;

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;

    if (g_enforce_crc)
    {
        ble_packet_t pkt;
        if (ble_decode_frame(&event->frame, event->meta.channel_index, &pkt) != 0 ||
            !ble_verify_crc(&pkt))
            return;
    }

    app_output_lock();
    {
        unsigned long packet_no = ++g_packet_count;
        if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
            print_ble_packet_summary(packet_no, event);
        else
            print_ble_packet_full(packet_no, event);
    }
    fflush(stdout);
    app_output_unlock();
}

/* -------------------------------------------------------------------------
 * Record backend (record-only raw IQ capture; moved from app_record.c)
 * -------------------------------------------------------------------------*/

static _Atomic unsigned int g_record_stop = 0u;
static sample_reader_t *g_record_reader = NULL;

static void record_handle_sigint(int sig)
{
    (void)sig;
    atomic_store_explicit(&g_record_stop, 1u, memory_order_release);
    /* Same pattern as session_request_stop(): wake the blocked pop so the
     * loop observes the flag promptly. */
    sample_reader_signal(g_record_reader);
}

static int app_record_run(const app_record_config_t *cfg)
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

    if (!cfg)
    {
        fprintf(stderr, "--record: missing config\n");
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
    if (!cfg->device_id &&
        session_get_default_device(NULL, NULL, 0u) != RADIO_SUCCESS)
    {
        fprintf(stderr, "--record: no devices found\n");
        return 1;
    }

    /* Always record to the directory the program was run in (cwd):
     * generate a self-describing filename here. */
    {
        char name[256];
        if (wav_build_recording_filename(cfg->lo_freq_hz,
                                         cfg->sample_rate_hz,
                                         name, sizeof(name)) != 0)
        {
            fprintf(stderr, "--record: cannot build filename\n");
            return 1;
        }
        int needed = snprintf(resolved, sizeof(resolved), "./%s", name);
        if (needed < 0 || (size_t)needed >= sizeof(resolved))
        {
            fprintf(stderr, "--record: output path too long\n");
            return 1;
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
            /* Caller-resolved gains (-g or device defaults). */
            .gain = cfg->gain,
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
    signal(SIGINT, record_handle_sigint);
    signal(SIGTERM, record_handle_sigint);

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
        const float complex *samples;
        unsigned int count;
        uint64_t base;
        (void)base;

        /* Raw mode: next() hands out pool blocks directly (zero copy);
         * each call releases the previous block. */
        if (sample_reader_next(&reader, &g_record_stop,
                               &samples, &count, &base) != 0)
            break;
        if (count == 0u)
            continue;
        if (wav_write_frames(&writer, samples, count) != 0)
        {
            fprintf(stderr, "--record: write failed\n");
            break;
        }
        samples_written += count;
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

/* -------------------------------------------------------------------------
 * BLE mode
 * -------------------------------------------------------------------------*/

static unsigned int ble_num_le_channels = BLE_SESSION_MAX_CHANNELS;
static unsigned int ble_bottom_le_channel = BLE_CH37_INDEX;
static int ble_channels_explicit = 0;
static session_t ble_session;
static int ble_session_initialized = 0;

static void ble_handle_sigint(int sig)
{
    (void)sig;
    if (ble_session_initialized)
        session_request_stop(&ble_session);
}

static void ble_print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "\nGeneral Options:\n");
    fprintf(stderr, "  %-30s Packet view style (default: summary)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of consecutive LE RF channels (1-%u, default: %u)\n",
            "-c, --channels N",
            BLE_SESSION_MAX_CHANNELS, ble_num_le_channels);
    fprintf(stderr, "  %-30s Bottom LE channel of the window (0-39, default: 37)\n",
            "-b, --bottom CH");
    app_print_device_usage_line();
    app_print_gain_usage_line();
    app_print_debug_usage_line();
    app_print_exhaustive_usage_line();
    fprintf(stderr, "\nLE Options:\n");
    app_print_enforce_crc_usage_line();
    app_print_other_usage_lines();
}

int ble_main(int argc, char *argv[])
{
    argv[0] = (char *)"supertooth le";
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"channels", required_argument, NULL, 'c'},
        {"bottom", required_argument, NULL, 'b'},
        {"device", optional_argument, NULL, 'd'},
        {"gain", required_argument, NULL, 'g'},
        {"version", no_argument, NULL, 'V'},
        {"debug", no_argument, NULL, APP_OPT_DEBUG},
        {"enforce-crc", required_argument, NULL, APP_OPT_ENFORCE_CRC},
        {"exhaustive", no_argument, NULL, APP_OPT_EXHAUSTIVE},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int list_devices = 0;
    const char *device_spec = NULL;
    int exhaustive = 0;
    app_device_spec_t spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int device_selected = 0;
    const char *gain_raw = NULL;
    radio_gain_spec_t gain_spec;
    int opt;
    int resolve_rc;
    unsigned int bottom_rf;
    unsigned int span_mhz;
    unsigned int rate_mhz;
    double lo_mhz;
    uint32_t tune_rate_hz;
    uint64_t tune_lo_hz;
    int is_file_input;
    uint64_t file_center_hz = 0u;
    uint32_t file_rate_hz = 0u;
    unsigned int adv_found;
    unsigned int rf;
    session_config_t config;
    session_ble_config_t ble_cfg;
    int result;

    spec_parsed.type = app_default_device_type();

    /* Default the LE window to what the radio can sustain (HackRF -> 10;
     * bladeRF -> 30; 80 Msps file replay -> 40). Re-resolved below when
     * the user did not pass -c and a different device was selected. */
    ble_num_le_channels = session_default_ble_count(spec_parsed.type);

    while ((opt = getopt_long(argc, argv, "v:c:b:d::g:Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'v':
            if (parse_view_mode(argv[0], optarg, ble_print_usage) != 0)
                return EXIT_FAILURE;
            break;
        case 'c':
            if (parse_le_channel_count(optarg, &ble_num_le_channels) != 0)
            {
                fprintf(stderr, "Invalid --channels value: %s (expected 1-%u)\n",
                        optarg, BLE_SESSION_MAX_CHANNELS);
                ble_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            ble_channels_explicit = 1;
            break;
        case 'b':
            if (parse_le_bottom_channel(optarg, &ble_bottom_le_channel) != 0)
            {
                fprintf(stderr, "Invalid --bottom value: %s (expected 0-39)\n",
                        optarg);
                ble_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'd':
            consume_device_arg(optarg, argc, argv, &list_devices, &device_spec);
            break;
        case APP_OPT_DEBUG:
            g_debug = 1;
            break;
        case 'g':
            gain_raw = optarg;
            break;
        case APP_OPT_EXHAUSTIVE:
            exhaustive = 1;
            break;
        case APP_OPT_ENFORCE_CRC:
            if (parse_enforce_crc_value(optarg, &g_enforce_crc) != 0)
            {
                fprintf(stderr, "Invalid --enforce-crc value: %s (expected on or off)\n",
                        optarg ? optarg : "on");
                ble_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'V':
            printf("supertooth le %s\n", supertooth_get_version());
            return EXIT_SUCCESS;
        case 'h':
            ble_print_usage(argv[0]);
            return EXIT_SUCCESS;
        default:
            ble_print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    resolve_rc = resolve_device(argv[0], list_devices, device_spec,
                                &spec_parsed, &device_selected,
                                ble_print_usage);
    if (resolve_rc > 0)
        return EXIT_SUCCESS;
    if (resolve_rc < 0)
        return EXIT_FAILURE;

    /* When the user did not pass -c, default to what the *selected* radio
     * can sustain (the pre-parse default assumed the build default). */
    if (!ble_channels_explicit)
    {
        radio_device_type_t dtype =
            device_selected ? spec_parsed.type : app_default_device_type();
        ble_num_le_channels = session_default_ble_count(dtype);
    }

    bottom_rf = ble_rf_for_channel_number(ble_bottom_le_channel);
    if (bottom_rf >= BLE_RF_CHANNEL_COUNT ||
        bottom_rf + ble_num_le_channels > BLE_RF_CHANNEL_COUNT)
    {
        fprintf(stderr,
                "Invalid window: LE ch%u with %u channel%s exceeds the LE band.\n",
                ble_bottom_le_channel, ble_num_le_channels,
                ble_num_le_channels == 1u ? "" : "s");
        return EXIT_FAILURE;
    }

    span_mhz = 2u * ble_num_le_channels;
    rate_mhz = (span_mhz == 2u) ? 4u : span_mhz;
    lo_mhz = 2401.0 + 2.0 * (double)bottom_rf + (double)ble_num_le_channels;

    /* The session owns layout validity: device bandwidth ceiling plus the
     * lane-split table (BLE spans are 2 MHz/RF). */
    {
        radio_device_type_t dtype =
            device_selected ? spec_parsed.type : app_default_device_type();
        switch (session_validate_layout(dtype, 1, 0, SESSION_REF_BLE,
                                        bottom_rf, ble_num_le_channels))
        {
            case SESSION_LAYOUT_OK:
                break;
            case SESSION_LAYOUT_RATE_EXCEEDED:
            {
                uint32_t max_rate =
                    session_device_max_rate_hz(dtype);
                fprintf(stderr,
                        "Invalid --channels %u for %s (max %u LE channels): "
                        "choose <= %u.\n",
                        ble_num_le_channels, session_device_type_name(dtype),
                        max_rate / 2000000u, max_rate / 2000000u);
                return EXIT_FAILURE;
            }
            case SESSION_LAYOUT_NO_LANE_SPLIT:
                fprintf(stderr,
                        "Invalid --channels %u: no even <=20 MHz lane split.\n",
                        ble_num_le_channels);
                return EXIT_FAILURE;
            case SESSION_LAYOUT_BAD_RANGE:
            default:
                fprintf(stderr,
                        "Invalid --channels %u (bottom RF %u): outside the LE band.\n",
                        ble_num_le_channels, bottom_rf);
                return EXIT_FAILURE;
        }
    }

    tune_rate_hz = rate_mhz * 1000000u;
    tune_lo_hz = (uint64_t)(lo_mhz * 1e6);
    is_file_input = device_selected &&
                    spec_parsed.type == RADIO_DEVICE_FILE;

    if (check_exhaustive(exhaustive, is_file_input) != 0)
        return EXIT_FAILURE;

    if (is_file_input)
    {
        if (check_file_compatible(spec_parsed.id, tune_rate_hz,
                                  &file_rate_hz, &file_center_hz) != 0)
            return EXIT_FAILURE;
    }

    if (resolve_gain(argv[0], device_selected, spec_parsed.type,
                     gain_raw, &gain_spec) != 0)
        return EXIT_FAILURE;

    printf("Supertooth LE\n");
    printf("==============\n");
    printf("Window      : %u LE channel%s from ch%u (RF %u-%u):",
           ble_num_le_channels, ble_num_le_channels == 1u ? "" : "s",
           ble_bottom_le_channel, bottom_rf, bottom_rf + ble_num_le_channels - 1u);
    for (rf = bottom_rf; rf < bottom_rf + ble_num_le_channels; rf++)
        printf(" %u", (unsigned int)ble_channel_number_for_rf(rf));
    printf("\n");
    printf("Advertising :");
    adv_found = 0u;
    for (rf = bottom_rf; rf < bottom_rf + ble_num_le_channels; rf++)
    {
        if (ble_rf_is_advertising(rf))
        {
            printf(" %u", (unsigned int)ble_channel_number_for_rf(rf));
            adv_found++;
        }
    }
    if (!adv_found)
        printf(" (none in window)");
    printf("\n");
    printf("View mode   : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("Enforce CRC : %s\n", g_enforce_crc ? "on" : "off");
    print_device_and_gain(device_selected, spec_parsed.type, spec_parsed.id,
                          &gain_spec);
    if (is_file_input)
        print_replay_banner(exhaustive, file_rate_hz, file_center_hz,
                            tune_lo_hz, lo_mhz);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");
    signal(SIGINT, ble_handle_sigint);

    config.device_type = spec_parsed.type;
    config.device_id = device_selected ? spec_parsed.id : NULL;
    config.debug = g_debug;
    config.gain = gain_spec;
    config.file_exhaustive = exhaustive;

    if (session_init(&ble_session, &config) != 0)
    {
        fprintf(stderr, "Failed to initialize session.\n");
        return EXIT_FAILURE;
    }
    ble_session_initialized = 1;

    ble_cfg.enforce_crc = g_enforce_crc ? 1u : 0u;
    session_enable_ble(&ble_session, &ble_cfg, handle_ble_packet, NULL);

    if (session_tune(&ble_session, SESSION_REF_BLE, bottom_rf, ble_num_le_channels) != 0)
    {
        fprintf(stderr, "Failed to tune session.\n");
        session_destroy(&ble_session);
        ble_session_initialized = 0;
        return EXIT_FAILURE;
    }

    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        app_summary_view_print_header();

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        g_device_view = app_device_view_start(&ble_session);

    result = session_run(&ble_session);

    if (g_device_view)
    {
        app_device_view_stop(g_device_view);
        g_device_view = NULL;
    }
    session_destroy(&ble_session);
    ble_session_initialized = 0;

    if (result != 0)
    {
        fprintf(stderr, "LE receiver failed.\n");
        return EXIT_FAILURE;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Window         : %u LE channel%s from ch%u (RF %u-%u)\n",
           ble_num_le_channels, ble_num_le_channels == 1u ? "" : "s",
           ble_bottom_le_channel, bottom_rf, bottom_rf + ble_num_le_channels - 1u);
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");

    if (g_debug)
        print_debug_summary(&ble_session, 1, 1, 0);

    return 0;
}

/* -------------------------------------------------------------------------
 * BR/EDR mode
 * -------------------------------------------------------------------------*/

static int bredr_lap_filter_enabled = 0;
static uint32_t bredr_lap_filter = 0u;

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                    const bredr_event_t *event,
                                    const bredr_connection_snapshot_t *connection);

static int connection_lap_cmp(const void *a, const void *b)
{
    const bredr_connection_snapshot_t *pa = *(const bredr_connection_snapshot_t *const *)a;
    const bredr_connection_snapshot_t *pb = *(const bredr_connection_snapshot_t *const *)b;
    uint32_t la = pa ? (pa->lap & 0xFFFFFFu) : 0u;
    uint32_t lb = pb ? (pb->lap & 0xFFFFFFu) : 0u;
    if (la < lb)
        return -1;
    if (la > lb)
        return 1;
    return 0;
}

static unsigned int current_master_clock_mhz(void)
{
    return g_num_bredr_channels == 2u ? 4u : g_num_bredr_channels;
}

static void print_packet_rssi(unsigned long packet_no,
                              const bredr_event_t *event,
                              const bredr_connection_snapshot_t *connection)
{
    (void)connection;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    bredr_connection_snapshot_t snapshots[BREDR_SESSION_MAX_CHANNELS * 2u];
    size_t count = session_get_bredr_connections(
        g_session, snapshots, sizeof(snapshots) / sizeof(snapshots[0]));
    const bredr_connection_snapshot_t **ordered =
        (const bredr_connection_snapshot_t **)malloc(sizeof(*ordered) * (count > 0u ? count : 1u));
    size_t used = 0u;
    size_t i;
    if (!ordered)
        return;

    for (i = 0; i < count; i++)
        ordered[used++] = &snapshots[i];
    qsort(ordered, used, sizeof(*ordered), connection_lap_cmp);

    bredr_print_rssi_snapshot(packet_no, frame, meta,
                              (const bredr_connection_snapshot_t *const *)ordered,
                              used, current_master_clock_mhz());
    free(ordered);
}

static packet_formatter_fn output_mode_formatter(app_output_mode_t mode)
{
    switch (mode)
    {
    case APP_OUTPUT_MODE_SUMMARY:
        return print_bredr_packet_summary;
    case APP_OUTPUT_MODE_RSSI:
        return print_packet_rssi;
    case APP_OUTPUT_MODE_FULL:
    default:
        return print_bredr_packet_full;
    }
}

/* Parse a "<type>:<id>" device spec (e.g. "hackrf:b25062dc22113a0b").
 * On success sets *out_type and *out_id (pointing into @p spec). */
static void bredr_print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "\nGeneral Options:\n");
    fprintf(stderr, "  %-30s Packet view style (default: summary)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-78, 79/\"all\" = full band, default: %u)\n",
            "-c, --channels N",
            g_num_bredr_channels);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    app_print_gain_usage_line();
    app_print_debug_usage_line();
    app_print_exhaustive_usage_line();
    fprintf(stderr, "\nBREDR Options:\n");
    fprintf(stderr, "  %-30s Only track/report this LAP (e.g. 0x1FC475)\n", "-l, --lap LAP");
    app_print_ac_errors_usage_line();
    app_print_other_usage_lines();
}

static void handle_bredr_packet(const bredr_event_t *event,
                                const bredr_connection_snapshot_t *connection,
                                void *user)
{
    (void)user;
    /* In devices mode the live table thread owns all output; the per-packet
     * path is suppressed so the two never interleave. */
    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;
    app_output_lock();
    g_packet_count++;
    output_mode_formatter(g_output_mode)(g_packet_count, event, connection);
    fflush(stdout);
    app_output_unlock();
}

int bredr_main(int argc, char *argv[])
{
    argv[0] = (char *)"supertooth bredr";
    static const struct option long_opts[] = {
        {"view",           required_argument, NULL, 'v'},
        {"lap",            required_argument, NULL, 'l'},
        {"channels",       required_argument, NULL, 'c'},
        {"bottom", required_argument, NULL, 'b'},
        {"device",         optional_argument, NULL, 'd'},
        {"gain",           required_argument, NULL, 'g'},
        {"ac-errors",      required_argument, NULL, APP_OPT_AC_ERRORS},
        {"exhaustive",     no_argument,       NULL, APP_OPT_EXHAUSTIVE},
        {"version",        no_argument,       NULL, 'V'},
        {"debug",          no_argument,       NULL, APP_OPT_DEBUG},
        {"help",           no_argument,       NULL, 'h'},
        {NULL,             0,                 NULL,  0 }
    };

    int list_devices = 0;
    const char *device_spec = NULL;
    int exhaustive = 0;
    app_device_spec_t spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int device_selected = 0;
    const char *gain_raw = NULL;
    radio_gain_spec_t gain_spec;
    const char *mode_name;
    unsigned int sample_rate;
    double tune_lo_mhz;
    uint64_t tune_lo_hz;
    int is_file_input;
    uint64_t file_center_hz = 0u;
    uint32_t file_rate_hz = 0u;
    session_config_t config;
    session_bredr_config_t bredr_cfg;
    int result;
    int resolve_rc;
    int opt;

    spec_parsed.type = app_default_device_type();

    /* Default the channel count to what the radio can actually sustain and
     * the service can stage (HackRF -> 20; bladeRF -> 60; 80 Msps file
     * replay -> 72). */
    g_num_bredr_channels =
        session_default_bredr_count(spec_parsed.type);

    while ((opt = getopt_long(argc, argv, "v:l:a:c:b:d::g:Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
            case 'v':
                if (parse_view_mode(argv[0], optarg, bredr_print_usage) != 0)
                    return EXIT_FAILURE;
                break;
            case 'l':
                if (parse_lap_filter(optarg, &bredr_lap_filter) != 0)
                {
                    fprintf(stderr, "Invalid LAP: %s\n", optarg);
                    bredr_print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                bredr_lap_filter_enabled = 1;
                break;
            case 'c':
        if (parse_bredr_channel_count(optarg, &g_num_bredr_channels) != 0)
        {
            fprintf(stderr, "Invalid --channels value: %s (expected even 2-78, 79 or \"all\")\n",
                    optarg);
            bredr_print_usage(argv[0]);
            return EXIT_FAILURE;
        }
                g_channels_explicit = 1;
                break;
            case 'b':
                if (parse_bredr_bottom_channel(optarg, &g_bottom_bredr_channel) != 0)
                {
                    fprintf(stderr, "Invalid --bottom value: %s (expected 0-%u)\n",
                            optarg, BREDR_MAX_CHANNEL);
                    bredr_print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_bottom_channel_explicit = 1;
                break;
            case 'd':
                consume_device_arg(optarg, argc, argv, &list_devices, &device_spec);
                break;
            case APP_OPT_DEBUG:
                g_debug = 1;
                break;
            case 'g':
                gain_raw = optarg;
                break;
            case APP_OPT_EXHAUSTIVE:
                exhaustive = 1;
                break;
            case APP_OPT_AC_ERRORS:
                if (parse_ac_errors_value(optarg, &g_ac_errors) != 0)
                {
                    fprintf(stderr, "Invalid --ac-errors value: %s (expected 0-64)\n",
                            optarg);
                    bredr_print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'V':
                printf("supertooth bredr %s\n", supertooth_get_version());
                return EXIT_SUCCESS;
            case 'h':
                bredr_print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                bredr_print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }
    if (optind != argc)
    {
        bredr_print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    resolve_rc = resolve_device(argv[0], list_devices, device_spec,
                                &spec_parsed, &device_selected,
                                bredr_print_usage);
    if (resolve_rc > 0)
        return EXIT_SUCCESS;
    if (resolve_rc < 0)
        return EXIT_FAILURE;

    if (resolve_bredr_window(argv[0], device_selected, spec_parsed.type,
                             0) != 0)
        return EXIT_FAILURE;

    mode_name =
        app_output_mode_name(g_output_mode, s_output_modes,
                             sizeof(s_output_modes) / sizeof(s_output_modes[0]));
    compute_bredr_tune(g_num_bredr_channels, g_bottom_bredr_channel,
                       &sample_rate, &tune_lo_mhz, &tune_lo_hz);
    is_file_input = device_selected &&
                    spec_parsed.type == RADIO_DEVICE_FILE;

    if (check_exhaustive(exhaustive, is_file_input) != 0)
        return EXIT_FAILURE;

    if (is_file_input)
    {
        if (check_file_compatible(spec_parsed.id, sample_rate,
                                  &file_rate_hz, &file_center_hz) != 0)
            return EXIT_FAILURE;
    }
    if (resolve_gain(argv[0], device_selected, spec_parsed.type,
                     gain_raw, &gain_spec) != 0)
        return EXIT_FAILURE;
    printf("Supertooth BR/EDR\n");
    printf("=================\n");
    printf("Channels    : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("View mode   : %s\n", mode_name);
    if (bredr_lap_filter_enabled)
        printf("LAP filter  : %06" PRIX32 "\n", bredr_lap_filter);
    else
        printf("LAP filter  : (none)\n");
    printf("AC errors   : %u\n", g_ac_errors);
    print_device_and_gain(device_selected, spec_parsed.type, spec_parsed.id,
                          &gain_spec);
    if (is_file_input)
        print_replay_banner(exhaustive, file_rate_hz, file_center_hz,
                            tune_lo_hz, tune_lo_mhz);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");

    config.device_type = spec_parsed.type;
    config.device_id = device_selected ? spec_parsed.id : NULL;
    config.debug = g_debug;
    config.gain = gain_spec;
    config.file_exhaustive = exhaustive;
    g_session = (session_t *)calloc(1, sizeof(*g_session));
    if (!g_session)
        return EXIT_FAILURE;
    if (session_init(g_session, &config) != 0)
    {
        free(g_session);
        g_session = NULL;
        return EXIT_FAILURE;
    }

    /* Install the Ctrl+C handler only once the session exists, so the signal
     * actually reaches a valid session and stops the capture loop. */
    app_install_sigint_handler(g_session);

    bredr_cfg.lap_filter = bredr_lap_filter;
    bredr_cfg.lap_filter_enabled = bredr_lap_filter_enabled;
    session_enable_bredr(g_session, &bredr_cfg, handle_bredr_packet, NULL);

    if (session_tune(g_session, SESSION_REF_BREDR, g_bottom_bredr_channel,
                     g_num_bredr_channels) != 0)
    {
        fprintf(stderr, "Failed to tune session.\n");
        session_destroy(g_session);
        free(g_session);
        g_session = NULL;
        return EXIT_FAILURE;
    }

    /* Apply the global access-code error tolerance before streaming begins.
     * The bitstream decoder is the sole access-code acceptance gate. */
    bredr_bitstream_decoder_set_global_max_ac_errors((uint8_t)g_ac_errors);

    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        app_summary_view_print_header();

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        g_device_view = app_device_view_start(g_session);

    result = session_run(g_session);

    if (result != 0)
        fprintf(stderr, "BR/EDR receiver failed.\n");

    if (g_device_view)
    {
        app_device_view_stop(g_device_view);
        g_device_view = NULL;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", mode_name);
    printf("  Channels       : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    if (bredr_lap_filter_enabled)
        printf("  LAP filter     : %06" PRIX32 "\n", bredr_lap_filter);
    else
        printf("  LAP filter     : (none)\n");
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    if (g_debug)
        print_debug_summary(g_session, 0, 0, 1);
    session_destroy(g_session);
    free(g_session);
    g_session = NULL;

    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

/* -------------------------------------------------------------------------
 * Hybrid mode
 * -------------------------------------------------------------------------*/

/* LE channels whose centers lie fully inside the capture span (same rule
 * the session's BLE fan-out uses): returns the count and collects the
 * advertising channel numbers among them for display. */
static unsigned int ble_channels_in_window(uint64_t lo_hz, uint32_t sample_rate,
                                           uint8_t adv_out[3])
{
    unsigned int count = 0u, adv_count = 0u;
    for (unsigned int rf = 0; rf < BLE_RF_CHANNEL_COUNT; rf++)
    {
        if (!ble_rf_in_capture_span(rf, lo_hz, sample_rate))
            continue;
        count++;
        if (ble_rf_is_advertising(rf) && adv_count < 3u)
            adv_out[adv_count++] = ble_channel_number_for_rf(rf);
    }
    return count;
}

static void hybrid_print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "\nGeneral Options:\n");
    fprintf(stderr, "  %-30s Packet view style (default: summary)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-78, 79/\"all\" = full band, default: %u)\n",
            "-c, --channels N",
            BREDR_SESSION_MAX_CHANNELS);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    app_print_gain_usage_line();
    app_print_debug_usage_line();
    app_print_exhaustive_usage_line();
    fprintf(stderr, "\nBREDR Options:\n");
    app_print_ac_errors_usage_line();
    fprintf(stderr, "\nLE Options:\n");
    app_print_enforce_crc_usage_line();
    app_print_other_usage_lines();
}

static void handle_hybrid_bredr_packet(const bredr_event_t *event,
                                       const bredr_connection_snapshot_t *connection,
                                       void *user)
{
    (void)user;
    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;
    app_output_lock();
    {
        unsigned long packet_no = ++g_packet_count;
        if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
            print_bredr_packet_summary(packet_no, event, connection);
        else
            print_hybrid_bredr_packet_full(packet_no, event, connection);
    }
    fflush(stdout);
    app_output_unlock();
}

int hybrid_main(int argc, char *argv[])
{
    argv[0] = (char *)"supertooth hybrid";
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"channels", required_argument, NULL, 'c'},
        {"bottom", required_argument, NULL, 'b'},
        {"device", optional_argument, NULL, 'd'},
        {"gain", required_argument, NULL, 'g'},
        {"ac-errors", required_argument, NULL, APP_OPT_AC_ERRORS},
        {"exhaustive", no_argument, NULL, APP_OPT_EXHAUSTIVE},
        {"version", no_argument, NULL, 'V'},
        {"debug", no_argument, NULL, APP_OPT_DEBUG},
        {"enforce-crc", required_argument, NULL, APP_OPT_ENFORCE_CRC},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int list_devices = 0;
    const char *device_spec = NULL;
    int exhaustive = 0;
    app_device_spec_t spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int device_selected = 0;
    const char *gain_raw = NULL;
    radio_gain_spec_t gain_spec;
    unsigned int channel_count;
    unsigned int bottom_channel;
    unsigned int sample_rate;
    double lo_mhz;
    uint64_t tune_lo_hz;
    int is_file_input;
    uint64_t file_center_hz = 0u;
    uint32_t file_rate_hz = 0u;
    uint8_t ble_adv[3] = {0u, 0u, 0u};
    unsigned int ble_count;
    session_config_t config;
    session_bredr_config_t bredr_cfg = { 0 };
    session_ble_config_t ble_cfg;
    int result;
    int resolve_rc;
    int opt;

    spec_parsed.type = app_default_device_type();

    /* Default the BR/EDR channel count to what the radio can sustain and
     * the service can stage (HackRF -> 20; bladeRF -> 60; 80 Msps file
     * replay -> 72). */
    g_num_bredr_channels =
        session_default_bredr_count(spec_parsed.type);

    while ((opt = getopt_long(argc, argv, "v:c:b:d::g:Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'v':
            if (parse_view_mode(argv[0], optarg, hybrid_print_usage) != 0)
                return EXIT_FAILURE;
            break;
        case 'c':
            if (parse_bredr_channel_count(optarg, &g_num_bredr_channels) != 0)
            {
                fprintf(stderr, "Invalid --channels value: %s (expected even 2-78, 79 or \"all\")\n",
                        optarg);
                hybrid_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            g_channels_explicit = 1;
            break;
        case 'b':
            if (parse_bredr_bottom_channel(optarg, &g_bottom_bredr_channel) != 0)
            {
                fprintf(stderr, "Invalid --bottom-channel value: %s (expected 0-%u)\n",
                        optarg, BREDR_MAX_CHANNEL);
                hybrid_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            g_bottom_channel_explicit = 1;
            break;
        case 'd':
            consume_device_arg(optarg, argc, argv, &list_devices, &device_spec);
            break;
        case APP_OPT_DEBUG:
            g_debug = 1;
            break;
        case 'g':
            gain_raw = optarg;
            break;
        case APP_OPT_EXHAUSTIVE:
            exhaustive = 1;
            break;
        case APP_OPT_ENFORCE_CRC:
            if (parse_enforce_crc_value(optarg, &g_enforce_crc) != 0)
            {
                fprintf(stderr, "Invalid --enforce-crc value: %s (expected on or off)\n",
                        optarg ? optarg : "on");
                hybrid_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case APP_OPT_AC_ERRORS:
            if (parse_ac_errors_value(optarg, &g_ac_errors) != 0)
            {
                fprintf(stderr, "Invalid --ac-errors value: %s (expected 0-64)\n",
                        optarg);
                hybrid_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'V':
            printf("supertooth hybrid %s\n", supertooth_get_version());
            return EXIT_SUCCESS;
        case 'h':
            hybrid_print_usage(argv[0]);
            return EXIT_SUCCESS;
        default:
            hybrid_print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    resolve_rc = resolve_device(argv[0], list_devices, device_spec,
                                &spec_parsed, &device_selected,
                                hybrid_print_usage);
    if (resolve_rc > 0)
        return EXIT_SUCCESS;
    if (resolve_rc < 0)
        return EXIT_FAILURE;

    if (resolve_bredr_window(argv[0], device_selected, spec_parsed.type,
                             1) != 0)
        return EXIT_FAILURE;

    channel_count  = g_num_bredr_channels;
    bottom_channel = g_bottom_bredr_channel;

    compute_bredr_tune(g_num_bredr_channels, g_bottom_bredr_channel,
                       &sample_rate, &lo_mhz, &tune_lo_hz);
    is_file_input = device_selected &&
                    spec_parsed.type == RADIO_DEVICE_FILE;

    if (check_exhaustive(exhaustive, is_file_input) != 0)
        return EXIT_FAILURE;

    if (is_file_input)
    {
        if (check_file_compatible(spec_parsed.id, sample_rate,
                                  &file_rate_hz, &file_center_hz) != 0)
            return EXIT_FAILURE;
    }

    if (resolve_gain(argv[0], device_selected, spec_parsed.type,
                     gain_raw, &gain_spec) != 0)
        return EXIT_FAILURE;

    ble_count =
        ble_channels_in_window((uint64_t)(lo_mhz * 1e6), sample_rate, ble_adv);

    printf("Supertooth Hybrid\n");
    printf("=================\n");
    printf("Channels    : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel,
           g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("LE fan-out : up to %u LE channels in window\n", ble_count);
    printf("View mode   : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("Enforce CRC : %s\n", g_enforce_crc ? "on" : "off");
    printf("AC errors   : %u\n", g_ac_errors);
    print_device_and_gain(device_selected, spec_parsed.type, spec_parsed.id,
                          &gain_spec);
    if (is_file_input)
        print_replay_banner(exhaustive, file_rate_hz, file_center_hz,
                            tune_lo_hz, lo_mhz);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");

    config.device_type = spec_parsed.type;
    config.device_id = device_selected ? spec_parsed.id : NULL;
    config.debug = g_debug;
    config.gain = gain_spec;
    config.file_exhaustive = exhaustive;
    g_session = (session_t *)calloc(1, sizeof(*g_session));
    if (!g_session)
        return EXIT_FAILURE;
    if (session_init(g_session, &config) != 0)
    {
        free(g_session);
        g_session = NULL;
        return EXIT_FAILURE;
    }

    /* Install the Ctrl+C handler only once the session exists, so the signal
     * actually reaches a valid session and stops the capture loop. */
    app_install_sigint_handler(g_session);

    session_enable_bredr(g_session, &bredr_cfg, handle_hybrid_bredr_packet, NULL);
    ble_cfg.enforce_crc = g_enforce_crc ? 1u : 0u;
    session_enable_ble(g_session, &ble_cfg, handle_ble_packet, NULL);

    if (session_tune(g_session, SESSION_REF_BREDR, bottom_channel, channel_count) != 0)
    {
        fprintf(stderr, "Failed to tune session.\n");
        session_destroy(g_session);
        free(g_session);
        g_session = NULL;
        return EXIT_FAILURE;
    }

    /* Apply the global access-code error tolerance before streaming begins.
     * The bitstream decoder is the sole access-code acceptance gate. */
    bredr_bitstream_decoder_set_global_max_ac_errors((uint8_t)g_ac_errors);

    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        app_summary_view_print_header();

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        g_device_view = app_device_view_start(g_session);

    result = session_run(g_session);

    if (result != 0)
        fprintf(stderr, "Hybrid receiver failed.\n");

    if (g_device_view)
    {
        app_device_view_stop(g_device_view);
        g_device_view = NULL;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Channels       : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    if (g_debug)
        print_debug_summary(g_session, 1, 1, 1);

    session_destroy(g_session);
    free(g_session);
    g_session = NULL;
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

/* -------------------------------------------------------------------------
 * Record mode
 * -------------------------------------------------------------------------*/

static void record_print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options]\n", argv0);
    fprintf(stderr, "\nGeneral Options:\n");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-78, 79/\"all\" = full band, default: device-dependent)\n",
            "-c, --channels N");
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    app_print_gain_usage_line();
    app_print_debug_usage_line();
    app_print_other_usage_lines();
}

int record_main(int argc, char *argv[])
{
    argv[0] = (char *)"supertooth record";
    static const struct option long_opts[] = {
        {"channels", required_argument, NULL, 'c'},
        {"bottom", required_argument, NULL, 'b'},
        {"device", optional_argument, NULL, 'd'},
        {"gain", required_argument, NULL, 'g'},
        {"version", no_argument, NULL, 'V'},
        {"debug", no_argument, NULL, APP_OPT_DEBUG},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int list_devices = 0;
    const char *device_spec = NULL;
    app_device_spec_t spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int device_selected = 0;
    const char *gain_raw = NULL;
    radio_gain_spec_t gain_spec;
    unsigned int sample_rate;
    double tune_lo_mhz;
    uint64_t tune_lo_hz;
    app_record_config_t rcfg;
    int resolve_rc;
    int opt;

    spec_parsed.type = app_default_device_type();

    /* Default the channel count to what the radio can actually sustain
     * (HackRF -> 20; bladeRF -> 60; 80 Msps file replay -> 72). */
    g_num_bredr_channels =
        session_default_bredr_count(spec_parsed.type);

    while ((opt = getopt_long(argc, argv, "c:b:d::g:Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'c':
            if (parse_bredr_channel_count(optarg, &g_num_bredr_channels) != 0)
            {
                fprintf(stderr, "Invalid --channels value: %s (expected even 2-78, 79 or \"all\")\n",
                        optarg);
                record_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            g_channels_explicit = 1;
            break;
        case 'b':
            if (parse_bredr_bottom_channel(optarg, &g_bottom_bredr_channel) != 0)
            {
                fprintf(stderr, "Invalid --bottom-channel value: %s (expected 0-%u)\n",
                        optarg, BREDR_MAX_CHANNEL);
                record_print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            g_bottom_channel_explicit = 1;
            break;
        case 'd':
            consume_device_arg(optarg, argc, argv, &list_devices, &device_spec);
            break;
        case APP_OPT_DEBUG:
            g_debug = 1;
            break;
        case 'g':
            gain_raw = optarg;
            break;
        case 'V':
            printf("supertooth record %s\n", supertooth_get_version());
            return EXIT_SUCCESS;
        case 'h':
            record_print_usage(argv[0]);
            return EXIT_SUCCESS;
        default:
            record_print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    }
    if (optind != argc)
    {
        record_print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    resolve_rc = resolve_device(argv[0], list_devices, device_spec,
                                &spec_parsed, &device_selected,
                                record_print_usage);
    if (resolve_rc > 0)
        return EXIT_SUCCESS;
    if (resolve_rc < 0)
        return EXIT_FAILURE;

    if (resolve_bredr_window(argv[0], device_selected, spec_parsed.type,
                             0) != 0)
        return EXIT_FAILURE;

    compute_bredr_tune(g_num_bredr_channels, g_bottom_bredr_channel,
                       &sample_rate, &tune_lo_mhz, &tune_lo_hz);

    if (device_selected &&
        spec_parsed.type == RADIO_DEVICE_FILE)
    {
        fprintf(stderr, "record cannot be used with file replay input.\n");
        return EXIT_FAILURE;
    }

    if (resolve_gain(argv[0], device_selected, spec_parsed.type,
                     gain_raw, &gain_spec) != 0)
        return EXIT_FAILURE;

    rcfg.device_type = spec_parsed.type;
    rcfg.device_id = device_selected ? spec_parsed.id : NULL;
    rcfg.lo_freq_hz = (uint32_t)tune_lo_hz;
    rcfg.sample_rate_hz = sample_rate;
    rcfg.gain = gain_spec;
    rcfg.debug = g_debug;
    return app_record_run(&rcfg) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
