#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <getopt.h>
#include "app_common.h"
#include "app_device_view.h"
#include "app_summary_view.h"
#include "version.h"
#include "bredr_display.h"
#include "radio_common.h"
#include "session.h"
#include "bredr_bitstream_decoder.h"

#define BREDR_MAX_CHANNEL 79u

/* -------------------------------------------------------------------------
 * Output modes
 * -------------------------------------------------------------------------*/

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                     const bredr_event_t *event,
                                     const bredr_piconet_snapshot_t *pnet);

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_SUMMARY;

/* Live device/piconet table view (started/stopped around session_run). */
static app_device_view_t *g_device_view = NULL;
static int g_debug = 0;
static int g_lap_filter_enabled = 0;
static uint32_t g_lap_filter = 0u;
/* Default BR/EDR channel count is derived from the radio's max sample rate
 * (see main()); this initial value is overwritten before use. */
static unsigned int g_num_bredr_channels = BREDR_SESSION_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;
/* Maximum access-code bit errors accepted by the BR/EDR bitstream decoder.
 * Defaults to 0 (strict, byte-perfect access-code match). */
static unsigned int g_ac_errors = 0u;

/* Counters. */
static unsigned long g_total_packets = 0UL;
static session_t *g_session = NULL;

/* -------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/

static int piconet_lap_cmp(const void *a, const void *b)
{
    const bredr_piconet_snapshot_t *pa = *(const bredr_piconet_snapshot_t *const *)a;
    const bredr_piconet_snapshot_t *pb = *(const bredr_piconet_snapshot_t *const *)b;
    uint32_t la = pa ? (pa->lap & 0xFFFFFFu) : 0u;
    uint32_t lb = pb ? (pb->lap & 0xFFFFFFu) : 0u;
    if (la < lb)
        return -1;
    if (la > lb)
        return 1;
    return 0;
}

static void print_session_piconets(void)
{
    size_t count = session_bredr_piconet_count(g_session);
    printf("=== BR/EDR Piconet Store (%zu piconet%s) ===\n",
           count, count == 1u ? "" : "s");
    for (size_t i = 0; i < count; i++)
    {
        bredr_piconet_snapshot_t snapshot;
        if (session_bredr_piconet_snapshot(g_session, i, &snapshot) == 0)
            bredr_print_piconet_snapshot(&snapshot);
    }
}

static unsigned int current_master_clock_mhz(void)
{
    return g_num_bredr_channels == 2u ? 4u : g_num_bredr_channels;
}

static void print_packet_full(unsigned long packet_no,
                               const bredr_event_t *event,
                               const bredr_piconet_snapshot_t *pnet)
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
    bredr_print_packet_details(frame, pnet, meta);

    printf("--------------------------------------------------\n");
}

static void print_packet_summary(unsigned long packet_no,
                                  const bredr_event_t *event,
                                  const bredr_piconet_snapshot_t *pnet)
{
    app_summary_view_print_bredr(packet_no, event, pnet);
}

static void print_packet_rssi(unsigned long packet_no,
                               const bredr_event_t *event,
                               const bredr_piconet_snapshot_t *pnet)
{
    (void)pnet;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    size_t count = session_bredr_piconet_count(g_session);
    const bredr_piconet_snapshot_t **ordered =
        (const bredr_piconet_snapshot_t **)malloc(sizeof(*ordered) * (count > 0u ? count : 1u));
    if (!ordered)
        return;

    size_t used = 0u;
    for (size_t i = 0; i < count; i++)
    {
        bredr_piconet_snapshot_t *cur =
            (bredr_piconet_snapshot_t *)malloc(sizeof(*cur));
        if (!cur)
            continue;
        if (session_bredr_piconet_snapshot(g_session, i, cur) == 0)
            ordered[used++] = cur;
        else
            free(cur);
    }
    qsort(ordered, used, sizeof(*ordered), piconet_lap_cmp);

    bredr_print_rssi_snapshot(packet_no, frame, meta,
                              (const bredr_piconet_snapshot_t *const *)ordered,
                              used, current_master_clock_mhz());
    for (size_t i = 0; i < used; i++)
        free((void *)ordered[i]);
    free(ordered);
}

static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
    {APP_OUTPUT_MODE_DEVICES, "devices"},
};

static packet_formatter_fn output_mode_formatter(app_output_mode_t mode)
{
    switch (mode)
    {
    case APP_OUTPUT_MODE_SUMMARY:
        return print_packet_summary;
    case APP_OUTPUT_MODE_RSSI:
        return print_packet_rssi;
    case APP_OUTPUT_MODE_FULL:
    default:
        return print_packet_full;
    }
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

static int parse_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 2ul || value > (unsigned long)BREDR_SESSION_MAX_CHANNELS ||
        (value & 1ul) != 0ul)
        return -1;

    *out_channels = (unsigned int)value;
    return 0;
}

static int parse_bottom_channel(const char *arg, unsigned int *out_bottom_channel)
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

/* Parse a "<type>:<id>" device spec (e.g. "hackrf:b25062dc22113a0b").
 * On success sets *out_type and *out_id (pointing into @p spec). */
static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary|devices] [-l|--lap LAP] "
            "[-c|--channels N] [-b|--bottom-channel CH] "
            "[-d|--device [<type>:<id>]] [--ac-errors N] [--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: summary)\n", "-v, --view");
    fprintf(stderr, "  %-30s Only track/report this LAP (e.g. 0x1FC475)\n", "-l, --lap LAP");
    fprintf(stderr, "  %-30s Max access-code bit errors (default: 0, strict)\n", "--ac-errors N");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            BREDR_SESSION_MAX_CHANNELS, g_num_bredr_channels);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    fprintf(stderr, "  %-30s Print version and exit\n", "-V, --version");
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "--debug");
}

static void handle_bredr_packet(const bredr_event_t *event,
                                  const bredr_piconet_snapshot_t *pnet,
                                  void *user)
{
    (void)user;
    /* In devices mode the live table thread owns all output; the per-packet
     * path is suppressed so the two never interleave. */
    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;
    app_output_lock();
    g_total_packets++;
    output_mode_formatter(g_output_mode)(g_total_packets, event, pnet);
    fflush(stdout);
    app_output_unlock();
}

/* -------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view",           required_argument, NULL, 'v'},
        {"lap",            required_argument, NULL, 'l'},
        {"channels",       required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"device",         optional_argument, NULL, 'd'},
        {"ac-errors",      required_argument, NULL, APP_OPT_AC_ERRORS},
        {"version",        no_argument,       NULL, 'V'},
        {"debug",          no_argument,       NULL, APP_OPT_DEBUG},
        {"help",           no_argument,       NULL, 'h'},
        {NULL,             0,                 NULL,  0 }
    };

    int g_list_devices = 0;
    const char *g_device_spec = NULL;
    app_device_spec_t g_device_spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int g_device_selected = 0;

    /* Default the channel count to what the radio can actually sustain:
     * max sample rate / 1 MHz per BR/EDR channel. For a HackRF (~20 MHz
     * ceiling) this is 20 channels instead of the full 79-channel band, so
     * the capture window fits within the radio's sample-rate limit. */
    {
        uint32_t max_rate = 0u;
        if (radio_get_max_sample_rate_for_type(g_device_spec_parsed.type,
                                               &max_rate) == 0 &&
            max_rate >= 1000000u)
            g_num_bredr_channels = max_rate / 1000000u;
    }

    int opt;
    while ((opt = getopt_long(argc, argv, "v:l:a:c:b:d::Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
            case 'v':
                if (app_parse_output_mode(optarg, s_output_modes,
                                          sizeof(s_output_modes) / sizeof(s_output_modes[0]),
                                          &g_output_mode) != 0)
                {
                    fprintf(stderr, "Invalid view mode: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'l':
                if (parse_lap_filter(optarg, &g_lap_filter) != 0)
                {
                    fprintf(stderr, "Invalid LAP: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_lap_filter_enabled = 1;
                break;
            case 'c':
        if (parse_channel_count(optarg, &g_num_bredr_channels) != 0)
        {
            fprintf(stderr, "Invalid --channels value: %s (expected even 2-%u)\n",
                    optarg, BREDR_SESSION_MAX_CHANNELS);
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
                break;
            case 'b':
                if (parse_bottom_channel(optarg, &g_bottom_bredr_channel) != 0)
                {
                    fprintf(stderr, "Invalid --bottom-channel value: %s (expected 0-%u)\n",
                            optarg, BREDR_MAX_CHANNEL);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_bottom_channel_explicit = 1;
                break;
            case 'd':
                g_list_devices = 1;
                g_device_spec = optarg;
                /* getopt's optional_argument doesn't attach a spaced arg
                 * to a short option, so consume the next token manually when
                 * it doesn't look like another option. */
                if (!g_device_spec && optind < argc && argv[optind][0] != '-')
                    g_device_spec = argv[optind++];
                break;
            case APP_OPT_DEBUG:
                g_debug = 1;
                break;
            case APP_OPT_AC_ERRORS:
            {
                char *end = NULL;
                unsigned long value = strtoul(optarg, &end, 0);
                if (end == optarg || *end != '\0' || value > 64ul)
                {
                    fprintf(stderr, "Invalid --ac-errors value: %s (expected 0-64)\n",
                            optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_ac_errors = (unsigned int)value;
                break;
            }
            case 'V':
                printf("supertooth-bredr %s\n", supertooth_get_version());
                return EXIT_SUCCESS;
            case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }
    if (optind != argc)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (g_list_devices)
    {
        if (g_device_spec)
        {
            if (app_parse_device_spec(g_device_spec, &g_device_spec_parsed) != 0)
            {
                fprintf(stderr, "Invalid device spec: %s (expected <type>:<id>)\n",
                        g_device_spec);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            if (app_validate_device_spec(argv[0], &g_device_spec_parsed) != 0)
                return EXIT_FAILURE;
            g_device_selected = 1;
        }
        else
        {
            return app_print_available_devices(argv[0]);
        }
    }

    if (g_bottom_channel_explicit)
    {
        unsigned int max_bottom_channel = BREDR_MAX_CHANNEL - (g_num_bredr_channels - 1u);
        if (g_bottom_bredr_channel > max_bottom_channel)
        {
            fprintf(stderr,
                    "Invalid --bottom-channel %u for --channels %u: out of BR/EDR band (0-%u).\n"
                    "For %u channels, the highest bottom channel would be %u.\n",
                    g_bottom_bredr_channel, g_num_bredr_channels, BREDR_MAX_CHANNEL,
                    g_num_bredr_channels, max_bottom_channel);
            return EXIT_FAILURE;
        }
    }

    const char *mode_name =
        app_output_mode_name(g_output_mode, s_output_modes,
                             sizeof(s_output_modes) / sizeof(s_output_modes[0]));
    unsigned int sample_rate = (g_num_bredr_channels == 2u) ? 4000000u : g_num_bredr_channels * 1000000u;
    unsigned int decim_factor = sample_rate / 2000000u;
    printf("Supertooth RX (BR/EDR)\n");
    printf("======================\n");
    printf("Sample rate : %u Msps\n", sample_rate / 1000000u);
    printf("Decimation  : /%u -> %u Msps demod input\n",
           decim_factor, 2u);
    printf("Channels    : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("View mode   : %s\n", mode_name);
    if (g_lap_filter_enabled)
        printf("LAP filter  : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("LAP filter  : (none)\n");
    if (g_device_selected)
        printf("Device      : %s:%s\n",
               radio_device_type_name(g_device_spec_parsed.type),
               g_device_spec_parsed.id);
    else
        printf("Device      : (default)\n");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");

    session_config_t config = {
        .device_type = g_device_spec_parsed.type,
        .device_id = g_device_selected ? g_device_spec_parsed.id : NULL,
        .debug = g_debug,
    };
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

    session_bredr_config_t bredr_cfg = {
        .lap_filter = g_lap_filter,
        .lap_filter_enabled = g_lap_filter_enabled,
    };
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

    printf("AC errors   : %u\n", g_ac_errors);

    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        app_summary_view_print_header();

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        g_device_view = app_device_view_start(g_session);

    int result = session_run(g_session);

    if (g_device_view)
    {
        app_device_view_stop(g_device_view);
        g_device_view = NULL;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", mode_name);
    if (g_lap_filter_enabled)
        printf("  LAP filter     : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("  LAP filter     : (none)\n");
    if (g_debug)
    {
        session_drop_breakdown_t drops;
        session_dropped_blocks_breakdown(g_session, &drops);
        app_print_drop_breakdown(&drops);
        printf("  BR/EDR frames emitted: %lu\n", session_bredr_frame_count(g_session));
    }
    printf("\n");
    print_session_piconets();
    session_destroy(g_session);
    free(g_session);
    g_session = NULL;

    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
