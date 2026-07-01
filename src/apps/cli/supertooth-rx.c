#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <getopt.h>
#include "app_common.h"
#include "bredr_display.h"
#include "radio_common.h"
#include "receiver_session.h"

/* Long-only options (no single-character short form) use val codes that
 * are outside the ASCII range used by the short-option string. */
enum
{
    OPT_DEBUG = 0x100,
};

#define BREDR_MAX_CHANNEL 79u

/* -------------------------------------------------------------------------
 * Output modes
 * -------------------------------------------------------------------------*/

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                    const bredr_event_t *event,
                                    const receiver_bredr_piconet_snapshot_t *pnet);

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_FULL;
static int g_debug = 0;
static int g_lap_filter_enabled = 0;
static uint32_t g_lap_filter = 0u;
static unsigned int g_rssi_averaging_window = RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW;
static unsigned int g_num_bredr_channels = RECEIVER_BREDR_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;

/* Counters. */
static unsigned long g_total_packets = 0UL;
static receiver_session_t *g_session = NULL;

/* -------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/

static int piconet_lap_cmp(const void *a, const void *b)
{
    const receiver_bredr_piconet_snapshot_t *pa = *(const receiver_bredr_piconet_snapshot_t *const *)a;
    const receiver_bredr_piconet_snapshot_t *pb = *(const receiver_bredr_piconet_snapshot_t *const *)b;
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
    size_t count = receiver_session_bredr_piconet_count(g_session);
    printf("=== BR/EDR Piconet Store (%zu piconet%s) ===\n",
           count, count == 1u ? "" : "s");
    for (size_t i = 0; i < count; i++)
    {
        receiver_bredr_piconet_snapshot_t snapshot;
        if (receiver_session_bredr_piconet_snapshot(g_session, i, &snapshot) == 0)
            bredr_print_piconet_snapshot(&snapshot);
    }
}

static unsigned int current_master_clock_mhz(void)
{
    return g_num_bredr_channels == 2u ? 4u : g_num_bredr_channels;
}

static void print_packet_full(unsigned long packet_no,
                              const bredr_event_t *event,
                              const receiver_bredr_piconet_snapshot_t *pnet)
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
                                 const receiver_bredr_piconet_snapshot_t *pnet)
{
    bredr_print_packet_summary_line(packet_no, &event->frame, pnet, &event->meta);
}

static void print_packet_rssi(unsigned long packet_no,
                              const bredr_event_t *event,
                              const receiver_bredr_piconet_snapshot_t *pnet)
{
    (void)pnet;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    size_t count = receiver_session_bredr_piconet_count(g_session);
    const receiver_bredr_piconet_snapshot_t **ordered =
        (const receiver_bredr_piconet_snapshot_t **)malloc(sizeof(*ordered) * (count > 0u ? count : 1u));
    if (!ordered)
        return;

    size_t used = 0u;
    for (size_t i = 0; i < count; i++)
    {
        receiver_bredr_piconet_snapshot_t *cur =
            (receiver_bredr_piconet_snapshot_t *)malloc(sizeof(*cur));
        if (!cur)
            continue;
        if (receiver_session_bredr_piconet_snapshot(g_session, i, cur) == 0)
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
    {APP_OUTPUT_MODE_RSSI, "rssi"},
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

static int parse_rssi_averaging(const char *arg, unsigned int *out_window)
{
    if (!arg || !out_window)
        return -1;
    if (strcmp(arg, "none") == 0)
    {
        *out_window = 0u;
        return 0;
    }

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 1000000ul)
        return -1;

    *out_window = (unsigned int)value;
    return 0;
}

static int parse_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 2ul || value > (unsigned long)RECEIVER_BREDR_MAX_CHANNELS ||
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

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary|rssi] [-l|--lap LAP] "
            "[--rssi-averaging N|none] [-c|--channels N] [-b|--bottom-channel CH] "
            "[-d|--device] [--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Only track/report this LAP (e.g. 0x1FC475)\n", "-l, --lap LAP");
    fprintf(stderr, "  %-30s EMA window for piconet RSSI (default: 16; 0/none disables)\n",
            "--rssi-averaging N|none");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            RECEIVER_BREDR_MAX_CHANNELS, RECEIVER_BREDR_MAX_CHANNELS);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    fprintf(stderr, "  %-30s List available radio devices and exit\n", "-d, --device");
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "--debug");
}

/* Enumerate every known radio device type, print one "<type>:<id>" line per
 * discovered device, then return. Used by -d/--device. */
static int print_available_devices(void)
{
    int enumerated = 0;

    printf("Found Available Devices:\n");
    for (int type = 0; type < (int)RADIO_DEVICE_TYPE_COUNT; type++)
    {
        const char *type_name = radio_device_type_name((radio_device_type_t)type);
        if (!type_name)
            continue;

        char **identifiers = NULL;
        size_t count = 0u;
        int result = radio_list_devices((radio_device_type_t)type,
                                        &identifiers, &count);
        if (result != RADIO_SUCCESS || count == 0u)
        {
            radio_free_device_list(&identifiers, count);
            continue;
        }

        for (size_t i = 0u; i < count; i++)
            printf("%s:%s\n", type_name, identifiers[i] ? identifiers[i] : "");

        radio_free_device_list(&identifiers, count);
        enumerated += (int)count;
    }

    if (enumerated == 0)
        printf("(no devices found)\n");

    return EXIT_SUCCESS;
}

static void handle_bredr_packet(const bredr_event_t *event,
                                const receiver_bredr_piconet_snapshot_t *pnet,
                                void *user)
{
    (void)user;
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
        {"rssi-averaging", required_argument, NULL, 'a'},
        {"channels",       required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"device",         no_argument,       NULL, 'd'},
        {"debug",          no_argument,       NULL, OPT_DEBUG},
        {"help",           no_argument,       NULL, 'h'},
        {NULL,             0,                 NULL,  0 }
    };

    int g_list_devices = 0;
    int opt;
    while ((opt = getopt_long(argc, argv, "v:l:a:c:b:dh", long_opts, NULL)) != -1)
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
            case 'a':
                if (parse_rssi_averaging(optarg, &g_rssi_averaging_window) != 0)
                {
                    fprintf(stderr, "Invalid --rssi-averaging value: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'c':
                if (parse_channel_count(optarg, &g_num_bredr_channels) != 0)
                {
                    fprintf(stderr, "Invalid --channels value: %s (expected even 2-%u)\n",
                            optarg, RECEIVER_BREDR_MAX_CHANNELS);
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
                break;
            case OPT_DEBUG:
                g_debug = 1;
                break;
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
        return print_available_devices();

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
    double lo_mhz = ((RECEIVER_BREDR_CHANNEL_0_FREQ + g_bottom_bredr_channel * 1000000.0) -
                    (-(g_num_bredr_channels / 2.0 - 0.5) * 1000000.0)) / 1e6;
    printf("Supertooth RX (BR/EDR)\n");
    printf("======================\n");
    printf("LO          : %.3f MHz\n", lo_mhz);
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
    if (g_rssi_averaging_window == 0u)
        printf("RSSI EMA    : disabled\n");
    else
        printf("RSSI EMA    : window=%u\n", g_rssi_averaging_window);
    printf("Block pool  : %u blocks, per-channel queue: %u\n",
           RECEIVER_BREDR_BLOCK_POOL_SIZE, RECEIVER_BREDR_CHANNEL_RING_SIZE);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");
    app_install_sigint_handler(&g_session);
    receiver_bredr_config_t config = {
        .channel_count = g_num_bredr_channels,
        .bottom_channel = g_bottom_bredr_channel,
        .rssi_averaging_window = g_rssi_averaging_window,
        .lap_filter = g_lap_filter,
        .lap_filter_enabled = g_lap_filter_enabled,
        .debug = g_debug,
    };
    receiver_bredr_callbacks_t callbacks = {
        .on_packet = handle_bredr_packet,
        .user = NULL,
    };
    receiver_bredr_stats_t stats;
    g_session = receiver_session_create();
    if (!g_session)
        return EXIT_FAILURE;
    int result = receiver_session_run_bredr(g_session, &config, &callbacks, &stats);

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", mode_name);
    if (g_lap_filter_enabled)
        printf("  LAP filter     : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("  LAP filter     : (none)\n");
    if (g_rssi_averaging_window == 0u)
        printf("  RSSI EMA       : disabled\n");
    else
    printf("  RSSI EMA       : window=%u\n", g_rssi_averaging_window);
    printf("  Total bits     : %" PRIu64 "\n", stats.total_bits);
    printf("  Header packets : %lu\n", stats.header_packets);
    printf("  ID packets     : %lu\n", stats.id_packets);
    if (g_debug)
        printf("  Dropped blocks : %lu\n",
               receiver_session_dispatcher_dropped_blocks(g_session));
    printf("\n");
    print_session_piconets();
    receiver_session_destroy(g_session);
    g_session = NULL;

    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
