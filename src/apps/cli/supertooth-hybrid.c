#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>

#include "app_common.h"
#include "ble_display.h"
#include "bredr_display.h"
#include "ble_bitstream_decoder.h"
#include "receiver_session.h"

#define BREDR_MAX_CHANNEL 79u

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static int g_enforce_crc = 1;   /* drop BLE frames whose CRC fails; default on */
static unsigned int g_num_bredr_channels = RECEIVER_BREDR_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;
static receiver_session_t *g_session = NULL;
static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
};

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_FULL;

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
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    if (ble_decode_frame(&event->frame, meta->channel_index, &packet) == 0)
        ble_print_packet(&packet);
    else
        printf("[BLE Decode Error]\n");
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const ble_event_t *event)
{
    ble_packet_t packet;
    if (ble_decode_frame(&event->frame, event->meta.channel_index, &packet) == 0)
    {
        ble_print_packet_summary_line(packet_no, &packet, &event->meta);
        return;
    }

    printf("pkt=%-6lu type=BLE pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           "DECODE_FAIL",
           event->meta.channel_index,
           "--",
           0u,
           "FAIL",
           event->meta.rssi_dbr);
}

static void print_bredr_packet_full(unsigned long packet_no,
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
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    bredr_print_packet_details(frame, pnet, meta);
    printf("--------------------------------------------------\n");
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                       const bredr_event_t *event,
                                       const receiver_bredr_piconet_snapshot_t *pnet)
{
    bredr_print_packet_summary_line(packet_no, &event->frame, pnet, &event->meta);
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

/* The single BLE advertising channel whose center lies inside the capture
 * window (at most one fits a <=20 MHz window), or 0 when none does — in
 * which case the hybrid BLE worker stays idle. */
static uint8_t ble_adv_channel_for_window(unsigned int bottom, unsigned int count)
{
    static const struct { uint8_t channel; uint32_t freq_hz; } adv[] = {
        { BLE_CH37_INDEX, BLE_CH37_FREQ_HZ },
        { BLE_CH38_INDEX, BLE_CH38_FREQ_HZ },
        { BLE_CH39_INDEX, BLE_CH39_FREQ_HZ },
    };
    double left_hz = (2401.5 + bottom) * 1e6;
    double right_hz = left_hz + count * 1e6;
    for (unsigned int i = 0; i < sizeof(adv) / sizeof(adv[0]); i++)
    {
        if ((double)adv[i].freq_hz >= left_hz && (double)adv[i].freq_hz <= right_hz)
            return adv[i].channel;
    }
    return 0u;
}

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary] [-c|--channels N] [-b|--bottom-channel CH] "
            "[-d|--device [<type>:<id>]] [--debug] [--enforce-crc on|off]\n",
            argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            RECEIVER_BREDR_MAX_CHANNELS, RECEIVER_BREDR_MAX_CHANNELS);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "--debug");
    fprintf(stderr, "  %-30s Drop BLE frames whose CRC fails (default: on)\n",
            "--enforce-crc on|off");
}

static void handle_hybrid_bredr_packet(const bredr_event_t *event,
                                       const receiver_bredr_piconet_snapshot_t *pnet,
                                       void *user)
{
    (void)user;
    app_output_lock();
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_bredr_packet_summary(packet_no, event, pnet);
    else
        print_bredr_packet_full(packet_no, event, pnet);
    fflush(stdout);
    app_output_unlock();
}

static void handle_hybrid_ble_packet(const ble_event_t *event,
                                      void *user)
{
    (void)user;

    /* When CRC enforcement is on, drop BLE frames whose CRC fails (or that
     * fail to decode) before emitting anything. BR/EDR frames are unaffected
     * and still flow through handle_hybrid_bredr_packet. */
    if (g_enforce_crc)
    {
        ble_packet_t pkt;
        if (ble_decode_frame(&event->frame, event->meta.channel_index, &pkt) != 0 ||
            !ble_verify_crc(&pkt))
            return;
    }

    app_output_lock();
    unsigned long packet_no = ++g_packet_count;
    if (g_output_mode == APP_OUTPUT_MODE_SUMMARY)
        print_ble_packet_summary(packet_no, event);
    else
        print_ble_packet_full(packet_no, event);
    fflush(stdout);
    app_output_unlock();
}

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"channels", required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"device", optional_argument, NULL, 'd'},
        {"debug", no_argument, NULL, APP_OPT_DEBUG},
        {"enforce-crc", required_argument, NULL, APP_OPT_ENFORCE_CRC},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int g_list_devices = 0;
    const char *g_device_spec = NULL;
    app_device_spec_t g_device_spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int g_device_selected = 0;
    int opt;
    while ((opt = getopt_long(argc, argv, "v:c:b:d::h", long_opts, NULL)) != -1)
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
            g_device_spec = optarg;
            if (!g_device_spec && optind < argc && argv[optind][0] != '-')
                g_device_spec = argv[optind++];
            break;
        case APP_OPT_DEBUG:
            g_debug = 1;
            break;
        case APP_OPT_ENFORCE_CRC:
        {
            const char *v = optarg ? optarg : "on";
            if (strcmp(v, "on") == 0 || strcmp(v, "1") == 0)
                g_enforce_crc = 1;
            else if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0)
                g_enforce_crc = 0;
            else
            {
                fprintf(stderr, "Invalid --enforce-crc value: %s (expected on or off)\n", v);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        }
        case 'h':
            print_usage(argv[0]);
            return EXIT_SUCCESS;
        default:
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
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

    uint8_t ble_channel =
        ble_adv_channel_for_window(g_bottom_bredr_channel, g_num_bredr_channels);
    unsigned int sample_rate =
        (g_num_bredr_channels == 2u) ? 4000000u : g_num_bredr_channels * 1000000u;
    double lo_mhz = 2402.0 + g_bottom_bredr_channel + (g_num_bredr_channels - 1u) / 2.0;

    printf("Supertooth Hybrid: BR/EDR ch%u-%u", g_bottom_bredr_channel,
           g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    if (ble_channel)
        printf(" + BLE ch%u\n", ble_channel);
    else
        printf(" (no BLE advertising channel in window — BLE idle)\n");
    printf("LO: %.1f MHz, %u BR/EDR channels%s, %u MHz bandwidth\n",
           lo_mhz, g_num_bredr_channels,
           ble_channel ? " + 1 BLE channel" : "",
           sample_rate / 1000000u);
    printf("View mode   : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    if (g_device_selected)
        printf("Device      : %s:%s\n",
               radio_device_type_name(g_device_spec_parsed.type),
               g_device_spec_parsed.id);
    else
        printf("Device      : (default)\n");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Enforce CRC : %s\n", g_enforce_crc ? "on" : "off");
    printf("Block pool  : %u blocks, per-channel queue: %u\n",
           RECEIVER_BREDR_BLOCK_POOL_SIZE, RECEIVER_BREDR_CHANNEL_RING_SIZE);

    app_install_sigint_handler(&g_session);

    receiver_hybrid_config_t config = {
        .channel_count = g_num_bredr_channels,
        .bottom_channel = g_bottom_bredr_channel,
        .le_grid = RECEIVER_BREDR_GRID_BREDR,
        .ble_channel = ble_channel,
        .device_type = g_device_spec_parsed.type,
        .device_id = g_device_selected ? g_device_spec_parsed.id : NULL,
        .debug = g_debug,
        .enforce_crc = g_enforce_crc,
    };
    receiver_hybrid_callbacks_t callbacks = {
        .on_bredr_packet = handle_hybrid_bredr_packet,
        .on_ble_packet = handle_hybrid_ble_packet,
        .user = NULL,
    };
    receiver_hybrid_stats_t stats;
    g_session = receiver_session_create();
    if (!g_session)
        return EXIT_FAILURE;

    printf("Receiving... Press Ctrl+C to stop.\n");
    int result = receiver_session_run_hybrid(g_session, &config, &callbacks, &stats);

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    printf("  Total packets  : %lu\n", stats.total_packets);
    if (g_debug)
    {
        printf("\n=== Debug Summary ===\n");
        printf("  Dropped blocks : %lu\n",
               receiver_session_dispatcher_dropped_blocks(g_session));
    }

    receiver_session_destroy(g_session);
    g_session = NULL;
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
