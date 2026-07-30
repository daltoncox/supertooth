#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>
#include <signal.h>
#include "session.h"
#include "app_common.h"
#include "version.h"
#include "ble_display.h"
#include "ble_bitstream_decoder.h"

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static int g_enforce_crc = 1;
static unsigned int g_num_le_channels = BLE_SESSION_MAX_CHANNELS;
static unsigned int g_bottom_le_channel = BLE_CH37_INDEX;
static session_t g_session;
static int g_session_initialized = 0;

static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
};

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_FULL;

static void handle_sigint(int sig)
{
    (void)sig;
    if (g_session_initialized)
        session_request_stop(&g_session);
}

static int parse_channel_count(const char *arg, unsigned int *out_channels)
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

static int parse_bottom_channel(const char *arg, unsigned int *out_bottom_channel)
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

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary] [-c|--channels N] [-b|--bottom-channel CH] "
            "[-d|--device [<type>:<id>]] [--debug] "
            "[--enforce-crc on|off]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of consecutive LE RF channels (1-%u, default: %u)\n",
            "-c, --channels N",
            BLE_SESSION_MAX_CHANNELS, BLE_SESSION_MAX_CHANNELS);
    fprintf(stderr, "  %-30s Bottom LE channel of the window (0-39, default: 37)\n",
            "-b, --bottom-channel CH");
    app_print_device_usage_line();
    fprintf(stderr, "  %-30s Print version and exit\n", "-V, --version");
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "--debug");
    fprintf(stderr, "  %-30s Drop frames whose BLE CRC fails (default: on)\n",
            "--enforce-crc on|off");
}

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

static void handle_ble_packet(const ble_event_t *event,
                               void *user)
{
    (void)user;

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
        {"version", no_argument, NULL, 'V'},
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
    while ((opt = getopt_long(argc, argv, "v:c:b:d::Vh", long_opts, NULL)) != -1)
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
            if (parse_channel_count(optarg, &g_num_le_channels) != 0)
            {
                fprintf(stderr, "Invalid --channels value: %s (expected 1-%u)\n",
                        optarg, BLE_SESSION_MAX_CHANNELS);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'b':
            if (parse_bottom_channel(optarg, &g_bottom_le_channel) != 0)
            {
                fprintf(stderr, "Invalid --bottom-channel value: %s (expected 0-39)\n",
                        optarg);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
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
        case 'V':
            printf("supertooth-ble %s\n", supertooth_get_version());
            return EXIT_SUCCESS;
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

    unsigned int bottom_rf = ble_rf_for_channel_number(g_bottom_le_channel);
    if (bottom_rf >= BLE_RF_CHANNEL_COUNT ||
        bottom_rf + g_num_le_channels > BLE_RF_CHANNEL_COUNT)
    {
        fprintf(stderr,
                "Invalid window: LE ch%u with %u channel%s exceeds the LE band.\n",
                g_bottom_le_channel, g_num_le_channels,
                g_num_le_channels == 1u ? "" : "s");
        return EXIT_FAILURE;
    }

    unsigned int span_mhz = 2u * g_num_le_channels;
    unsigned int rate_mhz = (span_mhz == 2u) ? 4u : span_mhz;
    double lo_mhz = 2401.0 + 2.0 * (double)bottom_rf + (double)g_num_le_channels;

    printf("BLE Packet Detector\n");
    printf("=====================\n");
    printf("Window      : %u LE channel%s from ch%u (RF %u-%u):",
           g_num_le_channels, g_num_le_channels == 1u ? "" : "s",
           g_bottom_le_channel, bottom_rf, bottom_rf + g_num_le_channels - 1u);
    for (unsigned int rf = bottom_rf; rf < bottom_rf + g_num_le_channels; rf++)
        printf(" %u", (unsigned int)ble_channel_number_for_rf(rf));
    printf("\n");
    printf("Advertising :");
    {
        unsigned int adv_found = 0u;
        for (unsigned int rf = bottom_rf; rf < bottom_rf + g_num_le_channels; rf++)
            if (ble_rf_is_advertising(rf))
                printf(" %u", (unsigned int)ble_channel_number_for_rf(rf)), adv_found++;
        if (!adv_found)
            printf(" (none in window)");
    }
    printf("\n");
    printf("LO          : %.1f MHz, %u Msps (channelized)\n", lo_mhz, rate_mhz);
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
    signal(SIGINT, handle_sigint);

    session_config_t config = {
        .device_type = g_device_spec_parsed.type,
        .device_id = g_device_selected ? g_device_spec_parsed.id : NULL,
        .debug = g_debug,
    };

    if (session_init(&g_session, &config) != 0)
    {
        fprintf(stderr, "Failed to initialize session.\n");
        return EXIT_FAILURE;
    }
    g_session_initialized = 1;

    session_ble_config_t ble_cfg = { .enforce_crc = g_enforce_crc ? 1u : 0u };
    session_enable_ble(&g_session, &ble_cfg, handle_ble_packet, NULL);

    if (session_tune(&g_session, SESSION_REF_BLE, bottom_rf, g_num_le_channels) != 0)
    {
        fprintf(stderr, "Failed to tune session.\n");
        session_destroy(&g_session);
        g_session_initialized = 0;
        return EXIT_FAILURE;
    }

    printf("Monitoring %u LE channel%s from ch%u...\n",
           g_num_le_channels, g_num_le_channels == 1u ? "" : "s",
           g_bottom_le_channel);
    printf("Press Ctrl+C to exit\n\n");

    int result = session_run(&g_session);
    session_destroy(&g_session);
    g_session_initialized = 0;

    if (result != 0)
    {
        fprintf(stderr, "BLE receiver failed.\n");
        return EXIT_FAILURE;
    }

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Window         : %u LE channel%s from ch%u (RF %u-%u)\n",
           g_num_le_channels, g_num_le_channels == 1u ? "" : "s",
           g_bottom_le_channel, bottom_rf, bottom_rf + g_num_le_channels - 1u);
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    printf("  Total packets  : %lu\n", g_packet_count);

    return 0;
}