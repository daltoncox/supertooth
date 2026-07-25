#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>
#include "app_common.h"
#include "version.h"
#include "ble_display.h"
#include "ble_bitstream_decoder.h"
#include "receiver_session.h"

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static int g_enforce_crc = 1;   /* drop BLE frames whose CRC fails; default on */
static uint8_t g_ble_channel = BLE_CH37_INDEX;
static unsigned int g_ble_rf = BLE_RF_ADV0_INDEX;   /* LE RF channel of g_ble_channel */
static receiver_session_t *g_session = NULL;

static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
};

static app_output_mode_t g_output_mode = APP_OUTPUT_MODE_FULL;

static int parse_ble_channel(const char *arg, uint8_t *out_channel, unsigned int *out_rf)
{
    if (!arg || !out_channel || !out_rf)
        return -1;

    char *endptr = NULL;
    unsigned long ch = strtoul(arg, &endptr, 10);
    if (*arg == '\0' || !endptr || *endptr != '\0')
        return -1;

    switch ((uint8_t)ch)
    {
    case BLE_CH37_INDEX:
        *out_channel = BLE_CH37_INDEX;
        *out_rf = BLE_RF_ADV0_INDEX;
        return 0;
    case BLE_CH38_INDEX:
        *out_channel = BLE_CH38_INDEX;
        *out_rf = BLE_RF_ADV1_INDEX;
        return 0;
    case BLE_CH39_INDEX:
        *out_channel = BLE_CH39_INDEX;
        *out_rf = BLE_RF_ADV2_INDEX;
        return 0;
    default:
        return -1;
    }
}

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary] [-b|--ble-channel 37|38|39] "
            "[-d|--device [<type>:<id>]] [--debug] "
            "[--enforce-crc on|off]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s BLE advertising channel (37, 38, or 39; default: 37)\n",
            "-b, --ble-channel 37|38|39");
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

    /* When CRC enforcement is on, drop frames whose CRC fails (or that fail
     * to decode) before emitting anything — no packet number is consumed and
     * no line is printed, mirroring the receiver going back to searching. */
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

// --- Main --------------------------------------------------------------------

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"ble-channel", required_argument, NULL, 'b'},
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
    while ((opt = getopt_long(argc, argv, "v:b:d::Vh", long_opts, NULL)) != -1)
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
        case 'b':
            if (parse_ble_channel(optarg, &g_ble_channel, &g_ble_rf) != 0)
            {
                fprintf(stderr, "Invalid BLE channel: %s (expected 37, 38, or 39)\n", optarg);
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

    printf("BLE Advertising Packet Detector\n");
    printf("================================\n");
    printf("Channel     : %u (%.3f MHz)\n", g_ble_channel,
           (double)ble_rf_channel_freq_hz(g_ble_rf) / 1e6);
    printf("LO          : %.1f MHz, 4 Msps (channelized)\n",
           (double)ble_rf_channel_freq_hz(g_ble_rf) / 1e6);
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
    app_install_sigint_handler(&g_session);

    /* Single-channel window on the selected advertising channel. The
     * session tunes a whole-MHz LO at the channel center (4 Msps for the
     * 2 MHz span) and channelizes down to 2 Msps. */
    receiver_ble_config_t config = {
        .bottom_le_channel = g_ble_rf,
        .le_channel_count = 1u,
        .device_type = g_device_spec_parsed.type,
        .device_id = g_device_selected ? g_device_spec_parsed.id : NULL,
        .debug = g_debug,
        .enforce_crc = g_enforce_crc,
    };
    receiver_ble_callbacks_t callbacks = {
        .on_packet = handle_ble_packet,
        .user = NULL,
    };
    g_session = receiver_session_create();
    if (!g_session)
    {
        fprintf(stderr, "Failed to create receiver session.\n");
        return EXIT_FAILURE;
    }

    printf("Monitoring BLE Channel %u (%.3f GHz) for advertising packets...\n",
           g_ble_channel, (double)ble_rf_channel_freq_hz(g_ble_rf) / 1e9);
    printf("Press Ctrl+C to exit\n\n");
        int result = receiver_session_run_ble(g_session, &config, &callbacks);
    receiver_session_destroy(g_session);
    g_session = NULL;
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
    printf("  BLE channel    : %u (%.3f MHz)\n", g_ble_channel, (double)ble_rf_channel_freq_hz(g_ble_rf) / 1e6);
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    printf("  Total packets  : %lu\n", g_packet_count);

    return 0;
}
