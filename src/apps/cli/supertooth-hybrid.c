#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <inttypes.h>

#include "app_common.h"
#include "app_device_view.h"
#include "app_summary_view.h"
#include "version.h"
#include "ble_display.h"
#include "bredr_display.h"
#include "ble_bitstream_decoder.h"
#include "session.h"
#include "bredr_bitstream_decoder.h"

#define BREDR_MAX_CHANNEL 79u

static unsigned long g_packet_count = 0;
static int g_debug = 0;
static int g_enforce_crc = 1;   /* drop BLE frames whose CRC fails; default on */
static unsigned int g_num_bredr_channels = BREDR_SESSION_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;
/* Maximum access-code bit errors accepted by the BR/EDR bitstream decoder.
 * Defaults to 0 (strict, byte-perfect access-code match). */
static unsigned int g_ac_errors = 0u;
static session_protocol_ref_t g_tune_ref = SESSION_REF_BREDR;
static session_t *g_session = NULL;
static app_device_view_t *g_device_view = NULL;
static const app_output_mode_option_t s_output_modes[] = {
    {APP_OUTPUT_MODE_FULL, "full"},
    {APP_OUTPUT_MODE_SUMMARY, "summary"},
    {APP_OUTPUT_MODE_DEVICES, "devices"},
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
    app_summary_view_print_ble(packet_no, event);
}

static void print_bredr_packet_full(unsigned long packet_no,
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
    printf("RSSI         : %.2f dBr\n\n", meta->rssi_dbr);
    bredr_print_packet_details(frame, pnet, meta);
    printf("--------------------------------------------------\n");
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                        const bredr_event_t *event,
                                        const bredr_piconet_snapshot_t *pnet)
{
    app_summary_view_print_bredr(packet_no, event, pnet);
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

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary|devices] [-c|--channels N] [-b|--bottom-channel CH] "
            "[--tune-ref bredr|ble] [-d|--device [<type>:<id>]] [--ac-errors N] [--debug] [--enforce-crc on|off]\n",
            argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            BREDR_SESSION_MAX_CHANNELS, g_num_bredr_channels);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    fprintf(stderr, "  %-30s Max access-code bit errors (default: 0, strict)\n", "--ac-errors N");
    fprintf(stderr, "  %-30s Which protocol's channel window sets the tuning grid (default: bredr)\n",
            "--tune-ref bredr|ble");
    app_print_device_usage_line();
    fprintf(stderr, "  %-30s Print version and exit\n", "-V, --version");
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "--debug");
    fprintf(stderr, "  %-30s Drop BLE frames whose CRC fails (default: on)\n",
            "--enforce-crc on|off");
}

static void handle_hybrid_bredr_packet(const bredr_event_t *event,
                                         const bredr_piconet_snapshot_t *pnet,
                                         void *user)
{
    (void)user;
    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;
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

    if (g_output_mode == APP_OUTPUT_MODE_DEVICES)
        return;

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
        {"tune-ref", required_argument, NULL, 'r'},
        {"device", optional_argument, NULL, 'd'},
        {"ac-errors", required_argument, NULL, APP_OPT_AC_ERRORS},
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

    /* Default the BR/EDR channel count to what the radio can sustain. BR/EDR
     * reference: max sample rate / 1 MHz per channel. BLE reference: each
     * "BR/EDR channel" maps to 2 MHz of LE span, so / 2 MHz. For a HackRF
     * (~20 MHz ceiling) this is 20 (BR/EDR ref) or 10 (BLE ref) channels. */
    {
        uint32_t max_rate = 0u;
        if (radio_get_max_sample_rate_for_type(g_device_spec_parsed.type,
                                               &max_rate) == 0 &&
            max_rate >= 1000000u)
        {
            unsigned int divisor = (g_tune_ref == SESSION_REF_BLE) ? 2000000u : 1000000u;
            g_num_bredr_channels = max_rate / divisor;
        }
    }

    int opt;
    while ((opt = getopt_long(argc, argv, "v:c:b:r:d::Vh", long_opts, NULL)) != -1)
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
                        optarg, BREDR_SESSION_MAX_CHANNELS);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'r':
            if (strcmp(optarg, "ble") == 0)
                g_tune_ref = SESSION_REF_BLE;
            else if (strcmp(optarg, "bredr") == 0)
                g_tune_ref = SESSION_REF_BREDR;
            else
            {
                fprintf(stderr, "Invalid --tune-ref value: %s (expected bredr or ble)\n", optarg);
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
            printf("supertooth-hybrid %s\n", supertooth_get_version());
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

    unsigned int channel_count, bottom_channel;
    if (g_tune_ref == SESSION_REF_BLE)
    {
        /* Translate the BR/EDR-style CLI options into an LE RF window for the
         * BLE reference. Each BR/EDR "channel" of the span is 2 MHz, so the LE
         * window is that many LE RF channels wide. */
        channel_count  = g_num_bredr_channels * 2u;
        bottom_channel = g_bottom_bredr_channel * 2u;
    }
    else
    {
        channel_count  = g_num_bredr_channels;
        bottom_channel = g_bottom_bredr_channel;
    }

    unsigned int sample_rate =
        (g_num_bredr_channels == 2u) ? 4000000u : g_num_bredr_channels * 1000000u;
    double lo_mhz = 2402.0 + g_bottom_bredr_channel + (g_num_bredr_channels - 1u) / 2.0;

    uint8_t ble_adv[3] = {0u, 0u, 0u};
    unsigned int ble_count =
        ble_channels_in_window((uint64_t)(lo_mhz * 1e6), sample_rate, ble_adv);

    printf("Supertooth Hybrid (tune-ref: %s)\n",
           g_tune_ref == SESSION_REF_BLE ? "BLE" : "BR/EDR");
    printf("  BR/EDR ch%u-%u + BLE fan-out (up to %u BLE channels in window)\n",
           g_bottom_bredr_channel,
           g_bottom_bredr_channel + g_num_bredr_channels - 1u, ble_count);
    printf("  LO: %.1f MHz, %u MHz bandwidth\n", lo_mhz, sample_rate / 1000000u);
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

    session_bredr_config_t bredr_cfg = { 0 };
    session_enable_bredr(g_session, &bredr_cfg, handle_hybrid_bredr_packet, NULL);
    session_ble_config_t ble_cfg = { .enforce_crc = g_enforce_crc ? 1u : 0u };
    session_enable_ble(g_session, &ble_cfg, handle_hybrid_ble_packet, NULL);

    if (session_tune(g_session, g_tune_ref, bottom_channel, channel_count) != 0)
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
    printf("Receiving... Press Ctrl+C to stop.\n");

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
    printf("  Output mode    : %s\n",
           app_output_mode_name(g_output_mode, s_output_modes,
                                sizeof(s_output_modes) / sizeof(s_output_modes[0])));
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Enforce CRC    : %s\n", g_enforce_crc ? "on" : "off");
    if (g_debug)
    {
        printf("\n=== Debug Summary ===\n");
        session_drop_breakdown_t drops;
        session_dropped_blocks_breakdown(g_session, &drops);
        app_print_drop_breakdown(&drops);
        unsigned long emitted = 0ul, confirmed = 0ul;
        session_ble_frame_counts(g_session, &emitted, &confirmed);
        printf("  BLE frames emitted   : %lu\n", emitted);
        printf("  BLE frames confirmed : %lu\n", confirmed);
        printf("  BR/EDR frames emitted: %lu\n", session_bredr_frame_count(g_session));
    }

    session_destroy(g_session);
    free(g_session);
    g_session = NULL;
    return result == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
