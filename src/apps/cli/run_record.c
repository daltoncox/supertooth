#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <strings.h>

#include "app_common.h"
#include "app_record.h"
#include "cli_modes.h"
#include "session.h"
#include "version.h"

#define BREDR_MAX_CHANNEL 79u

static int g_debug = 0;
static unsigned int g_num_bredr_channels = BREDR_SESSION_MAX_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;
static int g_channels_explicit = 0;

static int parse_channel_count(const char *arg, unsigned int *out_channels)
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
            "Usage: %s [-c|--channels N] [-b|--bottom-channel CH] "
            "[-d|--device [<type>:<id>]] [-g|--gain SPEC] [--debug]\n",
            argv0);
    fprintf(stderr, "General Options:\n");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-78, 79/\"all\" = full band, default: device-dependent)\n",
            "-c, --channels N");
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    app_print_device_usage_line();
    app_print_gain_usage_line();
    app_print_debug_usage_line();
    app_print_version_usage_line();
    app_print_help_usage_line();
}

int record_main(int argc, char *argv[])
{
    argv[0] = (char *)"supertooth record";
    static const struct option long_opts[] = {
        {"channels", required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"device", optional_argument, NULL, 'd'},
        {"gain", required_argument, NULL, 'g'},
        {"version", no_argument, NULL, 'V'},
        {"debug", no_argument, NULL, APP_OPT_DEBUG},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };

    int g_list_devices = 0;
    const char *g_device_spec = NULL;
    app_device_spec_t g_device_spec_parsed = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int g_device_selected = 0;
    const char *g_gain_raw = NULL;
    radio_gain_spec_t g_gain_spec;

    g_device_spec_parsed.type = app_default_device_type();

    /* Default the channel count to what the radio can actually sustain
     * (HackRF -> 20; bladeRF -> 60; 80 Msps file replay -> 72). */
    g_num_bredr_channels =
        session_default_bredr_count(g_device_spec_parsed.type);

    int opt;
    while ((opt = getopt_long(argc, argv, "c:b:d::g:Vh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'c':
            if (parse_channel_count(optarg, &g_num_bredr_channels) != 0)
            {
                fprintf(stderr, "Invalid --channels value: %s (expected even 2-78, 79 or \"all\")\n",
                        optarg);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            g_channels_explicit = 1;
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
        case 'g':
            g_gain_raw = optarg;
            break;
        case 'V':
            printf("supertooth record %s\n", supertooth_get_version());
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

    if (!g_device_selected)
    {
        if (app_require_default_device(argv[0]) != 0)
            return EXIT_FAILURE;
    }

    /* When the user did not pass -c, default to what the *selected* radio
     * can sustain (the pre-parse default assumed the build default). */
    if (!g_channels_explicit)
    {
        radio_device_type_t dtype =
            g_device_selected ? g_device_spec_parsed.type : app_default_device_type();
        g_num_bredr_channels = session_default_bredr_count(dtype);
    }

    if (g_bottom_channel_explicit)
    {
        /* "all" mode covers the whole band: bottom must be 0. */
        if (g_num_bredr_channels == BREDR_SESSION_MAX_CHANNELS &&
            g_bottom_bredr_channel != 0u)
        {
            fprintf(stderr,
                    "Invalid --bottom-channel %u for --channels all: "
                    "full-band capture always starts at 0.\n",
                    g_bottom_bredr_channel);
            return EXIT_FAILURE;
        }
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

    /* The tune must fit the selected radio's bandwidth ceiling (same
     * session-owned check the decode modes use). */
    {
        radio_device_type_t dtype =
            g_device_selected ? g_device_spec_parsed.type : app_default_device_type();
        switch (session_validate_layout(dtype, 0, 1, SESSION_REF_BREDR,
                                        g_bottom_bredr_channel,
                                        g_num_bredr_channels))
        {
        case SESSION_LAYOUT_OK:
            break;
        case SESSION_LAYOUT_RATE_EXCEEDED:
        {
            uint32_t max_rate =
                session_device_max_rate_hz(dtype);
            fprintf(stderr,
                    "Invalid --channels %u for %s (max %u MHz): choose <= %u channels.\n",
                    g_num_bredr_channels,
                    session_device_type_name(dtype),
                    max_rate / 1000000u, max_rate / 1000000u);
            return EXIT_FAILURE;
        }
        case SESSION_LAYOUT_NO_LANE_SPLIT:
            fprintf(stderr,
                    "Invalid --channels %u: no even <=20-channel lane split "
                    "(nearest supported: %u).\n",
                    g_num_bredr_channels,
                    session_snap_bredr_count(g_num_bredr_channels));
            return EXIT_FAILURE;
        case SESSION_LAYOUT_BAD_RANGE:
        default:
            fprintf(stderr,
                    "Invalid --channels %u (bottom %u): outside the BR/EDR band.\n",
                    g_num_bredr_channels, g_bottom_bredr_channel);
            return EXIT_FAILURE;
        }
    }

    /* "all" (c=79) captures the full 0..78 band at 80 Msps with the LO on
     * the exact band centre (2441 MHz, no half-channel premix). */
    unsigned int sample_rate = (g_num_bredr_channels == BREDR_SESSION_MAX_CHANNELS)
        ? 80000000u
        : ((g_num_bredr_channels == 2u) ? 4000000u : g_num_bredr_channels * 1000000u);
    double tune_lo_mhz = 2402.0 + (double)g_bottom_bredr_channel +
                         ((double)g_num_bredr_channels - 1.0) / 2.0;
    uint64_t tune_lo_hz = (uint64_t)(tune_lo_mhz * 1e6);

    if (g_device_selected &&
        g_device_spec_parsed.type == RADIO_DEVICE_FILE)
    {
        fprintf(stderr, "record cannot be used with file replay input.\n");
        return EXIT_FAILURE;
    }

    {
        radio_device_type_t rdtype =
            g_device_selected ? g_device_spec_parsed.type : app_default_device_type();
        if (app_resolve_gain_spec(argv[0], rdtype, g_gain_raw,
                                  &g_gain_spec) != 0)
            return EXIT_FAILURE;
    }

    app_record_config_t rcfg = {
        .device_type = g_device_spec_parsed.type,
        .device_id = g_device_selected ? g_device_spec_parsed.id : NULL,
        .lo_freq_hz = (uint32_t)tune_lo_hz,
        .sample_rate_hz = sample_rate,
        .gain = g_gain_spec,
        .debug = g_debug,
    };
    return app_record_run(&rcfg) == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
