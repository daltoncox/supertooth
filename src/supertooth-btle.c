#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include "hackrf.h"
#include "ble_phy.h"

// BTLE Advertising Channel 37 constants
#define BTLE_CH37_INDEX 37         // BLE channel index for 2.402 GHz
#define BTLE_CH37_FREQ 2402e6      // Channel 37 frequency (2.402 GHz)
#define BTLE_CH38_INDEX 38         // BLE channel index for 2.426 GHz
#define BTLE_CH38_FREQ 2426e6      // Channel 38 frequency (2.426 GHz)
#define BTLE_CH39_INDEX 39         // BLE channel index for 2.480 GHz
#define BTLE_CH39_FREQ 2480e6      // Channel 39 frequency (2.480 GHz)
#define ADVERTISING_AA 0x8E89BED6U // BTLE advertising access address

#define BTLE_SYMBOL_RATE 1e6 // 1 Mbps BTLE symbol rate
#define SAMPLES_PER_SYMBOL 2 // 2 MHz / 1 Mbps = 2 samples/symbol

// HackRF parameters
#define LNA_GAIN 32        // LNA gain
#define VGA_GAIN 32        // VGA gain
#define BUFFER_SIZE 262144 // HackRF transfer buffer size (bytes)
#define SAMPLE_RATE 2e6    // 2 MHz sample rate

// BTLE demodulation objects
static cpfskdem demod;

// BLE PHY processor
static ble_channel_processor_t ble_proc;

// Processing buffers
static float complex raw[BUFFER_SIZE / 2]; // complex samples

// RSSI tracking state
static unsigned long long total_samples = 0;
static long long pkt_start_abs = -1;
static ble_status_t prev_status = BLE_SEARCHING;
static unsigned long g_packet_count = 0;
static int g_debug = 0;
static unsigned long g_truncated_callback_blocks = 0;
static volatile sig_atomic_t g_stop = 0;
static uint8_t g_ble_channel = BTLE_CH37_INDEX;
static uint64_t g_ble_freq_hz = (uint64_t)BTLE_CH37_FREQ;

typedef enum
{
    OUTPUT_MODE_FULL = 0,
    OUTPUT_MODE_SUMMARY = 1
} output_mode_t;

static output_mode_t g_output_mode = OUTPUT_MODE_FULL;

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

static const char *ble_pdu_type_name(uint8_t pdu_type)
{
    switch (pdu_type & 0x0Fu)
    {
    case 0x00u:
        return "ADV_IND";
    case 0x01u:
        return "ADV_DIRECT_IND";
    case 0x02u:
        return "ADV_NONCONN_IND";
    case 0x03u:
        return "SCAN_REQ";
    case 0x04u:
        return "SCAN_RSP";
    case 0x05u:
        return "CONNECT_IND";
    case 0x06u:
        return "ADV_SCAN_IND";
    default:
        return "RESERVED";
    }
}

static int ble_primary_addr(const ble_packet_t *pkt, const uint8_t **addr_out)
{
    if (!pkt || !addr_out)
        return 0;

    uint8_t pdu_type = pkt->pdu[0] & 0x0Fu;
    uint8_t pay_len = pkt->pdu[1];
    if (pay_len < 6u)
        return 0;

    switch (pdu_type)
    {
    case 0x00u: // ADV_IND
    case 0x01u: // ADV_DIRECT_IND
    case 0x02u: // ADV_NONCONN_IND
    case 0x04u: // SCAN_RSP
    case 0x06u: // ADV_SCAN_IND
    case 0x03u: // SCAN_REQ (ScanA)
    case 0x05u: // CONNECT_IND (InitA)
        *addr_out = &pkt->pdu[2];
        return 1;
    default:
        return 0;
    }
}

static void format_ble_addr(char out[18], const uint8_t *addr)
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

static int parse_output_mode(const char *arg, output_mode_t *out_mode)
{
    if (!arg || !out_mode)
        return -1;
    if (strcmp(arg, "full") == 0)
    {
        *out_mode = OUTPUT_MODE_FULL;
        return 0;
    }
    if (strcmp(arg, "summary") == 0 || strcmp(arg, "ubertooth") == 0)
    {
        *out_mode = OUTPUT_MODE_SUMMARY;
        return 0;
    }
    return -1;
}

static int parse_ble_channel(const char *arg, uint8_t *out_channel, uint64_t *out_freq_hz)
{
    if (!arg || !out_channel || !out_freq_hz)
        return -1;

    char *endptr = NULL;
    unsigned long ch = strtoul(arg, &endptr, 10);
    if (*arg == '\0' || !endptr || *endptr != '\0')
        return -1;

    switch ((uint8_t)ch)
    {
    case BTLE_CH37_INDEX:
        *out_channel = BTLE_CH37_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH37_FREQ;
        return 0;
    case BTLE_CH38_INDEX:
        *out_channel = BTLE_CH38_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH38_FREQ;
        return 0;
    case BTLE_CH39_INDEX:
        *out_channel = BTLE_CH39_INDEX;
        *out_freq_hz = (uint64_t)BTLE_CH39_FREQ;
        return 0;
    default:
        return -1;
    }
}

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-b|--ble-channel 37|38|39] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s BLE advertising channel (37, 38, or 39; default: 37)\n",
            "-b, --ble-channel 37|38|39");
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "-d, --debug");
}

static void print_ble_packet_full(unsigned long packet_no,
                                  const ble_packet_t *pkt,
                                  float rssi_dbr,
                                  unsigned long long sample_index)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           sample_index, (unsigned int)(SAMPLE_RATE / 1000000u));
    printf("Type         : BLE\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(g_ble_freq_hz / 1000000u), g_ble_channel);
    printf("RSSI         : %.2f dBr\n", rssi_dbr);
    printf("\n");
    ble_print_packet(pkt);
    printf("--------------------------------------------------\n");
}

static void print_ble_packet_summary(unsigned long packet_no,
                                     const ble_packet_t *pkt,
                                     float rssi_dbr)
{
    uint8_t pdu_type = pkt->pdu[0] & 0x0Fu;
    const char *pdu_name = ble_pdu_type_name(pdu_type);
    const uint8_t *addr = NULL;
    char addr_buf[18];

    if (ble_primary_addr(pkt, &addr))
        format_ble_addr(addr_buf, addr);
    else
        snprintf(addr_buf, sizeof(addr_buf), "--");

    printf("pkt=%-6lu type=BLE pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           pdu_name,
           g_ble_channel,
           addr_buf,
           pkt->pdu[1],
           ble_verify_crc(pkt) ? "PASS" : "FAIL",
           rssi_dbr);
}

// --- HackRF RX Callback ------------------------------------------------------

int btle_rx_cb(hackrf_transfer *transfer)
{
    if (g_stop)
        return -1;

    // HackRF provides interleaved I/Q samples as int8_t.
    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_iq_bytes = transfer->valid_length;
    unsigned int num_complex = num_iq_bytes / 2; // I/Q pair per complex sample

    if (num_complex > (BUFFER_SIZE / 2))
    {
        g_truncated_callback_blocks++;
        if (g_debug)
            fprintf(stderr, "[debug] callback buffer truncated to %u complex samples (total=%lu)\n",
                    (unsigned int)(BUFFER_SIZE / 2), g_truncated_callback_blocks);
        num_complex = BUFFER_SIZE / 2;
    }

    // Record the absolute sample index of this buffer's first sample.
    unsigned long long buf_start = total_samples;
    total_samples += num_complex;

    // Convert int8_t samples to complex float range [-1.0, 1.0].
    for (unsigned int i = 0; i < num_complex; i++)
    {
        float i_sample = samples[2 * i] / 128.0f;
        float q_sample = samples[2 * i + 1] / 128.0f;
        raw[i] = i_sample + q_sample * _Complex_I;
    }

    // Demodulate BTLE signal and push bits to the BLE PHY processor
    unsigned int num_bits = num_complex / SAMPLES_PER_SYMBOL;
    for (unsigned int s = 0; s < num_bits; s++)
    {
        unsigned int sample_index = s * SAMPLES_PER_SYMBOL;
        unsigned int sym = cpfskdem_demodulate(demod, &raw[sample_index]);
        uint8_t bit = (uint8_t)(sym & 0x01);

        // Push the bit to the BLE PHY processor
        ble_status_t status = ble_push_bit(&ble_proc, bit);

        // Record the sample index when preamble+AA is first matched.
        if (prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            pkt_start_abs = (long long)(buf_start + sample_index);
        prev_status = status;

        if (status == BLE_VALID_PACKET)
        {
            // Compute relative RSSI (dBr): max instantaneous power over packet samples.
            float max_power = 0.0f;
            if (pkt_start_abs >= 0)
            {
                long long rel_start = pkt_start_abs - (long long)buf_start;
                unsigned int i_start = (rel_start < 0) ? 0 : (unsigned int)rel_start;
                unsigned int i_end = (s + 1) * SAMPLES_PER_SYMBOL;
                for (unsigned int i = i_start; i < i_end && i < num_complex; i++)
                {
                    float power = crealf(raw[i]) * crealf(raw[i]) +
                                  cimagf(raw[i]) * cimagf(raw[i]);
                    if (power > max_power)
                        max_power = power;
                }
            }
            float rssi_dbr = (max_power > 0.0f) ? 10.0f * log10f(max_power) : 0.0f;

            // Retrieve and print the completed packet
            ble_packet_t pkt;
            if (ble_get_packet(&ble_proc, &pkt) == 0)
            {
                unsigned long long abs_sample_index = buf_start + sample_index;
                unsigned long packet_no = ++g_packet_count;
                if (g_output_mode == OUTPUT_MODE_SUMMARY)
                    print_ble_packet_summary(packet_no, &pkt, rssi_dbr);
                else
                    print_ble_packet_full(packet_no, &pkt, rssi_dbr, abs_sample_index);
                fflush(stdout);
            }
        }
    }

    return g_stop ? -1 : 0;
}

// --- Main --------------------------------------------------------------------

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"ble-channel", required_argument, NULL, 'b'},
        {"debug", no_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "v:b:dh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
        case 'v':
            if (parse_output_mode(optarg, &g_output_mode) != 0)
            {
                fprintf(stderr, "Invalid view mode: %s\n", optarg);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'b':
            if (parse_ble_channel(optarg, &g_ble_channel, &g_ble_freq_hz) != 0)
            {
                fprintf(stderr, "Invalid BLE channel: %s (expected 37, 38, or 39)\n", optarg);
                print_usage(argv[0]);
                return EXIT_FAILURE;
            }
            break;
        case 'd':
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

    printf("BTLE Advertising Packet Detector\n");
    printf("================================\n");
    printf("Channel     : %u (%.3f MHz)\n", g_ble_channel, (double)g_ble_freq_hz / 1e6);
    printf("View mode   : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");

    // Initialize BLE PHY processor for selected advertising channel
    ble_processor_init(&ble_proc, g_ble_channel);

    // Create BTLE GFSK demodulator (1 bit/symbol, h ~ 0.5, etc.).
    unsigned int bps = 1;                                         // bits/symbol
    float h = 0.5f;                                               // modulation index
    unsigned int k = SAMPLES_PER_SYMBOL;                          // samples/symbol
    unsigned int m = 3;                                           // filter delay (symbols)
    float BT = 0.5f;                                              // BT product
    demod = cpfskdem_create(bps, h, k, m, BT, LIQUID_CPFSK_GMSK); // generic CPFSK GMSK

    if (!demod)
    {
        fprintf(stderr, "Error: Failed to create cpfskdem object.\n");
        return EXIT_FAILURE;
    }

    signal(SIGINT, handle_sigint);

    // HackRF setup and start RX.
    int result;
    hackrf_device *device = NULL;

    result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    hackrf_config_t config = {
        .lo_freq_hz = g_ble_freq_hz,
        .sample_rate = SAMPLE_RATE,
        .lna_gain = LNA_GAIN,
        .vga_gain = VGA_GAIN,
    };

    result = hackrf_configure(device, &config);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, btle_rx_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    printf("Monitoring BTLE Channel %u (%.3f GHz) for advertising packets...\n",
           g_ble_channel, (double)g_ble_freq_hz / 1e9);
    printf("Press Ctrl+C to exit\n\n");

    while (!g_stop)
        sleep(1);
    hackrf_stop_rx(device);
    hackrf_disconnect(device);
    cpfskdem_destroy(demod);

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  BLE channel    : %u (%.3f MHz)\n", g_ble_channel, (double)g_ble_freq_hz / 1e6);
    printf("  Total samples  : %" PRIu64 "\n", total_samples);
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("\n=== Debug Summary ===\n");
    printf("  Truncated callback blocks : %lu\n", g_truncated_callback_blocks);

    return 0;
}
