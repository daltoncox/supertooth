/**
 * @file bredr-scanner.c
 * @brief Real-time BR/EDR scanner with piconet tracking and UAP recovery.
 *
 * Signal path
 * -----------
 *  HackRF (20 Msps, int8 I/Q)
 *    → float complex
 *    → NCO mix-down  (channel → DC)
 *    → FIR decimate ×10  (20 Msps → 2 Msps)
 *    → GMSK demodulate   (k=2 → 1 Msym/s)
 *    → bredr_push_bit()
 *    → bredr_piconet_store_add_packet()
 *
 * For every decoded packet:
 *  - rx_clk_1600 is computed from the sample-counter position and stamped
 *    onto the packet before it enters the piconet store.
 *  - Packets with a decoded header are printed immediately.
 *  - Inquiry ID packets (GIAC/LIAC, no header) are counted silently.
 *  - When a brand-new LAP is seen for the first time a one-line announcement
 *    is printed.
 *  - On Ctrl+C the full store summary is printed before exit.
 *
 * Usage
 * -----
 *   bredr-scanner <channel>
 *
 *   channel  BR/EDR channel index 0–79; frequency = 2402 + channel MHz.
 *
 * Example
 * -------
 *   ./build/examples/bredr-scanner 31    # 2433 MHz
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>

#include "../src/hackrf.h"
#include "../src/bredr_phy.h"
#include "../src/bredr_piconet.h"
#include "../src/bredr_piconet_store.h"
#include "../src/rx_packet.h"

/* -------------------------------------------------------------------------
 * RF / DSP constants
 * -------------------------------------------------------------------------*/

#define SAMPLE_RATE     20000000u   /* 20 Msps                              */
#define DECIM_FACTOR    10u         /* 20 Msps → 2 Msps                     */
#define SYMBOL_STEP     2u          /* decimated samples per BR/EDR bit      */
#define LNA_GAIN        32u
#define VGA_GAIN        32u
#define LO_OFFSET_HZ    500000u     /* LO offset above channel centre        */

/* raw samples per bit: DECIM_FACTOR × SYMBOL_STEP */
#define RAW_SAMPS_PER_BIT   (DECIM_FACTOR * SYMBOL_STEP)

/* 1600 Hz slot clock period: 625 µs = 625 bit periods at 1 Mbps */
#define BITS_PER_RX_TIME_1600_TICK   625.0

#define BLOCK_MAX       131072u
#define BLOCK_DEC       (BLOCK_MAX / DECIM_FACTOR)

/* -------------------------------------------------------------------------
 * Global DSP + piconet state
 * -------------------------------------------------------------------------*/

static nco_crcf          g_nco;
static firdecim_crcf     g_firdec;
static cpfskdem          g_demod;
static bredr_processor_t g_proc;
static bredr_piconet_store_t g_store;

static float complex     g_raw[BLOCK_MAX];
static float complex     g_mixed[BLOCK_MAX];
static float complex     g_dec[BLOCK_DEC];

/* Running count of raw I/Q sample pairs received across all callbacks.
 * Captured at the start of each callback as the block's base sample index. */
static unsigned long long g_total_raw_samples = 0;
static unsigned long long g_block_base_sample = 0;

/* BR/EDR channel index (0–79); set in main() before streaming starts. */
static int g_channel = 0;

/* -------------------------------------------------------------------------
 * Statistics
 * -------------------------------------------------------------------------*/

static volatile sig_atomic_t g_stop = 0;
static unsigned long g_total_bits        = 0;
static unsigned long g_total_packets     = 0;  /* all packets with valid AC    */
static unsigned long g_header_packets    = 0;  /* packets with decoded header */
static unsigned long g_id_packets        = 0;  /* ID / shortened-AC packets   */

typedef struct
{
    uint8_t lt_addr;
    uint8_t type;
    uint8_t flow;
    uint8_t arqn;
    uint8_t seqn;
    uint8_t hec;
    int     hec_ok;
} bredr_decoded_header_t;

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"};

static int decode_header_with_clock(const bredr_packet_t *pkt,
                                    uint8_t uap,
                                    uint8_t clk6,
                                    bredr_decoded_header_t *out)
{
    if (!pkt || !pkt->has_header || !out)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3fu), bits);

    out->lt_addr = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
    out->type    = (bits[3]) | (uint8_t)(bits[4] << 1) | (uint8_t)(bits[5] << 2)
                              | (uint8_t)(bits[6] << 3);
    out->flow    = bits[7];
    out->arqn    = bits[8];
    out->seqn    = bits[9];

    out->hec = 0;
    for (int i = 0; i < 8; i++)
        out->hec |= (uint8_t)(bits[10 + i] << (7 - i));

    uint16_t hdr_data = (uint16_t)((out->lt_addr & 0x7u)
                                 | ((out->type   & 0xFu) << 3u)
                                 | ((out->flow   & 0x1u) << 7u)
                                 | ((out->arqn   & 0x1u) << 8u)
                                 | ((out->seqn   & 0x1u) << 9u));
    out->hec_ok = (bredr_compute_hec(hdr_data, uap) == out->hec);
    return out->hec_ok;
}

static void print_payload_preview(const bredr_packet_t *pkt)
{
    if (!pkt || pkt->payload_bytes == 0u) {
        printf("Payload      : (none)\n");
        return;
    }

    unsigned int show = pkt->payload_bytes < 32u ? pkt->payload_bytes : 32u;
    printf("Payload      : %u bytes", pkt->payload_bytes);
    for (unsigned int i = 0; i < show; i++) {
        if (i % 16u == 0u)
            printf("\n               ");
        printf("%02X ", pkt->payload[i]);
    }
    if (pkt->payload_bytes > show)
        printf("...");
    printf("\n");
}

static const char *tracking_state_desc(int tracking_state)
{
    if (tracking_state < 0)
        return "CLK1-6 never found";
    if (tracking_state == 0)
        return "CLK1-6 reacquire required";
    if (tracking_state >= 5)
        return "strong lock";
    return "tracking";
}

static void print_bredr_packet_report(const rx_packet_t *rx_pkt,
                                      const bredr_packet_t *pkt,
                                      const bredr_piconet_t *pnet)
{
    if (!rx_pkt || !pkt)
        return;

    unsigned int freq_mhz = rx_pkt->frequency_hz / 1000000u;
    unsigned int channel = (freq_mhz >= 2402u) ? (freq_mhz - 2402u) : 0u;
    int is_data = (rx_pkt->type == RX_PACKET_BREDR_DATA);

    printf("\n------------------ Packet #%lu --------------------\n", g_total_packets);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (20 Msps master clock)\n", rx_pkt->sample_index);
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n", freq_mhz, channel);
    printf("RSSI         : %.2f dBm\n", rx_pkt->rssi);

    printf("\n[%s Packet Info]\n",
           is_data ? "BR/EDR Data" : "BR/EDR Inquiry");
    printf("LAP          : 0x%06" PRIX32 "\n", pkt->lap & 0xFFFFFFu);
    if (pkt->has_header)
        printf("HEADER       : 0x%014" PRIX64 "\n",
               pkt->header_raw & 0x003FFFFFFFFFFFFFull);
    else
        printf("HEADER       : (none — shortened access code)\n");

    if (pkt->has_header) {
        bredr_decoded_header_t decoded = {0};
        int decoded_ok = 0;
        if (pnet && pnet->clk_known && pnet->uap_found)
            decoded_ok = decode_header_with_clock(pkt, pnet->uap,
                                                  pnet->central_clk_1_6,
                                                  &decoded);

        if (decoded_ok) {
            printf("\n[Decoded Header Info]\n");
            printf("HEC          : 0x%02X [PASS]\n", decoded.hec);
            printf("TYPE         : %s (%u)\n",
                   s_bredr_type_names[decoded.type & 0x0Fu], decoded.type & 0x0Fu);
            printf("LT_ADDR      : %u\n", decoded.lt_addr & 0x07u);
            printf("FLOW         : %u\n", decoded.flow & 1u);
            printf("ARQN         : %u\n", decoded.arqn & 1u);
            printf("SEQN         : %u\n", decoded.seqn & 1u);
        }

        print_payload_preview(pkt);
    }

    if (is_data && pnet) {
        printf("\n[Piconet Info]\n");
        printf("Packets      : %lu\n", pnet->total_packets);
        if (pnet->uap_found)
            printf("UAP          : 0x%02X\n", pnet->uap);
        else
            printf("UAP          : 0x??\n");
        printf("Tracking     : %d (%s)\n",
               pnet->tracking_state, tracking_state_desc(pnet->tracking_state));
        if (pnet->clk_known)
            printf("CLK1-6       : %u\n", pnet->central_clk_1_6);
        else
            printf("CLK1-6       : ??\n");
    }

    printf("--------------------------------------------------\n");
}

/* -------------------------------------------------------------------------
 * Signal handler
 * -------------------------------------------------------------------------*/

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* -------------------------------------------------------------------------
 * HackRF RX callback
 * -------------------------------------------------------------------------*/

static int rx_callback(hackrf_transfer *transfer)
{
    if (g_stop)
        return -1;

    int8_t       *buf   = (int8_t *)transfer->buffer;
    unsigned int  nsamp = (unsigned int)(transfer->valid_length / 2);
    if (nsamp > BLOCK_MAX)
        nsamp = BLOCK_MAX;

    /* Snapshot block base before updating the running total. */
    g_block_base_sample = g_total_raw_samples;
    g_total_raw_samples += nsamp;

    /* int8 I/Q → normalised float complex */
    for (unsigned int i = 0; i < nsamp; i++)
        g_raw[i] = buf[2*i] / 128.0f + (buf[2*i+1] / 128.0f) * _Complex_I;

    /* Mix channel offset to DC */
    nco_crcf_mix_block_down(g_nco, g_raw, g_mixed, nsamp);

    /* Decimate ×10 */
    unsigned int ndec = nsamp / DECIM_FACTOR;
    firdecim_crcf_execute_block(g_firdec, g_mixed, ndec, g_dec);

    /* Demodulate and push bits into the PHY processor */
    for (unsigned int i = 0; i + SYMBOL_STEP <= ndec; i += SYMBOL_STEP)
    {
        unsigned int raw_sym = cpfskdem_demodulate(g_demod, &g_dec[i]);
        uint8_t bit = (uint8_t)(raw_sym & 1u);

        bredr_status_t s = bredr_push_bit_and_samples(&g_proc, bit,
                                                      g_dec[i], g_dec[i + 1u]);
        g_total_bits++;

        if (s != BREDR_VALID_PACKET)
            continue;

        /* ---- Packet ready ------------------------------------------------ */
        g_total_packets++;
        bredr_packet_t pkt;
        bredr_get_packet(&g_proc, &pkt);

        /* Compute Bluetooth CLK at the ACCESS CODE detection point.
         *
         * i/SYMBOL_STEP is the bit index of the LAST collected bit (end of
         * packet).  For UAP recovery we need a timestamp that is the same
         * fixed offset from the AC for every packet, regardless of payload
         * length.  We back-calculate to the end of the sync word (= the
         * moment the AC was detected in STATE_SEARCHING) by subtracting the
         * bits consumed after that point:
         *   4  (trailer bits, STATE_DRAINING_TRAILER)
         *  54  (header FEC bits, always present for has_header packets)
         *  payload_bytes × 8  (payload FEC bits)
         *
         * For ID packets (has_header=0) the offset is 0; those packets are
         * not used by the piconet UAP logic so the absolute value does not
         * matter.
         */
        unsigned long long bit_in_block = (unsigned long long)(i / SYMBOL_STEP);
        unsigned long long bits_back = pkt.has_header
            ? (58ULL + (unsigned long long)pkt.payload_bytes * 8ULL)
            : 0ULL;
        unsigned long long ac_bit_in_block = (bit_in_block >= bits_back)
            ? (bit_in_block - bits_back) : 0ULL;
        unsigned long long abs_raw = g_block_base_sample
                                     + ac_bit_in_block * RAW_SAMPS_PER_BIT;
        unsigned long long abs_bit = abs_raw / RAW_SAMPS_PER_BIT;
        uint32_t clkn = (uint32_t)((abs_raw * 3200ULL + (unsigned long long)(SAMPLE_RATE / 2u)) /
                                   (unsigned long long)SAMPLE_RATE);
        pkt.rx_clk_ref = abs_raw;
        pkt.rx_clk_1600 = (uint32_t)((double)abs_bit / BITS_PER_RX_TIME_1600_TICK);

        /* RSSI is computed by the processor at AC detection time (pkt.rssi). */

        /* Route into piconet store — creates the piconet on first sight and
         * drives UAP/clock recovery internally via libbtbb. */
        bredr_piconet_t *pnet = bredr_piconet_store_add_packet(&g_store, &pkt,
                                                                 g_channel,
                                                                 clkn);

        if (pkt.has_header)
            g_header_packets++;
        else
            g_id_packets++;

        rx_packet_t rx_pkt = {
            .sample_index = pkt.rx_clk_ref,
            .type = pkt.has_header ? RX_PACKET_BREDR_DATA : RX_PACKET_BREDR_INQUIRY,
            .frequency_hz = (uint32_t)(2402u + (uint32_t)g_channel) * 1000000u,
            .rssi = pkt.rssi,
            .phy_packet = &pkt
        };

        print_bredr_packet_report(&rx_pkt, &pkt, pnet);

        fflush(stdout);
    }

    return g_stop ? -1 : 0;
}

/* -------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <channel>\n", argv[0]);
        fprintf(stderr, "  channel : BR/EDR channel 0–79 "
                        "(frequency = 2402 + channel MHz)\n");
        return EXIT_FAILURE;
    }

    int ch = atoi(argv[1]);
    if (ch < 0 || ch > 79)
    {
        fprintf(stderr, "Error: channel must be 0–79 (got %d)\n", ch);
        return EXIT_FAILURE;
    }

    uint64_t ch_freq_hz = (uint64_t)(2402 + ch) * 1000000ULL;
    uint64_t lo_freq_hz = ch_freq_hz + LO_OFFSET_HZ;

    printf("BR/EDR Scanner\n");
    printf("==============\n");
    printf("Channel     : %d  (%.3f MHz)\n", ch, ch_freq_hz / 1e6);
    printf("LO          : %.3f MHz  (+%u kHz offset)\n",
           lo_freq_hz / 1e6, LO_OFFSET_HZ / 1000u);
    printf("Sample rate : %u Msps → decimate ×%u → %u Msps\n",
           SAMPLE_RATE / 1000000u, DECIM_FACTOR,
           SAMPLE_RATE / DECIM_FACTOR / 1000000u);
    printf("AC errors   : %u max\n", BREDR_AC_ERRORS_DEFAULT);
    printf("Press Ctrl+C to stop.\n\n");

    /* ------------------------------------------------------------------
     * DSP setup
     * ------------------------------------------------------------------*/
    float nco_norm = -2.0f * (float)M_PI * (float)LO_OFFSET_HZ
                     / (float)SAMPLE_RATE;
    g_nco   = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(g_nco, nco_norm);

    g_firdec = firdecim_crcf_create_kaiser(DECIM_FACTOR, 7, 60.0f);
    g_demod  = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.5f,
                               LIQUID_CPFSK_GMSK);

    if (!g_nco || !g_firdec || !g_demod)
    {
        fprintf(stderr, "Failed to create DSP objects.\n");
        return EXIT_FAILURE;
    }

    g_channel = ch;
    bredr_processor_init(&g_proc, BREDR_AC_ERRORS_DEFAULT);
    bredr_piconet_store_init(&g_store);

    signal(SIGINT, handle_sigint);

    /* ------------------------------------------------------------------
     * HackRF setup
     * ------------------------------------------------------------------*/
    hackrf_device *device = NULL;

    int rc = hackrf_connect(&device);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n",
                hackrf_error_name(rc));
        bredr_piconet_store_free(&g_store);
        return EXIT_FAILURE;
    }

    hackrf_config_t cfg = {
        .lo_freq_hz  = lo_freq_hz,
        .sample_rate = SAMPLE_RATE,
        .lna_gain    = LNA_GAIN,
        .vga_gain    = VGA_GAIN,
    };

    rc = hackrf_configure(device, &cfg);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure() failed: %s\n",
                hackrf_error_name(rc));
        hackrf_disconnect(device);
        bredr_piconet_store_free(&g_store);
        return EXIT_FAILURE;
    }

    rc = hackrf_start_rx(device, rx_callback, NULL);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n",
                hackrf_error_name(rc));
        hackrf_disconnect(device);
        bredr_piconet_store_free(&g_store);
        return EXIT_FAILURE;
    }

    while (!g_stop)
        sleep(1);

    /* ------------------------------------------------------------------
     * Shutdown
     * ------------------------------------------------------------------*/
    hackrf_stop_rx(device);
    hackrf_disconnect(device);

    cpfskdem_destroy(g_demod);
    firdecim_crcf_destroy(g_firdec);
    nco_crcf_destroy(g_nco);

    /* ------------------------------------------------------------------
     * Final summary
     * ------------------------------------------------------------------*/
    printf("\n\n=== Session Summary ===\n");
    printf("  Channel         : %d  (%.3f MHz)\n", ch, ch_freq_hz / 1e6);
    printf("  Total bits      : %lu\n",  g_total_bits);
    printf("  Header packets  : %lu\n",  g_header_packets);
    printf("  ID packets      : %lu\n",  g_id_packets);
    printf("\n");
    bredr_piconet_store_print(&g_store);

    bredr_piconet_store_free(&g_store);

    return EXIT_SUCCESS;
}
