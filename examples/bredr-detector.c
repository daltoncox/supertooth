/**
 * @file bredr-detector.c
 * @brief Live BR/EDR packet detector — single channel, real HackRF.
 *
 * Signal path
 * -----------
 *  HackRF (20 Msps, int8 I/Q)
 *    → float complex
 *    → NCO mix-down  (channel → DC)
 *    → FIR decimate ×10  (20 Msps → 2 Msps)
 *    → GMSK demodulate   (k=2 → 1 Msym/s)
 *    → bredr_push_bit()
 *    → bredr_print_packet()
 *
 * Usage
 * -----
 *   bredr-detector <channel>
 *
 *   <channel>  BR/EDR channel index 0–79.
 *              Channel frequency = 2402 + channel MHz.
 *
 * Example
 * -------
 *   ./build/examples/bredr-detector 1     # listen on 2403 MHz (channel 1)
 *
 * Press Ctrl+C for a packet/LAP summary and clean exit.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <complex.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>

#include "../src/hackrf.h"
#include "../src/bredr_phy.h"

/* -------------------------------------------------------------------------
 * RF / DSP constants
 * -------------------------------------------------------------------------*/

#define SAMPLE_RATE      20000000u      /* 20 Msps                           */
#define DECIM_FACTOR     10u            /* 20 Msps → 2 Msps                 */
#define SYMBOL_STEP      2u             /* decimated samples per BR/EDR bit  */
#define LNA_GAIN         32u
#define VGA_GAIN         32u

/*
 * Place the LO 500 kHz above the target channel so the channel sits at
 * -500 kHz in baseband.  This keeps the desired signal away from DC and
 * avoids the DC spur that appears with direct-conversion receivers like
 * the HackRF.
 */
#define LO_OFFSET_HZ     500000u        /* LO above channel centre           */

/* HackRF callback delivers up to 262 144 bytes = 131 072 I/Q pairs. */
#define BLOCK_MAX        131072u
#define BLOCK_DEC        (BLOCK_MAX / DECIM_FACTOR)

/* -------------------------------------------------------------------------
 * Global DSP state (used by the callback — all initialised in main)
 * -------------------------------------------------------------------------*/

static nco_crcf          g_nco;
static firdecim_crcf     g_firdec;
static cpfskdem          g_demod;
static bredr_processor_t g_proc;

static float complex     g_raw[BLOCK_MAX];
static float complex     g_mixed[BLOCK_MAX];
static float complex     g_dec[BLOCK_DEC];

/* -------------------------------------------------------------------------
 * Statistics and unique-LAP tracking
 * -------------------------------------------------------------------------*/

static volatile sig_atomic_t g_stop = 0;
static unsigned long g_total_bits    = 0;
static unsigned long g_total_packets = 0;

typedef struct { uint32_t *laps; size_t count, cap; } LapSet;

static LapSet g_laps;

static void lapset_init(LapSet *s)
{
    s->cap  = 32;
    s->count = 0;
    s->laps  = malloc(s->cap * sizeof(uint32_t));
}

static void lapset_free(LapSet *s) { free(s->laps); }

static int lapset_contains(const LapSet *s, uint32_t lap)
{
    for (size_t i = 0; i < s->count; i++)
        if (s->laps[i] == lap) return 1;
    return 0;
}

static void lapset_add(LapSet *s, uint32_t lap)
{
    if (lapset_contains(s, lap)) return;
    if (s->count == s->cap) {
        s->cap *= 2;
        s->laps = realloc(s->laps, s->cap * sizeof(uint32_t));
    }
    s->laps[s->count++] = lap;
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

    int8_t       *buf  = (int8_t *)transfer->buffer;
    unsigned int  nsamp = (unsigned int)(transfer->valid_length / 2);
    if (nsamp > BLOCK_MAX)
        nsamp = BLOCK_MAX;

    /* Convert int8 I/Q → normalised float complex */
    for (unsigned int i = 0; i < nsamp; i++)
        g_raw[i] = buf[2*i] / 128.0f + (buf[2*i+1] / 128.0f) * _Complex_I;

    /* Mix channel from -500 kHz to DC */
    nco_crcf_mix_block_down(g_nco, g_raw, g_mixed, nsamp);

    /* Decimate ×10: 20 Msps → 2 Msps */
    unsigned int ndec = nsamp / DECIM_FACTOR;
    firdecim_crcf_execute_block(g_firdec, g_mixed, ndec, g_dec);

    /* Demodulate and feed the BR/EDR PHY */
    for (unsigned int i = 0; i + SYMBOL_STEP <= ndec; i += SYMBOL_STEP) {
        unsigned int raw_sym = cpfskdem_demodulate(g_demod, &g_dec[i]);
        uint8_t bit = (uint8_t)(raw_sym & 1u);

        bredr_status_t s = bredr_push_bit_and_samples(&g_proc, bit,
                                                      g_dec[i], g_dec[i + 1u]);
        g_total_bits++;

        if (s == BREDR_VALID_PACKET) {
            bredr_packet_t pkt;
            bredr_get_packet(&g_proc, &pkt);
            g_total_packets++;
            lapset_add(&g_laps, pkt.lap);

            printf("\n[Packet #%lu]\n", g_total_packets);
            bredr_print_packet(&pkt);
            printf("RSSI     : %.1f dBm\n", pkt.rssi);
            fflush(stdout);
        }
    }

    return g_stop ? -1 : 0;
}

/* -------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    if (argc != 2) {
        fprintf(stderr, "Usage: %s <channel>\n", argv[0]);
        fprintf(stderr, "  channel : BR/EDR channel 0–79 "
                        "(frequency = 2402 + channel MHz)\n");
        return EXIT_FAILURE;
    }

    int ch = atoi(argv[1]);
    if (ch < 0 || ch > 79) {
        fprintf(stderr, "Error: channel must be 0–79 (got %d)\n", ch);
        return EXIT_FAILURE;
    }

    /* Channel centre frequency and LO placement */
    uint64_t ch_freq_hz = (uint64_t)(2402 + ch) * 1000000ULL;
    uint64_t lo_freq_hz = ch_freq_hz + LO_OFFSET_HZ;

    printf("BR/EDR Live Detector\n");
    printf("====================\n");
    printf("Channel  : %d  (%.3f MHz)\n", ch, ch_freq_hz / 1e6);
    printf("LO       : %.3f MHz  (+%u kHz offset)\n",
           lo_freq_hz / 1e6, LO_OFFSET_HZ / 1000u);
    printf("Sample rate : %u Msps  →  decimate ×%u  →  %u Msps\n",
           SAMPLE_RATE / 1000000u, DECIM_FACTOR,
           SAMPLE_RATE / DECIM_FACTOR / 1000000u);
    printf("AC tolerance: %u bit errors\n", BREDR_AC_ERRORS_DEFAULT);
    printf("Press Ctrl+C to stop.\n\n");

    /* ------------------------------------------------------------------
     * DSP setup
     * ------------------------------------------------------------------*/

    /* NCO: channel sits at -LO_OFFSET_HZ in baseband; set NCO to that
     * negative offset so mix_block_down brings it to 0 Hz. */
    float nco_norm = -2.0f * (float)M_PI * (float)LO_OFFSET_HZ
                     / (float)SAMPLE_RATE;
    g_nco   = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(g_nco, nco_norm);

    /* Kaiser FIR decimator: factor 10, 7 half-lengths, 60 dB stop-band */
    g_firdec = firdecim_crcf_create_kaiser(DECIM_FACTOR, 7, 60.0f);

    /* GMSK demodulator — same parameters as supertooth-hybrid */
    g_demod = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.5f, LIQUID_CPFSK_GMSK);

    if (!g_nco || !g_firdec || !g_demod) {
        fprintf(stderr, "Failed to create DSP objects.\n");
        return EXIT_FAILURE;
    }

    /* BR/EDR PHY processor */
    bredr_processor_init(&g_proc, BREDR_AC_ERRORS_DEFAULT);

    /* Unique-LAP set */
    lapset_init(&g_laps);

    /* SIGINT → clean exit */
    signal(SIGINT, handle_sigint);

    /* ------------------------------------------------------------------
     * HackRF setup
     * ------------------------------------------------------------------*/
    hackrf_device *device = NULL;

    int rc = hackrf_connect(&device);
    if (rc != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(rc));
        return EXIT_FAILURE;
    }

    hackrf_config_t cfg = {
        .lo_freq_hz  = lo_freq_hz,
        .sample_rate = SAMPLE_RATE,
        .lna_gain    = LNA_GAIN,
        .vga_gain    = VGA_GAIN,
    };

    rc = hackrf_configure(device, &cfg);
    if (rc != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_configure() failed: %s\n", hackrf_error_name(rc));
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }

    rc = hackrf_start_rx(device, rx_callback, NULL);
    if (rc != HACKRF_SUCCESS) {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(rc));
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }

    /* Run until Ctrl+C */
    while (!g_stop)
        sleep(1);

    /* ------------------------------------------------------------------
     * Shutdown
     * ------------------------------------------------------------------*/
    hackrf_stop_rx(device);
    hackrf_disconnect(device);

    /* ------------------------------------------------------------------
     * Summary
     * ------------------------------------------------------------------*/
    printf("\n\n=== Session summary ===\n");
    printf("  Channel          : %d  (%.3f MHz)\n", ch, ch_freq_hz / 1e6);
    printf("  Total bits       : %lu\n", g_total_bits);
    printf("  Packets found    : %lu\n", g_total_packets);
    printf("  Unique LAPs      : %zu\n", g_laps.count);
    if (g_laps.count > 0) {
        printf("\nUnique LAPs:\n");
        for (size_t i = 0; i < g_laps.count; i++)
            printf("  0x%06X\n", g_laps.laps[i]);
    }

    /* Cleanup */
    cpfskdem_destroy(g_demod);
    firdecim_crcf_destroy(g_firdec);
    nco_crcf_destroy(g_nco);
    lapset_free(&g_laps);

    return EXIT_SUCCESS;
}
