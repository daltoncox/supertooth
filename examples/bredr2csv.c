/**
 * @file bredr2csv.c
 * @brief 20-channel BR/EDR receiver — writes detected packets to a CSV file.
 *
 * Signal path (per channel)
 * -------------------------
 *  HackRF (20 Msps, int8 I/Q)
 *    -> float complex
 *    -> NCO mix-down  (channel -> DC)
 *    -> FIR decimate x10  (20 Msps -> 2 Msps)
 *    -> GMSK demodulate   (k=2 -> 1 Msym/s)
 *    -> bredr_push_bit()
 *
 * CSV columns
 * -----------
 *  LAP      — 24-bit Lower Address Part (hex, 6 digits)
 *  RX_TIME  — absolute sample index at AC detection (20 Msps master clock)
 *  HEADER   — 18-bit FEC-decoded header, majority-voted from the 54-bit
 *             on-air FEC word; bits are still whitened (CLK1-6 unknown).
 *             Bit layout (LSB = bit 0):
 *               [2:0]  LT_ADDR   [6:3]  TYPE
 *               [7]    FLOW      [8]    ARQN      [9]    SEQN
 *               [17:10] HEC (bit 10 = first HEC bit transmitted)
 *  CHANNEL  — BR/EDR channel number (0..79)
 *
 * Flags
 * -----
 *  -e, --ac-errors <n>   Max Hamming distance for AC matching (default: 2, max: 3)
 *  -s, --strict-fec      Drop packets where any header FEC bit had a disagreement
 *                        among its three on-air copies (i.e. require all 18 groups
 *                        to be unanimous — no bit was corrected by majority vote).
 *
 * Usage
 * -----
 *   ./build/examples/bredr2csv [options] <output.csv>
 *
 * Only data packets (access code + header) are written; inquiry packets
 * (shortened access code, no header) are silently ignored.
 *
 * Covers BR/EDR channels 0..19 (2402..2421 MHz).
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <inttypes.h>
#include <pthread.h>
#include <getopt.h>

#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>

#include "../src/hackrf.h"
#include "../src/bredr_phy.h"

/* -------------------------------------------------------------------------
 * RF / DSP constants  (channels 0..19, LO centred between ch 9 and ch 10)
 * -------------------------------------------------------------------------*/

#define BREDR_CHANNEL_BW     1000000.0
#define BREDR_CHANNEL_0_FREQ 2402000000.0

#define LO_FREQ_HZ          2411500000ULL
#define SAMPLE_RATE         20000000u
#define LNA_GAIN            32u
#define VGA_GAIN            32u
#define NUM_CHANNELS        20u

#define DECIM_FACTOR        10u
#define SYMBOL_STEP         2u             /* decimated samples per BR/EDR bit */

#define BUFFER_SIZE         262144u
#define CHANNEL_BUFFER_SIZE (BUFFER_SIZE / DECIM_FACTOR)

#define RAW_SAMPS_PER_BIT   (DECIM_FACTOR * SYMBOL_STEP)  /* 20 samps/bit */
#define BITS_PER_CLK1600    625.0

/* -------------------------------------------------------------------------
 * Per-channel DSP + decoder context
 * -------------------------------------------------------------------------*/

typedef struct
{
    unsigned int  ctx_index;
    unsigned int  bredr_channel;
    nco_crcf      nco;
    firdecim_crcf firdec;
    cpfskdem      demod;
    bredr_processor_t proc;
    float complex mixed[BUFFER_SIZE];
    float complex decimated[CHANNEL_BUFFER_SIZE];
    int last_gen_processed;
} channel_ctx_t;

/* -------------------------------------------------------------------------
 * Globals
 * -------------------------------------------------------------------------*/

static float complex g_raw[BUFFER_SIZE];
static unsigned int  g_num_samples = 0u;

static channel_ctx_t g_ctx[NUM_CHANNELS];

/* Output CSV file — opened in main(), written under g_csv_mutex. */
static FILE *g_csv = NULL;

static pthread_t g_workers[NUM_CHANNELS];

static pthread_mutex_t g_work_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_work_cv    = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_done_cv    = PTHREAD_COND_INITIALIZER;
static unsigned int g_dispatch_count = 0u;
static unsigned int g_complete_count = 0u;
static int g_work_generation = 0;
static int g_shutdown        = 0;

static pthread_mutex_t g_csv_mutex = PTHREAD_MUTEX_INITIALIZER;

static volatile sig_atomic_t g_stop = 0;

static unsigned long long g_samples_received  = 0ULL;
static unsigned long long g_block_base_sample = 0ULL;

/* Options — set by argument parsing before any thread is started. */
static uint8_t g_max_ac_errors = BREDR_AC_ERRORS_DEFAULT;
static int     g_strict_fec    = 0;

/* -------------------------------------------------------------------------
 * Signal handler
 * -------------------------------------------------------------------------*/

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

/* -------------------------------------------------------------------------
 * FEC majority-vote decode
 *
 * Applies the 1/3-rate majority vote to the 54-bit header_raw field,
 * producing 18 whitened header bits (CLK1-6 unknown — no unwhitening).
 *
 * Output bit i = majority(header_raw[3i], header_raw[3i+1], header_raw[3i+2]).
 *
 * Sets *had_errors to 1 if any of the 18 three-bit groups were not
 * unanimous (i.e. at least one bit was corrected by the vote), 0 otherwise.
 * -------------------------------------------------------------------------*/

static uint32_t fec_decode_header(uint64_t header_raw, int *had_errors)
{
    uint32_t result = 0u;
    *had_errors = 0;
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = (uint8_t)((header_raw >> (3 * i + 0)) & 1u);
        uint8_t b = (uint8_t)((header_raw >> (3 * i + 1)) & 1u);
        uint8_t c = (uint8_t)((header_raw >> (3 * i + 2)) & 1u);
        if (a != b || b != c)
            *had_errors = 1;
        if ((a & b) | (b & c) | (c & a))
            result |= (uint32_t)1u << i;
    }
    return result;
}

/* -------------------------------------------------------------------------
 * Per-channel processing (called from worker threads)
 * -------------------------------------------------------------------------*/

static void process_channel(channel_ctx_t *ctx)
{
    nco_crcf_mix_block_down(ctx->nco, g_raw, ctx->mixed, g_num_samples);

    unsigned int ndec = g_num_samples / DECIM_FACTOR;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, ndec, ctx->decimated);

    for (unsigned int i = 0; i + SYMBOL_STEP <= ndec; i += SYMBOL_STEP)
    {
        uint8_t bit = (uint8_t)(cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]) & 1u);
        bredr_status_t s = bredr_push_bit_and_samples(&ctx->proc, bit,
                                                      ctx->decimated[i],
                                                      ctx->decimated[i + 1u]);

        if (s != BREDR_VALID_PACKET)
            continue;

        bredr_packet_t pkt;
        if (bredr_get_packet(&ctx->proc, &pkt) != 0)
            continue;

        /* Skip inquiry packets — they carry no header to decode. */
        if (!pkt.has_header)
            continue;

        int had_fec_errors;
        uint32_t hdr18 = fec_decode_header(pkt.header_raw, &had_fec_errors);

        if (g_strict_fec && had_fec_errors)
            continue;

        /* Compute rx_clk_ref at the access-code detection point.
         * i/SYMBOL_STEP is the decimated bit index of the last collected bit.
         * Back up past: 4 trailer bits + 54 header FEC bits + payload bits. */
        unsigned long long bit_in_block = (unsigned long long)(i / SYMBOL_STEP);
        unsigned long long bits_back    = 58ULL + (unsigned long long)pkt.payload_bytes * 8ULL;
        unsigned long long ac_bit       = (bit_in_block >= bits_back)
                                          ? (bit_in_block - bits_back) : 0ULL;
        unsigned long long rx_time      = g_block_base_sample + ac_bit * RAW_SAMPS_PER_BIT;

        pthread_mutex_lock(&g_csv_mutex);
        fprintf(g_csv, "%06" PRIX32 ",%" PRIu64 ",%05" PRIX32 ",%u\n",
                pkt.lap & 0xFFFFFFu,
                rx_time,
                hdr18   & 0x3FFFFu,
                ctx->bredr_channel);
        fflush(g_csv);
        pthread_mutex_unlock(&g_csv_mutex);
    }
}

/* -------------------------------------------------------------------------
 * Worker threads
 * -------------------------------------------------------------------------*/

static void *worker_fn(void *arg)
{
    channel_ctx_t *ctx = (channel_ctx_t *)arg;

    for (;;)
    {
        pthread_mutex_lock(&g_work_mutex);
        while (!g_shutdown && ctx->last_gen_processed == g_work_generation)
            pthread_cond_wait(&g_work_cv, &g_work_mutex);
        if (g_shutdown)
        {
            pthread_mutex_unlock(&g_work_mutex);
            break;
        }
        ctx->last_gen_processed = g_work_generation;
        pthread_mutex_unlock(&g_work_mutex);

        process_channel(ctx);

        pthread_mutex_lock(&g_work_mutex);
        if (++g_complete_count >= g_dispatch_count)
            pthread_cond_signal(&g_done_cv);
        pthread_mutex_unlock(&g_work_mutex);
    }
    return NULL;
}

/* -------------------------------------------------------------------------
 * HackRF callback
 * -------------------------------------------------------------------------*/

static int rx_cb(hackrf_transfer *transfer)
{
    if (g_stop)
        return -1;

    int8_t *buf = (int8_t *)transfer->buffer;
    g_num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (g_num_samples > BUFFER_SIZE)
        g_num_samples = BUFFER_SIZE;

    for (unsigned int i = 0; i < g_num_samples; i++)
        g_raw[i] = buf[2*i] / 128.0f + (buf[2*i+1] / 128.0f) * _Complex_I;

    unsigned long long base = g_samples_received;
    g_samples_received += g_num_samples;

    pthread_mutex_lock(&g_work_mutex);
    g_block_base_sample = base;
    g_dispatch_count    = NUM_CHANNELS;
    g_complete_count    = 0u;
    g_work_generation++;
    pthread_cond_broadcast(&g_work_cv);
    while (g_complete_count < g_dispatch_count && !g_shutdown)
        pthread_cond_wait(&g_done_cv, &g_work_mutex);
    pthread_mutex_unlock(&g_work_mutex);

    return g_stop ? -1 : 0;
}

/* -------------------------------------------------------------------------
 * Setup / teardown
 * -------------------------------------------------------------------------*/

static int setup_channels(void)
{
    float lowest_offset =
        -(NUM_CHANNELS / 2.0f - 0.5f) * (float)BREDR_CHANNEL_BW; /* -9.5 MHz */
    int lowest_bt_ch = (int)(((double)LO_FREQ_HZ + (double)lowest_offset
                               - BREDR_CHANNEL_0_FREQ) / BREDR_CHANNEL_BW);

    for (unsigned int i = 0; i < NUM_CHANNELS; i++)
    {
        channel_ctx_t *c = &g_ctx[i];
        c->ctx_index     = i;
        c->bredr_channel = (unsigned int)(lowest_bt_ch + (int)i);

        float ch_offset = (float)i * (float)BREDR_CHANNEL_BW + lowest_offset;
        float norm_freq = 2.0f * (float)M_PI * ch_offset / (float)SAMPLE_RATE;

        c->nco    = nco_crcf_create(LIQUID_NCO);
        c->firdec = firdecim_crcf_create_kaiser(DECIM_FACTOR, 7, 60.0f);
        c->demod  = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.5f, LIQUID_CPFSK_GMSK);

        if (!c->nco || !c->firdec || !c->demod)
            return -1;

        nco_crcf_set_frequency(c->nco, norm_freq);
        bredr_processor_init(&c->proc, g_max_ac_errors);
        c->last_gen_processed = -1;
    }
    return 0;
}

static void destroy_channels(void)
{
    for (unsigned int i = 0; i < NUM_CHANNELS; i++)
    {
        if (g_ctx[i].demod)  cpfskdem_destroy(g_ctx[i].demod);
        if (g_ctx[i].firdec) firdecim_crcf_destroy(g_ctx[i].firdec);
        if (g_ctx[i].nco)    nco_crcf_destroy(g_ctx[i].nco);
    }
}

static int start_workers(void)
{
    g_shutdown = 0;
    for (unsigned int i = 0; i < NUM_CHANNELS; i++)
    {
        if (pthread_create(&g_workers[i], NULL, worker_fn, &g_ctx[i]) != 0)
        {
            pthread_mutex_lock(&g_work_mutex);
            g_shutdown = 1;
            pthread_cond_broadcast(&g_work_cv);
            pthread_mutex_unlock(&g_work_mutex);
            for (unsigned int j = 0; j < i; j++)
                pthread_join(g_workers[j], NULL);
            return -1;
        }
    }
    return 0;
}

static void stop_workers(void)
{
    pthread_mutex_lock(&g_work_mutex);
    g_shutdown = 1;
    pthread_cond_broadcast(&g_work_cv);
    pthread_mutex_unlock(&g_work_mutex);

    for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        pthread_join(g_workers[i], NULL);
}

/* -------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------*/

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [options] <output.csv>\n", argv0);
    fprintf(stderr, "  -e, --ac-errors <n>  Max AC Hamming distance (default: %u, max: 3)\n",
            BREDR_AC_ERRORS_DEFAULT);
    fprintf(stderr, "  -s, --strict-fec     Drop packets with any corrected header FEC bit\n");
}

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"ac-errors",  required_argument, NULL, 'e'},
        {"strict-fec", no_argument,       NULL, 's'},
        {"help",       no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "e:sh", long_opts, NULL)) != -1)
    {
        switch (opt)
        {
            case 'e':
            {
                int n = atoi(optarg);
                if (n < 0 || n > 3)
                {
                    fprintf(stderr, "Error: --ac-errors must be 0–3 (got %d)\n", n);
                    return EXIT_FAILURE;
                }
                g_max_ac_errors = (uint8_t)n;
                break;
            }
            case 's':
                g_strict_fec = 1;
                break;
            case 'h':
                print_usage(argv[0]);
                return EXIT_SUCCESS;
            default:
                print_usage(argv[0]);
                return EXIT_FAILURE;
        }
    }

    if (optind != argc - 1)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    const char *csv_path = argv[optind];
    g_csv = fopen(csv_path, "w");
    if (!g_csv)
    {
        fprintf(stderr, "Cannot open '%s' for writing.\n", csv_path);
        return EXIT_FAILURE;
    }

    fprintf(g_csv, "LAP,RX_TIME,HEADER,CHANNEL\n");
    fflush(g_csv);

    fprintf(stderr, "bredr2csv  —  BR/EDR 20-channel receiver\n");
    fprintf(stderr, "LO          : %.3f MHz\n", (double)LO_FREQ_HZ / 1e6);
    fprintf(stderr, "Sample rate : %u Msps\n", SAMPLE_RATE / 1000000u);
    fprintf(stderr, "Channels    : %u  (0..19, 2402..2421 MHz)\n", NUM_CHANNELS);
    fprintf(stderr, "AC errors   : %u max\n", g_max_ac_errors);
    fprintf(stderr, "Strict FEC  : %s\n", g_strict_fec ? "on" : "off");
    fprintf(stderr, "Output      : %s\n", csv_path);
    fprintf(stderr, "Press Ctrl+C to stop.\n");

    if (setup_channels() != 0)
    {
        fprintf(stderr, "Failed to initialise DSP contexts.\n");
        destroy_channels();
        fclose(g_csv);
        return EXIT_FAILURE;
    }

    signal(SIGINT, handle_sigint);

    if (start_workers() != 0)
    {
        fprintf(stderr, "Failed to start worker threads.\n");
        destroy_channels();
        fclose(g_csv);
        return EXIT_FAILURE;
    }

    hackrf_device *device = NULL;
    int rc = hackrf_connect(&device);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect: %s\n", hackrf_error_name(rc));
        stop_workers();
        destroy_channels();
        fclose(g_csv);
        return EXIT_FAILURE;
    }

    hackrf_config_t cfg = {
        .lo_freq_hz  = LO_FREQ_HZ,
        .sample_rate = SAMPLE_RATE,
        .lna_gain    = LNA_GAIN,
        .vga_gain    = VGA_GAIN,
    };
    rc = hackrf_configure(device, &cfg);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure: %s\n", hackrf_error_name(rc));
        hackrf_disconnect(device);
        stop_workers();
        destroy_channels();
        fclose(g_csv);
        return EXIT_FAILURE;
    }

    rc = hackrf_start_rx(device, rx_cb, NULL);
    if (rc != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx: %s\n", hackrf_error_name(rc));
        hackrf_disconnect(device);
        stop_workers();
        destroy_channels();
        fclose(g_csv);
        return EXIT_FAILURE;
    }

    while (!g_stop)
        sleep(1);

    hackrf_stop_rx(device);
    hackrf_disconnect(device);

    stop_workers();
    destroy_channels();
    fclose(g_csv);

    return EXIT_SUCCESS;
}
