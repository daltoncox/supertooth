#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>
#include <btbb.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include "hackrf.h"
#include "ble_phy.h"
#include <pthread.h>

// Bluetooth BR/EDR constants
#define BREDR_CHANNEL_BW     1e6        // 1 MHz per BR/EDR Bluetooth channel
#define BREDR_CHANNEL_0_FREQ 2402e6     // Lowest Bluetooth BR/EDR channel frequency

// HackRF parameters
// LO at 2411.5 MHz: midpoint of BR/EDR ch0(2402)..ch19(2421) with +0.5 MHz shift
// so no BR/EDR channel falls at DC, giving all 20 channels usable.
#define LO_FREQ_HZ          2411500000ULL  // 2411.5 MHz as integer for hackrf_config_t
#define SAMPLE_RATE         20e6
#define LNA_GAIN            32
#define VGA_GAIN            32
#define NUM_BREDR_CHANNELS  20

#define BUFFER_SIZE         262144
#define CHANNEL_BUFFER_SIZE ((unsigned int)(BUFFER_SIZE / (SAMPLE_RATE / (BREDR_CHANNEL_BW * 8))) + 1)

// BLE advertising channel 37 = 2402 MHz, same frequency as BR/EDR ch0.
// Offset from LO: 2402e6 - 2411.5e6 = -9.5 MHz
#define BLE_CH37_FREQ_OFFSET -9.5e6
#define BLE_CH37_INDEX       37  // BLE channel index for dewhitening seed

// --- Global sample buffers ---------------------------------------------------

float complex raw[BUFFER_SIZE];
unsigned int num_samples;
static unsigned long long samples_received = 0;

// --- BR/EDR channel context --------------------------------------------------

typedef struct
{
    unsigned int ctx_index;
    unsigned int bredr_channel;
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;  // h=0.3 GMSK
    float complex mixed[BUFFER_SIZE];
    float complex decimated[CHANNEL_BUFFER_SIZE];
    uint8_t bits[CHANNEL_BUFFER_SIZE / 2];
    int last_gen_processed;
} bredr_channel_ctx_t;

bredr_channel_ctx_t bredr_ctx_arr[NUM_BREDR_CHANNELS];

// --- BLE channel context -----------------------------------------------------

typedef struct
{
    nco_crcf nco;              // same offset freq as BR/EDR ch0 (-9.5 MHz)
    firdecim_crcf firdec;
    cpfskdem demod;            // h=0.5 GFSK
    float complex mixed[BUFFER_SIZE];
    float complex decimated[CHANNEL_BUFFER_SIZE];
    ble_channel_processor_t ble_proc;
    ble_status_t prev_status;
    long long pkt_start_decimated;  // decimated-sample index of AA match
    int last_gen_processed;
} ble_ctx_t;

ble_ctx_t ble_ctx;

// --- Thread pool state -------------------------------------------------------

static pthread_t *worker_threads = NULL;
static unsigned int worker_count = 0;
static pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t work_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t work_cv = PTHREAD_COND_INITIALIZER;
static pthread_cond_t done_cv = PTHREAD_COND_INITIALIZER;
static unsigned int dispatch_count = 0;
static unsigned int complete_count = 0;
static int work_generation = 0;
static unsigned long long gen_samples_received = 0;
static int shutdown_requested = 0;

// --- Setup -------------------------------------------------------------------

void setup_channel_ctx(void)
{
    // Fix from supertooth-rx.c: use float, not int.
    // rx.c formula: (num_channels / 2 - 1) * -BW = -9e6 → channel at DC, only 19 usable.
    // New formula:  -(num_channels / 2 - 0.5) * BW = -9.5e6 → no DC channel, 20 usable.
    float lowest_ctx_freq_offset = -(NUM_BREDR_CHANNELS / 2.0f - 0.5f) * BREDR_CHANNEL_BW;

    int lowest_ctx_bt_channel = (int)((LO_FREQ_HZ + lowest_ctx_freq_offset - BREDR_CHANNEL_0_FREQ)
                                      / BREDR_CHANNEL_BW); // = 0

    for (unsigned int i = 0; i < NUM_BREDR_CHANNELS; i++)
    {
        bredr_ctx_arr[i].ctx_index    = i;
        bredr_ctx_arr[i].bredr_channel = (unsigned int)(lowest_ctx_bt_channel + i);

        float channel_offset_freq = i * BREDR_CHANNEL_BW + lowest_ctx_freq_offset;
        float normalized_freq = 2.0f * M_PI * channel_offset_freq / SAMPLE_RATE;
        bredr_ctx_arr[i].nco = nco_crcf_create(LIQUID_NCO);
        nco_crcf_set_frequency(bredr_ctx_arr[i].nco, normalized_freq);

        bredr_ctx_arr[i].firdec = firdecim_crcf_create_kaiser(10, 7, 60.0f);
        bredr_ctx_arr[i].demod  = cpfskdem_create(1, 0.3f, 2, 3, 0.5f, LIQUID_CPFSK_GMSK);
        bredr_ctx_arr[i].last_gen_processed = -1;
    }

    // BLE context: tunes to -9.5 MHz (same as BR/EDR ch0), but h=0.5 GFSK
    float ble_normalized_freq = 2.0f * M_PI * BLE_CH37_FREQ_OFFSET / SAMPLE_RATE;
    ble_ctx.nco   = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(ble_ctx.nco, ble_normalized_freq);
    ble_ctx.firdec = firdecim_crcf_create_kaiser(10, 7, 60.0f);
    ble_ctx.demod  = cpfskdem_create(1, 0.5f, 2, 3, 0.5f, LIQUID_CPFSK_GMSK);
    ble_processor_init(&ble_ctx.ble_proc, BLE_CH37_INDEX);
    ble_ctx.prev_status         = BLE_SEARCHING;
    ble_ctx.pkt_start_decimated = -1;
    ble_ctx.last_gen_processed  = -1;
}

// --- BR/EDR processing -------------------------------------------------------

static void process_bredr_channel(bredr_channel_ctx_t *ctx)
{
    nco_crcf_mix_block_down(ctx->nco, raw, ctx->mixed, num_samples);

    unsigned int decimated_samples = num_samples / 10;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, decimated_samples, ctx->decimated);

    for (unsigned int i = 0; i < decimated_samples; i += 2)
    {
        unsigned int bit = cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]);
        ctx->bits[i / 2] = (uint8_t)(bit & 1);
    }

    btbb_packet *pkt = NULL;
    int offset = btbb_find_ac((char *)ctx->bits, decimated_samples / 2, LAP_ANY, 2, &pkt);
    if (offset >= 0 && pkt != NULL)
    {
        uint32_t lap = btbb_packet_get_lap(pkt);
        if (lap == 0x9e8b33)
        {
            btbb_packet_unref(pkt);
            return;
        }

        unsigned long long absolute_sample = gen_samples_received + (offset * 20);
        unsigned long long bit_position = absolute_sample / 20;
        uint32_t clkn = (uint32_t)(bit_position / 312.5);
        btbb_packet_set_data(pkt, (char *)ctx->bits + offset, 400, ctx->bredr_channel, clkn);

        float max_power = 0.0f;
        for (unsigned int i = (unsigned int)offset * 2;
             i < (unsigned int)offset * 2 + 144 && i < decimated_samples; i++)
        {
            float power = crealf(ctx->decimated[i]) * crealf(ctx->decimated[i]) +
                          cimagf(ctx->decimated[i]) * cimagf(ctx->decimated[i]);
            if (power > max_power)
                max_power = power;
        }
        float rssi_dbm = (max_power > 0.0f) ? 10.0f * log10f(max_power) - 30.0f : 0.0f;

        pthread_mutex_lock(&print_mutex);
        printf("CTX %02d - BREDR_CH: %02d, CLKN: %06d, LAP: %06x, AC_ERR: %d, RSSI: %.2f\n",
               ctx->ctx_index, ctx->bredr_channel, clkn, lap,
               btbb_packet_get_ac_errors(pkt), rssi_dbm);
        fflush(stdout);
        if (btbb_packet_get_lap(pkt) != 0x9e8b33)
            btbb_process_packet(pkt, NULL);
        pthread_mutex_unlock(&print_mutex);

        btbb_packet_unref(pkt);
    }
}

static void *bredr_channel_worker(void *arg)
{
    bredr_channel_ctx_t *ctx = (bredr_channel_ctx_t *)arg;
    for (;;)
    {
        pthread_mutex_lock(&work_mutex);
        while (!shutdown_requested && ctx->last_gen_processed == work_generation)
            pthread_cond_wait(&work_cv, &work_mutex);
        if (shutdown_requested)
        {
            pthread_mutex_unlock(&work_mutex);
            break;
        }
        ctx->last_gen_processed = work_generation;
        pthread_mutex_unlock(&work_mutex);

        process_bredr_channel(ctx);

        pthread_mutex_lock(&work_mutex);
        complete_count++;
        if (complete_count >= dispatch_count)
            pthread_cond_signal(&done_cv);
        pthread_mutex_unlock(&work_mutex);
    }
    return NULL;
}

// --- BLE processing ----------------------------------------------------------

static void process_ble_channel(void)
{
    nco_crcf_mix_block_down(ble_ctx.nco, raw, ble_ctx.mixed, num_samples);

    unsigned int decimated_samples = num_samples / 10;
    firdecim_crcf_execute_block(ble_ctx.firdec, ble_ctx.mixed,
                                decimated_samples, ble_ctx.decimated);

    // buf_start in decimated-sample units (BLE PHY operates at 2 MHz post-decim)
    unsigned long long buf_start = gen_samples_received / 10;

    for (unsigned int i = 0; i < decimated_samples; i += 2)
    {
        unsigned int sym = cpfskdem_demodulate(ble_ctx.demod, &ble_ctx.decimated[i]);
        uint8_t bit = (uint8_t)(sym & 0x01);

        ble_status_t status = ble_push_bit(&ble_ctx.ble_proc, bit);

        if (ble_ctx.prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            ble_ctx.pkt_start_decimated = (long long)(buf_start + i);
        ble_ctx.prev_status = status;

        if (status == BLE_VALID_PACKET)
        {
            float max_power = 0.0f;
            if (ble_ctx.pkt_start_decimated >= 0)
            {
                long long rel = ble_ctx.pkt_start_decimated - (long long)buf_start;
                unsigned int i_start = (rel < 0) ? 0 : (unsigned int)rel;
                unsigned int i_end = i + 2;
                for (unsigned int j = i_start; j < i_end && j < decimated_samples; j++)
                {
                    float p = crealf(ble_ctx.decimated[j]) * crealf(ble_ctx.decimated[j]) +
                              cimagf(ble_ctx.decimated[j]) * cimagf(ble_ctx.decimated[j]);
                    if (p > max_power)
                        max_power = p;
                }
            }
            float rssi_dbm = (max_power > 0.0f) ? 10.0f * log10f(max_power) - 30.0f : 0.0f;

            ble_packet_t pkt;
            if (ble_get_packet(&ble_ctx.ble_proc, &pkt) == 0)
            {
                pthread_mutex_lock(&print_mutex);
                printf("\n");
                ble_print_packet(&pkt);
                printf("RSSI     : %.1f dBm\n", rssi_dbm);
                fflush(stdout);
                pthread_mutex_unlock(&print_mutex);
            }
        }
    }
}

static void *ble_channel_worker(void *arg)
{
    (void)arg;
    for (;;)
    {
        pthread_mutex_lock(&work_mutex);
        while (!shutdown_requested && ble_ctx.last_gen_processed == work_generation)
            pthread_cond_wait(&work_cv, &work_mutex);
        if (shutdown_requested)
        {
            pthread_mutex_unlock(&work_mutex);
            break;
        }
        ble_ctx.last_gen_processed = work_generation;
        pthread_mutex_unlock(&work_mutex);

        process_ble_channel();

        pthread_mutex_lock(&work_mutex);
        complete_count++;
        if (complete_count >= dispatch_count)
            pthread_cond_signal(&done_cv);
        pthread_mutex_unlock(&work_mutex);
    }
    return NULL;
}

// --- Thread pool -------------------------------------------------------------

static void init_thread_pool(void)
{
    worker_count   = NUM_BREDR_CHANNELS + 1; // 20 BR/EDR + 1 BLE
    worker_threads = (pthread_t *)malloc(sizeof(pthread_t) * worker_count);

    for (unsigned int i = 0; i < NUM_BREDR_CHANNELS; i++)
        (void)pthread_create(&worker_threads[i], NULL, bredr_channel_worker, &bredr_ctx_arr[i]);

    (void)pthread_create(&worker_threads[NUM_BREDR_CHANNELS], NULL, ble_channel_worker, NULL);
}

// --- HackRF callback ---------------------------------------------------------

int hybrid_cb(hackrf_transfer *transfer)
{
    int8_t *samples = (int8_t *)transfer->buffer;
    num_samples = transfer->valid_length / 2;
    for (unsigned int i = 0; i < num_samples; i++)
    {
        float i_sample = samples[2 * i] / 128.0f;
        float q_sample = samples[2 * i + 1] / 128.0f;
        raw[i] = i_sample + q_sample * _Complex_I;
    }
    samples_received += num_samples;

    pthread_mutex_lock(&work_mutex);
    gen_samples_received = samples_received;
    dispatch_count  = NUM_BREDR_CHANNELS + 1; // 20 BR/EDR + 1 BLE
    complete_count  = 0;
    work_generation++;
    pthread_cond_broadcast(&work_cv);
    while (complete_count < dispatch_count)
        pthread_cond_wait(&done_cv, &work_mutex);
    pthread_mutex_unlock(&work_mutex);

    return 0;
}

// --- Main --------------------------------------------------------------------

int main(void)
{
    printf("Supertooth Hybrid: BR/EDR ch0-19 + BLE ch37\n");
    printf("LO: %.1f MHz, %d BR/EDR channels + 1 BLE channel, 20 MHz bandwidth\n",
           LO_FREQ_HZ / 1e6, NUM_BREDR_CHANNELS);

    setup_channel_ctx();
    init_thread_pool();

    btbb_init(2);
    btbb_init_survey();

    int result;
    hackrf_device *device = NULL;

    result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        return EXIT_FAILURE;
    }

    hackrf_config_t config = {
        .lo_freq_hz  = LO_FREQ_HZ,
        .sample_rate = (uint32_t)SAMPLE_RATE,
        .lna_gain    = LNA_GAIN,
        .vga_gain    = VGA_GAIN,
    };

    result = hackrf_configure(device, &config);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure() failed\n");
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, hybrid_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }

    printf("Receiving... Press Ctrl+C to stop.\n");
    while (1)
        sleep(1);

    return 0;
}
