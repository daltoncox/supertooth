#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>
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
#define BLOCK_POOL_SIZE     64u
#define CHANNEL_RING_SIZE   8u

// BLE advertising channel 37 = 2402 MHz, same frequency as BR/EDR ch0.
// Offset from LO: 2402e6 - 2411.5e6 = -9.5 MHz
#define BLE_CH37_FREQ_OFFSET -9.5e6
#define BLE_CH37_INDEX       37  // BLE channel index for dewhitening seed

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
    unsigned int block_idx_ring[CHANNEL_RING_SIZE];
    unsigned int block_write_idx;
    unsigned int block_read_idx;
    unsigned int block_count;
    unsigned long dropped_blocks;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cv;
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
    unsigned int block_idx_ring[CHANNEL_RING_SIZE];
    unsigned int block_write_idx;
    unsigned int block_read_idx;
    unsigned int block_count;
    unsigned long dropped_blocks;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cv;
} ble_ctx_t;

ble_ctx_t ble_ctx;

typedef struct
{
    float complex samples[BUFFER_SIZE];
    unsigned int num_samples;
    unsigned long long block_base_sample;
    unsigned int refcount;
} rx_block_t;

// --- Thread pool state -------------------------------------------------------

static pthread_t *worker_threads = NULL;
static unsigned int worker_count = 0;
static pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;
static int shutdown_requested = 0;
static unsigned long g_packet_count = 0;
static unsigned long g_dropped_blocks = 0;
static unsigned long long samples_received = 0;
static rx_block_t g_block_pool[BLOCK_POOL_SIZE];
static unsigned int g_pool_write_idx = 0u;
static unsigned int g_channel_ctx_count = 0u;
static int g_ble_ctx_initialized = 0;
static int g_debug = 0;
static volatile sig_atomic_t g_stop = 0;

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
           (unsigned int)(BREDR_CHANNEL_0_FREQ / 1000000u), BLE_CH37_INDEX);
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

    printf("pkt=%-6lu type=BLE   pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           pdu_name,
           BLE_CH37_INDEX,
           addr_buf,
           pkt->pdu[1],
           ble_verify_crc(pkt) ? "PASS" : "FAIL",
           rssi_dbr);
}

static void print_bredr_packet_full(unsigned long packet_no,
                                    unsigned int ctx_index,
                                    unsigned int channel,
                                    uint32_t lap,
                                    uint32_t clkn,
                                    int ac_errors,
                                    float rssi_dbr,
                                    unsigned long long sample_index)
{
    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           sample_index, (unsigned int)(SAMPLE_RATE / 1000000u));
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(BREDR_CHANNEL_0_FREQ / 1000000u) + channel, channel);
    printf("RSSI         : %.2f dBr\n", rssi_dbr);

    printf("\n[BR/EDR Packet Info]\n");
    printf("Context      : %u\n", ctx_index);
    printf("LAP          : 0x%06" PRIX32 "\n", lap & 0xFFFFFFu);
    printf("CLKN         : %u\n", clkn);
    printf("AC Errors    : %d\n", ac_errors);
    printf("--------------------------------------------------\n");
}

static void print_bredr_packet_summary(unsigned long packet_no,
                                       unsigned int channel,
                                       uint32_t lap,
                                       uint32_t clkn,
                                       int ac_errors,
                                       float rssi_dbr)
{
    printf("pkt=%-6lu type=BREDR lap=%06" PRIX32 " ch=%02u ac=%d clkn=%u rssi=%.1f\n",
           packet_no,
           lap & 0xFFFFFFu,
           channel,
           ac_errors,
           clkn,
           rssi_dbr);
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

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "-d, --debug");
}

// --- Setup -------------------------------------------------------------------

static int setup_channel_ctx(void)
{
    g_channel_ctx_count = 0u;
    g_ble_ctx_initialized = 0;
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
        if (!bredr_ctx_arr[i].nco || !bredr_ctx_arr[i].firdec || !bredr_ctx_arr[i].demod)
            return -1;

        bredr_ctx_arr[i].block_write_idx = 0u;
        bredr_ctx_arr[i].block_read_idx = 0u;
        bredr_ctx_arr[i].block_count = 0u;
        bredr_ctx_arr[i].dropped_blocks = 0ul;
        if (pthread_mutex_init(&bredr_ctx_arr[i].queue_mutex, NULL) != 0)
            return -1;
        if (pthread_cond_init(&bredr_ctx_arr[i].queue_cv, NULL) != 0)
        {
            pthread_mutex_destroy(&bredr_ctx_arr[i].queue_mutex);
            return -1;
        }
        g_channel_ctx_count++;
    }

    // BLE context: tunes to -9.5 MHz (same as BR/EDR ch0), but h=0.5 GFSK
    float ble_normalized_freq = 2.0f * M_PI * BLE_CH37_FREQ_OFFSET / SAMPLE_RATE;
    ble_ctx.nco   = nco_crcf_create(LIQUID_NCO);
    nco_crcf_set_frequency(ble_ctx.nco, ble_normalized_freq);
    ble_ctx.firdec = firdecim_crcf_create_kaiser(10, 7, 60.0f);
    ble_ctx.demod  = cpfskdem_create(1, 0.5f, 2, 3, 0.5f, LIQUID_CPFSK_GMSK);
    if (!ble_ctx.nco || !ble_ctx.firdec || !ble_ctx.demod)
        return -1;
    ble_processor_init(&ble_ctx.ble_proc, BLE_CH37_INDEX);
    ble_ctx.prev_status         = BLE_SEARCHING;
    ble_ctx.pkt_start_decimated = -1;
    ble_ctx.block_write_idx = 0u;
    ble_ctx.block_read_idx = 0u;
    ble_ctx.block_count = 0u;
    ble_ctx.dropped_blocks = 0ul;
    if (pthread_mutex_init(&ble_ctx.queue_mutex, NULL) != 0)
        return -1;
    if (pthread_cond_init(&ble_ctx.queue_cv, NULL) != 0)
    {
        pthread_mutex_destroy(&ble_ctx.queue_mutex);
        return -1;
    }
    g_ble_ctx_initialized = 1;

    return 0;
}

static void destroy_channel_ctx(void)
{
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
    {
        if (bredr_ctx_arr[i].demod)
        {
            cpfskdem_destroy(bredr_ctx_arr[i].demod);
            bredr_ctx_arr[i].demod = NULL;
        }
        if (bredr_ctx_arr[i].firdec)
        {
            firdecim_crcf_destroy(bredr_ctx_arr[i].firdec);
            bredr_ctx_arr[i].firdec = NULL;
        }
        if (bredr_ctx_arr[i].nco)
        {
            nco_crcf_destroy(bredr_ctx_arr[i].nco);
            bredr_ctx_arr[i].nco = NULL;
        }
        pthread_cond_destroy(&bredr_ctx_arr[i].queue_cv);
        pthread_mutex_destroy(&bredr_ctx_arr[i].queue_mutex);
    }
    g_channel_ctx_count = 0u;

    if (g_ble_ctx_initialized && ble_ctx.demod)
    {
        cpfskdem_destroy(ble_ctx.demod);
        ble_ctx.demod = NULL;
    }
    if (g_ble_ctx_initialized && ble_ctx.firdec)
    {
        firdecim_crcf_destroy(ble_ctx.firdec);
        ble_ctx.firdec = NULL;
    }
    if (g_ble_ctx_initialized && ble_ctx.nco)
    {
        nco_crcf_destroy(ble_ctx.nco);
        ble_ctx.nco = NULL;
    }
    if (g_ble_ctx_initialized)
    {
        pthread_cond_destroy(&ble_ctx.queue_cv);
        pthread_mutex_destroy(&ble_ctx.queue_mutex);
        g_ble_ctx_initialized = 0;
    }
}

// --- BR/EDR processing -------------------------------------------------------

static void process_bredr_channel(bredr_channel_ctx_t *ctx, const rx_block_t *blk)
{
    nco_crcf_mix_block_down(ctx->nco, blk->samples, ctx->mixed, blk->num_samples);

    unsigned int decimated_samples = blk->num_samples / 10;
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

        unsigned long long absolute_sample = blk->block_base_sample + (unsigned long long)(offset * 20);
        unsigned long long bit_position = absolute_sample / 20;
        uint32_t clkn = (uint32_t)(bit_position / 312.5);
        btbb_packet_set_data(pkt, (char *)ctx->bits + offset, 400, ctx->bredr_channel, clkn);
        int ac_errors = btbb_packet_get_ac_errors(pkt);

        float max_power = 0.0f;
        for (unsigned int i = (unsigned int)offset * 2;
             i < (unsigned int)offset * 2 + 144 && i < decimated_samples; i++)
        {
            float power = crealf(ctx->decimated[i]) * crealf(ctx->decimated[i]) +
                          cimagf(ctx->decimated[i]) * cimagf(ctx->decimated[i]);
            if (power > max_power)
                max_power = power;
        }
        float rssi_dbr = (max_power > 0.0f) ? 10.0f * log10f(max_power) : 0.0f;

        pthread_mutex_lock(&print_mutex);
        unsigned long packet_no = ++g_packet_count;
        if (g_output_mode == OUTPUT_MODE_SUMMARY)
        {
            print_bredr_packet_summary(packet_no, ctx->bredr_channel, lap, clkn, ac_errors, rssi_dbr);
        }
        else
        {
            print_bredr_packet_full(packet_no, ctx->ctx_index, ctx->bredr_channel,
                                    lap, clkn, ac_errors, rssi_dbr, absolute_sample);
        }
        fflush(stdout);
        pthread_mutex_unlock(&print_mutex);

        btbb_packet_unref(pkt);
    }
}

static void *bredr_channel_worker(void *arg)
{
    bredr_channel_ctx_t *ctx = (bredr_channel_ctx_t *)arg;
    for (;;)
    {
        pthread_mutex_lock(&ctx->queue_mutex);
        while (!shutdown_requested && ctx->block_count == 0u)
            pthread_cond_wait(&ctx->queue_cv, &ctx->queue_mutex);
        if (shutdown_requested)
        {
            pthread_mutex_unlock(&ctx->queue_mutex);
            break;
        }
        unsigned int block_idx = ctx->block_idx_ring[ctx->block_read_idx];
        ctx->block_read_idx = (ctx->block_read_idx + 1u) % CHANNEL_RING_SIZE;
        ctx->block_count--;
        pthread_mutex_unlock(&ctx->queue_mutex);

        process_bredr_channel(ctx, &g_block_pool[block_idx]);
        __atomic_sub_fetch(&g_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    return NULL;
}

// --- BLE processing ----------------------------------------------------------

static void process_ble_channel(const rx_block_t *blk)
{
    nco_crcf_mix_block_down(ble_ctx.nco, blk->samples, ble_ctx.mixed, blk->num_samples);

    unsigned int decimated_samples = blk->num_samples / 10;
    firdecim_crcf_execute_block(ble_ctx.firdec, ble_ctx.mixed,
                                decimated_samples, ble_ctx.decimated);

    // buf_start in decimated-sample units (BLE PHY operates at 2 MHz post-decim)
    unsigned long long buf_start = blk->block_base_sample / 10;

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
            float rssi_dbr = (max_power > 0.0f) ? 10.0f * log10f(max_power) : 0.0f;

            ble_packet_t pkt;
            if (ble_get_packet(&ble_ctx.ble_proc, &pkt) == 0)
            {
                unsigned long long abs_sample = (buf_start + i) * 10ULL;
                pthread_mutex_lock(&print_mutex);
                unsigned long packet_no = ++g_packet_count;
                if (g_output_mode == OUTPUT_MODE_SUMMARY)
                    print_ble_packet_summary(packet_no, &pkt, rssi_dbr);
                else
                    print_ble_packet_full(packet_no, &pkt, rssi_dbr, abs_sample);
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
        pthread_mutex_lock(&ble_ctx.queue_mutex);
        while (!shutdown_requested && ble_ctx.block_count == 0u)
            pthread_cond_wait(&ble_ctx.queue_cv, &ble_ctx.queue_mutex);
        if (shutdown_requested)
        {
            pthread_mutex_unlock(&ble_ctx.queue_mutex);
            break;
        }
        unsigned int block_idx = ble_ctx.block_idx_ring[ble_ctx.block_read_idx];
        ble_ctx.block_read_idx = (ble_ctx.block_read_idx + 1u) % CHANNEL_RING_SIZE;
        ble_ctx.block_count--;
        pthread_mutex_unlock(&ble_ctx.queue_mutex);

        process_ble_channel(&g_block_pool[block_idx]);
        __atomic_sub_fetch(&g_block_pool[block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    return NULL;
}

// --- Thread pool -------------------------------------------------------------

static int init_thread_pool(void)
{
    shutdown_requested = 0;
    worker_count = g_channel_ctx_count + (g_ble_ctx_initialized ? 1u : 0u);
    worker_threads = (pthread_t *)calloc(worker_count, sizeof(pthread_t));
    if (!worker_threads)
        return -1;

    unsigned int created = 0u;
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
    {
        if (pthread_create(&worker_threads[i], NULL, bredr_channel_worker, &bredr_ctx_arr[i]) != 0)
        {
            worker_count = created;
            return -1;
        }
        created++;
    }

    if (g_ble_ctx_initialized &&
        pthread_create(&worker_threads[g_channel_ctx_count], NULL, ble_channel_worker, NULL) != 0)
    {
        worker_count = created;
        return -1;
    }

    return 0;
}

static void stop_thread_pool(void)
{
    shutdown_requested = 1;
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
    {
        pthread_mutex_lock(&bredr_ctx_arr[i].queue_mutex);
        pthread_cond_signal(&bredr_ctx_arr[i].queue_cv);
        pthread_mutex_unlock(&bredr_ctx_arr[i].queue_mutex);
    }
    if (g_ble_ctx_initialized)
    {
        pthread_mutex_lock(&ble_ctx.queue_mutex);
        pthread_cond_signal(&ble_ctx.queue_cv);
        pthread_mutex_unlock(&ble_ctx.queue_mutex);
    }

    for (unsigned int i = 0; i < worker_count; i++)
        pthread_join(worker_threads[i], NULL);

    free(worker_threads);
    worker_threads = NULL;
    worker_count = 0u;
}

// --- HackRF callback ---------------------------------------------------------

int hybrid_cb(hackrf_transfer *transfer)
{
    if (g_stop)
        return -1;

    if (!transfer || transfer->valid_length == 0)
        return g_stop ? -1 : 0;

    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_samples = transfer->valid_length / 2;
    if (num_samples > BUFFER_SIZE)
        num_samples = BUFFER_SIZE;

    unsigned long long block_base = samples_received;
    samples_received += num_samples;

    int block_idx = -1;
    for (unsigned int i = 0; i < BLOCK_POOL_SIZE; i++)
    {
        unsigned int idx = (g_pool_write_idx + i) % BLOCK_POOL_SIZE;
        if (__atomic_load_n(&g_block_pool[idx].refcount, __ATOMIC_ACQUIRE) == 0u)
        {
            block_idx = (int)idx;
            break;
        }
    }
    if (block_idx < 0)
    {
        g_dropped_blocks++;
        if (g_debug)
            fprintf(stderr, "[debug] dropped callback block: block pool exhausted (%u)\n",
                    BLOCK_POOL_SIZE);
        return g_stop ? -1 : 0;
    }

    rx_block_t *blk = &g_block_pool[(unsigned int)block_idx];
    blk->num_samples = num_samples;
    blk->block_base_sample = block_base;
    blk->refcount = 0u;

    for (unsigned int i = 0; i < num_samples; i++)
    {
        float i_sample = samples[2 * i] / 128.0f;
        float q_sample = samples[2 * i + 1] / 128.0f;
        blk->samples[i] = i_sample + q_sample * _Complex_I;
    }
    g_pool_write_idx = ((unsigned int)block_idx + 1u) % BLOCK_POOL_SIZE;
    __atomic_thread_fence(__ATOMIC_RELEASE);

    for (unsigned int ch = 0; ch < g_channel_ctx_count; ch++)
    {
        bredr_channel_ctx_t *ctx = &bredr_ctx_arr[ch];
        pthread_mutex_lock(&ctx->queue_mutex);
        if (ctx->block_count == CHANNEL_RING_SIZE)
        {
            unsigned int old_idx = ctx->block_idx_ring[ctx->block_read_idx];
            ctx->block_read_idx = (ctx->block_read_idx + 1u) % CHANNEL_RING_SIZE;
            ctx->block_count--;
            ctx->dropped_blocks++;
            if (g_debug)
                fprintf(stderr, "[debug] ch=%02u queue full (%u), dropping oldest (total=%lu)\n",
                        ctx->bredr_channel, CHANNEL_RING_SIZE, ctx->dropped_blocks);
            __atomic_sub_fetch(&g_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
        }
        ctx->block_idx_ring[ctx->block_write_idx] = (unsigned int)block_idx;
        ctx->block_write_idx = (ctx->block_write_idx + 1u) % CHANNEL_RING_SIZE;
        ctx->block_count++;
        __atomic_add_fetch(&g_block_pool[(unsigned int)block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
        pthread_cond_signal(&ctx->queue_cv);
        pthread_mutex_unlock(&ctx->queue_mutex);
    }

    if (!g_ble_ctx_initialized)
        return 0;

    pthread_mutex_lock(&ble_ctx.queue_mutex);
    if (ble_ctx.block_count == CHANNEL_RING_SIZE)
    {
        unsigned int old_idx = ble_ctx.block_idx_ring[ble_ctx.block_read_idx];
        ble_ctx.block_read_idx = (ble_ctx.block_read_idx + 1u) % CHANNEL_RING_SIZE;
        ble_ctx.block_count--;
        ble_ctx.dropped_blocks++;
        if (g_debug)
            fprintf(stderr, "[debug] ble ch=%02u queue full (%u), dropping oldest (total=%lu)\n",
                    BLE_CH37_INDEX, CHANNEL_RING_SIZE, ble_ctx.dropped_blocks);
        __atomic_sub_fetch(&g_block_pool[old_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    }
    ble_ctx.block_idx_ring[ble_ctx.block_write_idx] = (unsigned int)block_idx;
    ble_ctx.block_write_idx = (ble_ctx.block_write_idx + 1u) % CHANNEL_RING_SIZE;
    ble_ctx.block_count++;
    __atomic_add_fetch(&g_block_pool[(unsigned int)block_idx].refcount, 1u, __ATOMIC_ACQ_REL);
    pthread_cond_signal(&ble_ctx.queue_cv);
    pthread_mutex_unlock(&ble_ctx.queue_mutex);

    return g_stop ? -1 : 0;
}

// --- Main --------------------------------------------------------------------

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"debug", no_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "v:dh", long_opts, NULL)) != -1)
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

    printf("Supertooth Hybrid: BR/EDR ch0-19 + BLE ch37\n");
    printf("LO: %.1f MHz, %d BR/EDR channels + 1 BLE channel, 20 MHz bandwidth\n",
           LO_FREQ_HZ / 1e6, NUM_BREDR_CHANNELS);
    printf("View mode   : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Block pool  : %u blocks, per-channel queue: %u\n",
           BLOCK_POOL_SIZE, CHANNEL_RING_SIZE);

    if (setup_channel_ctx() != 0)
    {
        fprintf(stderr, "Failed to initialize channel DSP contexts.\n");
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }
    if (init_thread_pool() != 0)
    {
        fprintf(stderr, "Failed to initialize worker threads.\n");
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    signal(SIGINT, handle_sigint);

    btbb_init(2);
    btbb_init_survey();

    int result;
    hackrf_device *device = NULL;

    result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        stop_thread_pool();
        destroy_channel_ctx();
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
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, hybrid_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    printf("Receiving... Press Ctrl+C to stop.\n");
    while (!g_stop)
        sleep(1);

    hackrf_stop_rx(device);
    hackrf_disconnect(device);
    stop_thread_pool();
    unsigned long ch_drops_total = 0ul;
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
        ch_drops_total += bredr_ctx_arr[i].dropped_blocks;

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", g_output_mode == OUTPUT_MODE_SUMMARY ? "summary" : "full");
    printf("  Debug mode     : %s\n", g_debug ? "enabled" : "disabled");
    printf("  Total packets  : %lu\n", g_packet_count);
    printf("  Dropped blocks : %lu\n", g_dropped_blocks);

    printf("\n=== Debug Summary ===\n");
    printf("  BR/EDR queue drops (total): %lu\n", ch_drops_total);
    printf("  BR/EDR queue drops (per-channel):\n");
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
    {
        printf("    ch=%02u dropped=%lu\n",
               bredr_ctx_arr[i].bredr_channel,
               bredr_ctx_arr[i].dropped_blocks);
    }
    printf("  BLE queue drops (total): %lu\n", ble_ctx.dropped_blocks);
    printf("  BLE queue drops (per-channel):\n");
    printf("    ch=%02u dropped=%lu\n", BLE_CH37_INDEX, ble_ctx.dropped_blocks);

    destroy_channel_ctx();

    return 0;
}
