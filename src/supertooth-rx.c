#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <getopt.h>
#include <pthread.h>

#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>

#include "hackrf.h"
#include "bredr_phy.h"
#include "bredr_piconet.h"
#include "bredr_piconet_store.h"

/* -------------------------------------------------------------------------
 * RF / DSP constants
 * -------------------------------------------------------------------------*/

#define BREDR_CHANNEL_BW     1000000.0
#define BREDR_CHANNEL_0_FREQ 2402000000.0
#define BREDR_MAX_CHANNEL    79u

#define MAX_BREDR_CHANNELS     20u
#define DEFAULT_BREDR_CHANNELS MAX_BREDR_CHANNELS

#define SYMBOL_STEP         2u

#define BUFFER_SIZE         262144u
#define BLOCK_POOL_SIZE     64u
#define CHANNEL_RING_SIZE   8u

#define RX_CLK1600_TICKS_PER_SECOND 1600u
#define CLKN_TICKS_PER_SECOND       3200u

/* -------------------------------------------------------------------------
 * Output modes
 * -------------------------------------------------------------------------*/

typedef enum
{
    OUTPUT_MODE_FULL = 0,
    OUTPUT_MODE_SUMMARY = 1,
    OUTPUT_MODE_RSSI = 2
} output_mode_t;

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

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                    const bredr_packet_t *pkt,
                                    const bredr_piconet_t *pnet,
                                    unsigned int channel,
                                    float rssi_dbm);

typedef struct
{
    output_mode_t mode;
    const char *name;
    packet_formatter_fn fn;
} output_mode_spec_t;

/* -------------------------------------------------------------------------
 * Global runtime state
 * -------------------------------------------------------------------------*/

typedef struct
{
    unsigned int ctx_index;
    unsigned int bredr_channel;
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;
    bredr_processor_t proc;
    float complex mixed[BUFFER_SIZE];
    float complex decimated[BUFFER_SIZE];
    unsigned int block_idx_ring[CHANNEL_RING_SIZE];
    unsigned int block_write_idx;
    unsigned int block_read_idx;
    unsigned int block_count;
    unsigned long dropped_blocks;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cv;
} bredr_channel_ctx_t;

typedef struct
{
    float complex samples[BUFFER_SIZE];
    unsigned int num_samples;
    unsigned long long block_base_sample;
    unsigned int refcount;
} rx_block_t;

static bredr_channel_ctx_t g_bredr_ctx[MAX_BREDR_CHANNELS];
static bredr_piconet_store_t g_store;

static pthread_t *g_worker_threads = NULL;
static unsigned int g_worker_count = 0u;
static int g_shutdown_requested = 0;

/* Shared block pool + per-channel rings. */
static rx_block_t g_block_pool[BLOCK_POOL_SIZE];
static unsigned int g_pool_write_idx = 0u;
static unsigned long g_dropped_blocks = 0ul;

/* Packet processing and output lock (required by task request). */
static pthread_mutex_t g_packet_mutex = PTHREAD_MUTEX_INITIALIZER;

static volatile sig_atomic_t g_stop = 0;
static output_mode_t g_output_mode = OUTPUT_MODE_FULL;
static int g_debug = 0;
static int g_lap_filter_enabled = 0;
static uint32_t g_lap_filter = 0u;
static unsigned int g_rssi_averaging_window = 16u;
static unsigned int g_num_bredr_channels = DEFAULT_BREDR_CHANNELS;
static unsigned int g_bottom_bredr_channel = 0u;
static int g_bottom_channel_explicit = 0;

/* RF runtime configuration. */
static double g_bredr_channel_bw_hz = BREDR_CHANNEL_BW;
static double g_bredr_channel_0_freq_hz = BREDR_CHANNEL_0_FREQ;
static uint64_t g_lo_freq_hz = 2411500000ULL;
static uint32_t g_sample_rate = 20000000u;
static uint32_t g_lna_gain = 32u;
static uint32_t g_vga_gain = 32u;
static uint32_t g_demod_input_rate_hz = 2000000u;
static unsigned int g_decim_factor = 10u;
static unsigned int g_raw_samps_per_bit = 20u;
static unsigned int g_channel_ctx_count = 0u;

/* Sample-clock tracking. */
static unsigned long long g_samples_received = 0ULL;

/* Counters. */
static unsigned long long g_total_bits = 0ULL;
static unsigned long g_total_packets = 0UL;
static unsigned long g_header_packets = 0UL;
static unsigned long g_id_packets = 0UL;

static const char *const s_bredr_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"
};

/* -------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------*/

static void handle_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
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

static int piconet_has_active_track(const bredr_piconet_t *pnet)
{
    return pnet && pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0;
}

static void format_piconet_id(char out[16],
                              const bredr_packet_t *pkt,
                              const bredr_piconet_t *pnet)
{
    uint32_t lap = pkt ? (pkt->lap & 0xFFFFFFu) : 0u;
    if (pnet)
        lap = pnet->lap & 0xFFFFFFu;

    if (pnet && pnet->uap_found)
        snprintf(out, 16, "0x%02X%06" PRIX32, pnet->uap, lap);
    else
        snprintf(out, 16, "0x??%06" PRIX32, lap);
}

static void format_rssi_value(char out[8], int seen, float value)
{
    if (seen)
        snprintf(out, 8, "%6.1f", value);
    else
        snprintf(out, 8, "  --.-");
}

static int piconet_lap_cmp(const void *a, const void *b)
{
    const bredr_piconet_t *pa = *(const bredr_piconet_t *const *)a;
    const bredr_piconet_t *pb = *(const bredr_piconet_t *const *)b;
    uint32_t la = pa ? (pa->lap & 0xFFFFFFu) : 0u;
    uint32_t lb = pb ? (pb->lap & 0xFFFFFFu) : 0u;
    if (la < lb)
        return -1;
    if (la > lb)
        return 1;
    return 0;
}

static uint32_t raw_samples_to_rx_clk_1600(unsigned long long raw_sample_index)
{
    /* Rounded conversion: rx_clk_1600 = t * 1600 Hz. */
    unsigned long long num =
        raw_sample_index * (unsigned long long)RX_CLK1600_TICKS_PER_SECOND +
        (unsigned long long)(g_sample_rate / 2u);
    return (uint32_t)(num / (unsigned long long)g_sample_rate);
}

static uint32_t raw_samples_to_clkn(unsigned long long raw_sample_index)
{
    /* Rounded conversion: clkn = t * 3200 Hz (half-slot ticks). */
    unsigned long long num =
        raw_sample_index * (unsigned long long)CLKN_TICKS_PER_SECOND +
        (unsigned long long)(g_sample_rate / 2u);
    return (uint32_t)(num / (unsigned long long)g_sample_rate);
}

static int decode_header_with_clock(const bredr_packet_t *pkt,
                                    uint8_t uap,
                                    uint8_t clk6,
                                    bredr_decoded_header_t *out)
{
    if (!pkt || !pkt->has_header || !out)
        return 0;

    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3Fu), bits);

    out->lt_addr = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
    out->type    = (bits[3]) | (uint8_t)(bits[4] << 1) | (uint8_t)(bits[5] << 2)
                              | (uint8_t)(bits[6] << 3);
    out->flow    = bits[7];
    out->arqn    = bits[8];
    out->seqn    = bits[9];

    out->hec = 0u;
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
    if (!pkt || pkt->payload_bytes == 0u)
    {
        printf("Payload      : (none)\n");
        return;
    }

    unsigned int show = pkt->payload_bytes < 32u ? pkt->payload_bytes : 32u;
    printf("Payload      : %u bytes", pkt->payload_bytes);
    for (unsigned int i = 0; i < show; i++)
    {
        if (i % 16u == 0u)
            printf("\n               ");
        printf("%02X ", pkt->payload[i]);
    }
    if (pkt->payload_bytes > show)
        printf("...");
    printf("\n");
}

static void print_packet_full(unsigned long packet_no,
                              const bredr_packet_t *pkt,
                              const bredr_piconet_t *pnet,
                              unsigned int channel,
                              float rssi_dbm)
{
    if (!pkt)
        return;

    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           pkt->rx_clk_ref, g_sample_rate / 1000000u);
    printf("Type         : BR/EDR\n");
    printf("Frequency    : %u MHz (Channel %u)\n",
           (unsigned int)(g_bredr_channel_0_freq_hz / 1e6) + channel, channel);
    printf("RSSI         : %.2f dBm\n", rssi_dbm);

    printf("\n[%s Packet Info]\n",
           pkt->has_header ? "BR/EDR Data" : "BR/EDR Inquiry");
    printf("LAP          : 0x%06" PRIX32 "\n", pkt->lap & 0xFFFFFFu);
    if (pkt->has_header)
        printf("HEADER       : 0x%014" PRIX64 "\n",
               pkt->header_raw & 0x003FFFFFFFFFFFFFull);
    else
        printf("HEADER       : (none — shortened access code)\n");

    if (pkt->has_header)
    {
        bredr_decoded_header_t decoded = {0};
        int decoded_ok = 0;
        if (pnet && pnet->clk_known && pnet->uap_found)
            decoded_ok = decode_header_with_clock(pkt, pnet->uap,
                                                  pnet->central_clk_1_6, &decoded);

        if (decoded_ok)
        {
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

    if (pkt->has_header && pnet)
    {
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

static void print_packet_summary(unsigned long packet_no,
                                 const bredr_packet_t *pkt,
                                 const bredr_piconet_t *pnet,
                                 unsigned int channel,
                                 float rssi_dbm)
{
    if (!pkt)
        return;

    if (pkt->has_header)
    {
        char uap_buf[8];
        char clk_buf[8];
        if (pnet && pnet->uap_found)
            snprintf(uap_buf, sizeof(uap_buf), "%02X", pnet->uap);
        else
            snprintf(uap_buf, sizeof(uap_buf), "??");
        if (pnet && pnet->clk_known)
            snprintf(clk_buf, sizeof(clk_buf), "%02u", pnet->central_clk_1_6);
        else
            snprintf(clk_buf, sizeof(clk_buf), "??");

        printf("pkt=%-6lu lap=%06" PRIX32 " uap=%s ch=%02u ac=%u clk=%s track=%d rssi=%.1f\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               uap_buf,
               channel,
               pkt->ac_errors,
               clk_buf,
               pnet ? pnet->tracking_state : -1,
               rssi_dbm);
    }
    else
    {
        printf("pkt=%-6lu lap=%06" PRIX32 " uap=?? ch=%02u ac=%u clk=?? track=%d rssi=%.1f\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               channel,
               pkt->ac_errors,
               pnet ? pnet->tracking_state : -1,
               rssi_dbm);
    }
}

static void print_packet_rssi(unsigned long packet_no,
                              const bredr_packet_t *pkt,
                              const bredr_piconet_t *pnet,
                              unsigned int channel,
                              float rssi_dbm)
{
    (void)pnet;
    (void)channel;
    (void)rssi_dbm;
    if (!pkt)
        return;

    size_t count = bredr_piconet_store_count(&g_store);
    const bredr_piconet_t **ordered =
        (const bredr_piconet_t **)malloc(sizeof(*ordered) * (count > 0u ? count : 1u));
    if (!ordered)
        return;

    size_t used = 0u;
    for (size_t i = 0; i < count; i++)
    {
        const bredr_piconet_t *cur = bredr_piconet_store_get(&g_store, i);
        if (cur)
            ordered[used++] = cur;
    }
    qsort(ordered, used, sizeof(*ordered), piconet_lap_cmp);

    printf("\n================ RSSI Snapshot (Packet #%lu) ================\n", packet_no);
    printf("Sample Index : %" PRIu64 " (%u Msps master clock)\n",
           pkt->rx_clk_ref, g_sample_rate / 1000000u);
    printf("Piconets     : %zu\n", used);
    printf("--------------------------------------------------------------\n");

    for (size_t i = 0; i < used; i++)
    {
        const bredr_piconet_t *cur = ordered[i];
        char piconet_id[16];
        format_piconet_id(piconet_id, pkt, cur);

        if (!piconet_has_active_track(cur))
        {
            char combined_buf[8];
            format_rssi_value(combined_buf, cur->combined_rssi_seen, cur->combined_rssi);
            printf("Piconet: %-10s | Track: %2d | Combined: %s dBm\n",
                   piconet_id,
                   cur->tracking_state,
                   combined_buf);
            continue;
        }

        char central_buf[8];
        format_rssi_value(central_buf, cur->master_rssi_seen, cur->master_rssi);
        printf("Piconet: %-10s | Track: %2d | Central: %s dBm",
               piconet_id,
               cur->tracking_state,
               central_buf);

        int periph_seen = 0;
        for (int lt = 1; lt <= 7; lt++)
        {
            if (!cur->slave_rssi_seen[lt])
                continue;
            periph_seen = 1;
            char pbuf[8];
            format_rssi_value(pbuf, 1, cur->slave_rssi[lt]);
            printf(" | Periph[%d]: %s dBm", lt, pbuf);
        }
        if (!periph_seen)
            printf(" | Periph: (none yet)");
        printf("\n");
    }

    printf("==============================================================\n");
    free(ordered);
}

static const output_mode_spec_t s_output_modes[] = {
    {OUTPUT_MODE_FULL,      "full",      print_packet_full},
    {OUTPUT_MODE_SUMMARY,   "summary",   print_packet_summary},
    {OUTPUT_MODE_RSSI,      "rssi",      print_packet_rssi},
};

static const output_mode_spec_t *output_mode_spec(output_mode_t mode)
{
    for (size_t i = 0; i < sizeof(s_output_modes) / sizeof(s_output_modes[0]); i++)
    {
        if (s_output_modes[i].mode == mode)
            return &s_output_modes[i];
    }
    return &s_output_modes[0];
}

static int parse_output_mode(const char *arg, output_mode_t *out_mode)
{
    if (!arg || !out_mode)
        return -1;
    for (size_t i = 0; i < sizeof(s_output_modes) / sizeof(s_output_modes[0]); i++)
    {
        if (strcmp(arg, s_output_modes[i].name) == 0)
        {
            *out_mode = s_output_modes[i].mode;
            return 0;
        }
    }
    if (strcmp(arg, "ubertooth") == 0)
    {
        *out_mode = OUTPUT_MODE_SUMMARY;
        return 0;
    }
    return -1;
}

static int parse_lap_filter(const char *arg, uint32_t *out_lap)
{
    if (!arg || !out_lap)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 0xFFFFFFul)
        return -1;

    *out_lap = (uint32_t)value;
    return 0;
}

static int parse_rssi_averaging(const char *arg, unsigned int *out_window)
{
    if (!arg || !out_window)
        return -1;
    if (strcmp(arg, "none") == 0)
    {
        *out_window = 0u;
        return 0;
    }

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value > 1000000ul)
        return -1;

    *out_window = (unsigned int)value;
    return 0;
}

static int parse_channel_count(const char *arg, unsigned int *out_channels)
{
    if (!arg || !out_channels)
        return -1;

    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        value < 2ul || value > (unsigned long)MAX_BREDR_CHANNELS ||
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

static void update_receiver_layout(unsigned int channel_count)
{
    /* Configure channels [bottom, bottom+channel_count-1] and center LO on that span. */
    g_num_bredr_channels = channel_count;
    g_sample_rate = (uint32_t)(channel_count * (unsigned int)BREDR_CHANNEL_BW);
    if (channel_count == 2u)
        g_sample_rate = 4000000u; /* Keep FIR decimation active: 4 Msps -> /2 -> 2 Msps demod input. */
    g_decim_factor = (unsigned int)(g_sample_rate / g_demod_input_rate_hz);
    g_raw_samps_per_bit = g_decim_factor * SYMBOL_STEP;

    double lowest_ctx_freq_offset =
        -(channel_count / 2.0 - 0.5) * g_bredr_channel_bw_hz;
    double lowest_channel_freq_hz =
        g_bredr_channel_0_freq_hz + g_bottom_bredr_channel * g_bredr_channel_bw_hz;
    double lo_hz = lowest_channel_freq_hz - lowest_ctx_freq_offset;
    g_lo_freq_hz = (uint64_t)llround(lo_hz);
}

static void print_usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [-v|--view full|summary|rssi] [-l|--lap LAP] "
            "[--rssi-averaging N|none] [-c|--channels N] [-b|--bottom-channel CH] [-d|--debug]\n", argv0);
    fprintf(stderr, "  %-30s Packet view style (default: full)\n", "-v, --view");
    fprintf(stderr, "  %-30s Only track/report this LAP (e.g. 0x1FC475)\n", "-l, --lap LAP");
    fprintf(stderr, "  %-30s EMA window for piconet RSSI (default: 16; 0/none disables)\n",
            "--rssi-averaging N|none");
    fprintf(stderr, "  %-30s Number of BR/EDR channels from bottom (even 2-%u, default: %u)\n",
            "-c, --channels N",
            MAX_BREDR_CHANNELS, DEFAULT_BREDR_CHANNELS);
    fprintf(stderr, "  %-30s Lowest BR/EDR channel to process (0-%u, default: 0)\n",
            "-b, --bottom-channel CH",
            BREDR_MAX_CHANNEL);
    fprintf(stderr, "  %-30s Print drop/debug diagnostics\n", "-d, --debug");
}

/* -------------------------------------------------------------------------
 * Channel / thread setup
 * -------------------------------------------------------------------------*/

static int setup_channel_ctx(void)
{
    g_channel_ctx_count = 0u;
    float lowest_ctx_freq_offset =
        -(g_num_bredr_channels / 2.0f - 0.5f) * (float)g_bredr_channel_bw_hz;

    for (unsigned int i = 0; i < g_num_bredr_channels; i++)
    {
        bredr_channel_ctx_t *ctx = &g_bredr_ctx[i];
        ctx->ctx_index = i;
        ctx->bredr_channel = g_bottom_bredr_channel + i;

        float channel_offset_freq =
            (float)i * (float)g_bredr_channel_bw_hz + lowest_ctx_freq_offset;
        float normalized_freq = 2.0f * (float)M_PI * channel_offset_freq / (float)g_sample_rate;

        ctx->nco = nco_crcf_create(LIQUID_NCO);
        ctx->firdec = firdecim_crcf_create_kaiser(g_decim_factor, 7, 60.0f);
        ctx->demod = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.5f,
                                     LIQUID_CPFSK_GMSK);
        if (!ctx->nco || !ctx->firdec || !ctx->demod)
            return -1;
        nco_crcf_set_frequency(ctx->nco, normalized_freq);

        bredr_processor_init(&ctx->proc, BREDR_AC_ERRORS_DEFAULT);
        ctx->block_write_idx = 0u;
        ctx->block_read_idx = 0u;
        ctx->block_count = 0u;
        ctx->dropped_blocks = 0ul;
        if (pthread_mutex_init(&ctx->queue_mutex, NULL) != 0)
            return -1;
        if (pthread_cond_init(&ctx->queue_cv, NULL) != 0)
        {
            pthread_mutex_destroy(&ctx->queue_mutex);
            return -1;
        }
        g_channel_ctx_count++;
    }
    return 0;
}

static void destroy_channel_ctx(void)
{
    for (unsigned int i = 0; i < g_channel_ctx_count; i++)
    {
        if (g_bredr_ctx[i].demod)
        {
            cpfskdem_destroy(g_bredr_ctx[i].demod);
            g_bredr_ctx[i].demod = NULL;
        }
        if (g_bredr_ctx[i].firdec)
        {
            firdecim_crcf_destroy(g_bredr_ctx[i].firdec);
            g_bredr_ctx[i].firdec = NULL;
        }
        if (g_bredr_ctx[i].nco)
        {
            nco_crcf_destroy(g_bredr_ctx[i].nco);
            g_bredr_ctx[i].nco = NULL;
        }
        pthread_cond_destroy(&g_bredr_ctx[i].queue_cv);
        pthread_mutex_destroy(&g_bredr_ctx[i].queue_mutex);
    }
    g_channel_ctx_count = 0u;
}

/* -------------------------------------------------------------------------
 * Packet processing
 * -------------------------------------------------------------------------*/

static void process_bredr_channel(bredr_channel_ctx_t *ctx, const rx_block_t *blk)
{
    nco_crcf_mix_block_down(ctx->nco, blk->samples, ctx->mixed, blk->num_samples);

    unsigned int decimated_samples = blk->num_samples / g_decim_factor;
    firdecim_crcf_execute_block(ctx->firdec, ctx->mixed, decimated_samples, ctx->decimated);

    unsigned long long local_bits = 0ULL;

    for (unsigned int i = 0; i + SYMBOL_STEP <= decimated_samples; i += SYMBOL_STEP)
    {
        unsigned int raw_sym = cpfskdem_demodulate(ctx->demod, &ctx->decimated[i]);
        uint8_t bit = (uint8_t)(raw_sym & 1u);

        bredr_status_t s = bredr_push_bit_and_samples(&ctx->proc, bit,
                                                      ctx->decimated[i],
                                                      ctx->decimated[i + 1u]);
        local_bits++;

        if (s != BREDR_VALID_PACKET)
            continue;

        bredr_packet_t pkt;
        if (bredr_get_packet(&ctx->proc, &pkt) != 0)
            continue;
        if (g_lap_filter_enabled && ((pkt.lap & 0xFFFFFFu) != g_lap_filter))
            continue;

        /* Timestamp at access-code detection point */
        unsigned long long bit_in_block = (unsigned long long)(i / SYMBOL_STEP);
        unsigned long long bits_back = pkt.has_header
            ? (58ULL + (unsigned long long)pkt.payload_bytes * 8ULL)
            : 0ULL;
        unsigned long long ac_bit_in_block = (bit_in_block >= bits_back)
            ? (bit_in_block - bits_back)
            : 0ULL;
        unsigned long long abs_raw =
            blk->block_base_sample + ac_bit_in_block * g_raw_samps_per_bit;
        uint32_t clkn = raw_samples_to_clkn(abs_raw);
        pkt.rx_clk_ref = abs_raw;
        pkt.rx_clk_1600 = raw_samples_to_rx_clk_1600(abs_raw);

        // RSSI is computed by the processor at AC detection time (pkt.rssi).

        pthread_mutex_lock(&g_packet_mutex);
        bredr_piconet_t *pnet =
            bredr_piconet_store_add_packet(&g_store, &pkt, (int)ctx->bredr_channel, clkn);

        g_total_packets++;
        if (pkt.has_header)
            g_header_packets++;
        else
            g_id_packets++;

        const output_mode_spec_t *spec = output_mode_spec(g_output_mode);
        spec->fn(g_total_packets, &pkt, pnet, ctx->bredr_channel, pkt.rssi);
        fflush(stdout);
        pthread_mutex_unlock(&g_packet_mutex);
    }

    __atomic_add_fetch(&g_total_bits, local_bits, __ATOMIC_RELAXED);
}

static void *bredr_channel_worker(void *arg)
{
    bredr_channel_ctx_t *ctx = (bredr_channel_ctx_t *)arg;
    for (;;)
    {
        pthread_mutex_lock(&ctx->queue_mutex);
        while (!g_shutdown_requested && ctx->block_count == 0u)
            pthread_cond_wait(&ctx->queue_cv, &ctx->queue_mutex);
        if (g_shutdown_requested)
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

static int init_thread_pool(void)
{
    g_shutdown_requested = 0;
    g_worker_count = g_num_bredr_channels;
    g_worker_threads = (pthread_t *)calloc(g_worker_count, sizeof(pthread_t));
    if (!g_worker_threads)
        return -1;

    unsigned int created = 0u;
    for (unsigned int i = 0; i < g_worker_count; i++)
    {
        if (pthread_create(&g_worker_threads[i], NULL, bredr_channel_worker,
                           &g_bredr_ctx[i]) != 0)
        {
            g_worker_count = created;
            return -1;
        }
        created++;
    }
    return 0;
}

static void stop_thread_pool(void)
{
    g_shutdown_requested = 1;
    for (unsigned int i = 0; i < g_worker_count; i++)
    {
        pthread_mutex_lock(&g_bredr_ctx[i].queue_mutex);
        pthread_cond_signal(&g_bredr_ctx[i].queue_cv);
        pthread_mutex_unlock(&g_bredr_ctx[i].queue_mutex);
    }

    for (unsigned int i = 0; i < g_worker_count; i++)
        pthread_join(g_worker_threads[i], NULL);

    free(g_worker_threads);
    g_worker_threads = NULL;
    g_worker_count = 0u;
}

/* -------------------------------------------------------------------------
 * HackRF callback
 * -------------------------------------------------------------------------*/

static int bredr_cb(hackrf_transfer *transfer)
{
    if (g_stop)
        return -1;

    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > BUFFER_SIZE)
        num_samples = BUFFER_SIZE;

    unsigned long long block_base = g_samples_received;
    g_samples_received += num_samples;

    /* Find a free block slot; drop this callback block if all are in use. */
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
        float i_sample = samples[2u * i] / 128.0f;
        float q_sample = samples[2u * i + 1u] / 128.0f;
        blk->samples[i] = i_sample + q_sample * _Complex_I;
    }

    g_pool_write_idx = ((unsigned int)block_idx + 1u) % BLOCK_POOL_SIZE;
    __atomic_thread_fence(__ATOMIC_RELEASE);

    for (unsigned int ch = 0; ch < g_num_bredr_channels; ch++)
    {
        bredr_channel_ctx_t *ctx = &g_bredr_ctx[ch];
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

    return g_stop ? -1 : 0;
}

/* -------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view",           required_argument, NULL, 'v'},
        {"lap",            required_argument, NULL, 'l'},
        {"rssi-averaging", required_argument, NULL, 'a'},
        {"channels",       required_argument, NULL, 'c'},
        {"bottom-channel", required_argument, NULL, 'b'},
        {"debug",          no_argument,       NULL, 'd'},
        {"help",           no_argument,       NULL, 'h'},
        {NULL,             0,                 NULL,  0 }
    };

    update_receiver_layout(g_num_bredr_channels);

    int opt;
    while ((opt = getopt_long(argc, argv, "v:l:a:c:b:dh", long_opts, NULL)) != -1)
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
            case 'l':
                if (parse_lap_filter(optarg, &g_lap_filter) != 0)
                {
                    fprintf(stderr, "Invalid LAP: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                g_lap_filter_enabled = 1;
                break;
            case 'a':
                if (parse_rssi_averaging(optarg, &g_rssi_averaging_window) != 0)
                {
                    fprintf(stderr, "Invalid --rssi-averaging value: %s\n", optarg);
                    print_usage(argv[0]);
                    return EXIT_FAILURE;
                }
                break;
            case 'c':
                if (parse_channel_count(optarg, &g_num_bredr_channels) != 0)
                {
                    fprintf(stderr, "Invalid --channels value: %s (expected even 2-%u)\n",
                            optarg, MAX_BREDR_CHANNELS);
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
    if (optind != argc)
    {
        print_usage(argv[0]);
        return EXIT_FAILURE;
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

    update_receiver_layout(g_num_bredr_channels);

    const output_mode_spec_t *mode_spec = output_mode_spec(g_output_mode);
    printf("Supertooth RX (BR/EDR)\n");
    printf("======================\n");
    printf("LO          : %.3f MHz\n", (double)g_lo_freq_hz / 1e6);
    printf("Sample rate : %u Msps\n", g_sample_rate / 1000000u);
    printf("Decimation  : /%u -> %u Msps demod input\n",
           g_decim_factor, g_demod_input_rate_hz / 1000000u);
    printf("Channels    : %u (%u..%u)\n", g_num_bredr_channels,
           g_bottom_bredr_channel, g_bottom_bredr_channel + g_num_bredr_channels - 1u);
    printf("View mode   : %s\n", mode_spec->name);
    if (g_lap_filter_enabled)
        printf("LAP filter  : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("LAP filter  : (none)\n");
    if (g_rssi_averaging_window == 0u)
        printf("RSSI EMA    : disabled\n");
    else
        printf("RSSI EMA    : window=%u\n", g_rssi_averaging_window);
    printf("Block pool  : %u blocks, per-channel queue: %u\n",
           BLOCK_POOL_SIZE, CHANNEL_RING_SIZE);
    printf("Debug       : %s\n", g_debug ? "enabled" : "disabled");
    printf("Press Ctrl+C to stop.\n\n");

    if (setup_channel_ctx() != 0)
    {
        fprintf(stderr, "Failed to initialize BR/EDR channel DSP contexts.\n");
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
    bredr_piconet_set_rssi_averaging(g_rssi_averaging_window);
    bredr_piconet_store_init(&g_store);
    signal(SIGINT, handle_sigint);

    hackrf_device *device = NULL;
    int result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        bredr_piconet_store_free(&g_store);
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    hackrf_config_t config = {
        .lo_freq_hz  = g_lo_freq_hz,
        .sample_rate = g_sample_rate,
        .lna_gain    = g_lna_gain,
        .vga_gain    = g_vga_gain,
    };
    result = hackrf_configure(device, &config);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        bredr_piconet_store_free(&g_store);
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, bredr_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        bredr_piconet_store_free(&g_store);
        stop_thread_pool();
        destroy_channel_ctx();
        return EXIT_FAILURE;
    }

    while (!g_stop)
        sleep(1);

    hackrf_stop_rx(device);
    hackrf_disconnect(device);

    stop_thread_pool();
    destroy_channel_ctx();

    printf("\n\n=== Session Summary ===\n");
    printf("  Output mode    : %s\n", mode_spec->name);
    if (g_lap_filter_enabled)
        printf("  LAP filter     : %06" PRIX32 "\n", g_lap_filter);
    else
        printf("  LAP filter     : (none)\n");
    if (g_rssi_averaging_window == 0u)
        printf("  RSSI EMA       : disabled\n");
    else
    printf("  RSSI EMA       : window=%u\n", g_rssi_averaging_window);
    printf("  Total bits     : %" PRIu64 "\n", g_total_bits);
    printf("  Header packets : %lu\n", g_header_packets);
    printf("  ID packets     : %lu\n", g_id_packets);
    printf("  Dropped blocks : %lu\n", g_dropped_blocks);
    if (g_debug)
    {
        unsigned long ch_drops_total = 0ul;
        for (unsigned int i = 0; i < g_num_bredr_channels; i++)
            ch_drops_total += g_bredr_ctx[i].dropped_blocks;
        printf("  Channel queue drops (total): %lu\n", ch_drops_total);
        for (unsigned int i = 0; i < g_num_bredr_channels; i++)
        {
            if (g_bredr_ctx[i].dropped_blocks > 0ul)
                printf("    ch=%02u dropped=%lu\n",
                       g_bredr_ctx[i].bredr_channel,
                       g_bredr_ctx[i].dropped_blocks);
        }
    }
    printf("\n");
    bredr_piconet_store_print(&g_store);
    bredr_piconet_store_free(&g_store);

    return EXIT_SUCCESS;
}
