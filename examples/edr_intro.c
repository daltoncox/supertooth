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

#include "../src/hackrf.h"
#include "bredr_phy.h"
#include "bredr_piconet.h"
#include "bredr_piconet_store.h"

/* -------------------------------------------------------------------------
 * RF / DSP constants
 * -------------------------------------------------------------------------*/

#define BREDR_CHANNEL_BW     1000000.0
#define BREDR_CHANNEL_0_FREQ 2402000000.0

/* 20-channel BR/EDR layout (channels 0..19) with no BR/EDR channel at DC. */
#define LO_FREQ_HZ          2411500000ULL
#define SAMPLE_RATE         20000000u
#define LNA_GAIN            32u
#define VGA_GAIN            32u
#define NUM_BREDR_CHANNELS  20u

#define DECIM_FACTOR        10u
#define SYMBOL_STEP         2u

#define BUFFER_SIZE         262144u
#define CHANNEL_BUFFER_SIZE (BUFFER_SIZE / DECIM_FACTOR)

#define RAW_SAMPS_PER_BIT   (DECIM_FACTOR * SYMBOL_STEP) /* 20 samples/bit */
#define BITS_PER_RX_CLK1600_TICK 625.0

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

typedef enum
{
    EDR_INTRO_MODE_NONE = 0,
    EDR_INTRO_MODE_2M   = 2,
    EDR_INTRO_MODE_3M   = 3
} edr_intro_mode_t;

typedef struct
{
    uint8_t bytes[BREDR_MAX_PAYLOAD_BYTES];
    unsigned int payload_bytes;
} edr_intro_br_payload_t;

typedef struct
{
    edr_intro_mode_t mode;
    unsigned int bit_count;
    uint8_t bits[(BREDR_SYMBOLS_MAX + 7u) / 8u];
} edr_intro_edr_payload_t;

typedef struct
{
    const bredr_packet_t *base_pkt;
    uint8_t is_edr;
    void *payload;
} edr_intro_packet_t;

typedef void (*packet_formatter_fn)(unsigned long packet_no,
                                    const edr_intro_packet_t *intro_pkt,
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
    cpfskdem edr2_dem;
    cpfskdem edr3_dem;
    bredr_processor_t proc;
    float complex mixed[BUFFER_SIZE];
    float complex decimated[CHANNEL_BUFFER_SIZE];
    int last_gen_processed;
} bredr_channel_ctx_t;

static float complex g_raw[BUFFER_SIZE];
static unsigned int  g_num_samples = 0u;

static bredr_channel_ctx_t g_bredr_ctx[NUM_BREDR_CHANNELS];
static bredr_piconet_store_t g_store;

static pthread_t *g_worker_threads = NULL;
static unsigned int g_worker_count = 0u;

/* Work scheduling primitives. */
static pthread_mutex_t g_work_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_work_cv = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  g_done_cv = PTHREAD_COND_INITIALIZER;
static unsigned int g_dispatch_count = 0u;
static unsigned int g_complete_count = 0u;
static int g_work_generation = 0;
static int g_shutdown_requested = 0;

/* Packet processing and output lock (required by task request). */
static pthread_mutex_t g_packet_mutex = PTHREAD_MUTEX_INITIALIZER;

static volatile sig_atomic_t g_stop = 0;
static output_mode_t g_output_mode = OUTPUT_MODE_FULL;

/* Sample-clock tracking. */
static unsigned long long g_samples_received = 0ULL;
static unsigned long long g_block_base_sample = 0ULL;

/* Counters. */
static unsigned long long g_total_bits = 0ULL;
static unsigned long g_total_packets = 0UL;
static unsigned long g_header_packets = 0UL;
static unsigned long g_id_packets = 0UL;
static unsigned long g_edr_packets = 0UL;

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

static edr_intro_mode_t choose_edr_mode(const bredr_decoded_header_t *decoded,
                                        int decoded_ok,
                                        const bredr_packet_t *pkt)
{
    if (!pkt || !pkt->has_header || pkt->payload_bytes == 0u)
        return EDR_INTRO_MODE_NONE;

    if (!decoded_ok)
        return EDR_INTRO_MODE_NONE;

    /* Heuristic starter mapping for intro/demo purposes. */
    if (decoded->type == 11u || decoded->type == 13u)
        return EDR_INTRO_MODE_2M;
    if (decoded->type == 15u)
        return EDR_INTRO_MODE_3M;
    return EDR_INTRO_MODE_NONE;
}

static void push_bit_to_buffer(uint8_t *buf, unsigned int bit_pos, uint8_t bit)
{
    unsigned int byte_pos = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    if (bit)
        buf[byte_pos] |= (uint8_t)(1u << bit_idx);
}

static void demod_edr_payload_bits(bredr_channel_ctx_t *ctx,
                                   unsigned int payload_start_bit,
                                   unsigned int payload_bits,
                                   edr_intro_mode_t mode,
                                   edr_intro_edr_payload_t *out)
{
    memset(out, 0, sizeof(*out));
    out->mode = mode;
    out->bit_count = payload_bits;

    cpfskdem dem = (mode == EDR_INTRO_MODE_3M) ? ctx->edr3_dem : ctx->edr2_dem;
    for (unsigned int bit_i = 0u; bit_i < payload_bits; bit_i++)
    {
        unsigned int sample_idx = (payload_start_bit + bit_i) * SYMBOL_STEP;
        if (sample_idx + SYMBOL_STEP > CHANNEL_BUFFER_SIZE)
            break;
        uint8_t b = (uint8_t)(cpfskdem_demodulate(dem, &ctx->decimated[sample_idx]) & 1u);
        push_bit_to_buffer(out->bits, bit_i, b);
    }
}

static void print_payload_preview(const edr_intro_packet_t *intro_pkt,
                                  const bredr_packet_t *pkt)
{
    if (!pkt || !intro_pkt)
        return;

    if (!intro_pkt->is_edr)
    {
        const edr_intro_br_payload_t *br = (const edr_intro_br_payload_t *)intro_pkt->payload;
        if (!br || br->payload_bytes == 0u)
        {
            printf("Payload      : (none)\n");
            return;
        }
        unsigned int show = br->payload_bytes < 32u ? br->payload_bytes : 32u;
        printf("Payload      : %u bytes (BR)", br->payload_bytes);
        for (unsigned int i = 0; i < show; i++)
        {
            if (i % 16u == 0u)
                printf("\n               ");
            printf("%02X ", br->bytes[i]);
        }
        if (br->payload_bytes > show)
            printf("...");
        printf("\n");
        return;
    }

    const edr_intro_edr_payload_t *edr = (const edr_intro_edr_payload_t *)intro_pkt->payload;
    if (!edr || edr->bit_count == 0u)
    {
        printf("Payload      : (none)\n");
        return;
    }
    unsigned int payload_bytes = edr->bit_count / 8u;
    unsigned int show = payload_bytes < 32u ? payload_bytes : 32u;
    printf("Payload      : %u bits (EDR-%u)", edr->bit_count, (unsigned int)edr->mode);
    for (unsigned int i = 0; i < show; i++)
    {
        if (i % 16u == 0u)
            printf("\n               ");
        printf("%02X ", edr->bits[i]);
    }
    if (payload_bytes > show)
        printf("...");
    printf("\n");
}

static void print_packet_full(unsigned long packet_no,
                              const edr_intro_packet_t *intro_pkt,
                              const bredr_packet_t *pkt,
                              const bredr_piconet_t *pnet,
                              unsigned int channel,
                              float rssi_dbm)
{
    if (!pkt)
        return;

    printf("\n------------------ Packet #%lu --------------------\n", packet_no);
    printf("[RX Info]\n");
    printf("Sample Index : %" PRIu64 " (20 Msps master clock)\n", pkt->rx_clk_ref);
    printf("Type         : BR/EDR\n");
    printf("Air mode     : %s\n", intro_pkt && intro_pkt->is_edr ? "EDR" : "BR");
    printf("Frequency    : %u MHz (Channel %u)\n", 2402u + channel, channel);
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

        print_payload_preview(intro_pkt, pkt);
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
                                 const edr_intro_packet_t *intro_pkt,
                                 const bredr_packet_t *pkt,
                                 const bredr_piconet_t *pnet,
                                 unsigned int channel,
                                 float rssi_dbm)
{
    if (!pkt)
        return;
    const char *air_mode = (intro_pkt && intro_pkt->is_edr) ? "EDR" : "BR";

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

        printf("pkt=%-6lu lap=%06" PRIX32 " uap=%s ch=%02u ac=%u clk=%s track=%d rssi=%.1f mode=%s\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               uap_buf,
               channel,
               pkt->ac_errors,
               clk_buf,
               pnet ? pnet->tracking_state : -1,
               rssi_dbm,
               air_mode);
    }
    else
    {
        printf("pkt=%-6lu lap=%06" PRIX32 " uap=?? ch=%02u ac=%u clk=?? track=%d rssi=%.1f mode=%s\n",
               packet_no,
               pkt->lap & 0xFFFFFFu,
               channel,
               pkt->ac_errors,
               pnet ? pnet->tracking_state : -1,
               rssi_dbm,
               air_mode);
    }
}

static void print_packet_rssi(unsigned long packet_no,
                              const edr_intro_packet_t *intro_pkt,
                              const bredr_packet_t *pkt,
                              const bredr_piconet_t *pnet,
                              unsigned int channel,
                              float rssi_dbm)
{
    (void)intro_pkt;
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
    printf("Sample Index : %" PRIu64 " (20 Msps master clock)\n", pkt->rx_clk_ref);
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

static void print_usage(const char *argv0)
{
    fprintf(stderr, "Usage: %s [-v|--view full|summary|rssi]\n", argv0);
    fprintf(stderr, "  -v, --view       Packet view style (default: full)\n");
    fprintf(stderr, "                    (EDR demods engage only after a packet is captured)\n");
}

/* -------------------------------------------------------------------------
 * Channel / thread setup
 * -------------------------------------------------------------------------*/

static int setup_channel_ctx(void)
{
    float lowest_ctx_freq_offset =
        -(NUM_BREDR_CHANNELS / 2.0f - 0.5f) * (float)BREDR_CHANNEL_BW; /* -9.5 MHz */

    int lowest_ctx_bt_channel = (int)(((double)LO_FREQ_HZ +
                                        (double)lowest_ctx_freq_offset -
                                        BREDR_CHANNEL_0_FREQ) / BREDR_CHANNEL_BW);

    for (unsigned int i = 0; i < NUM_BREDR_CHANNELS; i++)
    {
        bredr_channel_ctx_t *ctx = &g_bredr_ctx[i];
        ctx->ctx_index = i;
        ctx->bredr_channel = (unsigned int)(lowest_ctx_bt_channel + (int)i);

        float channel_offset_freq =
            (float)i * (float)BREDR_CHANNEL_BW + lowest_ctx_freq_offset;
        float normalized_freq = 2.0f * (float)M_PI * channel_offset_freq / (float)SAMPLE_RATE;

        ctx->nco = nco_crcf_create(LIQUID_NCO);
        ctx->firdec = firdecim_crcf_create_kaiser(DECIM_FACTOR, 7, 60.0f);
        ctx->demod = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.5f,
                                     LIQUID_CPFSK_GMSK);
        ctx->edr2_dem = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.35f,
                                        LIQUID_CPFSK_GMSK);
        ctx->edr3_dem = cpfskdem_create(1, 0.3f, SYMBOL_STEP, 3, 0.20f,
                                        LIQUID_CPFSK_GMSK);
        if (!ctx->nco || !ctx->firdec || !ctx->demod || !ctx->edr2_dem || !ctx->edr3_dem)
            return -1;
        nco_crcf_set_frequency(ctx->nco, normalized_freq);

        bredr_processor_init(&ctx->proc, BREDR_AC_ERRORS_DEFAULT);
        ctx->last_gen_processed = -1;
    }
    return 0;
}

static void destroy_channel_ctx(void)
{
    for (unsigned int i = 0; i < NUM_BREDR_CHANNELS; i++)
    {
        if (g_bredr_ctx[i].edr3_dem)
            cpfskdem_destroy(g_bredr_ctx[i].edr3_dem);
        if (g_bredr_ctx[i].edr2_dem)
            cpfskdem_destroy(g_bredr_ctx[i].edr2_dem);
        if (g_bredr_ctx[i].demod)
            cpfskdem_destroy(g_bredr_ctx[i].demod);
        if (g_bredr_ctx[i].firdec)
            firdecim_crcf_destroy(g_bredr_ctx[i].firdec);
        if (g_bredr_ctx[i].nco)
            nco_crcf_destroy(g_bredr_ctx[i].nco);
    }
}

/* -------------------------------------------------------------------------
 * Packet processing
 * -------------------------------------------------------------------------*/

static void process_bredr_channel(bredr_channel_ctx_t *ctx)
{
    nco_crcf_mix_block_down(ctx->nco, g_raw, ctx->mixed, g_num_samples);

    unsigned int decimated_samples = g_num_samples / DECIM_FACTOR;
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

        /* Timestamp at access-code detection point, like bredr-scanner. */
        unsigned long long bit_in_block = (unsigned long long)(i / SYMBOL_STEP);
        unsigned long long bits_back = pkt.has_header
            ? (58ULL + (unsigned long long)pkt.payload_bytes * 8ULL)
            : 0ULL;
        unsigned long long ac_bit_in_block = (bit_in_block >= bits_back)
            ? (bit_in_block - bits_back)
            : 0ULL;
        unsigned long long abs_raw =
            g_block_base_sample + ac_bit_in_block * RAW_SAMPS_PER_BIT;
        unsigned long long abs_bit = abs_raw / RAW_SAMPS_PER_BIT;
        uint32_t clkn = (uint32_t)((abs_raw * 3200ULL + (unsigned long long)(SAMPLE_RATE / 2u)) /
                                   (unsigned long long)SAMPLE_RATE);
        pkt.rx_clk_ref = abs_raw;
        pkt.rx_clk_1600 =
            (uint32_t)((double)abs_bit / BITS_PER_RX_CLK1600_TICK);

        /* RSSI is computed by the processor at AC detection time (pkt.rssi). */

        pthread_mutex_lock(&g_packet_mutex);
        bredr_piconet_t *pnet =
            bredr_piconet_store_add_packet(&g_store, &pkt, (int)ctx->bredr_channel, clkn);

        bredr_decoded_header_t decoded = {0};
        int decoded_ok = 0;
        if (pnet && pnet->clk_known && pnet->uap_found)
            decoded_ok = decode_header_with_clock(&pkt, pnet->uap,
                                                  pnet->central_clk_1_6, &decoded);

        edr_intro_br_payload_t br_payload = {0};
        memcpy(br_payload.bytes, pkt.payload, pkt.payload_bytes);
        br_payload.payload_bytes = pkt.payload_bytes;

        edr_intro_edr_payload_t edr_payload = {0};
        edr_intro_mode_t mode = choose_edr_mode(&decoded, decoded_ok, &pkt);
        if (mode != EDR_INTRO_MODE_NONE)
        {
            unsigned int payload_bits = pkt.payload_bytes * 8u;
            unsigned int bit_in_block_u = (unsigned int)bit_in_block;
            unsigned int payload_start_bit = (bit_in_block_u + 1u > payload_bits)
                ? (bit_in_block_u + 1u - payload_bits)
                : 0u;
            demod_edr_payload_bits(ctx, payload_start_bit, payload_bits, mode, &edr_payload);
        }

        edr_intro_packet_t intro_pkt = {
            .base_pkt = &pkt,
            .is_edr = (mode != EDR_INTRO_MODE_NONE) ? 1u : 0u,
            .payload = (mode != EDR_INTRO_MODE_NONE) ? (void *)&edr_payload
                                                     : (void *)&br_payload,
        };

        g_total_packets++;
        if (pkt.has_header)
            g_header_packets++;
        else
            g_id_packets++;
        if (intro_pkt.is_edr)
            g_edr_packets++;

        const output_mode_spec_t *spec = output_mode_spec(g_output_mode);
        spec->fn(g_total_packets, &intro_pkt, &pkt, pnet, ctx->bredr_channel, pkt.rssi);
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
        pthread_mutex_lock(&g_work_mutex);
        while (!g_shutdown_requested && ctx->last_gen_processed == g_work_generation)
            pthread_cond_wait(&g_work_cv, &g_work_mutex);
        if (g_shutdown_requested)
        {
            pthread_mutex_unlock(&g_work_mutex);
            break;
        }
        ctx->last_gen_processed = g_work_generation;
        pthread_mutex_unlock(&g_work_mutex);

        process_bredr_channel(ctx);

        pthread_mutex_lock(&g_work_mutex);
        g_complete_count++;
        if (g_complete_count >= g_dispatch_count)
            pthread_cond_signal(&g_done_cv);
        pthread_mutex_unlock(&g_work_mutex);
    }
    return NULL;
}

static int init_thread_pool(void)
{
    g_shutdown_requested = 0;
    g_worker_count = NUM_BREDR_CHANNELS;
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
    pthread_mutex_lock(&g_work_mutex);
    g_shutdown_requested = 1;
    pthread_cond_broadcast(&g_work_cv);
    pthread_mutex_unlock(&g_work_mutex);

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
    g_num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (g_num_samples > BUFFER_SIZE)
        g_num_samples = BUFFER_SIZE;

    for (unsigned int i = 0; i < g_num_samples; i++)
    {
        float i_sample = samples[2u * i] / 128.0f;
        float q_sample = samples[2u * i + 1u] / 128.0f;
        g_raw[i] = i_sample + q_sample * _Complex_I;
    }

    unsigned long long block_base = g_samples_received;
    g_samples_received += g_num_samples;

    pthread_mutex_lock(&g_work_mutex);
    g_block_base_sample = block_base;
    g_dispatch_count = NUM_BREDR_CHANNELS;
    g_complete_count = 0u;
    g_work_generation++;
    pthread_cond_broadcast(&g_work_cv);
    while (g_complete_count < g_dispatch_count && !g_shutdown_requested)
        pthread_cond_wait(&g_done_cv, &g_work_mutex);
    pthread_mutex_unlock(&g_work_mutex);

    return g_stop ? -1 : 0;
}

/* -------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{
    static const struct option long_opts[] = {
        {"view", required_argument, NULL, 'v'},
        {"help", no_argument,       NULL, 'h'},
        {NULL,   0,                 NULL,  0 }
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "v:h", long_opts, NULL)) != -1)
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

    const output_mode_spec_t *mode_spec = output_mode_spec(g_output_mode);
    printf("EDR Intro RX (BR/EDR ch0-19)\n");
    printf("=============================\n");
    printf("LO          : %.3f MHz\n", (double)LO_FREQ_HZ / 1e6);
    printf("Sample rate : %u Msps\n", SAMPLE_RATE / 1000000u);
    printf("Channels    : %u (0..19)\n", NUM_BREDR_CHANNELS);
    printf("View mode   : %s\n", mode_spec->name);
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
        .lo_freq_hz  = LO_FREQ_HZ,
        .sample_rate = SAMPLE_RATE,
        .lna_gain    = LNA_GAIN,
        .vga_gain    = VGA_GAIN,
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
    printf("  Total bits     : %" PRIu64 "\n", g_total_bits);
    printf("  Header packets : %lu\n", g_header_packets);
    printf("  ID packets     : %lu\n", g_id_packets);
    printf("  EDR packets    : %lu\n", g_edr_packets);
    printf("\n");
    bredr_piconet_store_print(&g_store);
    bredr_piconet_store_free(&g_store);

    return EXIT_SUCCESS;
}
