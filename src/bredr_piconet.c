/**
 * @file bredr_piconet.c
 * @brief BR/EDR piconet tracking implementation.
 *
 * UAP resolution is fully delegated to libbtbb by the calling application.
 * Once the application determines the UAP (and the corresponding CLK1-6 via
 * libbtbb), it calls bredr_piconet_set_uap() to put the piconet into
 * clock-tracking mode.
 *
 * Clock tracking
 * --------------
 * With the UAP known, each subsequent header packet is used to verify the
 * current CLK1-6 estimate: the header is unwhitened with the expected CLK1-6,
 * and the HEC is recomputed with the known UAP.  If the HEC matches, the
 * estimate is updated in-place.  If it does not match, CLK1-6 offsets of
 * ±1 and ±2 are tried; the first match corrects the estimate.  Tracking
 * confidence is managed by tracking_state.
 *
 * Header unwhitening uses bredr_decode_header_bits() from bredr_phy.c/h,
 * which holds the whitening tables (sourced from libbtbb's bluetooth_packet.c).
 */

#include "bredr_piconet.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>

/* ---------------------------------------------------------------------------
 * RSSI averaging configuration
 * ---------------------------------------------------------------------------*/

#define BREDR_RSSI_AVG_WINDOW_DEFAULT 16u

static unsigned int g_rssi_avg_window = BREDR_RSSI_AVG_WINDOW_DEFAULT;
static float g_rssi_avg_alpha = 2.0f / ((float)BREDR_RSSI_AVG_WINDOW_DEFAULT + 1.0f);
static float g_rssi_avg_one_minus_alpha = 1.0f - (2.0f / ((float)BREDR_RSSI_AVG_WINDOW_DEFAULT + 1.0f));

static float update_rssi_value(float current, int seen, float sample)
{
    if (!seen || g_rssi_avg_window == 0u)
        return sample;

    return g_rssi_avg_alpha * sample + g_rssi_avg_one_minus_alpha * current;
}

/* ---------------------------------------------------------------------------
 * HEC verification for a given CLK1-6 and known UAP
 * ---------------------------------------------------------------------------*/

/**
 * Returns 1 if clk6 + uap produce a header with matching HEC, 0 otherwise.
 *
 * Delegates FEC decode and unwhitening to bredr_decode_header_bits().
 * bredr_compute_hec() returns bit 7 = first transmitted; we assemble
 * received_hec with the same convention (bit 7 = bits[10]).
 */
static int check_hec_for_clk6(const bredr_packet_t *pkt, uint8_t uap, int clk6)
{
    uint8_t bits[18];
    bredr_decode_header_bits(pkt, (uint8_t)(clk6 & 0x3f), bits);

    uint16_t hdr_data = 0;
    for (int i = 0; i < 10; i++)
        hdr_data |= (uint16_t)(bits[i] << i);

    uint8_t received_hec = 0;
    for (int i = 0; i < 8; i++)
        received_hec |= (uint8_t)(bits[10 + i] << (7 - i));

    return (bredr_compute_hec(hdr_data, uap) == received_hec);
}

/* ---------------------------------------------------------------------------
 * Clock tracking
 * ---------------------------------------------------------------------------*/

/**
 * Verify the current CLK1-6 estimate against the incoming packet's header.
 * Tries the expected CLK1-6 (derived from last-success rx_clk_1600 delta),
 * then ±1 and ±2.  Updates central_clk_1_6 and
 * last_successful_rx_clk_1600 on success. Increments tracking_state on
 * success (capped at 5), decrements on failure, and clears clk_known once
 * tracking_state reaches 0.
 */
static int update_clock(bredr_piconet_t *pnet, const bredr_packet_t *pkt)
{
    uint32_t rx_clk_1600_delta = pkt->rx_clk_1600 - pnet->last_successful_rx_clk_1600;
    int expected_clk6 = (int)((pnet->central_clk_1_6 + rx_clk_1600_delta) & 0x3f);

    static const int offsets[] = {0, 1, -1, 2, -2};
    for (int k = 0; k < 5; k++)
    {
        int candidate = ((expected_clk6 + offsets[k]) + 64) % 64;
        if (check_hec_for_clk6(pkt, pnet->uap, candidate))
        {
            pnet->central_clk_1_6 = (uint8_t)candidate;
            pnet->last_successful_rx_clk_1600 = pkt->rx_clk_1600;
            if (pnet->tracking_state < 5)
                pnet->tracking_state++;
            pnet->clk_known = 1;
            return 1;
        }
    }

    if (pnet->tracking_state > 0)
        pnet->tracking_state--;

    if (pnet->tracking_state == 0)
        pnet->clk_known = 0;

    return 0;
}

/* ---------------------------------------------------------------------------
 * bredr_piconet_t implementation
 * ---------------------------------------------------------------------------*/

void bredr_piconet_init(bredr_piconet_t *pnet, uint32_t lap)
{
    if (!pnet)
        return;

    memset(pnet, 0, sizeof(*pnet));
    pnet->lap = lap & 0xFFFFFFu;
    pnet->tracking_state = -1;

    /* GIAC/LIAC: UAP is the well-known DCI value (0x00). */
    if (pnet->lap == BREDR_LAP_GIAC || pnet->lap == BREDR_LAP_LIAC)
    {
        pnet->uap = BREDR_DCI;
        pnet->uap_found = 1;
    }
}

void bredr_piconet_set_uap(bredr_piconet_t *pnet, uint8_t uap,
                           uint8_t central_clk_1_6,
                           uint32_t last_successful_rx_clk_1600)
{
    if (!pnet)
        return;

    pnet->uap = uap;
    pnet->uap_found = 1;
    pnet->central_clk_1_6 = central_clk_1_6 & 0x3fu;
    pnet->last_successful_rx_clk_1600 = last_successful_rx_clk_1600;
    pnet->clk_known = 1;
    pnet->tracking_state = 1;
}

void bredr_piconet_set_uap_only(bredr_piconet_t *pnet, uint8_t uap)
{
    if (!pnet)
        return;

    pnet->uap = uap;
    pnet->uap_found = 1;
    pnet->clk_known = 0;
    pnet->tracking_state = -1;
}

void bredr_piconet_set_rssi_averaging(unsigned int window)
{
    g_rssi_avg_window = window;
    if (window == 0u)
    {
        g_rssi_avg_alpha = 1.0f;
        g_rssi_avg_one_minus_alpha = 0.0f;
    }
    else
    {
        g_rssi_avg_alpha = 2.0f / ((float)window + 1.0f);
        g_rssi_avg_one_minus_alpha = 1.0f - g_rssi_avg_alpha;
    }
}

void bredr_piconet_add_packet(bredr_piconet_t *pnet,
                              const bredr_packet_t *pkt)
{
    if (!pnet || !pkt)
        return;

    /* Copy into ring buffer. */
    pnet->queue[pnet->queue_head] = *pkt;
    pnet->queue_head = (pnet->queue_head + 1u) % BREDR_PICONET_QUEUE_SIZE;
    if (pnet->queue_fill < BREDR_PICONET_QUEUE_SIZE)
        pnet->queue_fill++;

    pnet->total_packets++;

    if (pnet->total_packets == 1u)
        pnet->first_seen = pkt->rx_clk_1600;
    pnet->last_seen = pkt->rx_clk_1600;

    int has_active_track = (pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0);

    /* Before track lock, keep only latest aggregate RSSI. */
    if (!has_active_track && pkt->rssi != 0.0f)
    {
        pnet->combined_rssi =
            update_rssi_value(pnet->combined_rssi, pnet->combined_rssi_seen, pkt->rssi);
        pnet->combined_rssi_seen = 1;
    }

    /* Directional RSSI accumulation requires active track + header packet. */
    if (!has_active_track || !pkt->has_header)
        return;

    /* Update per-role RSSI only when current packet passes HEC under tracking. */
    int hec_ok = 0;
    if (pkt->ac_errors <= 1u)
        hec_ok = update_clock(pnet, pkt);
    if (!hec_ok || pkt->rssi == 0.0f)
        return;

    /* rx_clk_1600 is slot clock (CLKN >> 1), so bit0 == CLK1 parity.
     * Master transmits when CLK1 == 0, slave when CLK1 == 1. */
    if ((pkt->rx_clk_1600 & 1u) == 0u)
    {
        pnet->master_rssi =
            update_rssi_value(pnet->master_rssi, pnet->master_rssi_seen, pkt->rssi);
        pnet->master_rssi_seen = 1;
    }
    else
    {
        uint8_t bits[18];
        bredr_decode_header_bits(pkt, pnet->central_clk_1_6, bits);
        uint8_t lt = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
        pnet->slave_rssi[lt] =
            update_rssi_value(pnet->slave_rssi[lt], pnet->slave_rssi_seen[lt], pkt->rssi);
        pnet->slave_rssi_seen[lt] = 1;
    }
}

float bredr_piconet_master_rssi(const bredr_piconet_t *pnet)
{
    if (!pnet || !pnet->master_rssi_seen)
        return (float)NAN;
    return pnet->master_rssi;
}

float bredr_piconet_slave_rssi(const bredr_piconet_t *pnet, uint8_t lt_addr)
{
    if (!pnet || lt_addr > 7u || !pnet->slave_rssi_seen[lt_addr])
        return (float)NAN;
    return pnet->slave_rssi[lt_addr];
}

uint32_t bredr_piconet_get_first_seen(const bredr_piconet_t *pnet)
{
    return pnet ? pnet->first_seen : 0u;
}

uint32_t bredr_piconet_get_last_seen(const bredr_piconet_t *pnet)
{
    return pnet ? pnet->last_seen : 0u;
}

void bredr_piconet_print(const bredr_piconet_t *pnet)
{
    if (!pnet)
        return;

    printf("  LAP: 0x%06" PRIX32, pnet->lap & 0xFFFFFFu);

    if (pnet->uap_found)
        printf("  UAP: 0x%02X", pnet->uap);
    else
        printf("  UAP: ??");

    if (pnet->clk_known)
    {
        printf("  CLK1-6: %02d [state=%d]",
               pnet->central_clk_1_6,
               pnet->tracking_state);
    }
    else
    {
        printf("  CLK: unknown [state=%d]", pnet->tracking_state);
    }

    double first_s = pnet->first_seen * 625.0e-6;
    double last_s = pnet->last_seen * 625.0e-6;
    printf("  First: CLK %07" PRIu32 " (~%.3f s)"
           "  Last: CLK %07" PRIu32 " (~%.3f s)",
           pnet->first_seen, first_s,
           pnet->last_seen, last_s);

    printf("  Pkts: %lu", pnet->total_packets);

    /* RSSI summary */
    if (pnet->combined_rssi_seen)
        printf("  Combined RSSI: %.1f dBr", pnet->combined_rssi);

    float mr = bredr_piconet_master_rssi(pnet);
    if (!isnan(mr))
        printf("  Master RSSI: %.1f dBr", mr);

    for (int i = 1; i <= 7; i++)
    {
        float sr = bredr_piconet_slave_rssi(pnet, (uint8_t)i);
        if (!isnan(sr))
            printf("  Slave[%d] RSSI: %.1f dBr", i, sr);
    }

    printf("\n");
}
