/**
 * @file bredr_piconet.c
 * @brief BR/EDR piconet tracking implementation.
 *
 * UAP resolution is delegated to the recovery backend by the calling
 * application.  Once the application determines the UAP (and the corresponding
 * CLK1-6 via the recovery backend), it calls bredr_piconet_set_uap() to put
 * the piconet into clock-tracking mode.
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
 * Header unwhitening uses bredr_decode_header_bits() from bredr_codec.c/h,
 * which holds the whitening tables.
 */

#include "bredr_piconet.h"
#include "bredr_codec.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

static uint64_t now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000u + (uint64_t)tv.tv_usec / 1000u;
}

/* ---------------------------------------------------------------------------
 * RSSI averaging configuration
 * ---------------------------------------------------------------------------*/

static float update_rssi_value(float current, int seen, float sample,
                               unsigned int window, float alpha,
                               float one_minus_alpha)
{
    if (!seen || window == 0u)
        return sample;

    return alpha * sample + one_minus_alpha * current;
}

static uint32_t bredr_sample_to_rx_clk_1600(const bredr_event_t *event)
{
    uint64_t num;
    unsigned int radio_sample_rate_hz;

    if (!event)
        return 0u;

    radio_sample_rate_hz = event->meta.radio_sample_rate_hz;
    if (radio_sample_rate_hz == 0u)
        return 0u;

    num = event->meta.radio_start_sample_index * 1600u +
          (uint64_t)(radio_sample_rate_hz / 2u);
    return (uint32_t)(num / (uint64_t)radio_sample_rate_hz);
}

static unsigned int bredr_queue_oldest_index(const bredr_piconet_t *pnet,
                                             unsigned int fill)
{
    return (pnet->queue_head + BREDR_PICONET_QUEUE_SIZE - fill) %
           BREDR_PICONET_QUEUE_SIZE;
}

static unsigned int bredr_queue_index_at(const bredr_piconet_t *pnet,
                                         unsigned int fill,
                                         unsigned int logical_pos)
{
    return (bredr_queue_oldest_index(pnet, fill) + logical_pos) %
           BREDR_PICONET_QUEUE_SIZE;
}

static int bredr_queue_insert_event(bredr_piconet_t *pnet,
                                    const bredr_event_t *event)
{
    unsigned int fill;
    unsigned int insert_pos = 0u;
    int newest;
    bredr_event_t carry;

    if (!pnet || !event)
        return 0;

    fill = pnet->queue_fill;
    while (insert_pos < fill)
    {
        unsigned int idx = bredr_queue_index_at(pnet, fill, insert_pos);
        if (pnet->queue[idx].meta.radio_start_sample_index >
            event->meta.radio_start_sample_index)
            break;
        insert_pos++;
    }

    newest = (insert_pos == fill);

    if (fill == BREDR_PICONET_QUEUE_SIZE)
    {
        if (insert_pos == 0u)
            return 0;

        fill--;
        pnet->queue_fill = fill;
        insert_pos--;
    }

    carry = *event;
    for (unsigned int pos = insert_pos; pos < fill; pos++)
    {
        unsigned int idx = bredr_queue_index_at(pnet, fill, pos);
        bredr_event_t displaced = pnet->queue[idx];
        pnet->queue[idx] = carry;
        carry = displaced;
    }

    pnet->queue[pnet->queue_head] = carry;
    pnet->queue_head = (pnet->queue_head + 1u) % BREDR_PICONET_QUEUE_SIZE;
    pnet->queue_fill = fill + 1u;
    return newest;
}

/* ---------------------------------------------------------------------------
 * Clock tracking
 * ---------------------------------------------------------------------------*/

/**
 * Verify the current CLK1-6 estimate against the incoming frame's header.
 * Tries the expected CLK1-6 (derived from last-success rx_clk_1600 delta),
 * then ±1 and ±2.  Updates central_clk_1_6 and
 * last_successful_rx_clk_1600 on success. Increments tracking_state on
 * success (capped at 5), decrements on failure, and clears clk_known once
 * tracking_state reaches 0.
 */
static int update_clock(bredr_piconet_t *pnet,
                        const bredr_frame_t *frame,
                        uint32_t rx_clk_1600)
{
    uint32_t rx_clk_1600_delta = rx_clk_1600 - pnet->last_successful_rx_clk_1600;
    int expected_clk6 = (int)((pnet->central_clk_1_6 + rx_clk_1600_delta) & 0x3f);

    static const int offsets[] = {0, 1, -1, 2, -2};
    for (int k = 0; k < 5; k++)
    {
        int candidate = ((expected_clk6 + offsets[k]) + 64) % 64;
        if (bredr_hec_ok_for_clk6(frame, pnet->uap, (uint8_t)candidate))
        {
            pnet->central_clk_1_6 = (uint8_t)candidate;
            pnet->last_successful_rx_clk_1600 = rx_clk_1600;
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

int bredr_piconet_add_packet(bredr_piconet_t *pnet,
                             const bredr_event_t *event,
                             unsigned int rssi_window,
                             float rssi_alpha,
                             float rssi_one_minus_alpha)
{
    int packet_is_newest;
    int has_active_track;
    uint32_t rx_clk_1600;
    uint64_t now = now_ms();

    if (!pnet || !event)
        return 0;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    rx_clk_1600 = bredr_sample_to_rx_clk_1600(event);

    pnet->total_packets++;
    packet_is_newest = bredr_queue_insert_event(pnet, event);

    if (pnet->total_packets == 1u)
        pnet->first_seen = rx_clk_1600;
    if (packet_is_newest)
        pnet->last_seen = rx_clk_1600;

    has_active_track = (pnet->uap_found && pnet->clk_known && pnet->tracking_state > 0);

    /* Before track lock, keep only latest aggregate RSSI. */
    if (packet_is_newest && !has_active_track && !isnan(meta->rssi_dbr))
    {
        pnet->combined_rssi =
            update_rssi_value(pnet->combined_rssi, pnet->combined_rssi_seen, meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->combined_rssi_seen = 1;
        rssi_window_add(&pnet->combined_rssi_win, meta->rssi_dbr, now);
    }

    /* Directional RSSI accumulation requires active track + header packet. */
    if (!packet_is_newest || !has_active_track || !frame->has_header)
        return packet_is_newest;

    /* Update per-role RSSI only when current frame passes HEC under tracking. */
    int hec_ok = 0;
    if (frame->ac_errors <= 1u)
        hec_ok = update_clock(pnet, frame, rx_clk_1600);
    if (!hec_ok || isnan(meta->rssi_dbr))
        return packet_is_newest;

    uint8_t packet_clk6 = pnet->central_clk_1_6;

    /* Direction comes from the recovered central clock for this packet.
     * The local receive timeline is only used to recover that clock, not to
     * classify packet role directly. Master transmits when CLK1 == 0 and
     * slave transmits when CLK1 == 1. */
    if ((packet_clk6 & 1u) == 0u)
    {
        pnet->master_rssi =
            update_rssi_value(pnet->master_rssi, pnet->master_rssi_seen, meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->master_rssi_seen = 1;
        rssi_window_add(&pnet->master_rssi_win, meta->rssi_dbr, now);
        pnet->master_pkts++;
    }
    else
    {
        uint8_t bits[18];
        bredr_decode_header_bits(frame, packet_clk6, bits);
        uint8_t lt = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
        pnet->slave_rssi[lt] =
            update_rssi_value(pnet->slave_rssi[lt], pnet->slave_rssi_seen[lt], meta->rssi_dbr,
                              rssi_window, rssi_alpha, rssi_one_minus_alpha);
        pnet->slave_rssi_seen[lt] = 1;
        rssi_window_add(&pnet->slave_rssi_win[lt], meta->rssi_dbr, now);
        pnet->slave_pkts[lt]++;
    }

    return packet_is_newest;
}
