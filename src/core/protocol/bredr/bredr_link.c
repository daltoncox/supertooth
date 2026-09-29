/**
 * @file bredr_link.c
 * @brief BR/EDR connection link implementation.
 *
 * UAP resolution is delegated to the recovery backend by the calling
 * application.  Once the application determines the UAP (and the corresponding
 * CLK1-6 via the recovery backend), it calls bredr_link_set_uap() to put
 * the link into clock-tracking mode.
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

#include "bredr_link.h"
#include "bredr_codec.h"
#include "bredr_clock_recovery.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

uint32_t bredr_sample_to_rx_clk_1600(const bredr_event_t *event)
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

uint32_t bredr_sample_to_clkn(const bredr_event_t *event)
{
    uint64_t num;
    unsigned int radio_sample_rate_hz;

    if (!event)
        return 0u;

    radio_sample_rate_hz = event->meta.radio_sample_rate_hz;
    if (radio_sample_rate_hz == 0u)
        return 0u;

    num = event->meta.radio_start_sample_index * 3200u +
           (uint64_t)(radio_sample_rate_hz / 2u);
    return (uint32_t)(num / (uint64_t)radio_sample_rate_hz);
}

static unsigned int bredr_queue_oldest_index(const bredr_link_t *link,
                                             unsigned int fill)
{
    return (link->queue_head + BREDR_LINK_QUEUE_SIZE - fill) %
           BREDR_LINK_QUEUE_SIZE;
}

static unsigned int bredr_queue_index_at(const bredr_link_t *link,
                                         unsigned int fill,
                                         unsigned int logical_pos)
{
    return (bredr_queue_oldest_index(link, fill) + logical_pos) %
           BREDR_LINK_QUEUE_SIZE;
}

static int bredr_queue_insert_event(bredr_link_t *link,
                                    const bredr_event_t *event)
{
    unsigned int fill;
    unsigned int insert_pos = 0u;
    int newest;
    bredr_event_t carry;

    if (!link || !event)
        return 0;

    fill = link->queue_fill;
    while (insert_pos < fill)
    {
        unsigned int idx = bredr_queue_index_at(link, fill, insert_pos);
        if (link->queue[idx].meta.radio_start_sample_index >
            event->meta.radio_start_sample_index)
            break;
        insert_pos++;
    }

    newest = (insert_pos == fill);

    if (fill == BREDR_LINK_QUEUE_SIZE)
    {
        if (insert_pos == 0u)
            return 0;

        fill--;
        link->queue_fill = fill;
        insert_pos--;
    }

    carry = *event;
    for (unsigned int pos = insert_pos; pos < fill; pos++)
    {
        unsigned int idx = bredr_queue_index_at(link, fill, pos);
        bredr_event_t displaced = link->queue[idx];
        link->queue[idx] = carry;
        carry = displaced;
    }

    link->queue[link->queue_head] = carry;
    link->queue_head = (link->queue_head + 1u) % BREDR_LINK_QUEUE_SIZE;
    link->queue_fill = fill + 1u;
    return newest;
}

/* ---------------------------------------------------------------------------
 * bredr_link_t implementation
 * ---------------------------------------------------------------------------*/

void bredr_link_init(bredr_link_t *link, uint32_t lap)
{
    if (!link)
        return;

    memset(link, 0, sizeof(*link));
    link->lap = lap & 0xFFFFFFu;
    link->tracking_state = -1;
    link->drift_candidate = 0;

    rssi_tracker_init(&link->combined_rssi_track);
    rssi_tracker_init(&link->central_rssi_track);
    for (int i = 0; i < 8; i++)
        rssi_tracker_init(&link->peripheral_rssi_track[i]);

    bredr_recovery_reset(link);

    /* GIAC/LIAC: UAP is the well-known DCI value (0x00). */
    if (link->lap == BREDR_LAP_GIAC || link->lap == BREDR_LAP_LIAC)
    {
        link->uap = BREDR_DCI;
        link->uap_valid = 1;
    }
}

void bredr_link_set_uap(bredr_link_t *link, uint8_t uap,
                           uint8_t central_clk_1_6,
                           uint32_t rx_clk_1600)
{
    if (!link)
        return;

    link->uap = uap;
    link->uap_valid = 1;
    link->clock_offset = (int)((central_clk_1_6 - rx_clk_1600) & 0x3Fu);
    link->clk_known = 1;
    link->tracking_state = 1;
    link->drift_candidate = 0;
}

void bredr_link_set_uap_only(bredr_link_t *link, uint8_t uap)
{
    if (!link)
        return;

    link->uap = uap;
    link->uap_valid = 1;
    link->clk_known = 0;
    link->tracking_state = -1;
    link->drift_candidate = 0;
}

uint8_t bredr_link_central_clk_1_6(const bredr_link_t *link,
                                      uint32_t rx_clk_1600)
{
    if (!link)
        return 0u;

    return (uint8_t)((rx_clk_1600 + link->clock_offset) & 0x3Fu);
}

int bredr_link_add_packet(bredr_link_t *link,
                              const bredr_event_t *event)
{
    int packet_is_newest;
    int has_active_track;
    uint32_t rx_clk_1600;

    if (!link || !event)
        return 0;
    const bredr_frame_t *frame = &event->frame;
    const rx_metadata_t *meta = &event->meta;
    rx_clk_1600 = bredr_sample_to_rx_clk_1600(event);

    link->total_packets++;
    packet_is_newest = bredr_queue_insert_event(link, event);

    if (link->total_packets == 1u)
        link->first_seen = rx_clk_1600;
    if (packet_is_newest)
        link->last_seen = rx_clk_1600;

    has_active_track = (link->uap_valid && link->clk_known && link->tracking_state > 0);

    /* Before track lock, accumulate aggregate RSSI from the newest packet. */
    if (packet_is_newest && !has_active_track && !isnan(meta->rssi_dbr))
        rssi_tracker_add(&link->combined_rssi_track, meta);

    /* Only the newest packet drives recovery / tracking. */
    if (!packet_is_newest)
        return packet_is_newest;

    /* Drive UAP/clock recovery.  While the clock is unknown this acquires the
     * UAP and clock offset; once known it merely corrects for clock drift. */
    int lock_ok = frame->has_header ? bredr_recovery_process(link, event) : 0;

    /* Directional RSSI is accumulated only for packets whose HEC validated on
     * an already-active track; the packet that first establishes the clock is
     * not itself routed. */
    if (!has_active_track || !lock_ok || isnan(meta->rssi_dbr))
        return packet_is_newest;

    uint8_t packet_clk6 = bredr_link_central_clk_1_6(link, rx_clk_1600);

    /* Direction comes from the recovered central clock for this packet.
     * The local receive timeline is only used to recover that clock, not to
     * classify packet role directly. Central transmits when CLK1 == 0 and
     * peripheral transmits when CLK1 == 1. */
    if ((packet_clk6 & 1u) == 0u)
    {
        rssi_tracker_add(&link->central_rssi_track, meta);
        link->central_pkts++;
    }
    else
    {
        uint8_t bits[18];
        bredr_decode_header_bits(frame, packet_clk6, bits);
        uint8_t lt = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
        rssi_tracker_add(&link->peripheral_rssi_track[lt], meta);
        link->peripheral_pkts[lt]++;
    }

    return packet_is_newest;
}
