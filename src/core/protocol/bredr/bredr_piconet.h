/**
 * @file bredr_piconet.h
 * @brief BR/EDR piconet tracking — per-piconet packet queues, clock tracking,
 *        per-device RSSI accumulation, and a multi-piconet store.
 *
 * Overview
 * --------
 * A `bredr_piconet_t` represents a single observed Bluetooth piconet,
 * identified by its 24-bit LAP.  It maintains a circular ring buffer of the
 * 1024 most recently received BR/EDR events.
 *
 * UAP and initial clock resolution is performed natively by the recovery
 * module (bredr_clock_recovery.c) operating directly on these fields.  Once
 * the UAP and an initial CLK1-6 value are known, the recovery module calls
 * `bredr_piconet_set_uap()` to transition the piconet into clock-tracking
 * mode.  In this mode every subsequent header packet is used to verify and —
 * if necessary — correct the clock by checking the HEC with the known UAP and
 * trying the tracked clock offset and its ±1, ±2 neighbours to absorb drift.
 *
 * The piconet does not store an absolute central clock; instead it keeps a
 * single `clock_offset` such that the central CLK1-6 at any received packet is
 * `(rx_clk_1600 + clock_offset) mod 64`.  Once the clock is established,
 * incoming packets are tagged as master or slave transmissions using slot
 * parity (CLK1): even slot = master, odd slot = slave. Per-role RSSI is
 * tracked as an exponentially averaged value by default (configurable,
 * including disabled mode).
 *
 * Memory notes
 * ------------
 * Each `bredr_piconet_t` is approximately 380 KB in size (1024-packet ring
 * buffer dominates).  Always allocate piconets on the heap; never declare
 * them as local variables.  The store manages allocation automatically.
 *
 * Bluetooth Core Specification references
 * ----------------------------------------
 * - Vol 2, Part B, §7.4   — HEC LFSR
 * - Vol 2, Part B, §6.3   — Access code and LAP
 * - Vol 2, Part B, §1.2   — Bluetooth clock (CLK / CLKN)
 */

#ifndef BREDR_PICONET_H
#define BREDR_PICONET_H

#include <stdint.h>
#include <stddef.h>

#include "receive_event_models.h"
#include "bredr_bitstream_decoder.h"
#include "rssi_tracker.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Number of packets retained in each piconet's ring buffer. */
#define BREDR_PICONET_QUEUE_SIZE 1024u

/** Number of CLK1-6 candidates tracked during UAP/clock acquisition. */
#define BREDR_CLK6_CANDIDATES 64

    /* ---------------------------------------------------------------------------
     * bredr_piconet_t
     * ---------------------------------------------------------------------------*/

    /**
     * @brief State for a single observed BR/EDR piconet.
     *
     * Identified by its 24-bit LAP.  All memory is owned by this struct; heap-
     * allocate via `malloc(sizeof(...))` or through a `bredr_piconet_store_t`.
     */
    typedef struct
    {
        /* -- Identity (info) --------------------------------------------------- */

        /** 24-bit Lower Address Part. */
        uint32_t lap;

        /** 8-bit Upper Address Part.  Valid only when uap_found != 0. */
        uint8_t uap;

        /** Non-zero once the UAP is known (tentative during recovery, then confirmed). */
        int uap_found;

        /* -- Clock tracking (valid once uap_found && clk_known) ---------------- */

        /** Non-zero once CLK1-6 has been established via bredr_piconet_set_uap(). */
        int clk_known;

        /**
         * Offset between the piconet's central clock and the receiver clock.
         * The central CLK1-6 of any received packet is:
         *   (rx_clk_1600 + clock_offset) mod 64.
         * Tracking tries this offset, then ±1 and ±2, correcting clock_offset
         * when the two clocks have drifted.
         */
        int clock_offset;

        /**
         * Clock tracking confidence:
         *  -1: no clock lock has ever been attained
         *   0: clock lock was lost; reacquisition required
         *   1..5: lock confidence, where 5 is strongest
         */
        int tracking_state;

        /* -- Recovery: UAP / CLK1-6 acquisition working state ------------------ */

        /**
         * Per CLK1-6 index (0–63): the tentative UAP recovered for that clock
         * candidate, or -1 if the candidate has been pruned.  Managed by the
         * recovery module (bredr_clock_recovery.c) while acquiring the UAP.
         */
        int recovery_candidates[BREDR_CLK6_CANDIDATES];

        /** clkn>>1 of the first packet fed to acquisition (anchors candidate clocks). */
        uint32_t recovery_first_pkt_time;

        /** Non-zero once at least one packet has been fed to acquisition. */
        int recovery_got_first_packet;

        /**
         * Per CLK1-6 candidate slot (0–63): bitmask of LT_ADDRs (bits 1–7) seen
         * carrying an eSCO (EV4/EV5) payload, or an ACL (DM/DH/DV) payload,
         * under that candidate's clock.  Bluetooth dedicates an eSCO-assigned
         * LT_ADDR to eSCO packets only; a candidate that decodes the same
         * LT_ADDR as both is invalid and is pruned.
         */
        uint8_t recovery_esco_lt_mask[BREDR_CLK6_CANDIDATES];
        uint8_t recovery_acl_lt_mask[BREDR_CLK6_CANDIDATES];

        /* -- Aggregate RSSI (used before tracking lock) ----------------------- */

        /** RSSI tracker over the last ~1 s, aggregate (pre-track-lock). */
        rssi_tracker_t combined_rssi_track;

        /* -- Per-role RSSI (valid with active track + HEC-pass packet) -------- */

        /** RSSI tracker over the last ~1 s, master transmissions (CLK1 == 0). */
        rssi_tracker_t master_rssi_track;

        /**
         * RSSI tracker over the last ~1 s for slave transmissions, indexed by
         * LT_ADDR (0–7). Index 0 = broadcast / unaddressed frames from slaves.
         */
        rssi_tracker_t slave_rssi_track[8];

        /* -- Frames: ring buffer ---------------------------------------------- */

        /** Circular queue of the 1024 most recently received BR/EDR events. */
        bredr_event_t queue[BREDR_PICONET_QUEUE_SIZE];

        /** Index of the next slot to overwrite (0 … QUEUE_SIZE-1). */
        unsigned int queue_head;

        /** Packets currently stored in the ring buffer (0 … QUEUE_SIZE). */
        unsigned int queue_fill;

        /** All-time count of packets received by this piconet. */
        unsigned long total_packets;

        /* -- Per-member packet counts (derived device rows) ------------------- */

        /** Packets classified as master transmissions (CLK1 == 0). */
        unsigned long master_pkts;

        /** Packets classified as slave transmissions, indexed by LT_ADDR (0–7). */
        unsigned long slave_pkts[8];

        /* -- Timestamps (slot clock, 625 µs per tick / 1600 Hz) ----------------- */

        /** rx_clk_1600 of the first packet ever added to this piconet. */
        uint32_t first_seen;

        /** rx_clk_1600 of the most recently added packet. */
        uint32_t last_seen;

    } bredr_piconet_t;

    /* ---------------------------------------------------------------------------
     * bredr_piconet_t API
     * ---------------------------------------------------------------------------*/

    /**
     * @brief Initialise a piconet for a given LAP.
     *
     * For the GIAC and LIAC LAPs the UAP is pre-set to the well-known DCI value
     * (0x00) and uap_found is set.
     *
     * @param pnet  Must not be NULL.
     * @param lap   24-bit Lower Address Part.
     */
    void bredr_piconet_init(bredr_piconet_t *pnet, uint32_t lap);

    /**
    * @brief Add a received event to the piconet ring buffer.
     *
    * Copies the event into the ring buffer (overwriting the oldest entry once
     * full) and updates first_seen and last_seen.
     *
     * If the piconet is in clock-tracking mode (uap_found && clk_known) and the
     * event carries a decoded header (has_header != 0), the central CLK1-6
     * estimate is verified and corrected by trying expected, ±1, ±2
     * candidates using
     * the known UAP's HEC.
     *
     * If event metadata includes a valid RSSI value and the clock is known,
     * the latest role RSSI is updated: master (CLK1 == 0) or slave
     * (CLK1 == 1, indexed by LT_ADDR).
     *
     * @param pnet  Must not be NULL and must have been initialised.
     * @param event BR/EDR event to add. Must not be NULL.
     */
    int bredr_piconet_add_packet(bredr_piconet_t *pnet,
                           const bredr_event_t *event);

    /**
     * @brief Record the UAP and initial CLK1-6, as solved by the recovery backend.
     *
     * Transitions the piconet into clock-tracking mode.  Subsequent calls to
     * bredr_piconet_add_packet() will verify and maintain the CLK1-6 estimate
     * using HEC checks.
     *
     * @param pnet          Must not be NULL.
     * @param uap           Solved 8-bit Upper Address Part.
     * @param central_clk_1_6  Central CLK1-6 value (0–63) valid at the packet
     *                          whose rx_clk_1600 is given below.
     * @param rx_clk_1600       rx_clk_1600 of the packet at which
     *                          central_clk_1_6 is known-good.
     */
    void bredr_piconet_set_uap(bredr_piconet_t *pnet, uint8_t uap,
                                uint8_t central_clk_1_6,
                                uint32_t rx_clk_1600);

    /**
     * @brief Record only UAP (clock still unknown).
     *
     * Use this when UAP is known but no packet has yet produced a validated
     * CLK1-6 candidate. Clock tracking remains disabled until a successful
     * HEC-based clock acquisition occurs.
     */
    void bredr_piconet_set_uap_only(bredr_piconet_t *pnet, uint8_t uap);

    /**
     * @brief Compute the central CLK1-6 for a packet received at rx_clk_1600.
     *
     * Uses the tracked clock_offset: central = (rx_clk_1600 + clock_offset) mod 64.
     *
     * @param pnet        Must not be NULL.
     * @param rx_clk_1600 Receiver slot-clock timestamp (625 µs/tick, 1600 Hz).
     * @return            Central CLK1-6 value (0–63).
     */
    uint8_t bredr_piconet_central_clk_1_6(const bredr_piconet_t *pnet,
                                          uint32_t rx_clk_1600);

    /**
     * @brief Convert a received event's start sample index to its rx_clk_1600
     *        slot-clock timestamp (625 µs per tick, 1600 Hz).
     */
    uint32_t bredr_sample_to_rx_clk_1600(const bredr_event_t *event);

    /**
     * @brief Convert a received event's start sample index to its CLKN
     *        timestamp (312.5 µs per tick, 3200 Hz).  CLKN = 2 * rx_clk_1600.
     */
    uint32_t bredr_sample_to_clkn(const bredr_event_t *event);


#ifdef __cplusplus
}
#endif

#endif /* BREDR_PICONET_H */
