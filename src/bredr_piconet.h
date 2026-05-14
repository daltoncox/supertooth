/**
 * @file bredr_piconet.h
 * @brief BR/EDR piconet tracking — per-piconet packet queues, clock tracking,
 *        per-device RSSI accumulation, and a multi-piconet store.
 *
 * Overview
 * --------
 * A `bredr_piconet_t` represents a single observed Bluetooth piconet,
 * identified by its 24-bit LAP.  It maintains a circular ring buffer of the
 * 1024 most recently received packets.
 *
 * UAP and initial clock resolution is delegated entirely to libbtbb by the
 * calling application.  Once the UAP and an initial CLK1-6 value are known,
 * the caller invokes `bredr_piconet_set_uap()` to transition the piconet into
 * clock-tracking mode.  In this mode every subsequent header packet is used
 * to verify and — if necessary — correct the central CLK1-6 estimate by
 * checking the HEC with the known UAP and trying CLK1-6 offsets of ±1 and
 * ±2.
 *
 * Once the clock is established, incoming packets are tagged as master or
 * slave transmissions using slot parity (CLK1): even slot = master, odd slot
 * = slave. Per-role RSSI is tracked as an exponentially averaged value by
 * default (configurable, including disabled mode).
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

#include "bredr_phy.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Number of packets retained in each piconet's ring buffer. */
#define BREDR_PICONET_QUEUE_SIZE 1024u

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
        /* -- Identity ---------------------------------------------------------- */

        /** 24-bit Lower Address Part. */
        uint32_t lap;

        /** 8-bit Upper Address Part.  Valid only when uap_found != 0. */
        uint8_t uap;

        /** Non-zero once the UAP has been provided via bredr_piconet_set_uap(). */
        int uap_found;

        /* -- Clock tracking (valid once uap_found && clk_known) ---------------- */

        /** Non-zero once CLK1-6 has been established via bredr_piconet_set_uap(). */
        int clk_known;

        /** Current best central CLK1-6 estimate (0–63). */
        uint8_t central_clk_1_6;

        /**
         * Packet rx_clk_1600 timestamp of the last packet that successfully passed
         * HEC with the tracked central_clk_1_6.
         */
        uint32_t last_successful_rx_clk_1600;

        /**
         * Clock tracking confidence:
         *  -1: no clock lock has ever been attained
         *   0: clock lock was lost; reacquisition required
         *   1..5: lock confidence, where 5 is strongest
         */
        int tracking_state;

        /* -- Aggregate RSSI (used before tracking lock) ---------------------- */

        /** Most recent RSSI (dBr) while no active clock track exists. */
        float combined_rssi;

        /** Non-zero if combined_rssi contains a valid value. */
        int combined_rssi_seen;

        /* -- Per-role RSSI (valid only with active track + HEC-pass packet) ----- */

        /**
         * Most recent RSSI (dBr) for master transmissions (CLK1 == 0).
         */
        float master_rssi;

        /** Non-zero if master_rssi contains a valid value. */
        int master_rssi_seen;

        /**
         * Most recent RSSI (dBr) for slave transmissions, indexed by LT_ADDR
         * (0–7). Index 0 = broadcast / unaddressed frames from slaves.
         */
        float slave_rssi[8];

        /** Non-zero if slave_rssi[i] contains a valid value. */
        int slave_rssi_seen[8];

        /* -- Ring buffer -------------------------------------------------------- */

        /** Circular queue of the 1024 most recently received packets. */
        bredr_packet_t queue[BREDR_PICONET_QUEUE_SIZE];

        /** Index of the next slot to overwrite (0 … QUEUE_SIZE-1). */
        unsigned int queue_head;

        /** Packets currently stored in the ring buffer (0 … QUEUE_SIZE). */
        unsigned int queue_fill;

        /** All-time count of packets received by this piconet. */
        unsigned long total_packets;

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
     * @brief Add a received packet to the piconet ring buffer.
     *
     * Copies the packet into the ring buffer (overwriting the oldest entry once
     * full) and updates first_seen and last_seen.
     *
     * If the piconet is in clock-tracking mode (uap_found && clk_known) and the
     * packet carries a clean decoded header (has_header != 0, ac_errors == 0),
     * the central CLK1-6 estimate is verified and corrected by trying expected,
     * ±1, ±2
     * candidates using
     * the known UAP's HEC.
     *
     * If a non-zero pkt->rssi is present and the clock is known, the latest
     * role RSSI is updated: master (CLK1 == 0) or slave (CLK1 == 1, indexed by
     * LT_ADDR).
     *
     * @param pnet  Must not be NULL and must have been initialised.
     * @param pkt   Packet to add.  Must not be NULL.
     */
    void bredr_piconet_add_packet(bredr_piconet_t *pnet,
                                  const bredr_packet_t *pkt);

    /**
     * @brief Record the UAP and initial CLK1-6, as solved by libbtbb.
     *
     * Transitions the piconet into clock-tracking mode.  Subsequent calls to
     * bredr_piconet_add_packet() will verify and maintain the CLK1-6 estimate
     * using HEC checks.
     *
     * @param pnet          Must not be NULL.
     * @param uap           Solved 8-bit Upper Address Part.
     * @param central_clk_1_6           Central CLK1-6 value (0–63) valid at
     *                                  last_successful_rx_clk_1600.
     * @param last_successful_rx_clk_1600  rx_clk_1600 of the packet at which
     *                                      central_clk_1_6 is known-good.
     */
    void bredr_piconet_set_uap(bredr_piconet_t *pnet, uint8_t uap,
                               uint8_t central_clk_1_6,
                               uint32_t last_successful_rx_clk_1600);

    /**
     * @brief Record only UAP (clock still unknown).
     *
     * Use this when UAP is known but no packet has yet produced a validated
     * CLK1-6 candidate. Clock tracking remains disabled until a successful
     * HEC-based clock acquisition occurs.
     */
    void bredr_piconet_set_uap_only(bredr_piconet_t *pnet, uint8_t uap);

    /**
     * @brief Configure exponential RSSI averaging for piconet RSSI fields.
     *
     * Applies to combined, master, and slave RSSI values maintained by
     * bredr_piconet_add_packet().
     *
     * Averaging uses an EMA with alpha = 2 / (window + 1). For window=0,
     * averaging is disabled and values are replaced by each new sample.
     *
     * @param window  EMA window in packets. 0 disables averaging.
     */
    void bredr_piconet_set_rssi_averaging(unsigned int window);

    /**
     * @brief Return the latest RSSI (dBr) for master transmissions.
     *
     * @return Latest RSSI in dBr, or NAN if no master packets with RSSI have
     *         been received.
     */
    float bredr_piconet_master_rssi(const bredr_piconet_t *pnet);

    /**
     * @brief Return the latest RSSI (dBr) for a specific slave LT_ADDR.
     *
     * @param lt_addr  LT_ADDR to query (0–7).
     * @return Latest RSSI in dBr, or NAN if no packets with RSSI have been
     *         received for this LT_ADDR.
     */
    float bredr_piconet_slave_rssi(const bredr_piconet_t *pnet, uint8_t lt_addr);

    /**
     * @brief Return the first packet rx_clk_1600 value.
     *
     * Returns 0 if no packets have been added yet.
     */
    uint32_t bredr_piconet_get_first_seen(const bredr_piconet_t *pnet);

    /**
     * @brief Return the most recent packet rx_clk_1600 value.
     *
     * Returns 0 if no packets have been added yet.
     */
    uint32_t bredr_piconet_get_last_seen(const bredr_piconet_t *pnet);

    /**
     * @brief Print a human-readable summary of this piconet to stdout.
     */
    void bredr_piconet_print(const bredr_piconet_t *pnet);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_PICONET_H */
