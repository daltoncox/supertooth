/**
 * @file bredr_clock_recovery.h
 * @brief BR/EDR UAP / CLK1-6 clock recovery and tracking.
 *
 * Native reimplementation of libbtbb's UAP/CLK1-6 recovery.  All low-level
 * primitives (1/3 and 2/3 FEC decode, dewhitening, HEC/CRC verification) are
 * provided by bredr_codec.c; this module implements only the recovery
 * orchestration (the 64 clock-candidate solve) and ongoing clock tracking on
 * top of them.  Every function operates directly on a bredr_piconet_t: the
 * piconet owns the recovery working state and the tracked clock_offset.
 */

#ifndef BREDR_CLOCK_RECOVERY_H
#define BREDR_CLOCK_RECOVERY_H

#include <stdint.h>
#include <stdio.h>

#include "bredr_piconet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Recovery result returned by bredr_recovery_process_packet().
 */
typedef struct
{
    uint8_t uap;       /* Recovered Upper Address Part. */
    uint8_t clk6_hint; /* Recovered central CLK1-6 hint (0-63) at this packet. */
} bredr_recovery_result_t;

/**
 * @brief One-time global init of the recovery module.
 *
 * @param max_ac_errors  Maximum access-code bit errors tolerated by the
 *                       recovery engine (currently advisory; the engine uses
 *                       BREDR_AC_ERRORS_DEFAULT for header acceptance).
 */
void bredr_recovery_global_init(uint8_t max_ac_errors);

/**
 * @brief Reset the acquisition working state (and only that) of a piconet.
 *
 * Clears the recovery candidates, tentative UAP/clock fields and clock_offset.
 * The ring buffer, statistics and confirmed clock lock are preserved; callers
 * that want a full reset should instead re-initialise the piconet.
 */
void bredr_piconet_recovery_reset(bredr_piconet_t *pnet);

/**
 * @brief Feed one header packet to the acquisition engine while the piconet
 *        has no confirmed clock.
 *
 * Accumulates candidate UAPs/CLK1-6.  On a confident solve it records the UAP
 * (uap_found) and, via the clk6_hint result, the recovered clock offset.  The
 * clock is not established here — bredr_clock_recovery_acquire() drives the
 * full solve including CLK1-6 disambiguation.
 *
 * @return non-zero once a UAP has been determined for this packet.
 */
int bredr_recovery_process_packet(bredr_piconet_t *pnet,
                                   const bredr_frame_t *frame,
                                   int channel,
                                   uint32_t clkn,
                                   bredr_recovery_result_t *out);

/**
 * @brief Verify and, if necessary, correct the tracked clock_offset for a
 *        header packet received at rx_clk_1600.
 *
 * Tries the current offset, then ±1 and ±2, validating each with the known
 * UAP's HEC.  On a match, clock_offset is corrected for drift and tracking
 * confidence is raised; on failure it is lowered (and cleared at zero).
 *
 * @return non-zero if the HEC validated (the clock was maintained).
 */
int bredr_clock_track_packet(bredr_piconet_t *pnet,
                             const bredr_frame_t *frame,
                             uint32_t rx_clk_1600);

/**
 * @brief Acquisition entry point driven by bredr_piconet_add_packet().
 *
 * Runs the recovery engine and, once a UAP is known, narrows the 64 CLK1-6
 * candidates against the piconet's historical packets, then establishes the
 * clock via bredr_piconet_set_uap() when unambiguous (or via the recovery hint
 * when a single candidate cannot be isolated).
 *
 * @return non-zero if the clock became known as a result of this packet.
 */
int bredr_clock_recovery_acquire(bredr_piconet_t *pnet,
                                 const bredr_event_t *event,
                                 uint32_t clkn,
                                 uint32_t rx_clk_1600);

/**
 * @brief Enable recording of every acquired frame to a binary dump file.
 *
 * When @p file is non-NULL, each header fed to acquisition is appended in a
 * self-describing binary format so it can be replayed offline (see
 * test_replay_frame_dump).  Pass NULL to disable.  The file is owned by the
 * caller, who is responsible for opening/closing it; this module never closes
 * it.
 */
void bredr_clock_recovery_set_frame_dump(FILE *file);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CLOCK_RECOVERY_H */
