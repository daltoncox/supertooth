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
 *
 * The single entry point is bredr_recovery_process(): while the piconet has no
 * confirmed clock it acquires the UAP and clock offset, and once those are
 * known it merely tracks and corrects for clock drift.
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
 * @brief Reset the acquisition working state (and only that) of a piconet.
 *
 * Clears the recovery candidates, tentative UAP/clock fields and clock_offset.
 * The ring buffer, statistics and confirmed clock lock are preserved; callers
 * that want a full reset should instead re-initialise the piconet.
 */
void bredr_recovery_reset(bredr_piconet_t *pnet);

/**
 * @brief Drive UAP/clock recovery for a single received event.
 *
 * While the piconet has no confirmed clock it acquires the UAP and clock
 * offset (bredr_piconet_set_uap() is called once both are found).  Once the
 * UAP and clock offset are known this function only tracks clock drift,
 * correcting the tracked clock_offset when the two clocks have drifted.
 *
 * @return non-zero if the clock is locked for this packet (acquired, or the
 *         HEC validated while tracking).
 */
int bredr_recovery_process(bredr_piconet_t *pnet,
                           const bredr_event_t *event);

/**
 * @brief Enable recording of every acquired frame to a binary dump file.
 *
 * When @p file is non-NULL, each header fed to acquisition is appended in a
 * self-describing binary format so it can be replayed offline (see
 * test_replay_frame_dump).  Pass NULL to disable.  The file is owned by the
 * caller, who is responsible for opening/closing it; this module never closes
 * it.
 */
void bredr_recovery_set_frame_dump(FILE *file);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CLOCK_RECOVERY_H */
