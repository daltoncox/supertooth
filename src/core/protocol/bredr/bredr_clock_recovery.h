/**
 * @file bredr_clock_recovery.h
 * @brief BR/EDR UAP / CLK1-6 clock recovery.
 *
 * Native reimplementation of libbtbb's UAP/CLK1-6 recovery.  All low-level
 * primitives (1/3 and 2/3 FEC decode, dewhitening, HEC/CRC verification) are
 * provided by bredr_codec.c; this module implements only the recovery
 * orchestration (the 64 clock-candidate solve) on top of them.
 */

#ifndef BREDR_CLOCK_RECOVERY_H
#define BREDR_CLOCK_RECOVERY_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bredr_recovery_state bredr_recovery_state_t;

typedef struct
{
    uint8_t uap;
    uint8_t clk6_hint;
} bredr_recovery_result_t;

void bredr_recovery_global_init(uint8_t max_ac_errors);
bredr_recovery_state_t *bredr_recovery_state_create(uint32_t lap);
void bredr_recovery_state_destroy(bredr_recovery_state_t *state);
void bredr_recovery_state_reset(bredr_recovery_state_t *state, uint32_t lap);

int bredr_recovery_process_packet(bredr_recovery_state_t *state,
                                  const bredr_frame_t *frame,
                                  int channel,
                                  uint32_t clkn,
                                  bredr_recovery_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CLOCK_RECOVERY_H */
