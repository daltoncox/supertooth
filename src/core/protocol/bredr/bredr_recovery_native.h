/**
 * @file bredr_recovery_native.h
 * @brief Repository-owned BR/EDR UAP / CLK1-6 recovery backend.
 *
 * This replaces the old transitional libbtbb-backed backend.  It reuses the
 * codec primitives already present in the tree (header FEC decode, dewhitening
 * and HEC/CRC verification) and implements only the recovery *orchestration*
 * (the 64 clock-candidate solve) that libbtbb used to provide.
 */

#ifndef BREDR_RECOVERY_NATIVE_H
#define BREDR_RECOVERY_NATIVE_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"
#include "bredr_recovery_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bredr_recovery_native_state bredr_recovery_native_state_t;

void bredr_recovery_native_global_init(uint8_t max_ac_errors);
bredr_recovery_native_state_t *bredr_recovery_native_state_create(uint32_t lap);
void bredr_recovery_native_state_destroy(bredr_recovery_native_state_t *state);
void bredr_recovery_native_state_reset(bredr_recovery_native_state_t *state, uint32_t lap);

int bredr_recovery_native_process_packet(bredr_recovery_native_state_t *state,
                                         const bredr_frame_t *frame,
                                         int channel,
                                         uint32_t clkn,
                                         uint8_t *uap_out,
                                         uint8_t *clk6_hint_out);

const bredr_recovery_backend_ops_t *bredr_recovery_native_backend(void);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_RECOVERY_NATIVE_H */
