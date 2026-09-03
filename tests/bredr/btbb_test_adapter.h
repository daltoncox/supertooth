/**
 * @file btbb_test_adapter.h
 * @brief Test-only adapter feeding supertooth frames into the real libbtbb.
 *
 * The vendored libbtbb (see libbtbb/ at the repository root) is compiled into
 * a static library that is linked EXCLUSIVELY into test executables -- it is
 * never linked into supertooth_core, keeping the GPL code out of the product.
 *
 * This adapter rebuilds the libbtbb symbol layout expected by
 * btbb_packet_set_data() from a supertooth bredr_frame_t, exactly like the
 * former transitional bredr_recovery_btbb.c backend did:
 *
 *   symbols[0..63]    sync word regenerated from the frame LAP
 *   symbols[64..67]   AC trailer derived from the last sync-word bit
 *   symbols[68..121]  raw 54 FEC-encoded header bits (air order)
 *   symbols[122..]    raw on-air payload bits (air order)
 */

#ifndef BTBB_TEST_ADAPTER_H
#define BTBB_TEST_ADAPTER_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "btbb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create a libbtbb packet holding the same content as @p frame.
 *
 * Note: btbb_packet_set_data() internally stores `clkn >> 1` (CLK1), which is
 * the clock domain libbtbb uses for all CLK1-6 whitening math.
 *
 * @return New packet (caller must btbb_packet_unref()), or NULL.
 */
btbb_packet *btbb_test_packet_from_frame(const bredr_frame_t *frame,
                                         int channel,
                                         uint32_t clkn);

/** Create a libbtbb piconet initialised with @p lap. */
btbb_piconet *btbb_test_piconet_new(uint32_t lap);

/**
 * Run one packet through libbtbb's UAP recovery (btbb_uap_from_header).
 *
 * Mirrors the "have LAP, need UAP" path of btbb_process_packet() without the
 * survey / hop-reversal machinery.  stdout is temporarily suppressed while
 * libbtbb runs because the library prints progress messages.
 *
 * @param[out] uap_out       Recovered UAP (valid when 1 is returned).
 * @param[out] clk6_hint_out CLK offset hint (valid when 1 is returned).
 * @return 1 when libbtbb has determined the UAP, 0 otherwise.
 */
int btbb_test_run_uap_recovery(btbb_piconet *pn,
                               const bredr_frame_t *frame,
                               int channel,
                               uint32_t clkn,
                               uint8_t *uap_out,
                               uint8_t *clk6_hint_out);

#ifdef __cplusplus
}
#endif

#endif /* BTBB_TEST_ADAPTER_H */
