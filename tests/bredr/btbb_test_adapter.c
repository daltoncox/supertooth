/**
 * @file btbb_test_adapter.c
 * @brief Test-only adapter feeding supertooth frames into the real libbtbb.
 */

#include "btbb_test_adapter.h"

#include "bredr_codec.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BTBB_TEST_BASE_PACKET_BITS ((BREDR_AC_BITS - 4u) + 54u)
#define BTBB_TEST_MAX_PACKET_BITS  (BTBB_TEST_BASE_PACKET_BITS + BR_MAX_AIR_PAYLOAD_BITS)

static uint8_t frame_air_payload_bit(const bredr_frame_t *frame, unsigned int bit_pos)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    return (uint8_t)((frame->air_payload[byte_idx] >> bit_idx) & 1u);
}

btbb_packet *btbb_test_packet_from_frame(const bredr_frame_t *frame,
                                         int channel,
                                         uint32_t clkn)
{
    if (!frame || !frame->has_header)
        return NULL;

    static char symbols[BTBB_TEST_MAX_PACKET_BITS];

    unsigned int air_payload_bits = frame->air_payload_bits;
    if (air_payload_bits > BR_MAX_AIR_PAYLOAD_BITS)
        air_payload_bits = BR_MAX_AIR_PAYLOAD_BITS;

    memset(symbols, 0, sizeof(symbols));

    uint64_t sw = bredr_gen_syncword(frame->lap & 0xFFFFFFu);
    for (unsigned int i = 0; i < 64u; i++)
        symbols[i] = (char)((sw >> i) & 1u);

    uint8_t sw_last = (uint8_t)((sw >> 63u) & 1u);
    uint8_t trailer = sw_last ? 0xAu : 0x5u;
    for (unsigned int i = 0; i < 4u; i++)
        symbols[64u + i] = (char)((trailer >> i) & 1u);

    for (unsigned int i = 0; i < 54u; i++)
        symbols[68u + i] = (char)((frame->header_raw >> i) & 1u);

    for (unsigned int i = 0; i < air_payload_bits; i++)
        symbols[122u + i] = (char)frame_air_payload_bit(frame, i);

    btbb_packet *bp = btbb_packet_new();
    if (!bp)
        return NULL;

    btbb_packet_set_data(bp,
                         symbols,
                         (int)(BTBB_TEST_BASE_PACKET_BITS + air_payload_bits),
                         (uint8_t)channel,
                         clkn);
    btbb_packet_set_flag(bp, BTBB_WHITENED, 1);
    return bp;
}

btbb_piconet *btbb_test_piconet_new(uint32_t lap)
{
    btbb_piconet *pn = btbb_piconet_new();
    if (pn)
        btbb_init_piconet(pn, lap & 0xFFFFFFu);
    return pn;
}

int btbb_test_run_uap_recovery(btbb_piconet *pn,
                               const bredr_frame_t *frame,
                               int channel,
                               uint32_t clkn,
                               uint8_t *uap_out,
                               uint8_t *clk6_hint_out)
{
    if (!pn || !frame || !frame->has_header)
        return 0;

    btbb_packet *bp = btbb_test_packet_from_frame(frame, channel, clkn);
    if (!bp)
        return 0;

    /* libbtbb prints progress messages ("UAP = 0x.. found after ..");
     * silence stdout for the duration of the call. */
    fflush(stdout);
    int saved_stdout = dup(STDOUT_FILENO);
    int devnull = -1;
    if (saved_stdout >= 0)
    {
        devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0)
            dup2(devnull, STDOUT_FILENO);
    }

    btbb_uap_from_header(bp, pn);

    fflush(stdout);
    if (saved_stdout >= 0)
    {
        dup2(saved_stdout, STDOUT_FILENO);
        close(saved_stdout);
    }
    if (devnull >= 0)
        close(devnull);

    int recovered = 0;
    if (btbb_piconet_get_flag(pn, BTBB_UAP_VALID))
    {
        if (uap_out)
            *uap_out = btbb_piconet_get_uap(pn);
        if (clk6_hint_out)
            *clk6_hint_out =
                (uint8_t)(((uint32_t)btbb_piconet_get_clk_offset(pn)) & 0x3Fu);
        recovered = 1;
    }

    btbb_packet_unref(bp);
    return recovered;
}
