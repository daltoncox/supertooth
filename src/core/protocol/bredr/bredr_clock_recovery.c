/**
 * @file bredr_clock_recovery.c
 * @brief BR/EDR UAP / CLK1-6 clock recovery.
 *
 * Clean-room reimplementation of libbtbb's BR/EDR UAP/CLK1-6 recovery.  All
 * low-level primitives (1/3 and 2/3 FEC decode, dewhitening, HEC decode,
 * payload CRC) come from bredr_codec.c; this file reproduces libbtbb's
 * candidate-selection algorithm (btbb_uap_from_header) and per-type CRC logic
 * (crc_check / fhs / DM / DH / EV3 / EV4 / EV5 / HV).
 *
 * Invariants that must match libbtbb:
 *   - Recovery clock is CLK1 (clkn >> 1).  libbtbb stores pkt->clkn = clkn>>1
 *     in btbb_packet_set_data(); every whitening/clock computation uses it.
 *   - Packet type numbering follows libbtbb: DM3=0xA, DH3=0xB, EV4=0xC,
 *     EV5=0xD, DM5=0xE, DH5=0xF (HV3/EV3 share 0x7, DV=0x8, AUX1=0x9).
 *   - crc_check() contract: 0 hard negative (only FHS/DM1/HV1 may prune),
 *     1 inconclusive, 2 inconclusive-but-likely, >1 (10/1000) positive.
 *     Non-FHS/DM1/HV1 CRC failures are downgraded to inconclusive.  EV3/EV5
 *     positives are downgraded to inconclusive (high false-positive rate).
 */

#include "bredr_clock_recovery.h"

#include "bredr_codec.h"
#include "bredr_bitstream_decoder.h"

#include <stdlib.h>
#include <string.h>

#define CLK6_CANDIDATES 64

#define PT_FHS   2
#define PT_DV    8
#define PT_DM1   3
#define PT_DM3   10
#define PT_DM5   14
#define PT_DH1   4
#define PT_DH3   11
#define PT_DH5   15
#define PT_HV1   5
#define PT_HV2   6
#define PT_HV3   7  /* HV3 / EV3 */
#define PT_AUX1  9
#define PT_EV4   12
#define PT_EV5   13

struct bredr_recovery_state
{
    int clock6_candidates[CLK6_CANDIDATES];
    uint8_t uap;
    int clk_offset;
    uint32_t first_pkt_time;
    int packets_observed;
    int got_first_packet;
    int uap_valid;
    int clk6_valid;
};

static void state_clear(bredr_recovery_state_t *st)
{
    for (int i = 0; i < CLK6_CANDIDATES; i++)
        st->clock6_candidates[i] = -1;
    st->uap = 0u;
    st->clk_offset = 0;
    st->first_pkt_time = 0u;
    st->packets_observed = 0;
    st->got_first_packet = 0;
    st->uap_valid = 0;
    st->clk6_valid = 0;
}

void bredr_recovery_global_init(uint8_t max_ac_errors)
{
    (void)max_ac_errors;
}

bredr_recovery_state_t *bredr_recovery_state_create(uint32_t lap)
{
    (void)lap;
    bredr_recovery_state_t *state =
        (bredr_recovery_state_t *)calloc(1, sizeof(*state));
    if (!state)
        return NULL;
    state_clear(state);
    return state;
}

void bredr_recovery_state_destroy(bredr_recovery_state_t *state)
{
    free(state);
}

void bredr_recovery_state_reset(bredr_recovery_state_t *state, uint32_t lap)
{
    (void)lap;
    if (state)
        state_clear(state);
}

/* --- header decode (mirror libbtbb try_clock) --- */

static uint8_t try_clock(const bredr_frame_t *frame,
                         uint8_t clock,
                         uint8_t *type_out)
{
    uint8_t packed_header[7];
    uint8_t header[18];
    uint8_t unwhitened[18];
    unsigned int decoded_bits = 0u;
    int be;

    bredr_pack_header_raw(frame->header_raw, packed_header);
    be = bredr_fec_decode_1_3(packed_header, 54u, header, &decoded_bits);
    if (be < 0 || decoded_bits != 18u || be >= 4)
        return 0;

    bredr_dewhiten_air_payload_bytes(header, 18u, clock, 0u, unwhitened,
                                     sizeof(unwhitened));

    uint16_t hdr_data = (uint16_t)bredr_read_packed_field(unwhitened, 0u, 10u);
    uint8_t hec = (uint8_t)bredr_read_packed_field(unwhitened, 10u, 8u);
    *type_out = (uint8_t)bredr_read_packed_field(unwhitened, 3u, 4u);
    return bredr_decode_uap_from_hec(hdr_data, hec);
}

/* --- payload CRC checkers (mirror libbtbb fhs/DM/DH/EV3/EV4/EV5/HV) --- */
/* return: 0 hard negative, 1 inconclusive, 2 likely, 10 positive, 1000 FHS. */

static int fhs(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
{
    int size = (int)frame->air_payload_bits;
    int payload_length = 20;
    uint8_t raw[300];
    uint8_t corrected[21];
    uint8_t payload[20];
    unsigned int cb = 0u;
    uint16_t crc, chk;

    if (size < payload_length * 12)
        return 1;

    bredr_copy_packed_bits(frame->air_payload, 0u, (unsigned)(payload_length * 12), raw);
    if (bredr_fec_decode_2_3(raw, (unsigned)(payload_length * 12), corrected, &cb) < 0)
        return 0;

    memset(payload, 0, sizeof(payload));
    bredr_dewhiten_air_payload_bytes(corrected, 160u, clock, 18u, payload,
                                     sizeof(payload));

    crc = bredr_payload_crc(payload, 144u, uap); /* (20-2)*8 */
    chk = (uint16_t)bredr_read_packed_field(payload, 144u, 16u);
    if (crc == chk)
        return 1000;

    for (int c = 32; c < 64; c++)
    {
        uint8_t p2[20];
        memset(p2, 0, sizeof(p2));
        bredr_dewhiten_air_payload_bytes(corrected, 160u, (uint8_t)c, 18u, p2,
                                         sizeof(p2));
        crc = bredr_payload_crc(p2, 144u, uap);
        chk = (uint16_t)bredr_read_packed_field(p2, 144u, 16u);
        if (crc == chk)
            return 1000;
    }
    return 0;
}

static int decode_payload_header(const uint8_t *stream,
                                 unsigned int sym_off,
                                 int size,
                                 int fec,
                                 int header_bytes,
                                 uint8_t clock,
                                 int *payload_length_out)
{
    uint8_t ph[3] = {0};

    if (header_bytes == 2)
    {
        if (size < 16)
            return 0;
        if (fec)
        {
            uint8_t raw[4];
            uint8_t cor[2];
            unsigned int cb = 0u;
            if (size < 30)
                return 0;
            bredr_copy_packed_bits(stream, sym_off, 24u, raw);
            if (bredr_fec_decode_2_3(raw, 24u, cor, &cb) < 0)
                return 0;
            bredr_dewhiten_air_payload_bytes(cor, 16u, clock, 18u, ph, sizeof(ph));
        }
        else
        {
            uint8_t raw[2];
            bredr_copy_packed_bits(stream, sym_off, 16u, raw);
            bredr_dewhiten_air_payload_bytes(raw, 16u, clock, 18u, ph, sizeof(ph));
        }
        *payload_length_out = (int)bredr_read_packed_field(ph, 3u, 10u) + 4;
    }
    else
    {
        if (size < 8)
            return 0;
        if (fec)
        {
            uint8_t raw[2];
            uint8_t cor[1];
            unsigned int cb = 0u;
            if (size < 15)
                return 0;
            bredr_copy_packed_bits(stream, sym_off, 12u, raw);
            if (bredr_fec_decode_2_3(raw, 12u, cor, &cb) < 0)
                return 0;
            bredr_dewhiten_air_payload_bytes(cor, 8u, clock, 18u, ph, sizeof(ph));
        }
        else
        {
            uint8_t raw[1];
            bredr_copy_packed_bits(stream, sym_off, 8u, raw);
            bredr_dewhiten_air_payload_bytes(raw, 8u, clock, 18u, ph, sizeof(ph));
        }
        *payload_length_out = (int)bredr_read_packed_field(ph, 3u, 5u) + 3;
    }
    return 1;
}

static int dm(const bredr_frame_t *frame, uint8_t clock, uint8_t type,
              uint8_t uap)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    int header_bytes = 2;
    int max_length;
    unsigned int sym_off = 0u;
    int payload_length;

    switch (type & 0x0Fu)
    {
    case PT_DV:
        sym_off = 80u;
        size -= 80;
        header_bytes = 1;
        max_length = 12;
        break;
    case PT_DM1:
        header_bytes = 1;
        max_length = 20;
        break;
    case PT_DM3:
        max_length = 125;
        break;
    case PT_DM5:
        max_length = 228;
        break;
    default:
        return 0;
    }

    if (!decode_payload_header(stream, sym_off, size, 1, header_bytes,
                               clock, &payload_length))
        return 0;
    if (payload_length > max_length)
        return 1;

    int bitlength = payload_length * 8;
    if (bitlength > size)
        return 1;

    unsigned int encoded = (unsigned int)bitlength * 15u / 10u;
    if (encoded > (unsigned)size)
        return 1;

    uint8_t raw[400];
    uint8_t corrected[300];
    uint8_t payload[300];
    unsigned int cb = 0u;

    bredr_copy_packed_bits(stream, sym_off, encoded, raw);
    if (bredr_fec_decode_2_3(raw, encoded, corrected, &cb) < 0)
        return 0;

    memset(payload, 0, sizeof(payload));
    bredr_dewhiten_air_payload_bytes(corrected, (unsigned)bitlength, clock, 18u,
                                     payload, sizeof(payload));

    uint16_t crc = bredr_payload_crc(payload, (unsigned)(payload_length - 2) * 8u,
                                     uap);
    uint16_t chk = (uint16_t)bredr_read_packed_field(payload, (unsigned)(payload_length - 2) * 8u, 16u);
    if (crc == chk)
        return 10;
    return 2;
}

static int dh(const bredr_frame_t *frame, uint8_t clock, uint8_t type,
              uint8_t uap)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    int header_bytes = 2;
    int max_length;
    int payload_length;

    switch (type & 0x0Fu)
    {
    case PT_AUX1:
    case PT_DH1:
        header_bytes = 1;
        max_length = 30;
        break;
    case PT_DH3:
        max_length = 187;
        break;
    case PT_DH5:
        max_length = 343;
        break;
    default:
        return 0;
    }

    if (!decode_payload_header(stream, 0u, size, 0, header_bytes, clock,
                               &payload_length))
        return 0;
    if (payload_length > max_length)
        return 1;

    int bitlength = payload_length * 8;
    if (bitlength > size)
        return 1;

    uint8_t payload[300];
    memset(payload, 0, sizeof(payload));
    bredr_dewhiten_air_payload_bytes(stream, (unsigned)bitlength, clock, 18u,
                                     payload, sizeof(payload));

    if (type == PT_AUX1)
        return 2;

    uint16_t crc = bredr_payload_crc(payload, (unsigned)(payload_length - 2) * 8u,
                                     uap);
    uint16_t chk = (uint16_t)bredr_read_packed_field(payload, (unsigned)(payload_length - 2) * 8u, 16u);
    if (crc == chk)
        return 10;
    return 2;
}

static int ev3(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    int maxlength = 32;
    uint8_t payload[32];
    uint8_t chunk[1];

    memset(payload, 0, sizeof(payload));

    for (int payload_length = 0; payload_length < maxlength; payload_length++)
    {
        int bits = payload_length * 8;
        if ((bits + 8) > size)
            return 1;

        memset(chunk, 0, sizeof(chunk));
        bredr_dewhiten_air_payload_bytes(stream, 8u, clock,
                                         (unsigned)(18 + bits), chunk, 1);
        payload[payload_length] = chunk[0];

        if (payload_length > 2)
        {
            unsigned int data_bits = (unsigned)(payload_length - 2) * 8u;
            uint16_t crc = bredr_payload_crc(payload, data_bits, uap);
            uint16_t chk = (uint16_t)bredr_read_packed_field(payload, data_bits, 16u);
            if (crc == chk)
                return 10;
        }
    }
    return 2;
}

static int ev4(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    int maxlength = 1470;
    int minlength = 45;
    int syms = 0;
    int bits = 0;
    int payload_length = 1;
    uint8_t payload[200];
    uint8_t corrected[2];
    uint8_t chunk[2];
    unsigned int cb = 0u;

    memset(payload, 0, sizeof(payload));

    while (syms < maxlength)
    {
        if (syms + 15 > size)
            return 1;

        uint8_t raw[2];
        bredr_copy_packed_bits(stream, (unsigned)syms, 15u, raw);
        if (bredr_fec_decode_2_3(raw, 15u, corrected, &cb) < 0)
        {
            if (syms < minlength)
                return 0;
            return 1;
        }

        memset(chunk, 0, sizeof(chunk));
        bredr_dewhiten_air_payload_bytes(corrected, 10u, clock,
                                         (unsigned)(18 + bits), chunk, 2);
        for (int i = 0; i < 10; i++)
            bredr_set_packed_bit(payload, (unsigned)(bits + i),
                                 bredr_get_packed_bit(chunk, (unsigned)i));

        while (payload_length > 2 && payload_length * 8 <= bits)
        {
            unsigned int data_bits = (unsigned)(payload_length - 2) * 8u;
            uint16_t crc = bredr_payload_crc(payload, data_bits, uap);
            uint16_t chk = (uint16_t)bredr_read_packed_field(payload, data_bits, 16u);
            if (crc == chk)
                return 10;
            payload_length++;
        }

        syms += 15;
        bits += 10;
    }
    return 2;
}

static int ev5(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    int maxlength = 182;
    uint8_t payload[200];
    uint8_t chunk[1];

    memset(payload, 0, sizeof(payload));

    for (int payload_length = 0; payload_length < maxlength; payload_length++)
    {
        int bits = payload_length * 8;
        if ((bits + 8) > size)
            return 1;

        memset(chunk, 0, sizeof(chunk));
        bredr_dewhiten_air_payload_bytes(stream, 8u, clock,
                                         (unsigned)(18 + bits), chunk, 1);
        payload[payload_length] = chunk[0];

        if (payload_length > 2)
        {
            unsigned int data_bits = (unsigned)(payload_length - 2) * 8u;
            uint16_t crc = bredr_payload_crc(payload, data_bits, uap);
            uint16_t chk = (uint16_t)bredr_read_packed_field(payload, data_bits, 16u);
            if (crc == chk)
                return 10;
        }
    }
    return 2;
}

static int hv(const bredr_frame_t *frame, uint8_t clock, uint8_t type)
{
    const uint8_t *stream = frame->air_payload;
    int size = (int)frame->air_payload_bits;
    uint8_t corrected[10];
    uint8_t payload[30];
    unsigned int cb = 0u;

    if (size < 240)
        return 1;

    switch (type & 0x0Fu)
    {
    case PT_HV1:
        if (bredr_fec_decode_1_3(stream, 240u, corrected, &cb) < 0)
            return 0;
        memset(payload, 0, sizeof(payload));
        bredr_dewhiten_air_payload_bytes(corrected, 80u, clock, 18u, payload,
                                         sizeof(payload));
        break;
    case PT_HV2:
    {
        uint8_t raw[20];
        uint8_t cor[20];
        bredr_copy_packed_bits(stream, 0u, 240u, raw);
        if (bredr_fec_decode_2_3(raw, 240u, cor, &cb) < 0)
            return 0;
        memset(payload, 0, sizeof(payload));
        bredr_dewhiten_air_payload_bytes(cor, 160u, clock, 18u, payload,
                                         sizeof(payload));
        break;
    }
    case PT_HV3:
        memset(payload, 0, sizeof(payload));
        bredr_dewhiten_air_payload_bytes(stream, 240u, clock, 18u, payload,
                                         sizeof(payload));
        break;
    default:
        return 1;
    }
    return 2;
}

static int crc_check(const bredr_frame_t *frame, uint8_t clock,
                     uint8_t type, uint8_t uap)
{
    int retval = 1;

    switch (type & 0x0Fu)
    {
    case PT_FHS:
        retval = fhs(frame, clock, uap);
        break;
    case PT_DV:
    case PT_DM1:
    case PT_DM3:
    case PT_DM5:
        retval = dm(frame, clock, type, uap);
        break;
    case PT_DH1:
    case PT_DH3:
    case PT_DH5:
        retval = dh(frame, clock, type, uap);
        break;
    case PT_HV3: /* EV3 */
        retval = ev3(frame, clock, uap);
        break;
    case PT_EV4:
        retval = ev4(frame, clock, uap);
        break;
    case PT_EV5:
        retval = ev5(frame, clock, uap);
        break;
    case PT_HV1:
        retval = hv(frame, clock, type);
        break;
    default:
        break;
    }

    if (retval == 0 && (type != PT_FHS && type != PT_DM1 && type != PT_HV1))
        return 1;
    if (retval > 1 && (type == PT_HV3 || type == PT_EV5))
        return 1;
    return retval;
}

static int process(bredr_recovery_state_t *st,
                   const bredr_frame_t *frame,
                   uint32_t clkn,
                   bredr_recovery_result_t *out)
{
    if (!st || !frame || !frame->has_header)
        return 0;

    uint32_t clk1 = clkn >> 1;

    if (!st->got_first_packet)
        st->first_pkt_time = clk1;

    st->packets_observed++;

    int remaining = 0;
    int first_clock = 0;

    for (int count = 0; count < CLK6_CANDIDATES; count++)
    {
        if (st->clock6_candidates[count] > -1 || !st->got_first_packet)
        {
            int clock = (count + (int)(clk1 - st->first_pkt_time)) & 0x3f;

            uint8_t type = 0u;
            uint8_t UAP = try_clock(frame, (uint8_t)clock, &type);

            int crc_chk = -1;

            if (!st->got_first_packet || UAP == (uint8_t)st->clock6_candidates[count])
                crc_chk = crc_check(frame, (uint8_t)clock, type, UAP);

            if (st->uap_valid && UAP != st->uap)
                crc_chk = -1;

            switch (crc_chk)
            {
            case -1:
            case 0:
                st->clock6_candidates[count] = -1;
                break;
            case 1:
            case 2:
                st->clock6_candidates[count] = (int)UAP;
                first_clock = count;
                remaining++;
                break;
            default:
                st->clk_offset = (count - (int)(st->first_pkt_time & 0x3fu)) & 0x3f;
                st->uap = UAP;
                st->uap_valid = 1;
                st->clk6_valid = 1;
                out->uap = UAP;
                out->clk6_hint = (uint8_t)st->clk_offset;
                return 1;
            }
        }
    }

    st->got_first_packet = 1;

    if (remaining == 1)
    {
        st->clk_offset = (first_clock - (int)(st->first_pkt_time & 0x3fu)) & 0x3f;
        st->uap = (uint8_t)st->clock6_candidates[first_clock];
        st->uap_valid = 1;
        st->clk6_valid = 1;
        out->uap = st->uap;
        out->clk6_hint = (uint8_t)st->clk_offset;
        return 1;
    }

    if (remaining == 0)
    {
        state_clear(st);
        return 0;
    }

    return 0;
}

int bredr_recovery_process_packet(bredr_recovery_state_t *state,
                                  const bredr_frame_t *frame,
                                  int channel,
                                  uint32_t clkn,
                                  bredr_recovery_result_t *out)
{
    (void)channel;
    if (!out)
        return 0;
    return process(state, frame, clkn, out);
}
