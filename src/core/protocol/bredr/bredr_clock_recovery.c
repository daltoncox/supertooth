/**
 * @file bredr_clock_recovery.c
 * @brief BR/EDR UAP / CLK1-6 clock recovery and tracking.
 *
 * Clean-room reimplementation of libbtbb's BR/EDR UAP/CLK1-6 recovery.  All
 * low-level primitives (1/3 and 2/3 FEC decode, dewhitening, HEC decode,
 * payload CRC) come from bredr_codec.c; this file reproduces libbtbb's
 * candidate-selection algorithm (btbb_uap_from_header) and per-type CRC logic
 * (crc_check / fhs / DM / DH / EV3 / EV4 / EV5 / HV), and then owns ongoing
 * clock tracking (the clock_offset correction in bredr_clock_track_packet).
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
 *
 * All recovery/tracking state lives in the target bredr_piconet_t; there is no
 * separate recovery struct.
 */

#include "bredr_clock_recovery.h"

#include "bredr_codec.h"
#include "bredr_bitstream_decoder.h"

#include <string.h>
#include <stdio.h>

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

/* ---------------------------------------------------------------------------
 * Frame dump (offline recovery replay)
 * --------------------------------------------------------------------------- */

#define FRAME_DUMP_MAGIC   0x53544C44u /* "STLD" */
#define FRAME_DUMP_VERSION 1u

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t lap;
    int32_t channel;
    uint32_t clkn;
    uint64_t header_raw;
    uint32_t air_payload_bits;
    uint8_t air_payload[BR_MAX_AIR_PAYLOAD_BYTES];
} frame_dump_rec_t;

static FILE *g_frame_dump = NULL;

void bredr_clock_recovery_set_frame_dump(FILE *file)
{
    g_frame_dump = file;
}

/* ---------------------------------------------------------------------------
 * Acquisition working-state helpers
 * --------------------------------------------------------------------------- */

static void state_clear(bredr_piconet_t *pnet)
{
    for (int i = 0; i < BREDR_CLK6_CANDIDATES; i++)
        pnet->recovery_candidates[i] = -1;
    pnet->uap = 0u;
    pnet->uap_found = 0;
    pnet->clock_offset = 0;
    pnet->recovery_first_pkt_time = 0u;
    pnet->recovery_got_first_packet = 0;
}

void bredr_recovery_global_init(uint8_t max_ac_errors)
{
    (void)max_ac_errors;
}

void bredr_piconet_recovery_reset(bredr_piconet_t *pnet)
{
    if (pnet)
        state_clear(pnet);
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

/* ---------------------------------------------------------------------------
 * Acquisition: 64 CLK1-6 candidate solve
 * --------------------------------------------------------------------------- */

static int process(bredr_piconet_t *pnet,
                   const bredr_frame_t *frame,
                   uint32_t clkn,
                   bredr_recovery_result_t *out)
{
    if (!pnet || !frame || !frame->has_header)
        return 0;

    uint32_t clk1 = clkn >> 1;

    if (!pnet->recovery_got_first_packet)
        pnet->recovery_first_pkt_time = clk1;

    int remaining = 0;
    int first_clock = 0;

    for (int count = 0; count < BREDR_CLK6_CANDIDATES; count++)
    {
        if (pnet->recovery_candidates[count] > -1 || !pnet->recovery_got_first_packet)
        {
            int clock = (count + (int)(clk1 - pnet->recovery_first_pkt_time)) & 0x3f;

            uint8_t type = 0u;
            uint8_t UAP = try_clock(frame, (uint8_t)clock, &type);

            int crc_chk = -1;

            if (!pnet->recovery_got_first_packet ||
                UAP == (uint8_t)pnet->recovery_candidates[count])
                crc_chk = crc_check(frame, (uint8_t)clock, type, UAP);

            if (pnet->uap_found && UAP != pnet->uap)
                crc_chk = -1;

            switch (crc_chk)
            {
            case -1:
            case 0:
                pnet->recovery_candidates[count] = -1;
                break;
            case 1:
            case 2:
                pnet->recovery_candidates[count] = (int)UAP;
                first_clock = count;
                remaining++;
                break;
            default:
                pnet->clock_offset =
                    (count - (int)(pnet->recovery_first_pkt_time & 0x3fu)) & 0x3f;
                pnet->uap = UAP;
                pnet->uap_found = 1;
                out->uap = UAP;
                out->clk6_hint = (uint8_t)pnet->clock_offset;
                return 1;
            }
        }
    }

    pnet->recovery_got_first_packet = 1;

    if (remaining == 1)
    {
        pnet->clock_offset =
            (first_clock - (int)(pnet->recovery_first_pkt_time & 0x3fu)) & 0x3f;
        pnet->uap = (uint8_t)pnet->recovery_candidates[first_clock];
        pnet->uap_found = 1;
        out->uap = pnet->uap;
        out->clk6_hint = (uint8_t)pnet->clock_offset;
        return 1;
    }

    if (remaining == 0)
    {
        state_clear(pnet);
        return 0;
    }

    return 0;
}

int bredr_recovery_process_packet(bredr_piconet_t *pnet,
                                   const bredr_frame_t *frame,
                                   int channel,
                                   uint32_t clkn,
                                   bredr_recovery_result_t *out)
{
    (void)channel;
    if (!out)
        return 0;
    return process(pnet, frame, clkn, out);
}

/* ---------------------------------------------------------------------------
 * CLK1-6 disambiguation against historical packets
 * --------------------------------------------------------------------------- */

/** Maximum age (rx_clk_1600 ticks) for historical packets used in CLK1-6
 *  narrowing.  625 µs × 8000 ≈ 5 seconds. */
#define CLK1_6_HISTORY_CUTOFF_CLK1600 8000u

static int narrow_clk6_candidates(const bredr_piconet_t *pnet,
                                   const bredr_event_t *cur_event,
                                   uint8_t uap,
                                   int candidates[64],
                                   int n)
{
    uint32_t cur_clk;

    if (n <= 1 || pnet->queue_fill < 2)
        return n;

    cur_clk = bredr_sample_to_rx_clk_1600(cur_event);

    /* The queue is maintained in start-sample order. The current packet is at
     * the logical tail, so walk backwards through older history and stop once
     * packets fall outside the history window. */
    for (unsigned int i = 1; i < pnet->queue_fill; i++)
    {
        unsigned int idx =
            (pnet->queue_head + BREDR_PICONET_QUEUE_SIZE - 1u - i) % BREDR_PICONET_QUEUE_SIZE;
        const bredr_event_t *hist_event = &pnet->queue[idx];
        const bredr_frame_t *hist_frame = &hist_event->frame;
        uint32_t hist_clk = bredr_sample_to_rx_clk_1600(hist_event);

        /* Older entries only get farther away in time as we walk backward. */
        if ((cur_clk - hist_clk) > CLK1_6_HISTORY_CUTOFF_CLK1600)
            break;

        /* Skip packets without a decodable header inside configured AC tolerance. */
        if (!hist_frame->has_header || hist_frame->ac_errors > BREDR_AC_ERRORS_DEFAULT)
            continue;

        /* CLK1-6 advances one tick per rx_clk_1600 slot.  The CLK1-6 at the
         * historical packet is: (c_current - delta) mod 64. */
        uint32_t delta_mod64 = (cur_clk - hist_clk) & 0x3Fu;

        int j = 0;
        for (int k = 0; k < n; k++)
        {
            uint8_t c_at_hist =
                (uint8_t)((candidates[k] - (int)delta_mod64 + 64) & 0x3F);
            if (bredr_hec_ok_for_clk6(hist_frame, uap, c_at_hist))
                candidates[j++] = candidates[k];
        }
        n = j;

        if (n <= 1)
            break;
    }

    return n;
}

int bredr_clock_recovery_acquire(bredr_piconet_t *pnet,
                                  const bredr_event_t *event,
                                  uint32_t clkn,
                                  uint32_t rx_clk_1600)
{
    const bredr_frame_t *frame = &event->frame;

    if (!pnet || !frame->has_header || frame->ac_errors > BREDR_AC_ERRORS_DEFAULT)
        return 0;
    if (pnet->uap_found && pnet->clk_known)
        return 0;

    /* Optional frame dump for offline recovery replay. */
    if (g_frame_dump)
    {
        frame_dump_rec_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.magic = FRAME_DUMP_MAGIC;
        rec.version = FRAME_DUMP_VERSION;
        rec.lap = frame->lap & 0xFFFFFFu;
        rec.channel = (int32_t)event->meta.channel_index;
        rec.clkn = clkn;
        rec.header_raw = frame->header_raw;
        rec.air_payload_bits = frame->air_payload_bits;
        if (rec.air_payload_bits > BR_MAX_AIR_PAYLOAD_BITS)
            rec.air_payload_bits = BR_MAX_AIR_PAYLOAD_BITS;
        unsigned int nbytes = (rec.air_payload_bits + 7u) / 8u;
        if (nbytes > sizeof(rec.air_payload))
            nbytes = sizeof(rec.air_payload);
        memcpy(rec.air_payload, frame->air_payload, nbytes);
        if (fwrite(&rec, sizeof(rec), 1u, g_frame_dump) == 1u)
            fflush(g_frame_dump);
    }

    bredr_recovery_result_t result = {0};
    if (bredr_recovery_process_packet(pnet, frame, (int)event->meta.channel_index, clkn, &result))
    {
        uint8_t uap = result.uap;
        uint8_t btbb_clk6 = result.clk6_hint; /* recovery backend CLK1-6 hint */
        if (!pnet->uap_found)
            bredr_piconet_set_uap_only(pnet, uap);

        /* Collect all CLK1-6 values that produce a valid HEC for this packet. */
        int valid_clk[64];
        int valid_n = 0;
        for (int c = 0; c < 64; c++)
        {
            if (bredr_hec_ok_for_clk6(frame, uap, (uint8_t)c))
                valid_clk[valid_n++] = c;
        }

        /* Narrow the candidates using historical packets in the ring buffer. */
        valid_n = narrow_clk6_candidates(pnet, event, uap, valid_clk, valid_n);

        if (valid_n == 1)
        {
            /* Unambiguous — use directly. */
            bredr_piconet_set_uap(pnet, uap, (uint8_t)valid_clk[0], rx_clk_1600);
            return 1;
        }
        else if (valid_n > 1)
        {
            /* Still ambiguous after history scan — fall back to the candidate
             * closest to the recovery backend's clock-offset hint. */
            int best = valid_clk[0];
            int best_dist = 64;
            for (int i = 0; i < valid_n; i++)
            {
                int d = valid_clk[i] - (int)btbb_clk6;
                if (d < 0)
                    d = -d;
                if (d > 32)
                    d = 64 - d;
                if (d < best_dist)
                {
                    best_dist = d;
                    best = valid_clk[i];
                }
            }
            bredr_piconet_set_uap(pnet, uap, (uint8_t)best, rx_clk_1600);
            return 1;
        }
        /* valid_n == 0: UAP may be wrong — leave state unchanged and let
         * acquisition accumulate more packets before trying again. */
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * Ongoing clock tracking (post-acquisition)
 * --------------------------------------------------------------------------- */

int bredr_clock_track_packet(bredr_piconet_t *pnet,
                             const bredr_frame_t *frame,
                             uint32_t rx_clk_1600)
{
    if (!pnet || !frame)
        return 0;

    /* Central CLK1-6 expected for this packet, given the tracked offset. */
    uint8_t base = (uint8_t)((rx_clk_1600 + pnet->clock_offset) & 0x3Fu);

    static const int offsets[] = {0, 1, -1, 2, -2};
    for (int k = 0; k < 5; k++)
    {
        int candidate = ((base + offsets[k]) + 64) % 64;
        if (bredr_hec_ok_for_clk6(frame, pnet->uap, (uint8_t)candidate))
        {
            /* Correct the offset for any drift detected between the two clocks. */
            pnet->clock_offset = (uint8_t)((candidate - (int)rx_clk_1600) & 0x3Fu);
            if (pnet->tracking_state < 5)
                pnet->tracking_state++;
            pnet->clk_known = 1;
            return 1;
        }
    }

    if (pnet->tracking_state > 0)
        pnet->tracking_state--;

    if (pnet->tracking_state == 0)
        pnet->clk_known = 0;

    return 0;
}
