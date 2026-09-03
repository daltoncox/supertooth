/**
 * @file bredr_clock_recovery.c
 * @brief BR/EDR UAP / CLK1-6 clock recovery and tracking.
 *
 * Clean-room reimplementation of libbtbb's BR/EDR UAP/CLK1-6 recovery.  All
 * low-level primitives (1/3 and 2/3 FEC decode, dewhitening, HEC decode,
 * payload CRC) come from bredr_codec.c; this file reproduces libbtbb's
 * candidate-selection algorithm (btbb_uap_from_header) and per-type CRC logic
 * (fhs / DM / DH / EV3 / EV4 / EV5 / HV), and then owns ongoing clock tracking
 * (the clock_offset correction in recover_clock_drift).
 *
 * Invariants that must match libbtbb:
 *   - Recovery clock is CLK1 (clkn >> 1).  libbtbb stores pkt->clkn = clkn>>1
 *     in btbb_packet_set_data(); every whitening/clock computation uses it.
 *   - Packet type numbering follows libbtbb: DM3=0xA, DH3=0xB, EV4=0xC,
 *     EV5=0xD, DM5=0xE, DH5=0xF (HV3/EV3 share 0x7, DV=0x8, AUX1=0x9).
 *   - verify_payload_crc() contract: 0 hard negative (only FHS/DM1/HV1 may
 *     prune), 1 inconclusive, 2 inconclusive-but-likely, >1 (10/1000) positive.
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
#define PT_POLL  1
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

void bredr_recovery_set_frame_dump(FILE *file)
{
    g_frame_dump = file;
}

/* ---------------------------------------------------------------------------
 * Acquisition working-state helpers
 * --------------------------------------------------------------------------- */

void bredr_recovery_reset(bredr_piconet_t *pnet)
{
    if (!pnet)
        return;
    for (int i = 0; i < BREDR_CLK6_CANDIDATES; i++)
        pnet->recovery_candidates[i] = -1;
    memset(pnet->recovery_esco_lt_mask, 0, sizeof(pnet->recovery_esco_lt_mask));
    memset(pnet->recovery_acl_lt_mask, 0, sizeof(pnet->recovery_acl_lt_mask));
    /* Preserve pnet->uap (last known value for device-row display); only
     * validity is cleared here. */
    pnet->uap_valid = 0;
    pnet->clock_offset = 0;
    pnet->drift_candidate = 0;
    pnet->recovery_first_pkt_time = 0u;
    pnet->recovery_got_first_packet = 0;
}

/* --- header decode (mirror libbtbb try_clock) --- */

/* FEC-decode and unwhiten the header for a single CLK1-6 candidate, returning
 * the candidate UAP via the HEC or 0 on decode failure.  When hdr_out is
 * non-NULL the decoded header fields are also returned so callers can run
 * slot-role sanity checks.  When fec_errors_out is non-NULL it receives the
 * 1/3-FEC error count of the decode (0 == a 100% correct header, which is the
 * only kind of header we ever drive recovery from). */
static uint8_t decode_uap_for_clock(const bredr_frame_t *frame,
                                    uint8_t clock,
                                    uint8_t *type_out,
                                    bredr_decoded_header_t *hdr_out,
                                    int *fec_errors_out)
{
    uint8_t packed_header[7];
    uint8_t header[18];
    uint8_t unwhitened[18];
    unsigned int decoded_bits = 0u;
    int be;

    if (hdr_out)
        memset(hdr_out, 0, sizeof(*hdr_out));
    if (fec_errors_out)
        *fec_errors_out = -1;

    bredr_pack_header_raw(frame->header_raw, packed_header);
    be = bredr_fec_decode_1_3(packed_header, 54u, header, &decoded_bits);
    if (fec_errors_out)
        *fec_errors_out = be;
    if (be < 0 || decoded_bits != 18u || be >= 4)
        return 0;

    bredr_xor_whitening_payload(header, 18u, clock, 0u, unwhitened,
                                sizeof(unwhitened));

    uint16_t hdr_data = (uint16_t)bredr_read_packed_field(unwhitened, 0u, 10u);
    uint8_t hec = (uint8_t)bredr_read_packed_field(unwhitened, 10u, 8u);
    *type_out = (uint8_t)bredr_read_packed_field(unwhitened, 3u, 4u);

    if (hdr_out)
    {
        hdr_out->lt_addr = (uint8_t)(unwhitened[0] | (unwhitened[1] << 1) |
                                     (unwhitened[2] << 2));
        hdr_out->type = *type_out;
        hdr_out->flow = unwhitened[7];
        hdr_out->arqn = unwhitened[8];
        hdr_out->seqn = unwhitened[9];
        hdr_out->hec = hec;
    }

    return bredr_decode_uap_from_hec(hdr_data, hec);
}

/* Reject CLK1-6 candidates whose decoded header is illegal for the implied slot
 * role.  The master transmits in even slots (CLK1 == 0) and the peripheral
 * (slave) in odd slots (CLK1 == 1).  Returns 0 if the candidate is plausible,
 * -1 if it must be pruned.  Checks:
 *   - LT_ADDR == 0 (broadcast) never originates from a peripheral.
 *   - POLL (0x1) and FHS (0x2) are master-only, so never in a peripheral slot.
 *   - HV1/HV2/HV3 (0x5/0x6/0x7) reserve FLOW/ARQN/SEQN as 0, so any of those
 *     bits set indicates a malformed (or mis-clocked) header. */
static int sanity_check_header(const bredr_decoded_header_t *hdr,
                               uint8_t clk1_6)
{
    int peripheral_timeslot = (clk1_6 & 1u);

    if (peripheral_timeslot)
    {
        if (hdr->lt_addr == 0u)
            return -1; /* broadcast never originates from a peripheral */

        if (hdr->type == PT_POLL)
            return -1; /* POLL is a master-only packet */

        if (hdr->type == PT_FHS)
            return -1; /* FHS is a master-only packet */
    }

    if (hdr->type == PT_HV1 || hdr->type == PT_HV2 || hdr->type == PT_HV3)
        if (hdr->flow || hdr->arqn || hdr->seqn)
            return -1; /* SCO header reserves FLOW/ARQN/SEQN as 0 */

    return 0;
}

/* --- payload CRC checkers (mirror libbtbb fhs/DM/DH/EV3/EV4/EV5/HV) --- */
/* return: 0 hard negative, 1 inconclusive, 2 likely, 10 positive, 1000 FHS. */

static int verify_fhs_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
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
    bredr_xor_whitening_payload(corrected, 160u, clock, 18u, payload,
                                sizeof(payload));

    crc = bredr_payload_crc(payload, 144u, uap); /* (20-2)*8 */
    chk = (uint16_t)bredr_read_packed_field(payload, 144u, 16u);
    if (crc == chk)
        return 1000;

    for (int c = 32; c < 64; c++)
    {
        uint8_t p2[20];
        memset(p2, 0, sizeof(p2));
        bredr_xor_whitening_payload(corrected, 160u, (uint8_t)c, 18u, p2,
                                    sizeof(p2));
        crc = bredr_payload_crc(p2, 144u, uap);
        chk = (uint16_t)bredr_read_packed_field(p2, 144u, 16u);
        if (crc == chk)
            return 1000;
    }
    return 0;
}

/* Decode the 2- or 1-byte ACL payload header to recover the payload length. */
static int decode_acl_payload_length(const uint8_t *stream,
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
            bredr_xor_whitening_payload(cor, 16u, clock, 18u, ph, sizeof(ph));
        }
        else
        {
            uint8_t raw[2];
            bredr_copy_packed_bits(stream, sym_off, 16u, raw);
            bredr_xor_whitening_payload(raw, 16u, clock, 18u, ph, sizeof(ph));
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
            bredr_xor_whitening_payload(cor, 8u, clock, 18u, ph, sizeof(ph));
        }
        else
        {
            uint8_t raw[1];
            bredr_copy_packed_bits(stream, sym_off, 8u, raw);
            bredr_xor_whitening_payload(raw, 8u, clock, 18u, ph, sizeof(ph));
        }
        *payload_length_out = (int)bredr_read_packed_field(ph, 3u, 5u) + 3;
    }
    return 1;
}

static int verify_dm_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t type,
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

    if (!decode_acl_payload_length(stream, sym_off, size, 1, header_bytes,
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
    bredr_xor_whitening_payload(corrected, (unsigned)bitlength, clock, 18u,
                                payload, sizeof(payload));

    uint16_t crc = bredr_payload_crc(payload, (unsigned)(payload_length - 2) * 8u,
                                     uap);
    uint16_t chk = (uint16_t)bredr_read_packed_field(payload, (unsigned)(payload_length - 2) * 8u, 16u);
    if (crc == chk)
        return 10;
    return 2;
}

static int verify_dh_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t type,
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

    if (!decode_acl_payload_length(stream, 0u, size, 0, header_bytes, clock,
                                  &payload_length))
        return 0;
    if (payload_length > max_length)
        return 1;

    int bitlength = payload_length * 8;
    if (bitlength > size)
        return 1;

    uint8_t payload[300];
    memset(payload, 0, sizeof(payload));
    bredr_xor_whitening_payload(stream, (unsigned)bitlength, clock, 18u,
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

static int verify_ev3_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
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
        bredr_xor_whitening_payload(stream, 8u, clock,
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

static int verify_ev4_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
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
        bredr_xor_whitening_payload(corrected, 10u, clock,
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

static int verify_ev5_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t uap)
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
        bredr_xor_whitening_payload(stream, 8u, clock,
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

static int verify_hv_payload(const bredr_frame_t *frame, uint8_t clock, uint8_t type)
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
        bredr_xor_whitening_payload(corrected, 80u, clock, 18u, payload,
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
        bredr_xor_whitening_payload(cor, 160u, clock, 18u, payload,
                                    sizeof(payload));
        break;
    }
    case PT_HV3:
        memset(payload, 0, sizeof(payload));
        bredr_xor_whitening_payload(stream, 240u, clock, 18u, payload,
                                    sizeof(payload));
        break;
    default:
        return 1;
    }
    return 2;
}

static int verify_payload_crc(const bredr_frame_t *frame, uint8_t clock,
                              uint8_t type, uint8_t uap)
{
    int retval = 1;

    switch (type & 0x0Fu)
    {
    case PT_FHS:
        retval = verify_fhs_payload(frame, clock, uap);
        break;
    case PT_DV:
    case PT_DM1:
    case PT_DM3:
    case PT_DM5:
        retval = verify_dm_payload(frame, clock, type, uap);
        break;
    case PT_DH1:
    case PT_DH3:
    case PT_DH5:
        retval = verify_dh_payload(frame, clock, type, uap);
        break;
    case PT_HV3: /* EV3 */
        retval = verify_ev3_payload(frame, clock, uap);
        break;
    case PT_EV4:
        retval = verify_ev4_payload(frame, clock, uap);
        break;
    case PT_EV5:
        retval = verify_ev5_payload(frame, clock, uap);
        break;
    case PT_HV1:
        retval = verify_hv_payload(frame, clock, type);
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
 * eSCO LT_ADDR discipline
 * --------------------------------------------------------------------------- */

/* Enforce the Bluetooth rule that an LT_ADDR assigned to an eSCO link is used
 * for eSCO packets only.  For a confident (CRC-verified) decode of candidate
 * slot @p count, record the LT_ADDR as eSCO (EV4/EV5) or ACL (DM/DH/DV); if the
 * same LT_ADDR was already recorded under the opposite family, return -1 to
 * prune the candidate, else update the masks and return @p crc_chk unchanged. */
static int apply_esco_lt_addr_rule(bredr_piconet_t *pnet, int count,
                                   uint8_t lt_addr, uint8_t type, int crc_chk)
{
    if (crc_chk != 10 || lt_addr == 0u)
        return crc_chk;

    int is_esco = (type == PT_EV4 || type == PT_EV5);
    int is_acl  = (type == PT_DV  || type == PT_DM1 || type == PT_DM3 ||
                   type == PT_DM5 || type == PT_DH1 || type == PT_DH3 ||
                   type == PT_DH5);
    if (!is_esco && !is_acl)
        return crc_chk;

    uint8_t bit = (uint8_t)(1u << lt_addr);
    uint8_t *self_mask  = is_esco ? pnet->recovery_esco_lt_mask
                                  : pnet->recovery_acl_lt_mask;
    uint8_t *other_mask = is_esco ? pnet->recovery_acl_lt_mask
                                  : pnet->recovery_esco_lt_mask;

    if (other_mask[count] & bit)
        return -1; /* LT_ADDR already seen as the opposite family */

    self_mask[count] |= bit;
    return crc_chk;
}

/* ---------------------------------------------------------------------------
 * Acquisition: 64 CLK1-6 candidate solve
 * --------------------------------------------------------------------------- */

/* Accumulate UAP/CLK1-6 candidates from one header packet.  On a confident
 * solve it records pnet->uap, pnet->uap_valid and a tentative clock_offset
 * (the CLK1-6 hint), and returns 1.  Returns 0 while still ambiguous. */
static int solve_uap_clock_candidates(bredr_piconet_t *pnet,
                                     const bredr_frame_t *frame,
                                     uint32_t clkn)
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
            bredr_decoded_header_t hdr;
            int fec_err = 0;
            uint8_t UAP = decode_uap_for_clock(frame, (uint8_t)clock, &type, &hdr, &fec_err);

            int crc_chk = -1;

            /* Recovery is only ever driven by a 100% correct header: any FEC
             * error in the 1/3 decode means the packet is not reliable enough
             * to update UAP/clock state, so the candidate is pruned. */
            if (fec_err != 0)
                crc_chk = -1;
            else if (!pnet->recovery_got_first_packet ||
                     UAP == (uint8_t)pnet->recovery_candidates[count])
                crc_chk = verify_payload_crc(frame, (uint8_t)clock, type, UAP);

            if (pnet->uap_valid && UAP != pnet->uap)
                crc_chk = -1;

            /* Slot-role sanity checks on the decoded header.  An illegal
             * combination (e.g. broadcast/POLL/FHS in a peripheral timeslot, or
             * a non-zero SCO reserved bit) invalidates the clock candidate
             * regardless of the CRC result. */
            if (sanity_check_header(&hdr, (uint8_t)clock) != 0)
                crc_chk = -1;

            /* Enforce eSCO LT_ADDR discipline: an LT_ADDR must not be decoded
             * as both eSCO (EV4/EV5) and ACL (DM/DH) under the same clock. */
            crc_chk = apply_esco_lt_addr_rule(pnet, count, hdr.lt_addr,
                                              type, crc_chk);

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
                pnet->uap_valid = 1;
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
        pnet->uap_valid = 1;
        return 1;
    }

    if (remaining == 0)
    {
        bredr_recovery_reset(pnet);
        return 0;
    }

    return 0;
}

/* ---------------------------------------------------------------------------
 * CLK1-6 disambiguation against historical packets
 * --------------------------------------------------------------------------- */

/** Maximum age (rx_clk_1600 ticks) for historical packets used in CLK1-6
 *  narrowing.  625 µs × 8000 ≈ 5 seconds. */
#define CLK1_6_HISTORY_CUTOFF_CLK1600 8000u

/* Narrow the @p n CLK1-6 candidates in @p candidates against the piconet's
 * historical packets (which advance one CLK1-6 tick per rx_clk_1600 slot),
 * returning the number of surviving candidates.  The scan is bounded both by
 * the history age cutoff and by the ring-buffer depth
 * (BREDR_PICONET_QUEUE_SIZE recent packets). */
static int narrow_clock_via_history(const bredr_piconet_t *pnet,
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

        /* Skip packets without a decodable header.  Access-code acceptance
         * (including its error tolerance) is enforced solely by the bitstream
         * decoder, so no separate AC filter belongs here. */
        if (!hist_frame->has_header)
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

/* ---------------------------------------------------------------------------
 * Acquisition entry point (called by bredr_recovery_process while the clock
 * is not yet known)
 * --------------------------------------------------------------------------- */

/* Feed packets to the candidate solver; once a UAP is found, narrow the 64
 * CLK1-6 candidates against history and establish the clock via
 * bredr_piconet_set_uap() (or via the recovery hint when a single candidate
 * cannot be isolated).  Returns 1 if the clock became known. */
static int acquire_uap_and_clock(bredr_piconet_t *pnet,
                                 const bredr_event_t *event,
                                 uint32_t clkn,
                                 uint32_t rx_clk_1600)
{
    const bredr_frame_t *frame = &event->frame;

    if (!pnet || !frame->has_header)
        return 0;
    if (pnet->uap_valid && pnet->clk_known)
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

    if (!solve_uap_clock_candidates(pnet, frame, clkn))
        return 0;

    uint8_t uap = pnet->uap;
    uint8_t btbb_clk6 = (uint8_t)pnet->clock_offset; /* recovery backend CLK1-6 hint */
    if (!pnet->uap_valid)
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
    valid_n = narrow_clock_via_history(pnet, event, uap, valid_clk, valid_n);

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
    return 0;
}

/* ---------------------------------------------------------------------------
 * Ongoing clock tracking (post-acquisition)
 * --------------------------------------------------------------------------- */

/* Verify (and correct for drift) the tracked clock_offset for a header packet
 * received at rx_clk_1600.  The tracked offset is always tried first; a base
 * hit clears any pending drift suspicion and builds confidence.  A ±1/±2 hit
 * is only suspicion: it is parked in drift_candidate with clock_offset and
 * tracking_state left untouched, and applied only when the next packet misses
 * the base offset again with the SAME delta.  A conflicting delta clears the
 * suspicion without touching tracking state.  Only a packet matching no
 * candidate at all decays tracking_state (cleared at zero, as before).
 * The header must be 100% correct (zero FEC errors) before any clock state
 * is touched.  Returns 1 if the HEC validated on any tried candidate. */
static int recover_clock_drift(bredr_piconet_t *pnet,
                               const bredr_frame_t *frame,
                               uint32_t rx_clk_1600)
{
    if (!pnet || !frame)
        return 0;

    /* Central CLK1-6 expected for this packet, given the tracked offset. */
    uint8_t base = (uint8_t)((rx_clk_1600 + pnet->clock_offset) & 0x3Fu);

    static const int offsets[] = {0, 1, -1, 2, -2};
    int hit_delta = 0;
    int found = 0;
    for (int k = 0; k < 5; k++)
    {
        int candidate = ((base + offsets[k]) + 64) % 64;
        if (bredr_hec_ok_for_clk6_clean(frame, pnet->uap, (uint8_t)candidate))
        {
            hit_delta = offsets[k];
            found = 1;
            break;
        }
    }

    if (!found)
    {
        /* No valid clock for this packet: drop any pending suspicion and
         * decay lock confidence as before. */
        pnet->drift_candidate = 0;
        if (pnet->tracking_state > 0)
            pnet->tracking_state--;

        if (pnet->tracking_state == 0)
            pnet->clk_known = 0;

        return 0;
    }

    if (hit_delta == 0)
    {
        /* Tracked clock still correct: any prior suspicion is disproven. */
        pnet->drift_candidate = 0;
        if (pnet->tracking_state < 5)
            pnet->tracking_state++;
        pnet->clk_known = 1;
        return 1;
    }

    if (pnet->drift_candidate == 0)
    {
        /* First sighting of this drift: record it, but hold the tracked
         * offset and confidence steady until a second packet confirms. */
        pnet->drift_candidate = hit_delta;
        return 1;
    }

    if (pnet->drift_candidate == hit_delta)
    {
        /* Same delta on two consecutive packets: confirmed drift. */
        pnet->clock_offset = ((pnet->clock_offset + hit_delta) + 64) % 64;
        pnet->drift_candidate = 0;
        if (pnet->tracking_state < 5)
            pnet->tracking_state++;
        pnet->clk_known = 1;
        return 1;
    }

    /* Conflicting drift evidence: discard the suspicion, keep tracking. */
    pnet->drift_candidate = 0;
    return 1;
}

/* ---------------------------------------------------------------------------
 * Unified recovery entry point
 * --------------------------------------------------------------------------- */

/* Drive UAP/clock recovery for a single received event.  While the piconet
 * has no confirmed clock it acquires the UAP and clock offset; once those are
 * known it merely tracks and corrects for clock drift.  Returns 1 if the
 * clock is locked for this packet (acquired or HEC-validated). */
int bredr_recovery_process(bredr_piconet_t *pnet,
                           const bredr_event_t *event)
{
    if (!pnet || !event)
        return 0;

    const bredr_frame_t *frame = &event->frame;
    if (!frame->has_header)
        return 0;

    uint32_t rx_clk_1600 = bredr_sample_to_rx_clk_1600(event);
    uint32_t clkn = bredr_sample_to_clkn(event);

    if (pnet->uap_valid && pnet->clk_known)
        return recover_clock_drift(pnet, frame, rx_clk_1600);

    return acquire_uap_and_clock(pnet, event, clkn, rx_clk_1600);
}
