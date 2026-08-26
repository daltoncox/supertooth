/**
 * @file bredr_bitstream_decoder.c
 * @brief BR/EDR PHY-layer bitstream processor implementation.
 *
 * See bredr_bitstream_decoder.h for the public API and design notes.
 */

#include "bredr_bitstream_decoder.h"
#include "bredr_codec.h"

#include <string.h>   /* memset, memcpy */
#include <stddef.h>   /* NULL */

/* ---------------------------------------------------------------------------
 * Internal state constants
 * ---------------------------------------------------------------------------*/

#define STATE_SEARCHING 0
#define STATE_DRAINING_TRAILER 1
#define STATE_COLLECTING 2

/* ---------------------------------------------------------------------------
 * Barker code constants
 *
 * Bits 57–63 of any valid BR/EDR sync word form a 7-bit "barker code".
 * All valid sync words carry one of two complementary patterns:
 *   BARKER_A = 0x27 = 0b0100111  (distance 0)
 *   BARKER_B = 0x58 = 0b1011000  (complement of A)
 * We accept windows whose barker field is within Hamming distance 1 of
 * either pattern.
 * ---------------------------------------------------------------------------*/

#define BARKER_A 0x27u
#define BARKER_B 0x58u
#define BARKER_MAX_ERRORS 1u

/* ---------------------------------------------------------------------------
 * Global access-code error tolerance
 *
 * Single, process-wide control for the maximum Hamming distance accepted as a
 * valid access-code match.  Every BR/EDR bitstream decoder reads this value;
 * there is intentionally no per-decoder or per-layer override, so the decoder
 * is the sole gate for access-code acceptance.  Defaults to 0 (strict,
 * byte-perfect access-code match).  Set once before streaming begins.
 * ---------------------------------------------------------------------------*/

static uint8_t g_max_ac_errors = 0u;

void bredr_bitstream_decoder_set_global_max_ac_errors(uint8_t max_ac_errors)
{
    g_max_ac_errors = max_ac_errors;
}

uint8_t bredr_bitstream_decoder_get_global_max_ac_errors(void)
{
    return g_max_ac_errors;
}

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------------*/

/**
 * @brief Compute the 8-bit HEC for a 10-bit header and a known UAP.
 *
 * Implements the HEC LFSR described in Bluetooth Core Spec Vol 2, Part B,
 * §7.4 and Figure 7.3.
 *
 * Generator polynomial: g(D) = D^8 + D^7 + D^5 + D^2 + D + 1
 * XOR mask (D^7..D^0 terms): 0xA7 = 1010_0111
 *
 * The LFSR is pre-loaded with `uap` (UAP0 → leftmost register element =
 * most-significant bit of the byte), then clocked once per header bit
 * (LT_ADDR[0] first, SEQN last).  The resulting register holds the HEC
 * with the first-to-be-transmitted bit in bit 7.
 *
 * @param data  10 header bits, bit 0 = LT_ADDR[0] (first transmitted),
 *              bit 9 = SEQN (last transmitted before HEC).
 * @param uap   8-bit Upper Address Part (UAP0 in bit 0, UAP7 in bit 7).
 * @return      8-bit HEC register; bit 7 is the first bit transmitted.
 */
/**
 * @brief Count the number of set bits in a 64-bit word (Hamming weight).
 */
static uint8_t popcount64(uint64_t x)
{
#ifdef __GNUC__
    return (uint8_t)__builtin_popcountll(x);
#else
    uint8_t n = 0;
    while (x)
    {
        n += (uint8_t)(x & 1u);
        x >>= 1;
    }
    return n;
#endif
}

/**
 * @brief Count the number of set bits in a 7-bit value.
 */
static uint8_t popcount7(uint8_t x)
{
    x &= 0x7Fu;
    x = (uint8_t)((x & 0x55u) + ((x >> 1) & 0x55u));
    x = (uint8_t)((x & 0x33u) + ((x >> 2) & 0x33u));
    x = (uint8_t)((x + (x >> 4)) & 0x0Fu);
    return x;
}

/**
 * @brief Check whether a 64-bit sync-word window passes the barker pre-filter.
 *
 * Extracts the 7-bit barker field from bits 57–63 of the window and checks
 * its Hamming distance from both valid barker patterns.  Returns non-zero
 * if the minimum distance is <= BARKER_MAX_ERRORS.
 */
static int barker_ok(uint64_t window)
{
    uint8_t b = (uint8_t)((window >> 57) & 0x7Fu);
    uint8_t d0 = popcount7((uint8_t)(b ^ BARKER_A));
    uint8_t d1 = popcount7((uint8_t)(b ^ BARKER_B));
    uint8_t d = (d0 < d1) ? d0 : d1;
    return d <= BARKER_MAX_ERRORS;
}

/**
 * @brief Reset the collection portion of a processor back to SEARCHING.
 *
 * Does NOT clear sw_window or bits_seen — those run
 * continuously across packet boundaries.  (The access-code error tolerance is
 * a process-wide global, not per-processor state, so it is not touched here.)
 */
static void reset_collection(bredr_bitstream_decoder_t *proc)
{
    proc->state = STATE_SEARCHING;
    proc->drain_count = 0;
    proc->collected_header_raw = 0u;
    memset(proc->collected_air_payload, 0, sizeof(proc->collected_air_payload));
    proc->collected_air_payload_bits = 0u;
    proc->bits_collected = 0;
    proc->bits_to_collect = 0;
    proc->header_decoded = 0;
    proc->detected_lap = 0;
    proc->detected_ac_errors = 0;
    proc->detected_sw_last = 0;
    proc->trailer_bits = 0;
}

/**
 * @brief Pack one header bit into the raw header buffer.
 *
 * @return Non-zero if the bit was packed successfully, 0 on overflow.
 */
static int push_header_bit(bredr_bitstream_decoder_t *proc, uint8_t bit)
{
    if (proc->bits_collected >= 54u)
        return 0;

    if (bit & 1u)
        proc->collected_header_raw |= (uint64_t)1u << proc->bits_collected;

    proc->bits_collected++;
    return 1;
}

/**
 * @brief Pack one payload bit into the air payload buffer.
 */
static int push_air_payload_bit(bredr_bitstream_decoder_t *proc, uint8_t bit)
{
    unsigned int byte_idx = proc->collected_air_payload_bits / 8u;
    unsigned int bit_idx = proc->collected_air_payload_bits % 8u;

    if (byte_idx >= sizeof(proc->collected_air_payload))
        return 0;

    if (bit & 1u)
        proc->collected_air_payload[byte_idx] |= (uint8_t)(1u << bit_idx);

    proc->collected_air_payload_bits++;
    proc->bits_collected++;
    return 1;
}

/* ---------------------------------------------------------------------------
 * Public API implementation
 * ---------------------------------------------------------------------------*/

void bredr_bitstream_decoder_init(bredr_bitstream_decoder_t *proc)
{
    if (!proc)
        return;
    memset(proc, 0, sizeof(*proc));
    /* state = STATE_SEARCHING = 0, already zeroed */
}

bredr_status_t bredr_bitstream_decoder_push_bit(bredr_bitstream_decoder_t *proc, uint8_t bit)
{
    if (!proc)
        return BREDR_ERROR;

    uint8_t b = bit ? 1u : 0u;
    proc->total_bits_seen++;

    /* ======================================================================
     * STATE: SEARCHING
     * Slide the 64-bit window and look for a valid access code sync word.
     * ====================================================================== */
    if (proc->state == STATE_SEARCHING)
    {
        /* Shift oldest bit out of position 0, push new bit into position 63. */
        proc->sw_window = (proc->sw_window >> 1u) | ((uint64_t)b << 63u);

        if (proc->bits_seen < 64u)
            proc->bits_seen++;

        /* Only start checking once the window is fully populated. */
        if (proc->bits_seen < 64u)
            return BREDR_SEARCHING;

        /* Fast barker pre-filter: skip most non-AC windows cheaply. */
        if (!barker_ok(proc->sw_window))
            return BREDR_SEARCHING;

        /* Extract the candidate LAP from bits 34–57 of the sync word. */
        uint32_t lap = (uint32_t)((proc->sw_window >> 34u) & 0xFFFFFFu);

        /* Re-generate the expected sync word and measure Hamming distance. */
        uint64_t expected = bredr_gen_syncword(lap);
        uint8_t ac_errors = popcount64(proc->sw_window ^ expected);

        if (ac_errors > g_max_ac_errors)
            return BREDR_SEARCHING;

        /* Access code matched. */
        proc->detected_lap = lap;
        proc->detected_ac_errors = ac_errors;
        proc->detected_packet_start_bit = proc->total_bits_seen - 68u;
        /* GIAC/LIAC inquiry packets are always ID packets: a shortened access
         * code with no trailer, no header, and no payload.  FHS responses from
         * slaves use the slave's own Device Access Code, not the GIAC/LIAC, so
         * we will never see a full packet body on these LAPs.  Skip the trailer
         * phase entirely and report immediately. */
        if (lap == BREDR_LAP_GIAC || lap == BREDR_LAP_LIAC)
        {
            memset(&proc->last_frame, 0, sizeof(proc->last_frame));
            proc->last_frame.phy = RECEIVER_PHY_BR;
            proc->last_frame.start_bit_index = proc->detected_packet_start_bit;
            proc->last_frame.lap = lap;
            proc->last_frame.ac_errors = ac_errors;
            proc->last_frame.has_header = 0u;
            proc->last_frame.air_payload_bits = 0u;
            proc->packet_ready = 1;
            return BREDR_VALID_PACKET;
        }

        /* Non-inquiry packet — capture the last SW bit for trailer validation,
         * then begin draining the 4-bit trailer. */
        proc->detected_sw_last = (uint8_t)((proc->sw_window >> 63u) & 1u);
        proc->trailer_bits = 0u;
        proc->state = STATE_DRAINING_TRAILER;
        proc->drain_count = 4u;
        return BREDR_COLLECTING;
    }

    /* ======================================================================
     * STATE: DRAINING_TRAILER
     * Consume the 4-bit AC trailer and validate it before collecting the
     * packet header.  The trailer is an alternating sequence whose first bit
     * is the complement of the last transmitted sync-word bit (bit 63):
     *   sw_last == 1 → trailer = 0101 (nibble 0x5, LSB-first packed)
     *   sw_last == 0 → trailer = 1010 (nibble 0xA, LSB-first packed)
     * If the trailer does not match, this is a shortened access code (ID
     * packet or similar) with no header or payload.
     * ====================================================================== */
    if (proc->state == STATE_DRAINING_TRAILER)
    {
        /* Pack incoming bit LSB-first: first received → bit 0. */
        uint8_t trailer_pos = (uint8_t)(4u - proc->drain_count);
        proc->trailer_bits |= (uint8_t)(b << trailer_pos);
        proc->drain_count--;

        if (proc->drain_count > 0u)
            return BREDR_COLLECTING;

        /* All 4 trailer bits received — validate. */
        uint8_t expected_trailer = proc->detected_sw_last ? 0xAu : 0x5u;
        uint8_t trailer_nibble = proc->trailer_bits & 0xFu;

        if (trailer_nibble != expected_trailer)
        {
            /* Invalid trailer: shortened access code, no header or payload. */
            memset(&proc->last_frame, 0, sizeof(proc->last_frame));
            proc->last_frame.phy = RECEIVER_PHY_BR;
            proc->last_frame.start_bit_index = proc->detected_packet_start_bit;
            proc->last_frame.lap = proc->detected_lap;
            proc->last_frame.ac_errors = proc->detected_ac_errors;
            proc->last_frame.has_header = 0u;
            proc->last_frame.air_payload_bits = 0u;
            proc->packet_ready = 1;
            reset_collection(proc);
            return BREDR_VALID_PACKET;
        }

        /* Valid trailer — transition to header/payload collection. */
        proc->state = STATE_COLLECTING;
        proc->collected_header_raw = 0u;
        memset(proc->collected_air_payload, 0, sizeof(proc->collected_air_payload));
        proc->collected_air_payload_bits = 0u;
        proc->bits_collected = 0u;
        proc->bits_to_collect = 0u;
        proc->header_decoded = 0;
        return BREDR_COLLECTING;
    }

    /* ======================================================================
     * STATE: COLLECTING
     * Capture header bits into the raw header accumulator, then capture raw
     * payload bits directly into the payload buffer.
     * ====================================================================== */
    if ((!proc->header_decoded && !push_header_bit(proc, b)) ||
        (proc->header_decoded && !push_air_payload_bit(proc, b)))
    {
        /* Buffer overflow — something went wrong; reset. */
        reset_collection(proc);
        return BREDR_ERROR;
    }

    /* -------------------------------------------------------------------
        * After the 54-bit FEC-encoded header is collected, switch to a fixed
        * post-access-code ceiling.
     *
     * The BR/EDR header is whitened, so without CLK1-6 we cannot trust the
     * TYPE field yet. Sizing capture from the still-whitened header can stop
     * collection early and truncate a valid packet, so we retain the maximum
     * possible body instead.
     * ------------------------------------------------------------------- */
    if (!proc->header_decoded && proc->bits_collected >= 54u)
    {
        proc->bits_to_collect = 54u + BR_MAX_AIR_PAYLOAD_BITS;
        proc->header_decoded = 1;
    }

    /* -------------------------------------------------------------------
     * Determine how many bits we are waiting for.
     * Before the header is decoded, collect up to the maximum body size as
     * a safety ceiling (should not normally happen given the 54-bit trigger).
     * ------------------------------------------------------------------- */
    unsigned int target = proc->header_decoded
                              ? proc->bits_to_collect
                              : (54u + BR_MAX_AIR_PAYLOAD_BITS);

    if (proc->bits_collected < target)
        return BREDR_COLLECTING;

    /* ======================================================================
     * Frame complete — assemble bredr_frame_t and signal the caller.
     * ====================================================================== */
    bredr_frame_t *frame = &proc->last_frame;

    memset(frame, 0, sizeof(*frame));
    frame->phy = RECEIVER_PHY_BR;
    frame->start_bit_index = proc->detected_packet_start_bit;
    frame->lap = proc->detected_lap;
    frame->ac_errors = proc->detected_ac_errors;
    frame->has_header = 1u;
    frame->header_raw = proc->collected_header_raw;
    frame->air_payload_bits = proc->collected_air_payload_bits;
    memcpy(frame->air_payload, proc->collected_air_payload, sizeof(frame->air_payload));

    proc->packet_ready = 1;

    /* Reset to SEARCHING for the next packet, then return VALID. */
    reset_collection(proc);
    return BREDR_VALID_PACKET;
}

int bredr_bitstream_decoder_get_frame(bredr_bitstream_decoder_t *proc, bredr_frame_t *out)
{
    if (!proc || !out || !proc->packet_ready)
        return -1;

    memcpy(out, &proc->last_frame, sizeof(*out));
    memset(&proc->last_frame, 0, sizeof(proc->last_frame));
    proc->packet_ready = 0;
    return 0;
}
