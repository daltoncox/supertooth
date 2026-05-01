/**
 * @file bredr_phy.c
 * @brief BR/EDR PHY-layer bitstream processor implementation.
 *
 * See bredr_phy.h for the public API and design notes.
 */

#include "bredr_phy.h"

#include <string.h>   /* memset, memcpy */
#include <stddef.h>   /* NULL */
#include <stdio.h>    /* printf */
#include <inttypes.h> /* PRIx32 */
#include <complex.h>  /* float complex, crealf, cimagf */
#include <math.h>     /* log10f */

/* ---------------------------------------------------------------------------
 * Internal state constants
 * ---------------------------------------------------------------------------*/

#define STATE_SEARCHING 0
#define STATE_DRAINING_TRAILER 1
#define STATE_COLLECTING 2

/* ---------------------------------------------------------------------------
 * Sync-word generator matrix
 *
 * (64,30) linear block code, polynomial 0260534236651, modified for the
 * barker code.  Row i is XOR'd into the codeword when bit (23-i) of the
 * LAP is set.  Source: libbtbb bluetooth_packet.c (reference only).
 * ---------------------------------------------------------------------------*/

static const uint64_t sw_matrix[24] = {
    0xfe000002a0d1c014ULL, 0x01000003f0b9201fULL,
    0x008000033ae40edbULL, 0x004000035fca99b9ULL,
    0x002000036d5dd208ULL, 0x00100001b6aee904ULL,
    0x00080000db577482ULL, 0x000400006dabba41ULL,
    0x00020002f46d43f4ULL, 0x000100017a36a1faULL,
    0x00008000bd1b50fdULL, 0x000040029c3536aaULL,
    0x000020014e1a9b55ULL, 0x0000100265b5d37eULL,
    0x0000080132dae9bfULL, 0x000004025bd5ea0bULL,
    0x00000203ef526bd1ULL, 0x000001033511ab3cULL,
    0x000000819a88d59eULL, 0x00000040cd446acfULL,
    0x00000022a41aabb3ULL, 0x0000001390b5cb0dULL,
    0x0000000b0ae27b52ULL, 0x0000000585713da9ULL};

/* Default codeword (output when LAP == 0), already incorporating the PN
 * sequence and barker code modification. */
static const uint64_t SW_DEFAULT_CW = 0xb0000002c7820e7eULL;

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
 * Packet-type payload sizes
 *
 * Maximum on-air payload bits (including FEC overhead where applicable) for
 * each of the 16 BR/EDR packet TYPE values.  These are conservative maximums
 * used to bound bit collection when the full payload length is not decoded.
 *
 * Index = TYPE field value (4 bits, 0–15).
 * ---------------------------------------------------------------------------*/

static const unsigned int s_payload_bits[16] = {
    0,    /* 0  NULL  — no payload                              */
    0,    /* 1  POLL  — no payload                              */
    240,  /* 2  FHS   — 18 B, 2/3 FEC → 240 bits               */
    480,  /* 3  DM1   — max 17 B body, 2/3 FEC → ~480 bits      */
    216,  /* 4  DH1   — max 27 B, no FEC → 216 bits             */
    240,  /* 5  HV1   — 10 B, 1/3 FEC → 240 bits                */
    240,  /* 6  HV2   — 20 B, 2/3 FEC → 240 bits                */
    240,  /* 7  HV3   — 30 B, no FEC → 240 bits                 */
    560,  /* 8  DV    — voice + DM1 → ~560 bits                 */
    232,  /* 9  AUX1  — max 29 B, no FEC → 232 bits             */
    1500, /* 10 DM3   — max 121 B body, 2/3 FEC → ~1500 bits    */
    1464, /* 11 DH3   — max 183 B, no FEC → 1464 bits           */
    1440, /* 12 EV4   — max 120 B, 2/3 FEC → ~1440 bits         */
    1440, /* 13 EV5   — max 180 B, no FEC → 1440 bits           */
    2736, /* 14 DM5   — max 224 B body, 2/3 FEC → ~2736 bits    */
    2712, /* 15 DH5   — max 339 B, no FEC → 2712 bits           */
};

unsigned int bredr_on_air_payload_bits(uint8_t type_code)
{
    return s_payload_bits[type_code & 0x0Fu];
}

/* ---------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------------*/

/**
 * @brief Reverse the bits of a byte (bit 0 ↔ bit 7, bit 1 ↔ bit 6, …).
 */
static uint8_t reverse_byte(uint8_t b)
{
    b = (uint8_t)(((b & 0xF0u) >> 4) | ((b & 0x0Fu) << 4));
    b = (uint8_t)(((b & 0xCCu) >> 2) | ((b & 0x33u) << 2));
    b = (uint8_t)(((b & 0xAAu) >> 1) | ((b & 0x55u) << 1));
    return b;
}

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
uint8_t bredr_compute_hec(uint16_t data, uint8_t uap)
{
    uint8_t lfsr = uap;

    for (int i = 0; i < 10; i++)
    {
        uint8_t fb = ((lfsr >> 7u) & 1u) ^ ((data >> i) & 1u);

        lfsr = (uint8_t)(lfsr << 1u); /* shift left; bit 0 becomes 0 */
        if (fb)
            lfsr ^= 0xA7u; /* tap mask: D^7+D^5+D^2+D+1   */
    }
    return lfsr; /* bit 7 = first HEC bit to be transmitted */
}

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
 * Does NOT clear sw_window, bits_seen, max_ac_errors, ac_ring, or ac_ring_head
 * — those run continuously across packet boundaries.
 */
static void reset_collection(bredr_processor_t *proc)
{
    proc->state = STATE_SEARCHING;
    proc->drain_count = 0;
    proc->bits_collected = 0;
    proc->bits_to_collect = 0;
    proc->header_decoded = 0;
    proc->detected_lap = 0;
    proc->detected_ac_errors = 0;
    proc->detected_sw_last = 0;
    proc->trailer_bits = 0;
    memset(proc->raw_symbols, 0, sizeof(proc->raw_symbols));
}

/**
 * @brief Pack one bit into raw_symbols at the current collection position.
 *
 * Bits are packed LSB-first within each byte, matching BLE over-the-air
 * ordering and the convention used by ble_phy.c.
 *
 * @return Non-zero if the bit was packed successfully, 0 on overflow.
 */
static int push_symbol_bit(bredr_processor_t *proc, uint8_t bit)
{
    unsigned int byte_idx = proc->bits_collected / 8u;
    unsigned int bit_idx = proc->bits_collected % 8u;

    if (byte_idx >= sizeof(proc->raw_symbols))
        return 0;

    if (bit & 1u)
        proc->raw_symbols[byte_idx] |= (uint8_t)(1u << bit_idx);

    proc->bits_collected++;
    return 1;
}

/**
 * @brief Read one bit from raw_symbols at an arbitrary bit position.
 */
static uint8_t read_symbol_bit(const bredr_processor_t *proc,
                               unsigned int bit_pos)
{
    unsigned int byte_idx = bit_pos / 8u;
    unsigned int bit_idx = bit_pos % 8u;
    if (byte_idx >= sizeof(proc->raw_symbols))
        return 0u;
    return (proc->raw_symbols[byte_idx] >> bit_idx) & 1u;
}

/**
 * @brief Decode the 54-bit 1/3-FEC-encoded header into individual field bits.
 *
 * Each of the 18 header bits is transmitted three times (positions 3i, 3i+1,
 * 3i+2 in the FEC stream).  A majority vote recovers the original bit.
 *
 * Header field layout (Bluetooth Core Spec Vol 2, Part B, §7.3):
 *   bits  0– 2  LT_ADDR  (3 bits)
 *   bits  3– 6  TYPE     (4 bits)
 *   bit   7     FLOW
 *   bit   8     ARQN
 *   bit   9     SEQN
 *   bits 10–17  HEC      (8 bits)
 */
static void decode_fec_header(const bredr_processor_t *proc,
                              uint8_t *lt_addr, uint8_t *type,
                              uint8_t *flow, uint8_t *arqn,
                              uint8_t *seqn, uint8_t *hec)
{
    uint8_t hdr[18];
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = read_symbol_bit(proc, (unsigned int)(3 * i));
        uint8_t b = read_symbol_bit(proc, (unsigned int)(3 * i + 1));
        uint8_t c = read_symbol_bit(proc, (unsigned int)(3 * i + 2));
        /* majority vote */
        hdr[i] = (uint8_t)((a & b) | (b & c) | (c & a));
    }

    *lt_addr = (hdr[0]) | (uint8_t)(hdr[1] << 1) | (uint8_t)(hdr[2] << 2);
    *type = (hdr[3]) | (uint8_t)(hdr[4] << 1) | (uint8_t)(hdr[5] << 2) | (uint8_t)(hdr[6] << 3);
    *flow = hdr[7];
    *arqn = hdr[8];
    *seqn = hdr[9];
    *hec = (hdr[10]) | (uint8_t)(hdr[11] << 1) | (uint8_t)(hdr[12] << 2) | (uint8_t)(hdr[13] << 3) | (uint8_t)(hdr[14] << 4) | (uint8_t)(hdr[15] << 5) | (uint8_t)(hdr[16] << 6) | (uint8_t)(hdr[17] << 7);
}

/* ---------------------------------------------------------------------------
 * Public API implementation
 * ---------------------------------------------------------------------------*/

uint64_t bredr_gen_syncword(uint32_t lap)
{
    uint64_t codeword = SW_DEFAULT_CW;
    for (int i = 0; i < 24; i++)
        if (lap & (0x800000u >> i))
            codeword ^= sw_matrix[i];
    return codeword;
}

void bredr_processor_init(bredr_processor_t *proc, uint8_t max_ac_errors)
{
    if (!proc)
        return;
    memset(proc, 0, sizeof(*proc));
    proc->max_ac_errors = max_ac_errors;
    /* state = STATE_SEARCHING = 0, already zeroed */
}

bredr_status_t bredr_push_bit(bredr_processor_t *proc, uint8_t bit)
{
    if (!proc)
        return BREDR_ERROR;

    uint8_t b = bit ? 1u : 0u;

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

        if (ac_errors > proc->max_ac_errors)
            return BREDR_SEARCHING;

        /* Access code matched. */
        proc->detected_lap = lap;
        proc->detected_ac_errors = ac_errors;
        proc->last_packet.rssi = 0.0f;

        /* GIAC/LIAC inquiry packets are always ID packets: a shortened access
         * code with no trailer, no header, and no payload.  FHS responses from
         * slaves use the slave's own Device Access Code, not the GIAC/LIAC, so
         * we will never see a full packet body on these LAPs.  Skip the trailer
         * phase entirely and report immediately. */
        if (lap == BREDR_LAP_GIAC || lap == BREDR_LAP_LIAC)
        {
            proc->last_packet.lap        = lap;
            proc->last_packet.ac_errors  = ac_errors;
            proc->last_packet.has_header = 0u;
            proc->last_packet.payload_bytes = 0u;
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
            proc->last_packet.lap = proc->detected_lap;
            proc->last_packet.ac_errors = proc->detected_ac_errors;
            proc->last_packet.has_header = 0u;
            proc->last_packet.payload_bytes = 0u;
            proc->packet_ready = 1;
            reset_collection(proc);
            return BREDR_VALID_PACKET;
        }

        /* Valid trailer — transition to header/payload collection. */
        proc->state = STATE_COLLECTING;
        proc->bits_collected = 0u;
        proc->bits_to_collect = 0u;
        proc->header_decoded = 0;
        memset(proc->raw_symbols, 0, sizeof(proc->raw_symbols));
        return BREDR_COLLECTING;
    }

    /* ======================================================================
     * STATE: COLLECTING
     * Pack bits into raw_symbols.  Decode header after 54 bits, then wait
     * for the payload to complete.
     * ====================================================================== */
    if (!push_symbol_bit(proc, b))
    {
        /* Buffer overflow — something went wrong; reset. */
        reset_collection(proc);
        return BREDR_ERROR;
    }

    /* -------------------------------------------------------------------
     * After the 54-bit FEC-encoded header is collected, decode it to learn
     * the packet type and calculate the total collection target.
     * ------------------------------------------------------------------- */
    if (!proc->header_decoded && proc->bits_collected >= 54u)
    {
        uint8_t lt_addr, type, flow, arqn, seqn, hec;
        decode_fec_header(proc, &lt_addr, &type, &flow, &arqn, &seqn, &hec);

        /* Pack the 54 raw FEC bits into header_raw (bit 0 = first received). */
        uint64_t header_raw = 0u;
        for (unsigned int i = 0u; i < 54u; i++)
            if (read_symbol_bit(proc, i))
                header_raw |= (uint64_t)1u << i;
        proc->last_packet.header_raw = header_raw;

        /* Store decoded header fields directly into the staging packet so we
         * don't need to decode again at completion time. */
        proc->last_packet.lt_addr = lt_addr;
        proc->last_packet.type = type;
        proc->last_packet.flow = flow;
        proc->last_packet.arqn = arqn;
        proc->last_packet.seqn = seqn;
        proc->last_packet.hec = hec;

        unsigned int payload_bits = s_payload_bits[type & 0x0Fu];
        proc->bits_to_collect = 54u + payload_bits;
        /* Cap at the buffer ceiling. */
        if (proc->bits_to_collect > BREDR_SYMBOLS_MAX)
            proc->bits_to_collect = BREDR_SYMBOLS_MAX;

        proc->header_decoded = 1;
    }

    /* -------------------------------------------------------------------
     * Determine how many bits we are waiting for.
     * Before the header is decoded, collect up to BREDR_SYMBOLS_MAX bits as
     * a safety ceiling (should not normally happen given the 54-bit trigger).
     * ------------------------------------------------------------------- */
    unsigned int target = proc->header_decoded
                              ? proc->bits_to_collect
                              : BREDR_SYMBOLS_MAX;

    if (proc->bits_collected < target)
        return BREDR_COLLECTING;

    /* ======================================================================
     * Packet complete — assemble bredr_packet_t and signal the caller.
     * ====================================================================== */
    bredr_packet_t *pkt = &proc->last_packet;

    /* AC fields (header fields already stored during decode). */
    pkt->lap = proc->detected_lap;
    pkt->ac_errors = proc->detected_ac_errors;
    pkt->has_header = 1u;

    /* HEC verification — only possible for inquiry packets where the UAP
     * is the well-known DCI value (0x00).  Pack the 10 header bits into
     * a uint16_t with bit 0 = LT_ADDR[0] (first transmitted). */
    uint32_t lap = proc->detected_lap & 0xFFFFFFu;
    if (lap == BREDR_LAP_GIAC || lap == BREDR_LAP_LIAC)
    {
        uint16_t hdr_data =
            (uint16_t)((pkt->lt_addr & 0x7u) |
                       ((pkt->type & 0xFu) << 3u) |
                       ((pkt->flow & 0x1u) << 7u) |
                       ((pkt->arqn & 0x1u) << 8u) |
                       ((pkt->seqn & 0x1u) << 9u));
        /* compute_hec returns the LFSR register (bit 7 = first transmitted).
         * Our decoded pkt->hec has bit 0 = first received, so it is the
         * bit-reversal of the LFSR register output. */
        uint8_t expected_hec = bredr_compute_hec(hdr_data, BREDR_DCI);
        pkt->hec_valid = (expected_hec == reverse_byte(pkt->hec)) ? 1u : 2u;
    }
    else
    {
        pkt->hec_valid = 0u; /* UAP unknown — cannot verify */
    }

    /* Extract payload bytes from raw_symbols starting at bit 54.
     * Bits are packed LSB-first; read them out byte by byte. */
    unsigned int payload_bits = (proc->bits_collected > 54u)
                                    ? (proc->bits_collected - 54u)
                                    : 0u;
    unsigned int payload_bytes = payload_bits / 8u;
    if (payload_bytes > BREDR_MAX_PAYLOAD_BYTES)
        payload_bytes = BREDR_MAX_PAYLOAD_BYTES;

    for (unsigned int i = 0u; i < payload_bytes; i++)
    {
        uint8_t byte = 0u;
        for (unsigned int bit_idx = 0u; bit_idx < 8u; bit_idx++)
        {
            unsigned int pos = 54u + i * 8u + bit_idx;
            byte |= (uint8_t)(read_symbol_bit(proc, pos) << bit_idx);
        }
        pkt->payload[i] = byte;
    }
    pkt->payload_bytes = payload_bytes;

    proc->packet_ready = 1;

    /* Reset to SEARCHING for the next packet, then return VALID. */
    reset_collection(proc);
    return BREDR_VALID_PACKET;
}

static float compute_ac_ring_rssi_dbm(const bredr_processor_t *proc)
{
    float sum_power = 0.0f;
    for (unsigned int k = 0u; k < BREDR_AC_SAMPLES; k++)
    {
        float re = crealf(proc->ac_ring[k]);
        float im = cimagf(proc->ac_ring[k]);
        sum_power += re * re + im * im;
    }
    float mean_power = sum_power * (1.0f / (float)BREDR_AC_SAMPLES);
    return (mean_power > 0.0f)
        ? 10.0f * log10f(mean_power) - 30.0f
        : -99.0f;
}

bredr_status_t bredr_push_bit_and_samples(bredr_processor_t *proc, uint8_t bit,
                                          float complex sample_a,
                                          float complex sample_b)
{
    if (!proc)
        return BREDR_ERROR;

    int was_searching = (proc->state == STATE_SEARCHING);

    /* Feed both IQ samples for this bit into the sliding ring buffer so it
     * always holds the most recent BREDR_AC_SAMPLES decimated samples. */
    proc->ac_ring[proc->ac_ring_head] = sample_a;
    proc->ac_ring_head = (proc->ac_ring_head + 1u) % BREDR_AC_SAMPLES;
    proc->ac_ring[proc->ac_ring_head] = sample_b;
    proc->ac_ring_head = (proc->ac_ring_head + 1u) % BREDR_AC_SAMPLES;

    bredr_status_t status = bredr_push_bit(proc, bit);

    /* AC matched on this call (including immediate inquiry packet path). */
    if (was_searching && status != BREDR_SEARCHING)
        proc->last_packet.rssi = compute_ac_ring_rssi_dbm(proc);

    return status;
}

int bredr_get_packet(bredr_processor_t *proc, bredr_packet_t *out)
{
    if (!proc || !out || !proc->packet_ready)
        return -1;

    memcpy(out, &proc->last_packet, sizeof(*out));
    proc->packet_ready = 0;
    return 0;
}

/* ---------------------------------------------------------------------------
 * Whitening tables — identical to libbtbb bluetooth_packet.c
 * Used for header dewhitening in bredr_decode_header_bits().
 * ---------------------------------------------------------------------------*/

static const uint8_t s_whitening_indices[64] = {
    99, 85, 17, 50, 102, 58, 108, 45, 92, 62, 32, 118, 88, 11, 80, 2,
    37, 69, 55,  8,  20, 40,  74,114, 15,106, 30, 78, 53, 72, 28,26,
    68,  7, 39,113, 105, 77,  71, 25, 84, 49,  57, 44, 61,117, 10, 1,
   123,124, 22,125, 111, 23,  42,126,  6,112,  76, 24, 48, 43,116, 0
};

static const uint8_t s_whitening_data[127] = {
    1,1,1,0,0,0,1,1,1,0,1,1,0,0,0,1,0,1,0,0,1,0,1,1,1,1,1,0,
    1,0,1,0,1,0,0,0,0,1,0,1,1,0,1,1,1,1,0,0,1,1,1,0,0,1,0,1,
    0,1,1,0,0,1,1,0,0,0,0,0,1,1,0,1,1,0,1,0,1,1,1,0,1,0,0,0,
    1,1,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,1,1,0,1,0,0,1,
    1,1,1,0,1,1,1,0,0,0,0,1,1,1,1
};

/* ---------------------------------------------------------------------------
 * bredr_decode_header_bits — public shared helper
 * ---------------------------------------------------------------------------*/

void bredr_decode_header_bits(const bredr_packet_t *pkt,
                               uint8_t clk6, uint8_t bits[18])
{
    /* Step 1: majority-vote 1/3-FEC on the 54 raw bits to get 18 whitened bits. */
    for (int i = 0; i < 18; i++)
    {
        uint8_t a = (uint8_t)((pkt->header_raw >> (3*i + 0)) & 1u);
        uint8_t b = (uint8_t)((pkt->header_raw >> (3*i + 1)) & 1u);
        uint8_t c = (uint8_t)((pkt->header_raw >> (3*i + 2)) & 1u);
        bits[i] = (a & b) | (b & c) | (c & a);
    }

    /* Step 2: XOR with whitening PN sequence for the given CLK1-6. */
    int index = (int)s_whitening_indices[clk6 & 0x3fu];
    for (int i = 0; i < 18; i++)
    {
        bits[i] ^= s_whitening_data[index];
        index = (index + 1) % 127;
    }
}

/* ---------------------------------------------------------------------------
 * bredr_print_packet — human-readable packet summary (CLK1-6 unknown)
 * ---------------------------------------------------------------------------*/

static const char *const s_type_names[16] = {
    "NULL", "POLL", "FHS", "DM1",
    "DH1", "HV1", "HV2", "HV3",
    "DV", "AUX1", "DM3", "DH3",
    "EV4", "EV5", "DM5", "DH5"};

static void print_payload_hex(const bredr_packet_t *pkt)
{
    if (pkt->payload_bytes == 0u)
    {
        printf("Payload  : (none)\n");
        return;
    }
    unsigned int show = pkt->payload_bytes < 32u ? pkt->payload_bytes : 32u;
    printf("Payload  : %u bytes (raw, not dewhitened)", pkt->payload_bytes);
    for (unsigned int i = 0u; i < show; i++)
    {
        if (i % 16u == 0u)
            printf("\n           ");
        printf("%02X ", pkt->payload[i]);
    }
    if (pkt->payload_bytes > show)
        printf("...");
    printf("\n");
}

void bredr_print_packet(const bredr_packet_t *pkt)
{
    if (!pkt)
        return;

    printf("--- BR/EDR Packet ---\n");
    printf("LAP      : 0x%06" PRIX32 "\n", pkt->lap & 0xFFFFFFu);
    printf("AC Errors: %u\n", pkt->ac_errors);

    if (!pkt->has_header)
    {
        printf("Header   : (none — shortened access code)\n");
        return;
    }

    printf("Header   : 0x%014" PRIX64 " (54 FEC bits, not dewhitened)\n",
           pkt->header_raw & 0x003FFFFFFFFFFFFFull);
    printf("Decoded  : CLK1-6 unknown — header fields cannot be decoded\n");

    print_payload_hex(pkt);
}

/* ---------------------------------------------------------------------------
 * bredr_print_packet_decoded — print with known UAP and CLK1-6
 * ---------------------------------------------------------------------------*/

void bredr_print_packet_decoded(const bredr_packet_t *pkt,
                                 uint8_t uap, uint8_t clk6)
{
    if (!pkt)
        return;

    printf("--- BR/EDR Packet ---\n");
    printf("LAP      : 0x%06" PRIX32 "\n", pkt->lap & 0xFFFFFFu);
    printf("AC Errors: %u\n", pkt->ac_errors);

    if (!pkt->has_header)
    {
        printf("Header   : (none — shortened access code)\n");
        return;
    }

    printf("Header   : 0x%014" PRIX64 " (54 FEC bits raw)\n",
           pkt->header_raw & 0x003FFFFFFFFFFFFFull);

    /* Unwhiten header bits. */
    uint8_t bits[18];
    bredr_decode_header_bits(pkt, clk6, bits);

    /* Extract fields. */
    uint8_t lt_addr = (bits[0]) | (uint8_t)(bits[1] << 1) | (uint8_t)(bits[2] << 2);
    uint8_t type    = (bits[3]) | (uint8_t)(bits[4] << 1) | (uint8_t)(bits[5] << 2)
                                | (uint8_t)(bits[6] << 3);
    uint8_t flow    = bits[7];
    uint8_t arqn    = bits[8];
    uint8_t seqn    = bits[9];

    /* HEC: bits[10] is first received = LFSR bit 7 (MSB), bits[17] = LFSR bit 0.
     * Assemble with bit 7 = bits[10] to match bredr_compute_hec() output format. */
    uint8_t received_hec = 0;
    for (int i = 0; i < 8; i++)
        received_hec |= (uint8_t)(bits[10 + i] << (7 - i));

    /* Verify HEC. */
    uint16_t hdr_data = (uint16_t)((lt_addr & 0x7u)
                                 | ((type   & 0xFu) << 3u)
                                 | ((flow   & 0x1u) << 7u)
                                 | ((arqn   & 0x1u) << 8u)
                                 | ((seqn   & 0x1u) << 9u));
    uint8_t computed_hec = bredr_compute_hec(hdr_data, uap);
    int hec_ok = (computed_hec == received_hec);

    printf("CLK1-6   : %d  UAP: 0x%02X\n", clk6 & 0x3f, uap);
    printf("Type     : %s (%u)\n", s_type_names[type & 0x0Fu], type & 0x0Fu);
    printf("LT_ADDR  : %u\n", lt_addr & 0x07u);
    printf("FLOW     : %u  ARQN: %u  SEQN: %u\n",
           flow & 1u, arqn & 1u, seqn & 1u);
    printf("HEC      : 0x%02X [%s]\n", received_hec, hec_ok ? "PASS" : "FAIL");

    print_payload_hex(pkt);
}
