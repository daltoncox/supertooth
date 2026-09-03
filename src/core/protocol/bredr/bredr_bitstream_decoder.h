/**
 * @file bredr_bitstream_decoder.h
 * @brief BR/EDR PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module detects and captures BR/EDR (Classic Bluetooth) packets from a
 * raw 1-Mbps GFSK bitstream, one bit at a time.  The design mirrors the
 * `ble_phy` API: all decoder state lives inside a `bredr_bitstream_decoder_t` object
 * that the caller allocates and passes to every function, making it safe to
 * use from an SDR RX callback without global variables.
 *
 * Typical usage
 * -------------
 * @code
 *   bredr_bitstream_decoder_t proc;
 *   bredr_bitstream_decoder_init(&proc);
 *
 *   // For every demodulated bit from the 1-Mbps channel:
 *   bredr_status_t s = bredr_bitstream_decoder_push_bit(&proc, bit);
 *   if (s == BREDR_VALID_PACKET) {
 *       bredr_frame_t frame;
 *       bredr_bitstream_decoder_get_frame(&proc, &frame);
 *       // Use frame ...
 *   }
 * @endcode
 *
 * Access code detection
 * ---------------------
 * The 72-bit BR/EDR access code (AC) consists of a 4-bit preamble, a 64-bit
 * sync word derived from the device's LAP via a (64,30) linear block code,
 * and a 4-bit trailer.  Detection uses:
 *  1. A 64-bit sliding window over the incoming bitstream.
 *  2. A 7-bit "barker code" pre-filter (bits 57–63 of the sync word) that
 *     quickly rejects most non-AC windows without a full correlation.
 *  3. LAP extraction from bits 34–57, followed by sync-word re-generation
 *     and Hamming-distance verification.
 *
 * Limitation: bit errors that fall in the LAP portion of the sync word
 * (bits 34–57) may cause a miss because the extracted LAP will be wrong.
 * This simple approach works well at low BER; set the global access-code
 * error tolerance to 0 (the default) for the most reliable (but strictest)
 * detection.
 *
 * Packet collection
 * -----------------
 * After the AC is confirmed, the processor drains the 4-bit trailer, then
 * collects the 54-bit FEC-encoded packet header (18 header bits repeated
 * three times — 1/3 rate FEC with majority-vote decode). Because the BR/EDR
 * header is whitened and the whitening clock is not yet known at this stage,
 * the PHY does not trust the still-whitened TYPE field to determine packet
 * length. Instead it retains the maximum possible post-access-code body for a
 * 5-slot Basic Rate packet and leaves semantic decode to higher layers once
 * UAP/CLK context exists. Payload bytes are stored raw (air order, not
 * dewhitened).
 *
 * Thread safety
 * -------------
 * A single `bredr_bitstream_decoder_t` is NOT thread-safe.  Use one processor per
 * thread/channel, or protect with a mutex.
 *
 * Bluetooth Core Specification references
 * ----------------------------------------
 * - Vol 2, Part B, §6.3  — Access code format
 * - Vol 2, Part B, §7    — Packet format
 * - Vol 2, Part B, §7.4  — FEC (1/3 rate for packet header)
 * - Vol 2, Part B, §8    — Packet types and payload lengths
 */

#ifndef BREDR_BITSTREAM_DECODER_H
#define BREDR_BITSTREAM_DECODER_H

#include <stdint.h>
#include <complex.h>

#include "phy.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/


// Number of bits in a BR/EDR access code (preamble + sync word + trailer = 4 + 64 + 4)
#define BREDR_AC_BITS 72u

// Maximum retained on-air payload bits in a 5-slot Basic Rate packet.
#define BR_MAX_AIR_PAYLOAD_BITS  2999u

// Enough payload bytes to hold BR_MAX_AIR_PAYLOAD_BITS
#define BR_MAX_AIR_PAYLOAD_BYTES ((BR_MAX_AIR_PAYLOAD_BITS + 7u) / 8u)

/**
 * General Inquiry Access Code LAP (GIAC).
 * Packets with this LAP use DCI (0x00) as the UAP for HEC generation,
 * so the HEC can be verified without knowing the remote device's address.
 */
#define BREDR_LAP_GIAC  0x9E8B33u

/**
 * Limited Inquiry Access Code LAP (LIAC).
 * Same DCI-based HEC verification as GIAC.
 */
#define BREDR_LAP_LIAC  0x9E8B00u

/** Default Check Initialization value — UAP used for inquiry packet HECs. */
#define BREDR_DCI       0x00u

/**
 * Number of decimated IQ samples that span the access code.
 * At 2 Msps / 1 Mbps there are 2 samples per bit, so 72 bits × 2 = 144.
 */
#define BREDR_AC_SAMPLES (BREDR_AC_BITS * 2u)

/**
 * Number of decimated IQ samples available when the access code is detected.
 * Detection completes on the preamble+sync portion (68 bits) before the
 * 4-bit trailer is drained.
 */
#define BREDR_AC_DETECT_SAMPLES (68u * 2u)

/* ---------------------------------------------------------------------------
 * bredr_frame_t — a captured BR/EDR PHY frame
 * ---------------------------------------------------------------------------*/

/**
 * @brief A captured BR/EDR PHY frame.
 *
 * Populated by the processor after a complete packet has been collected.
 * The LAP is extracted from the access code sync word. The header is stored
 * as the raw 54-bit FEC-encoded field captured from the air. The air payload
 * is stored in reception order, has NOT been dewhitened, and is packed
 * LSB-first within each byte (first received bit goes to bit 0).
 *
 * On-air packet layout (bits transmitted LSB-first):
 *  | Preamble (4 b) | Sync Word (64 b) | Trailer (4 b) |
 *  | Header FEC (54 b) | Payload (variable) |
 */
typedef struct
{
    /** Physical-layer modulation/coding of the captured packet. */
    receiver_phy_t phy;

    /** Absolute bit index of the access-code start in the channel bitstream. */
    uint64_t start_bit_index;

    /** 24-bit Lower Address Part extracted from the access code sync word. */
    uint32_t lap;

    /** Number of bit errors found during access code detection (0–max). */
    uint8_t  ac_errors;

    /**
     * Raw 54 FEC-encoded header bits as received from the air.
     * Bit 0 = first received bit, bit 53 = last received bit.
     * Valid only when has_header != 0.
     */
    uint64_t header_raw;

    /* -- Air payload ------------------------------------------------------- */

    /**
     * On-air payload bytes in reception order. Not dewhitened.
     * Packed LSB-first within each byte.
     * Valid bytes: air_payload[0 .. bredr_frame_air_payload_bytes(frame)-1].
     */
    uint8_t  air_payload[BR_MAX_AIR_PAYLOAD_BYTES];

    /** Exact number of on-air payload bits captured into air_payload[]. */
    unsigned int air_payload_bits;

    /**
     * Non-zero when this frame contains a header and payload.
     * Zero when the frame is a shortened access code (trailer was absent
     * or invalid) — in that case only `lap` and `ac_errors` are valid.
     */
    uint8_t has_header;

} bredr_frame_t;

static inline unsigned int bredr_frame_air_payload_bytes(const bredr_frame_t *frame)
{
    return frame ? ((frame->air_payload_bits + 7u) / 8u) : 0u;
}

/* ---------------------------------------------------------------------------
 * bredr_status_t — return codes for bredr_bitstream_decoder_push_bit()
 * ---------------------------------------------------------------------------*/

/**
 * @brief Status codes returned by `bredr_bitstream_decoder_push_bit()`.
 *
 * Callers should treat any negative value as an error so that future error
 * codes can be added without breaking existing switch statements.
 */
typedef enum
{
    /** An unrecoverable internal error occurred (e.g. NULL processor). */
    BREDR_ERROR        = -1,

    /** Scanning for the next access code. */
    BREDR_SEARCHING    =  0,

    /**
     * Access code confirmed; the processor is collecting header/payload bits.
     * Keep calling `bredr_bitstream_decoder_push_bit()` — no action required from the caller.
     */
    BREDR_COLLECTING   =  1,

    /**
     * A complete packet has been captured.
     * Call `bredr_get_packet()` to retrieve it before the next packet begins.
     */
    BREDR_VALID_PACKET =  2,

} bredr_status_t;

/* ---------------------------------------------------------------------------
 * bredr_bitstream_decoder_t — per-channel decoder state
 * ---------------------------------------------------------------------------*/

/**
 * @brief All state required to capture packets on one BR/EDR channel.
 *
 * Treat this as an opaque type; access members only through the API below.
 */
typedef struct
{
    /* -- Sync-word detection ----------------------------------------------- */

    /**
     * 64-bit sliding window over the incoming bitstream.
     * bit 0 = oldest received bit, bit 63 = newest received bit.
     * Matches the "host order" representation used by libbtbb's
     * air_to_host64().
     */
    uint64_t sw_window;

    /** Total bits shifted into sw_window; saturates at 64. */
    unsigned int bits_seen;

    /** Absolute number of demodulated bits consumed by this processor. */
    uint64_t total_bits_seen;

    /* -- State machine ----------------------------------------------------- */

    /**
     * Internal state: 0 = SEARCHING, 1 = DRAINING_TRAILER, 2 = COLLECTING.
     * Not to be confused with the public bredr_status_t return values.
     */
    int      state;

    /** Trailer bits still to consume before collection begins (0–4). */
    unsigned int drain_count;

    /* -- Bit collection buffers -------------------------------------------- */

    /**
     * Raw 54 FEC-encoded header bits as received from the air.
     * Bit 0 = first received bit, bit 53 = last received bit.
     */
    uint64_t collected_header_raw;

    /** On-air payload bytes packed LSB-first within each byte. */
    uint8_t  collected_air_payload[BR_MAX_AIR_PAYLOAD_BYTES];

    /** Exact number of air payload bits captured so far. */
    unsigned int collected_air_payload_bits;

    /** Total body bits captured so far (54 header bits + air payload bits). */
    unsigned int bits_collected;

    /**
     * Total bits to collect (header FEC + payload on-air bits).
     * Valid only after the header has been decoded.
     */
    unsigned int bits_to_collect;

    /** Non-zero once the 54-bit FEC header has been decoded. */
    int      header_decoded;

    /* -- AC match results (valid when state != SEARCHING) ------------------ */

    /** LAP extracted from the matched sync word. */
    uint32_t detected_lap;

    /** AC bit errors for the current match. */
    uint8_t  detected_ac_errors;

    /** Absolute bit index of the current packet's access-code start. */
    uint64_t detected_packet_start_bit;

    /**
     * Last transmitted bit of the matched sync word (bit 63 of sw_window).
     * Used to determine the expected trailer pattern after AC detection.
     * 0 → expected trailer `1010`; 1 → expected trailer `0101`.
     */
    uint8_t  detected_sw_last;

    /**
     * Trailer bits accumulated during STATE_DRAINING_TRAILER, packed
     * LSB-first (first received bit in bit 0).  Valid only while
     * drain_count > 0.
     */
    uint8_t  trailer_bits;

    /* -- Last valid packet ------------------------------------------------- */

    /** Most recently completed packet; valid when packet_ready != 0. */
    bredr_frame_t last_frame;

    /** Non-zero when last_packet holds an unread valid packet. */
    int            packet_ready;

} bredr_bitstream_decoder_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

/**
 * @brief Initialise a channel processor.
 *
 * Must be called before the first `bredr_bitstream_decoder_push_bit()` on this processor.
 * Safe to call again at any time to reset all state.
 *
 * The access-code error tolerance is a single process-wide setting owned by
 * this module (see bredr_bitstream_decoder_set_global_max_ac_errors); it is
 * not configured per processor.
 *
 * @param proc  Pointer to the processor.  Must not be NULL.
 */
void           bredr_bitstream_decoder_init(bredr_bitstream_decoder_t *proc);

/**
 * @brief Set the process-wide maximum access-code Hamming distance.
 *
 * Applies to every BR/EDR bitstream decoder.  This is the sole access-code
 * acceptance gate; no other layer performs an access-code error filter.
 * Set once before streaming begins.  Defaults to 0 (strict, byte-perfect
 * access-code match).
 *
 * @param max_ac_errors  Maximum Hamming distance accepted as a valid AC match.
 */
void           bredr_bitstream_decoder_set_global_max_ac_errors(uint8_t max_ac_errors);

/**
 * @brief Return the current process-wide maximum access-code Hamming distance.
 *
 * @return  The configured maximum AC error tolerance.
 */
uint8_t        bredr_bitstream_decoder_get_global_max_ac_errors(void);

/**
 * @brief Push one demodulated bit into the processor.
 *
 * Call this function for every bit produced by your GFSK demodulator, in
 * reception order.
 *
 * @param proc  Pointer to an initialised processor.  Must not be NULL.
 * @param bit   The demodulated bit value.  Only the LSB is examined.
 *
 * @return  `BREDR_VALID_PACKET` when a complete packet is ready,
 *          `BREDR_COLLECTING` while collecting header/payload bits,
 *          `BREDR_SEARCHING` when scanning for the next AC, or
 *          `BREDR_ERROR` on invalid input.
 */
bredr_status_t bredr_bitstream_decoder_push_bit(bredr_bitstream_decoder_t *proc, uint8_t bit);

/**
 * @brief Retrieve the last captured packet.
 *
 * Copies the most recently completed packet into `*out` and marks the
 * internal buffer as consumed.
 *
 * @param proc  Pointer to the processor.  Must not be NULL.
 * @param out   Destination buffer.  Must not be NULL.
 *
 * @return  0 on success, -1 if no valid packet is available.
 */
int            bredr_bitstream_decoder_get_frame(bredr_bitstream_decoder_t *proc, bredr_frame_t *out);

/**
 * @brief Recover the candidate UAP from a 10-bit header value and the 8-bit HEC.
 *
 * This is the Bluetooth HEC *decode* (the inverse of the hardware HEC
 * generation).  It is exposed publicly so that external modules (e.g.
 * bredr_piconet) can recover the UAP without duplicating the LFSR logic.
 *
 * The HEC only narrows the UAP to one of (up to) 36 reachable values for a
 * given header; the payload CRC must be used to disambiguate the true UAP.
 *
 * @param data  10 header bits: bit 0 = LT_ADDR[0] (first transmitted),
 *              bit 9 = SEQN (last transmitted before HEC).
 * @param hec   8-bit HEC register; bit 7 is the first bit transmitted.
 * @return      candidate 8-bit UAP.
 */
#ifdef __cplusplus
}
#endif

#endif /* BREDR_BITSTREAM_DECODER_H */
