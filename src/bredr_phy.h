/**
 * @file bredr_phy.h
 * @brief BR/EDR PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module detects and captures BR/EDR (Classic Bluetooth) packets from a
 * raw 1-Mbps GFSK bitstream, one bit at a time.  The design mirrors the
 * `ble_phy` API: all decoder state lives inside a `bredr_processor_t` object
 * that the caller allocates and passes to every function, making it safe to
 * use from an SDR RX callback without global variables.
 *
 * Typical usage
 * -------------
 * @code
 *   bredr_processor_t proc;
 *   bredr_processor_init(&proc, BREDR_AC_ERRORS_DEFAULT);
 *
 *   // For every demodulated bit from the 1-Mbps channel:
 *   bredr_status_t s = bredr_push_bit(&proc, bit);
 *   if (s == BREDR_VALID_PACKET) {
 *       bredr_packet_t pkt;
 *       bredr_get_packet(&proc, &pkt);
 *       bredr_print_packet(&pkt);
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
 * This simple approach works well at low BER; use `max_ac_errors=0` for
 * the most reliable (but strictest) detection.
 *
 * Packet collection
 * -----------------
 * After the AC is confirmed, the processor drains the 4-bit trailer, then
 * collects the 54-bit FEC-encoded packet header (18 header bits repeated
 * three times — 1/3 rate FEC with majority-vote decode).  The TYPE field
 * from the header determines how many additional payload bits are collected.
 * Payload bytes are stored raw (air order, not dewhitened — clock bits
 * required for dewhitening are not available to a promiscuous receiver).
 *
 * Thread safety
 * -------------
 * A single `bredr_processor_t` is NOT thread-safe.  Use one processor per
 * thread/channel, or protect with a mutex.
 *
 * Bluetooth Core Specification references
 * ----------------------------------------
 * - Vol 2, Part B, §6.3  — Access code format
 * - Vol 2, Part B, §7    — Packet format
 * - Vol 2, Part B, §7.4  — FEC (1/3 rate for packet header)
 * - Vol 2, Part B, §8    — Packet types and payload lengths
 */

#ifndef BREDR_PHY_H
#define BREDR_PHY_H

#include <stdint.h>
#include <complex.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/

/** Maximum payload bytes stored per packet (DH5 payload = 339 bytes). */
#define BREDR_MAX_PAYLOAD_BYTES  343u

/**
 * Maximum on-air symbols (bits) that can be collected after the AC.
 * Covers the largest 5-slot BR/EDR packet at 1 Mbps.
 */
#define BREDR_SYMBOLS_MAX       3125u

/** Default maximum Hamming distance allowed in access-code matching. */
#define BREDR_AC_ERRORS_DEFAULT    2u

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
 * Number of bits in a BR/EDR access code (preamble + sync word = 4 + 64 + 4
 * trailer, but the sliding window detects the 64-bit sync word; the ring
 * buffer covers 72 bits to include preamble).
 */
#define BREDR_AC_BITS    72u

/**
 * Number of decimated IQ samples that span the access code.
 * At 2 Msps / 1 Mbps there are 2 samples per bit, so 72 bits × 2 = 144.
 */
#define BREDR_AC_SAMPLES (BREDR_AC_BITS * 2u)

/* ---------------------------------------------------------------------------
 * bredr_packet_t — a captured BR/EDR packet
 * ---------------------------------------------------------------------------*/

/**
 * @brief A captured BR/EDR packet.
 *
 * Populated by the processor after a complete packet has been collected.
 * The LAP is extracted from the access code sync word.  The header fields
 * (lt_addr … hec) are decoded via 1/3 FEC majority vote.  The payload is
 * stored as raw bytes in reception (air) order and has NOT been dewhitened.
 *
 * On-air packet layout (bits transmitted LSB-first):
 *  | Preamble (4 b) | Sync Word (64 b) | Trailer (4 b) |
 *  | Header FEC (54 b) | Payload (variable) |
 */
typedef struct
{
    /** 24-bit Lower Address Part extracted from the access code sync word. */
    uint32_t lap;

    /** Number of bit errors found during access code detection (0–max). */
    uint8_t  ac_errors;

    /**
     * Receiver timestamp in 1600 Hz slot ticks at reception time.
     * This is derived from rx_clk_ref using the configured RX sample clock
     * and Bluetooth slot duration (625 us).
     * Set by the caller before or immediately after bredr_get_packet().
     * Defaults to 0 if not set.
     */
    uint32_t rx_clk_1600;

    /**
     * Hardware receive timestamp reference at reception time.
     * This is the absolute RX sample-clock index used to derive rx_clk_1600.
     * Set by the caller before or immediately after bredr_get_packet().
     * Defaults to 0 if not set.
     */
    uint64_t rx_clk_ref;

    /* -- Decoded packet header fields (after 1/3 FEC majority vote) ------- */

    /** Logical Transport Address (3 bits). */
    uint8_t  lt_addr;

    /** Packet type field (4 bits).  See TYPE_NAMES[] in bredr_phy.c. */
    uint8_t  type;

    /** Flow control bit. */
    uint8_t  flow;

    /** Acknowledgement bit. */
    uint8_t  arqn;

    /** Sequence number bit. */
    uint8_t  seqn;

    /** Header error check byte (8 bits). */
    uint8_t  hec;

    /**
     * HEC verification result.
     *   0 — not verified (UAP is unknown; packet is not an inquiry packet)
     *   1 — HEC passed  (inquiry packet; DCI=0x00 used as UAP)
     *   2 — HEC failed  (inquiry packet; HEC does not match)
     */
    uint8_t  hec_valid;

    /**
     * Received signal strength in dBm at the time this packet was captured.
     * Set by the caller before or after bredr_get_packet().  Defaults to 0.0f
     * if the caller does not set it.
     */
    float    rssi;

    /**
     * Raw 54 FEC-encoded header bits as received from the air.
     * Bit 0 = first received bit, bit 53 = last received bit.
     * Valid only when has_header != 0.
     */
    uint64_t header_raw;

    /* -- Raw payload ------------------------------------------------------- */

    /**
     * Payload bytes in reception order.  Not dewhitened.
     * Valid bytes: payload[0 .. payload_bytes-1].
     */
    uint8_t  payload[BREDR_MAX_PAYLOAD_BYTES];

    /** Number of valid bytes in `payload[]`. */
    unsigned int payload_bytes;

    /**
     * Non-zero when this packet contains a decoded header and payload.
     * Zero when the packet is a shortened access code (trailer was absent
     * or invalid) — in that case only `lap` and `ac_errors` are valid.
     */
    uint8_t has_header;

} bredr_packet_t;

/* ---------------------------------------------------------------------------
 * bredr_status_t — return codes for bredr_push_bit()
 * ---------------------------------------------------------------------------*/

/**
 * @brief Status codes returned by `bredr_push_bit()`.
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
     * Keep calling `bredr_push_bit()` — no action required from the caller.
     */
    BREDR_COLLECTING   =  1,

    /**
     * A complete packet has been captured.
     * Call `bredr_get_packet()` to retrieve it before the next packet begins.
     */
    BREDR_VALID_PACKET =  2,

} bredr_status_t;

/* ---------------------------------------------------------------------------
 * bredr_processor_t — per-channel decoder state
 * ---------------------------------------------------------------------------*/

/**
 * @brief All state required to capture packets on one BR/EDR channel.
 *
 * Treat this as an opaque type; access members only through the API below.
 */
typedef struct
{
    /* -- Configuration ----------------------------------------------------- */

    /** Maximum AC Hamming distance accepted as a valid match. */
    uint8_t  max_ac_errors;

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

    /* -- State machine ----------------------------------------------------- */

    /**
     * Internal state: 0 = SEARCHING, 1 = DRAINING_TRAILER, 2 = COLLECTING.
     * Not to be confused with the public bredr_status_t return values.
     */
    int      state;

    /** Trailer bits still to consume before collection begins (0–4). */
    unsigned int drain_count;

    /* -- Symbol collection buffer ------------------------------------------ */

    /**
     * Packed collection buffer (LSB-first within each byte, air order).
     * Holds the FEC-encoded header bits followed by payload bits.
     * Sized to hold BREDR_SYMBOLS_MAX bits.
     */
    uint8_t  raw_symbols[(BREDR_SYMBOLS_MAX + 7u) / 8u];

    /** Bits packed into raw_symbols so far. */
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

    /* -- AC IQ sample ring buffer ----------------------------------------- */

    /**
     * Circular buffer of the most recent BREDR_AC_SAMPLES (144) decimated IQ
     * samples.  Two samples are written per bredr_push_bit_and_samples() call
     * (one per
     * bit at 2 samples/bit).  When an access code is detected this buffer
     * holds exactly the IQ samples that covered the AC, allowing RSSI to be
     * computed immediately at detection time rather than post-hoc.
     */
    float complex ac_ring[BREDR_AC_SAMPLES];

    /** Write index into ac_ring; advances mod BREDR_AC_SAMPLES. */
    unsigned int  ac_ring_head;

    /* -- Last valid packet ------------------------------------------------- */

    /** Most recently completed packet; valid when packet_ready != 0. */
    bredr_packet_t last_packet;

    /** Non-zero when last_packet holds an unread valid packet. */
    int            packet_ready;

} bredr_processor_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

/**
 * @brief Initialise a channel processor.
 *
 * Must be called before the first `bredr_push_bit()` on this processor.
 * Safe to call again at any time to reset all state.
 *
 * @param proc           Pointer to the processor.  Must not be NULL.
 * @param max_ac_errors  Maximum Hamming distance accepted as a valid AC
 *                       match.  Use BREDR_AC_ERRORS_DEFAULT (2) for
 *                       normal operation, 0 for strict matching only.
 */
void           bredr_processor_init(bredr_processor_t *proc,
                                    uint8_t max_ac_errors);

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
bredr_status_t bredr_push_bit(bredr_processor_t *proc, uint8_t bit);

/**
 * @brief Push one demodulated bit plus its two IQ samples into the processor.
 *
 * This is a convenience wrapper around `bredr_push_bit()` for callers that
 * have IQ available and want RSSI computed from the AC sample window.
 *
 * @param proc      Pointer to an initialised processor.  Must not be NULL.
 * @param bit       The demodulated bit value.  Only the LSB is examined.
 * @param sample_a  First of the two decimated IQ samples for this bit.
 * @param sample_b  Second of the two decimated IQ samples for this bit.
 *
 * @return Same status codes as `bredr_push_bit()`.
 */
bredr_status_t bredr_push_bit_and_samples(bredr_processor_t *proc, uint8_t bit,
                                          float complex sample_a,
                                          float complex sample_b);

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
int            bredr_get_packet(bredr_processor_t *proc, bredr_packet_t *out);

/**
 * @brief Print a human-readable summary of a captured BR/EDR packet.
 *
 * Displays the LAP, AC error count, decoded header fields (including the
 * packet type name), and a hex dump of the first payload bytes.  The payload
 * is printed in raw air order and has not been dewhitened.
 *
 * @param pkt  Pointer to the packet.  Must not be NULL.
 */

/**
 * @brief Compute the 8-bit HEC for a 10-bit header value and a known UAP.
 *
 * Exposed publicly so that external modules (e.g. bredr_piconet) can
 * verify HEC without duplicating the LFSR logic.
 *
 * @param data  10 header bits: bit 0 = LT_ADDR[0] (first transmitted),
 *              bit 9 = SEQN (last transmitted before HEC).
 * @param uap   8-bit Upper Address Part used to seed the LFSR.
 * @return      8-bit HEC register; bit 7 is the first bit transmitted.
 */
uint8_t        bredr_compute_hec(uint16_t data, uint8_t uap);

/**
 * @brief Apply 1/3-FEC majority vote and header unwhitening to a packet.
 *
 * Reconstructs the 18 whitened header bits from `pkt->header_raw` via
 * majority vote, then XORs with the PN-sequence for the given CLK1-6.
 * The result is 18 unwhitened bits in air order:
 *   bits  0– 2  LT_ADDR  (bit 0 = first transmitted)
 *   bits  3– 6  TYPE     (bit 0 = first transmitted)
 *   bit   7     FLOW
 *   bit   8     ARQN
 *   bit   9     SEQN
 *   bits 10–17  HEC      (bit 10 = first transmitted, i.e. LFSR bit 7)
 *
 * @param pkt   Packet with a valid `header_raw` field (has_header must be 1).
 * @param clk6  CLK1-6 whitening key (0–63).
 * @param bits  Output array of 18 bits, one per element (0 or 1).
 */
void           bredr_decode_header_bits(const bredr_packet_t *pkt,
                                        uint8_t clk6, uint8_t bits[18]);

/**
 * @brief Print a packet without decoded header fields.
 *
 * When CLK1-6 is unknown the majority-voted header bits are still whitened
 * and therefore meaningless.  This function prints the LAP, AC error count,
 * the raw 54-bit FEC-encoded header word, and a note that the header fields
 * cannot be decoded until the clock is known.  Payload bytes are shown raw.
 *
 * @param pkt  Pointer to the packet.  Must not be NULL.
 */
void           bredr_print_packet(const bredr_packet_t *pkt);

/**
 * @brief Print a packet with fully decoded header fields.
 *
 * Unwhitens the header with the given CLK1-6, decodes all fields, verifies
 * the HEC against the known UAP, and prints a human-readable summary.
 * Payload is still shown raw (dewhitening the payload requires additional
 * clock bits beyond CLK1-6).
 *
 * @param pkt   Pointer to the packet.  Must not be NULL.
 * @param uap   Known 8-bit Upper Address Part.
 * @param clk6  Known CLK1-6 (0–63) valid for this packet.
 */
void           bredr_print_packet_decoded(const bredr_packet_t *pkt,
                                          uint8_t uap, uint8_t clk6);

/**
 * @brief Generate a 64-bit BR/EDR sync word from a 24-bit LAP.
 *
 * Implements the (64,30) linear block code used to encode the LAP into
 * the access code sync word (Bluetooth Core Spec Vol 2, Part B, §6.3.3).
 * The result is in "host order" (bit 0 = first transmitted), matching the
 * convention used by libbtbb's btbb_gen_syncword().
 *
 * Exposed publicly so that test/example code can construct synthetic frames.
 *
 * @param lap  24-bit Lower Address Part.
 * @return     64-bit sync word in host order.
 */
uint64_t       bredr_gen_syncword(uint32_t lap);

/**
 * @brief Return the maximum on-air payload bits for a given TYPE code.
 *
 * This is the number of bits the PHY targets when collecting a packet of
 * the given type — including FEC overhead where applicable.  The TYPE code
 * is the raw 4-bit field value (0–15) as it appears in the packet header.
 *
 * Used by external modules (e.g. bredr_piconet) to correct the rx_clk_1600
 * estimate after the true TYPE is decoded with a known CLK1-6.
 *
 * @param type_code  4-bit packet TYPE (0–15).
 * @return           On-air payload bits, or 0 for NULL/POLL.
 */
unsigned int   bredr_on_air_payload_bits(uint8_t type_code);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_PHY_H */
