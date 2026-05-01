/**
 * @file ble_phy.h
 * @brief BLE PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module decodes a raw 1-Mbps BLE bitstream (GFSK, LSB-first) one bit at
 * a time.  It is intentionally stateless from the caller's perspective: all
 * decoder state lives inside a `ble_channel_processor_t` object that you
 * allocate and pass to every call.  This makes the API safe to use from an
 * SDR RX callback without any global variables.
 *
 * Typical usage
 * -------------
 * @code
 *   ble_channel_processor_t proc;
 *   ble_processor_init(&proc, 37);          // advertising channel 37
 *
 *   // Inside your SDR RX callback, per demodulated bit:
 *   ble_status_t status = ble_push_bit(&proc, bit);
 *   if (status == BLE_VALID_PACKET) {
 *       ble_packet_t pkt;
 *       ble_get_packet(&proc, &pkt);
 *       // Use pkt ...
 *   }
 * @endcode
 *
 * Thread safety
 * -------------
 * A single `ble_channel_processor_t` is NOT thread-safe.  Use one processor
 * per thread/channel, or protect access with a mutex.
 *
 * Bluetooth Core Specification references
 * ----------------------------------------
 * - Vol 6, Part B, §2.1  — Packet format
 * - Vol 6, Part B, §3.2  — Data whitening (LFSR)
 * - Vol 6, Part B, §2.3  — CRC
 */

#ifndef BLE_PHY_H
#define BLE_PHY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/

/** Maximum PDU size supported: 2-byte header + 255-byte payload + 1 spare. */
#define BLE_PDU_MAX_BYTES 258u

/** CRC length in bytes (24-bit CRC per BLE spec). */
#define BLE_CRC_BYTES 3u

/**
 * CRC initial value for BLE advertising channels.
 * (Bluetooth Core Spec Vol 6, Part B, §3.1.1)
 */
#define BLE_CRC_INIT_ADV 0x555555u

/** BLE advertising access address (Bluetooth Core Spec Vol 6, Part B, §2.1.2). */
#define BLE_ADVERTISING_AA 0x8E89BED6UL

    /* ---------------------------------------------------------------------------
     * ble_packet_t — a fully decoded BLE advertising packet
     * ---------------------------------------------------------------------------*/

    /**
     * @brief A decoded BLE advertising packet.
     *
     * Fields are populated by the processor in the order they arrive over-the-air.
     * All multi-byte integer fields are in host byte order.
     *
     * After `ble_get_packet()` returns, `pdu[0]` is the PDU header byte (PDU type
     * in bits [3:0], TxAdd/RxAdd flags in bits [6:7]) and `pdu[1]` is the payload
     * length in bytes.  Payload data starts at `pdu[2]`.
     *
     *  On-air packet layout (LSB first within each byte):
     *  | Preamble (1 B) | Access Address (4 B) | PDU header (2 B) | Payload | CRC (3 B) |
     */
    typedef struct
    {
        /** Preamble byte as received over-the-air (0x55 or 0xAA for BLE). */
        uint8_t preamble;

        /** Access address, host-endian, assembled from 4 over-the-air bytes. */
        uint32_t access_address;

        /**
         * Raw PDU bytes after dewhitening, in reception order.
         * pdu[0] : PDU header byte 0  (PDU type, TxAdd, RxAdd)
         * pdu[1] : PDU header byte 1  (payload length, 0–255)
         * pdu[2+]: Payload
         *
         * The CRC bytes are NOT stored here; use the `crc` field instead.
         */
        uint8_t pdu[BLE_PDU_MAX_BYTES];

        /**
         * CRC value computed over the PDU.  Only the lower 24 bits are valid.
         * A value of 0 indicates the CRC has not been verified (e.g. if the
         * packet was truncated).
         */
        uint32_t crc;

    } ble_packet_t;

    /* ---------------------------------------------------------------------------
     * ble_status_t — return codes for ble_push_bit()
     * ---------------------------------------------------------------------------*/

    /**
     * @brief Status codes returned by `ble_push_bit()`.
     *
     * Callers should treat any negative value as an error so that new error codes
     * can be added in the future without breaking existing switch statements.
     */
    typedef enum
    {
        /** An unrecoverable internal error occurred (e.g. NULL processor). */
        BLE_ERROR = -1,

        /** Normal operation; no packet boundary reached yet. */
        BLE_SEARCHING = 0,

        /**
         * Preamble and access address have been matched; the processor is now
         * collecting PDU bits.  The caller does not need to do anything special —
         * keep calling `ble_push_bit()` with subsequent bits.
         */
        BLE_COLLECTING = 1,

        /**
         * A complete packet has been assembled and dewhitened.
         * Call `ble_get_packet()` before the next `ble_push_bit()` call to
         * retrieve it; the internal buffer will be overwritten when the next
         * packet begins.
         */
        BLE_VALID_PACKET = 2,

    } ble_status_t;

    /* ---------------------------------------------------------------------------
     * ble_channel_processor_t — per-channel decoder state
     * ---------------------------------------------------------------------------*/

    /**
     * @brief All state required to decode one BLE advertising channel.
     *
     * Treat this as an opaque type; access members only through the API below.
     * Direct field access is not part of the stable API and may change.
     */
    typedef struct
    {
        /* -- Configuration (set once at init) ---------------------------------- */

        /** BLE channel index (0–39).  Used to seed the dewhitening LFSR. */
        uint8_t channel_index;

        /* -- Pattern detection ------------------------------------------------- */

        /**
         * 40-bit sliding window holding the most recent 40 received bits
         * (bit 0 = oldest, bit 39 = newest, LSB-first packing).
         * Used to detect [preamble || access address].
         */
        uint64_t bit_window;

        /* -- Packet collection ------------------------------------------------- */

        /** Non-zero while a packet is being assembled. */
        int collecting;

        /** Number of PDU bits collected so far during the current packet. */
        unsigned int bits_collected;

        /**
         * Raw collection buffer (before dewhitening).
         * Holds the PDU (header + payload) followed by the 3 CRC bytes, so it
         * must be large enough for the maximum PDU plus the CRC.
         */
        uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];

        /**
         * Non-zero once the 2-byte PDU header has been peeked at to determine
         * the payload length.  Set during collection, cleared by reset_collection.
         */
        int header_decoded;

        /**
         * Total number of bits to collect for the current packet
         * (2-byte header + payload + 3-byte CRC, all in bits).
         * Valid only when header_decoded != 0.
         */
        unsigned int bits_to_collect;

        /* -- Last valid packet ------------------------------------------------- */

        /** The most recently completed packet; valid when `packet_ready != 0`. */
        ble_packet_t last_packet;

        /** Non-zero when `last_packet` holds an unread valid packet. */
        int packet_ready;

    } ble_channel_processor_t;

    /* ---------------------------------------------------------------------------
     * API
     * ---------------------------------------------------------------------------*/

    /**
     * @brief Initialise a channel processor.
     *
     * Must be called before the first `ble_push_bit()` on this processor.
     * Safe to call again at any time to reset decoder state.
     *
     * @param proc          Pointer to the processor to initialise.  Must not be NULL.
     * @param channel_index BLE channel index (0–39).  Use 37, 38, or 39 for
     *                      advertising channels.
     */
    void ble_processor_init(ble_channel_processor_t *proc, uint8_t channel_index);

    /**
     * @brief Push one demodulated bit into the processor.
     *
     * Call this function for every bit produced by your GFSK demodulator, in
     * reception order (LSB of each byte first, as transmitted over-the-air).
     *
     * The function advances the internal state machine and returns the current
     * processing status.  The only value that requires action from the caller is
     * `BLE_VALID_PACKET`; all other values are informational.
     *
     * @param proc  Pointer to an initialised processor.  Must not be NULL.
     * @param bit   The demodulated bit value.  Only the LSB is examined; any
     *              non-zero value is treated as logic 1.
     *
     * @return  `BLE_VALID_PACKET` when a complete packet is ready (call
     *          `ble_get_packet()` to retrieve it), `BLE_COLLECTING` while
     *          assembling a packet, `BLE_SEARCHING` when scanning for the next
     *          preamble, or `BLE_ERROR` on invalid input.
     */
    ble_status_t ble_push_bit(ble_channel_processor_t *proc, uint8_t bit);

    /**
     * @brief Retrieve the last decoded packet.
     *
     * Copy the most recently completed packet into `*out`.  The internal packet
     * buffer is marked as consumed and will be overwritten when the next packet
     * completes.
     *
     * This function should be called promptly after `ble_push_bit()` returns
     * `BLE_VALID_PACKET`.  It is safe to call from an SDR RX callback.
     *
     * @param proc  Pointer to the processor.  Must not be NULL.
     * @param out   Destination buffer.  Must not be NULL.
     *
     * @return  0 on success, -1 if no valid packet is available (either `proc`
     *          or `out` is NULL, or `ble_push_bit()` has not yet returned
     *          `BLE_VALID_PACKET` since the last retrieval).
     */
    int ble_get_packet(ble_channel_processor_t *proc, ble_packet_t *out);

    /**
     * @brief Print a human-readable summary of a decoded BLE advertising packet
     *        to stdout.
     *
     * Decodes and displays:
     *  - PDU type name and description (e.g. ADV_IND, CONNECT_IND)
     *  - AdvA / InitA / ScanA addresses in colon-separated notation, with
     *    address type (Public / Random Static / Random Private Resolvable /
     *    Random Private Non-Resolvable) derived from the TxAdd/RxAdd header
     *    flags and the top 2 bits of the address MSB.
     *  - TargetA for ADV_DIRECT_IND
     *  - AdvData as hex + ASCII for packet types that carry it; Manufacturer
     *    Specific Data (AD type 0xFF) with company name lookup.
     *  - CRC value and pass/fail result.
     *
     * This function does not print RSSI — that information is not part of the
     * decoded packet and should be printed by the caller.
     *
     * @param pkt  Pointer to the decoded packet.  Must not be NULL.
     */
    void ble_print_packet(const ble_packet_t *pkt);

    /**
     * @brief Verify the CRC of a decoded BLE advertising packet.
     *
     * Computes the 24-bit CRC over the PDU (header + payload) using the
     * advertising channel initial value (BLE_CRC_INIT_ADV) and compares
     * the result against the received CRC stored in `pkt->crc`.
     *
     * CRC generator polynomial: x^24 + x^10 + x^9 + x^6 + x^4 + x^3 + x + 1
     * (Bluetooth Core Spec Vol 6, Part B, §3.1.1)
     *
     * @param pkt  Pointer to the packet to verify.  Must not be NULL.
     * @return 1 if the CRC matches, 0 if it does not or if `pkt` is NULL.
     */
    int ble_verify_crc(const ble_packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* BLE_PHY_H */