/**
 * @file ble_bitstream_decoder.h
 * @brief BLE PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module captures BLE frames from a demodulated bitstream. One unified
 * detector handles both advertising and data channel PDUs — there is no
 * mode to toggle:
 *
 *  1. SEARCH: an 8-bit sliding window flags a preamble (0x55 or 0xAA).
 *     The preamble alone is too common to accept a packet on, so every
 *     candidate is proven later.
 *  2. COLLECT_AA: the next 32 bits are the access address. If it is the
 *     advertising address (0x8E89BED6) the candidate is an advertising
 *     PDU; anything else is a possible data PDU. Classification is purely
 *     on the access address, regardless of channel.
 *  3. COLLECT_PDU: the dewhitened header length determines the candidate
 *     size (2 + len + 3 CRC bytes).
 *  4. Every candidate is emitted as captured. Advertising frames (and any
 *     CONNECT_IND they carry) are handed to the consumer ungate; data
 *     candidates are emitted raw too. Confirming a data candidate is a real
 *     packet — and recovering its CRCInit from CONNECT_IND seeding — is the
 *     consumer's job (the per-session piconet store owns CRC gating and
 *     CRCInit recovery). This keeps the decoder a pure PHY/framing stage,
 *     mirroring the BR/EDR path.
 *
 * Resync rule: a candidate that is not a real packet is detected later by
 * the consumer's CRC gate. To avoid false negatives when a false preamble
 * swallowed bits of a real packet, collection always resumes at the next
 * preamble found inside the buffered candidate (never trusting the
 * declared length to drop bits).
 *
 * Typical usage
 * -------------
 * @code
 *   ble_bitstream_decoder_t proc;
 *   ble_bitstream_decoder_init(&proc, 37);    // advertising ch 37
 *
 *   ble_status_t status = ble_bitstream_decoder_push_bit(&proc, bit);
 *   if (status == BLE_VALID_PACKET) {
 *       ble_frame_t frame;
 *       ble_bitstream_decoder_get_frame(&proc, &frame);
 *       // Pass frame to ble_decode_frame() ...
 *   }
 * @endcode
 */

#ifndef BLE_BITSTREAM_DECODER_H
#define BLE_BITSTREAM_DECODER_H

#include <math.h>
#include <stdint.h>

#include "phy.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * Public constants
 * ---------------------------------------------------------------------------*/

#define BLE_PDU_MAX_BYTES 258u
#define BLE_CRC_BYTES 3u
#define BLE_CRC_INIT_ADV 0x555555u
#define BLE_ADVERTISING_AA 0x8E89BED6UL
#define BLE_CH37_INDEX 37u
#define BLE_CH38_INDEX 38u
#define BLE_CH39_INDEX 39u
#define BLE_CH37_FREQ_HZ 2402000000u
#define BLE_CH38_FREQ_HZ 2426000000u
#define BLE_CH39_FREQ_HZ 2480000000u

/* LE RF channel geometry: 40 RF channels at 2 MHz spacing, RF k centered
 * on 2402 + 2k MHz. Advertising channels 37/38/39 occupy RF 0/12/39; data
 * channels 0..36 fill the rest. */
#define BLE_RF_CHANNEL_COUNT 40u
#define BLE_RF_CHANNEL_0_FREQ_HZ 2402000000u
#define BLE_RF_CHANNEL_BW_HZ 2000000u
#define BLE_RF_ADV0_INDEX 0u    /* LE 37 */
#define BLE_RF_ADV1_INDEX 12u   /* LE 38 */
#define BLE_RF_ADV2_INDEX 39u   /* LE 39 */

/* Candidate buffering: preamble (8) + access address (32) + max PDU+CRC. */
#define BLE_PREAMBLE_BITS 8u
#define BLE_AA_BITS 32u
#define BLE_CANDIDATE_MAX_BITS \
    (BLE_PREAMBLE_BITS + BLE_AA_BITS + (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES) * 8u)
#define BLE_CANDIDATE_MAX_BYTES ((BLE_CANDIDATE_MAX_BITS + 7u) / 8u + 6u)

/** Center frequency of LE RF channel @p rf (0..39) in Hz. */
static inline uint32_t ble_rf_channel_freq_hz(unsigned int rf)
{
    return BLE_RF_CHANNEL_0_FREQ_HZ + (uint32_t)rf * BLE_RF_CHANNEL_BW_HZ;
}

/** Nonzero when LE RF channel @p rf is an advertising channel (37/38/39). */
static inline int ble_rf_is_advertising(unsigned int rf)
{
    return rf == BLE_RF_ADV0_INDEX || rf == BLE_RF_ADV1_INDEX ||
           rf == BLE_RF_ADV2_INDEX;
}

/** LE channel number for RF channel @p rf (RF 0->37, 1..11->0..10,
 * 12->38, 13..38->11..36, 39->39). */
static inline uint8_t ble_channel_number_for_rf(unsigned int rf)
{
    if (rf == BLE_RF_ADV0_INDEX) return BLE_CH37_INDEX;
    if (rf == BLE_RF_ADV1_INDEX) return BLE_CH38_INDEX;
    if (rf >= BLE_RF_ADV2_INDEX) return BLE_CH39_INDEX;
    if (rf < BLE_RF_ADV1_INDEX) return (uint8_t)(rf - 1u);
    return (uint8_t)(rf - 2u);
}

/** LE RF channel carrying LE channel number @p ch (inverse of
 * ble_channel_number_for_rf). Returns 0xFF for out-of-range input. */
static inline unsigned int ble_rf_for_channel_number(unsigned int ch)
{
    if (ch == BLE_CH37_INDEX) return BLE_RF_ADV0_INDEX;
    if (ch == BLE_CH38_INDEX) return BLE_RF_ADV1_INDEX;
    if (ch == BLE_CH39_INDEX) return BLE_RF_ADV2_INDEX;
    if (ch <= 10u) return ch + 1u;
    if (ch <= 36u) return ch + 2u;
    return 0xFFu;
}

/** Nonzero when LE RF channel @p rf is fully inside a capture span of
 * @p sample_rate Hz around @p lo_hz (the same rule the channelizer uses to
 * place a processor). */
static inline int ble_rf_in_capture_span(unsigned int rf, uint64_t lo_hz,
                                         uint32_t sample_rate)
{
    double offset_hz = (double)ble_rf_channel_freq_hz(rf) - (double)lo_hz;
    return fabs(offset_hz) + 500000.0 <= (double)sample_rate / 2.0;
}

/* ---------------------------------------------------------------------------
 * ble_frame_t — a captured BLE frame (advertising or data)
 * ---------------------------------------------------------------------------*/

/** Identifies which PDU family a captured frame carries. */
typedef enum
{
    BLE_FRAME_ADVERTISING = 0,
    BLE_FRAME_DATA = 1,
} ble_frame_kind_t;

typedef struct
{
    /** Physical-layer modulation/coding of the captured packet. */
    receiver_phy_t phy;
    /** Advertising-channel or data-channel PDU. */
    ble_frame_kind_t kind;
    uint8_t preamble;
    uint32_t access_address;
    /** CRCInit used for this frame: the confirmed per-connection value for
     * data frames, BLE_CRC_INIT_ADV for advertising frames. */
    uint32_t crc_init;
    /** Stamped 1 by the framer for emitted data frames (they are CRC-gated);
     * 0 for advertising frames (the codec computes crc_ok instead). */
    uint8_t crc_ok;
    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    uint16_t raw_pdu_bytes;
} ble_frame_t;

/* ---------------------------------------------------------------------------
 * ble_status_t — return codes for ble_bitstream_decoder_push_bit()
 * ---------------------------------------------------------------------------*/

typedef enum
{
    BLE_ERROR = -1,
    BLE_SEARCHING = 0,
    BLE_COLLECTING = 1,
    BLE_VALID_PACKET = 2,
} ble_status_t;

/* ---------------------------------------------------------------------------
 * ble_bitstream_decoder_t — per-channel decoder state
 * ---------------------------------------------------------------------------*/

/** Internal framing states. */
typedef enum
{
    BLE_DEC_SEARCH = 0,
    BLE_DEC_COLLECT_AA = 1,
    BLE_DEC_COLLECT_PDU = 2,
} ble_dec_state_t;

/** Maximum frames queued between a push_bit and get_frame calls. */
#define BLE_DEC_FRAME_QUEUE 8u

typedef struct
{
    uint8_t channel_index;

    ble_dec_state_t state;
    uint8_t detected_preamble;
    uint32_t access_address;   /* classified candidate AA */

    /* Candidate bit buffer, LSB-first air order: preamble at bit 0, AA at
     * bit 8, PDU at bit 40 (PDU bytes are byte-aligned at cand[5..]). */
    uint8_t cand[BLE_CANDIDATE_MAX_BYTES];
    unsigned int cand_bits;

    int is_data_candidate;
    int header_decoded;
    unsigned int pdu_target_bits;   /* PDU+CRC bits once the header is known */

    ble_frame_t last_frame;
    int frame_ready;   /* transient: set while assembling a frame */

    /* Frames emitted by the decoder but not yet retrieved by the consumer.
     * A single push_bit may complete several candidates (a real packet
     * recovered from inside a rejected false candidate), so emission is
     * queued and drained by ble_bitstream_decoder_get_frame(). */
    ble_frame_t pending[BLE_DEC_FRAME_QUEUE];
    uint8_t pending_count;
} ble_bitstream_decoder_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_bitstream_decoder_init(ble_bitstream_decoder_t *proc,
                                 uint8_t channel_index);
ble_status_t ble_bitstream_decoder_push_bit(ble_bitstream_decoder_t *proc, uint8_t bit);
int ble_bitstream_decoder_get_frame(ble_bitstream_decoder_t *proc, ble_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_BITSTREAM_DECODER_H */
