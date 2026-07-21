/**
 * @file ble_bitstream_decoder.h
 * @brief BLE PHY-layer bitstream processor — real-time, push-bit API.
 *
 * Overview
 * --------
 * This module captures BLE advertising frames from a demodulated bitstream.
 * Framing state lives directly inside `ble_bitstream_decoder_t`, which keeps
 * ownership aligned with BR/EDR: the PHY owns capture and framing, while the
 * codec layer later turns a captured frame into a decoded packet model.
 *
 * Typical usage
 * -------------
 * @code
 *   ble_bitstream_decoder_t proc;
 *   ble_bitstream_decoder_init(&proc, 37);          // advertising channel 37
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

#include <stdint.h>

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

/* ---------------------------------------------------------------------------
 * ble_frame_t — a captured BLE advertising frame
 * ---------------------------------------------------------------------------*/

typedef struct
{
    uint8_t preamble;
    uint32_t access_address;
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

typedef struct
{
    uint8_t channel_index;
    uint64_t bit_window;
    int collecting;
    unsigned int bits_collected;
    uint8_t raw_pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    int header_decoded;
    unsigned int bits_to_collect;
    unsigned int frame_bytes;
    uint8_t detected_preamble;
    ble_frame_t last_frame;
    int frame_ready;
} ble_bitstream_decoder_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_bitstream_decoder_init(ble_bitstream_decoder_t *proc, uint8_t channel_index);
ble_status_t ble_bitstream_decoder_push_bit(ble_bitstream_decoder_t *proc, uint8_t bit);
int ble_bitstream_decoder_get_frame(ble_bitstream_decoder_t *proc, ble_frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_BITSTREAM_DECODER_H */