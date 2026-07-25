/**
 * @file ble_bitstream_decoder.c
 * @brief BLE PHY-layer bitstream processor implementation.
 *
 * See ble_bitstream_decoder.h for the public API and design notes.
 */

#include "ble_bitstream_decoder.h"

#include "ble_codec.h"

#include <string.h>

#define BLE_MAX_BITS_TO_COLLECT ((BLE_PDU_MAX_BYTES + BLE_CRC_BYTES) * 8u)
#define BLE_DETECTION_WINDOW_BITS 40u
#define BLE_PATTERN_AA_SHIFT 8u
#define BLE_PATTERN_PREAMBLE1 0x55ULL
#define BLE_PATTERN_PREAMBLE2 0xAAULL
#define BLE_PATTERN1 (BLE_PATTERN_PREAMBLE1 | (BLE_ADVERTISING_AA << BLE_PATTERN_AA_SHIFT))
#define BLE_PATTERN2 (BLE_PATTERN_PREAMBLE2 | (BLE_ADVERTISING_AA << BLE_PATTERN_AA_SHIFT))
#define BLE_WINDOW_MASK ((1ULL << BLE_DETECTION_WINDOW_BITS) - 1ULL)

static int ble_window_matches(uint64_t window, uint8_t *preamble_out)
{
    if (window == BLE_PATTERN1)
    {
        if (preamble_out)
            *preamble_out = (uint8_t)BLE_PATTERN_PREAMBLE1;
        return 1;
    }

    if (window == BLE_PATTERN2)
    {
        if (preamble_out)
            *preamble_out = (uint8_t)BLE_PATTERN_PREAMBLE2;
        return 1;
    }

    return 0;
}

static void ble_reset_collection(ble_bitstream_decoder_t *proc)
{
    proc->collecting = 0;
    proc->bits_collected = 0;
    proc->header_decoded = 0;
    proc->bits_to_collect = 0;
}

void ble_bitstream_decoder_init(ble_bitstream_decoder_t *proc, uint8_t channel_index)
{
    if (!proc)
        return;

    memset(proc, 0, sizeof(*proc));
    proc->channel_index = channel_index;
}

ble_status_t ble_bitstream_decoder_push_bit(ble_bitstream_decoder_t *proc, uint8_t bit)
{
    if (!proc)
        return BLE_ERROR;

    uint8_t b = bit ? 1u : 0u;

    if (proc->collecting)
    {
        unsigned int byte_idx = proc->bits_collected / 8u;
        unsigned int bit_idx = proc->bits_collected % 8u;

        if (byte_idx >= (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES))
        {
            ble_reset_collection(proc);
            return BLE_ERROR;
        }

        if (b)
            proc->raw_pdu[byte_idx] |= (uint8_t)(1u << bit_idx);

        proc->bits_collected++;

        if (!proc->header_decoded && proc->bits_collected >= 16u)
        {
            uint8_t header[2];
            memcpy(header, proc->raw_pdu, sizeof(header));
            ble_dewhiten(header, sizeof(header), proc->channel_index);
            proc->bits_to_collect =
                (2u + ble_payload_length_from_header(header) + BLE_CRC_BYTES) * 8u;
            proc->header_decoded = 1;
        }

        unsigned int target =
            proc->header_decoded ? proc->bits_to_collect : BLE_MAX_BITS_TO_COLLECT;
        if (proc->bits_collected >= target)
        {
            unsigned int frame_bytes = target / 8u;
            memcpy(proc->last_frame.raw_pdu, proc->raw_pdu, frame_bytes);
            proc->last_frame.phy = RECEIVER_PHY_LE_1M;
            proc->last_frame.preamble = proc->detected_preamble;
            proc->last_frame.access_address = BLE_ADVERTISING_AA;
            proc->last_frame.raw_pdu_bytes = (uint16_t)frame_bytes;
            proc->frame_ready = 1;
            proc->frame_bytes = frame_bytes;
            memset(proc->raw_pdu, 0, sizeof(proc->raw_pdu));
            ble_reset_collection(proc);
            return BLE_VALID_PACKET;
        }

        return BLE_COLLECTING;
    }

    proc->bit_window =
        ((proc->bit_window >> 1u) | ((uint64_t)b << 39u)) & BLE_WINDOW_MASK;
    if (ble_window_matches(proc->bit_window, &proc->detected_preamble))
    {
        proc->collecting = 1;
        proc->bits_collected = 0u;
        proc->bit_window = 0u;
        proc->frame_ready = 0;
        proc->frame_bytes = 0u;
        memset(proc->raw_pdu, 0, sizeof(proc->raw_pdu));
        return BLE_COLLECTING;
    }

    return BLE_SEARCHING;
}

int ble_bitstream_decoder_get_frame(ble_bitstream_decoder_t *proc, ble_frame_t *out)
{
    if (!proc || !out || !proc->frame_ready)
        return -1;

    memcpy(out, &proc->last_frame, sizeof(*out));
    memset(&proc->last_frame, 0, sizeof(proc->last_frame));
    proc->frame_ready = 0;
    proc->frame_bytes = 0u;
    return 0;
}
