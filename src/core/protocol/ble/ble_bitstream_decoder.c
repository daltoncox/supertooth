/**
 * @file ble_bitstream_decoder.c
 * @brief BLE PHY-layer bitstream processor implementation.
 *
 * See ble_bitstream_decoder.h for the public API and design notes.
 */

#include "ble_bitstream_decoder.h"

#include "ble_codec.h"
#include "ble_piconet.h"

#include <string.h>

#define BLE_PREAMBLE_PATTERN_1 0x55u
#define BLE_PREAMBLE_PATTERN_2 0xAAu
#define BLE_AA_END_BITS (BLE_PREAMBLE_BITS + BLE_AA_BITS)   /* 40 */
#define BLE_HEADER_BITS 16u

/* ---------------------------------------------------------------------------
 * Candidate buffer bit helpers (LSB-first air order)
 * ---------------------------------------------------------------------------*/

static uint8_t ble_get_bit(const uint8_t *buf, unsigned int i)
{
    return (uint8_t)((buf[i / 8u] >> (i % 8u)) & 1u);
}

static void ble_set_bit(uint8_t *buf, unsigned int i, uint8_t b)
{
    if (b)
        buf[i / 8u] |= (uint8_t)(1u << (i % 8u));
    else
        buf[i / 8u] &= (uint8_t)~(1u << (i % 8u));
}

/* Extract the 8-bit value at bit offset @p from (LSB-first air order). */
static uint8_t ble_window8(const uint8_t *buf, unsigned int from)
{
    uint8_t w = 0u;
    for (unsigned int j = 0; j < BLE_PREAMBLE_BITS; j++)
        w |= (uint8_t)(ble_get_bit(buf, from + j) << j);
    return w;
}

/* Discard the first @p k bits of the candidate buffer. */
static void ble_shift_bits(ble_bitstream_decoder_t *proc, unsigned int k)
{
    if (k == 0u)
        return;
    if (k >= proc->cand_bits)
    {
        proc->cand_bits = 0u;
        memset(proc->cand, 0, sizeof(proc->cand));
        return;
    }

    unsigned int remaining = proc->cand_bits - k;
    for (unsigned int i = 0; i < remaining; i++)
        ble_set_bit(proc->cand, i, ble_get_bit(proc->cand, i + k));
    for (unsigned int i = remaining; i < proc->cand_bits; i++)
        ble_set_bit(proc->cand, i, 0u);
    proc->cand_bits = remaining;
}

/* First preamble (0x55/0xAA) at any bit offset >= @p from, or -1. */
static int ble_find_next_preamble(const ble_bitstream_decoder_t *proc,
                                  unsigned int from)
{
    if (proc->cand_bits < BLE_PREAMBLE_BITS)
        return -1;

    for (unsigned int i = from; i + BLE_PREAMBLE_BITS <= proc->cand_bits; i++)
    {
        uint8_t w = ble_window8(proc->cand, i);
        if (w == BLE_PREAMBLE_PATTERN_1 || w == BLE_PREAMBLE_PATTERN_2)
            return (int)i;
    }
    return -1;
}

/* ---------------------------------------------------------------------------
 * State transitions
 * ---------------------------------------------------------------------------*/

/* Go back to SEARCH keeping only the last few buffered bits: they may be
 * the start of a preamble that completes with future bits. */
static void ble_restart_search(ble_bitstream_decoder_t *proc)
{
    unsigned int keep =
        (proc->cand_bits > (BLE_PREAMBLE_BITS - 1u)) ? (BLE_PREAMBLE_BITS - 1u)
                                                     : proc->cand_bits;
    ble_shift_bits(proc, proc->cand_bits - keep);
    proc->state = BLE_DEC_SEARCH;
    proc->header_decoded = 0;
    proc->pdu_target_bits = 0u;
}

/* CRC gate failed: reject only the bits up to the next preamble found
 * inside the buffered candidate, then resume there. */
static void ble_reject_and_rescan(ble_bitstream_decoder_t *proc)
{
    int k = ble_find_next_preamble(proc, 1u);
    if (k < 0)
    {
        ble_restart_search(proc);
        return;
    }

    ble_shift_bits(proc, (unsigned int)k);
    proc->detected_preamble = proc->cand[0];
    proc->state = BLE_DEC_COLLECT_AA;
    proc->header_decoded = 0;
    proc->pdu_target_bits = 0u;
}

/* A candidate was fully consumed (emitted): discard its bits and scan the
 * remainder for the next candidate preamble. */
static void ble_finish_candidate(ble_bitstream_decoder_t *proc)
{
    unsigned int consumed = BLE_AA_END_BITS + proc->pdu_target_bits;
    ble_shift_bits(proc, consumed);

    int k = ble_find_next_preamble(proc, 0u);
    if (k < 0)
    {
        ble_restart_search(proc);
        return;
    }

    ble_shift_bits(proc, (unsigned int)k);
    proc->detected_preamble = proc->cand[0];
    proc->state = BLE_DEC_COLLECT_AA;
    proc->header_decoded = 0;
    proc->pdu_target_bits = 0u;
}

static void ble_classify(ble_bitstream_decoder_t *proc)
{
    proc->access_address =
        (uint32_t)proc->cand[1] | ((uint32_t)proc->cand[2] << 8u) |
        ((uint32_t)proc->cand[3] << 16u) | ((uint32_t)proc->cand[4] << 24u);

    /* Classification is purely on the access address, regardless of
     * channel: the advertising AA always takes the advertising path,
     * anything else is a data candidate. Advertising PDUs showing up on a
     * data channel are interesting (a genuine bug somewhere, or a misread
     * worth diagnosing) and must stay visible rather than being masked. */
    proc->is_data_candidate =
        (proc->access_address != BLE_ADVERTISING_AA) ? 1 : 0;
    proc->state = BLE_DEC_COLLECT_PDU;
}

/* ---------------------------------------------------------------------------
 * Frame completion
 * ---------------------------------------------------------------------------*/

/* CONNECT_IND PDUs carry the connection's AA + CRCInit: seed them as
 * piconet candidates (they must still prove themselves on a data packet).
 * Only CRC-valid CONNECT_INDs seed — a corrupt one would seed garbage. */
static void ble_maybe_seed_connect_ind(ble_bitstream_decoder_t *proc,
                                       const uint8_t *raw_pdu,
                                       unsigned int frame_bytes)
{
    if (!proc->store ||
        frame_bytes < (2u + 2u * BLE_ADDR_LEN + BLE_CONNECT_IND_DATA_MAX_BYTES +
                       BLE_CRC_BYTES))
        return;

    uint8_t pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(pdu, raw_pdu, frame_bytes);
    ble_dewhiten(pdu, frame_bytes, proc->channel_index);

    if ((pdu[0] & 0x0Fu) != BLE_PDU_CONNECT_IND)
        return;

    unsigned int payload_len = ble_payload_length_from_header(pdu);
    if (payload_len < (2u * BLE_ADDR_LEN + BLE_CONNECT_IND_DATA_MAX_BYTES) ||
        frame_bytes < (2u + payload_len + BLE_CRC_BYTES))
        return;

    uint32_t rx_crc = ble_extract_crc(&pdu[2u + payload_len]);
    if (ble_crc_calc(pdu, 2u + payload_len, BLE_CRC_INIT_ADV) != rx_crc)
        return;

    ble_connect_ind_params_t params;
    if (ble_connect_ind_parse(&pdu[2u + 2u * BLE_ADDR_LEN], &params) != 0)
        return;

    ble_piconet_store_seed_candidate(proc->store, params.access_address,
                                     params.crc_init);
}

static ble_status_t ble_complete_adv_frame(ble_bitstream_decoder_t *proc)
{
    unsigned int frame_bytes = proc->pdu_target_bits / 8u;

    memcpy(proc->last_frame.raw_pdu, &proc->cand[5], frame_bytes);
    proc->last_frame.phy = RECEIVER_PHY_LE_1M;
    proc->last_frame.kind = BLE_FRAME_ADVERTISING;
    proc->last_frame.preamble = proc->detected_preamble;
    proc->last_frame.access_address = proc->access_address;
    proc->last_frame.crc_init = BLE_CRC_INIT_ADV;
    proc->last_frame.crc_ok = 0u;   /* computed by the codec, as before */
    proc->last_frame.raw_pdu_bytes = (uint16_t)frame_bytes;
    proc->frame_ready = 1;

    ble_maybe_seed_connect_ind(proc, &proc->cand[5], frame_bytes);
    ble_finish_candidate(proc);
    return BLE_VALID_PACKET;
}

/* CRC-gate a completed data candidate against the piconet store.
 * Returns nonzero when the frame was accepted and emitted. */
static int ble_gate_data_candidate(ble_bitstream_decoder_t *proc)
{
    unsigned int frame_bytes = proc->pdu_target_bits / 8u;
    unsigned int pdu_bytes = frame_bytes - BLE_CRC_BYTES;

    uint8_t pdu[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(pdu, &proc->cand[5], frame_bytes);
    ble_dewhiten(pdu, frame_bytes, proc->channel_index);

    uint32_t rx_crc = ble_extract_crc(&pdu[pdu_bytes]);
    uint32_t crc_init_used = 0u;
    if (ble_piconet_store_gate_data_pdu(proc->store, proc->access_address,
                                        pdu, pdu_bytes, rx_crc,
                                        &crc_init_used) != BLE_GATE_ACCEPT)
        return 0;

    memcpy(proc->last_frame.raw_pdu, &proc->cand[5], frame_bytes);
    proc->last_frame.phy = RECEIVER_PHY_LE_1M;
    proc->last_frame.kind = BLE_FRAME_DATA;
    proc->last_frame.preamble = proc->detected_preamble;
    proc->last_frame.access_address = proc->access_address;
    proc->last_frame.crc_init = crc_init_used;
    proc->last_frame.crc_ok = 1u;
    proc->last_frame.raw_pdu_bytes = (uint16_t)frame_bytes;
    proc->frame_ready = 1;

    ble_finish_candidate(proc);
    return 1;
}

/* ---------------------------------------------------------------------------
 * Drain loop: advance the state machine over buffered bits
 * ---------------------------------------------------------------------------*/

static ble_status_t ble_drain(ble_bitstream_decoder_t *proc)
{
    for (;;)
    {
        if (proc->state == BLE_DEC_SEARCH)
        {
            if (proc->cand_bits < BLE_PREAMBLE_BITS)
                return BLE_SEARCHING;

            /* Only the newest 8-bit window can be a fresh preamble. */
            uint8_t w = ble_window8(proc->cand,
                                    proc->cand_bits - BLE_PREAMBLE_BITS);
            if (w == BLE_PREAMBLE_PATTERN_1 || w == BLE_PREAMBLE_PATTERN_2)
            {
                /* Keep only the preamble bits as the candidate start. */
                ble_shift_bits(proc, proc->cand_bits - BLE_PREAMBLE_BITS);
                proc->detected_preamble = w;
                proc->state = BLE_DEC_COLLECT_AA;
                continue;
            }

            /* Not a preamble: drop the oldest bit and wait for more. */
            ble_shift_bits(proc, 1u);
            return BLE_SEARCHING;
        }

        if (proc->state == BLE_DEC_COLLECT_AA)
        {
            if (proc->cand_bits < BLE_AA_END_BITS)
                return BLE_COLLECTING;
            ble_classify(proc);
            continue;
        }

        /* BLE_DEC_COLLECT_PDU */
        unsigned int pdu_bits = proc->cand_bits - BLE_AA_END_BITS;
        if (!proc->header_decoded)
        {
            if (pdu_bits < BLE_HEADER_BITS)
                return BLE_COLLECTING;

            uint8_t header[2];
            header[0] = proc->cand[5];
            header[1] = proc->cand[6];
            ble_dewhiten(header, sizeof(header), proc->channel_index);
            proc->pdu_target_bits =
                (2u + ble_payload_length_from_header(header) + BLE_CRC_BYTES) * 8u;
            proc->header_decoded = 1;
        }
        if (pdu_bits < proc->pdu_target_bits)
            return BLE_COLLECTING;

        if (!proc->is_data_candidate)
            return ble_complete_adv_frame(proc);

        if (ble_gate_data_candidate(proc))
            return BLE_VALID_PACKET;

        ble_reject_and_rescan(proc);
        if (proc->state == BLE_DEC_SEARCH)
            return BLE_SEARCHING;
        /* Otherwise loop and re-drain the rescan backlog. */
    }
}

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

void ble_bitstream_decoder_init(ble_bitstream_decoder_t *proc,
                                uint8_t channel_index,
                                struct ble_piconet_store *store)
{
    if (!proc)
        return;

    memset(proc, 0, sizeof(*proc));
    proc->channel_index = channel_index;
    proc->store = store;
    proc->state = BLE_DEC_SEARCH;
}

ble_status_t ble_bitstream_decoder_push_bit(ble_bitstream_decoder_t *proc, uint8_t bit)
{
    if (!proc)
        return BLE_ERROR;

    uint8_t b = bit ? 1u : 0u;

    if (proc->cand_bits >= BLE_CANDIDATE_MAX_BITS)
    {
        /* Defensive: the buffer cannot fill legitimately (candidates are
         * length-bounded below the capacity). Reject the candidate. */
        ble_restart_search(proc);
        return BLE_ERROR;
    }

    ble_set_bit(proc->cand, proc->cand_bits, b);
    proc->cand_bits++;

    return ble_drain(proc);
}

int ble_bitstream_decoder_get_frame(ble_bitstream_decoder_t *proc, ble_frame_t *out)
{
    if (!proc || !out || !proc->frame_ready)
        return -1;

    memcpy(out, &proc->last_frame, sizeof(*out));
    memset(&proc->last_frame, 0, sizeof(proc->last_frame));
    proc->frame_ready = 0;
    return 0;
}
