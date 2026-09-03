/**
 * @file ble_test_stream.h
 * @brief Test helper: build air-order BLE bitstreams and feed a decoder.
 *
 * Streams are packed LSB-first (air order), exactly as the demodulator
 * delivers them: preamble, access address, then whitened PDU+CRC. Whitening
 * uses ble_dewhiten() (an involution) and CRCs ble_crc_calc(), so vectors
 * are self-generated — no external fixtures.
 */

#ifndef BLE_TEST_STREAM_H
#define BLE_TEST_STREAM_H

#include <stdint.h>
#include <string.h>

#include "ble_bitstream_decoder.h"
#include "ble_codec.h"
#include "ble_piconet.h"

#define BTS_MAX_BITS 8192u
#define BTS_MAX_FRAMES 16u

typedef struct
{
    uint8_t bytes[BTS_MAX_BITS / 8u];
    unsigned int count;   /* number of bits */
} ble_test_stream_t;

typedef struct
{
    ble_frame_t frames[BTS_MAX_FRAMES];
    unsigned int count;
} ble_test_out_t;

static inline void bts_reset(ble_test_stream_t *s)
{
    memset(s, 0, sizeof(*s));
}

static inline void bts_push_bit(ble_test_stream_t *s, uint8_t b)
{
    if (s->count >= BTS_MAX_BITS)
        return;
    if (b)
        s->bytes[s->count / 8u] |= (uint8_t)(1u << (s->count % 8u));
    s->count++;
}

static inline void bts_push_byte(ble_test_stream_t *s, uint8_t byte)
{
    for (unsigned int i = 0; i < 8u; i++)
        bts_push_bit(s, (uint8_t)((byte >> i) & 1u));
}

static inline void bts_push_bytes(ble_test_stream_t *s,
                                  const uint8_t *data,
                                  unsigned int len)
{
    for (unsigned int i = 0; i < len; i++)
        bts_push_byte(s, data[i]);
}

static inline void bts_push_preamble(ble_test_stream_t *s, uint8_t preamble)
{
    bts_push_byte(s, preamble);
}

/* Realistic preamble for an access address (Core Vol 6 2.1.1): the
 * preamble's last air bit complements the AA's first air bit, forcing an
 * edge — 0xAA (ends in 1) when AA bit0 is 0, 0x55 (ends in 0) when 1. */
static inline uint8_t bts_preamble_for_aa(uint32_t aa)
{
    return (aa & 1u) ? 0x55u : 0xAAu;
}

static inline void bts_push_aa(ble_test_stream_t *s, uint32_t aa)
{
    for (unsigned int i = 0; i < 4u; i++)
        bts_push_byte(s, (uint8_t)((aa >> (8u * i)) & 0xFFu));
}

/* Append PDU + CRC (whitened) for the given channel/CRCInit.
 * Returns the number of PDU+CRC bytes pushed. */
static inline unsigned int bts_push_pdu(ble_test_stream_t *s,
                                        const uint8_t *pdu,
                                        unsigned int pdu_bytes,
                                        uint8_t channel_index,
                                        uint32_t crc_init)
{
    uint8_t air[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(air, pdu, pdu_bytes);

    uint32_t crc = ble_crc_calc(pdu, pdu_bytes, crc_init) & 0xFFFFFFu;
    air[pdu_bytes + 0u] = ble_bit_reverse_byte((uint8_t)((crc >> 16u) & 0xFFu));
    air[pdu_bytes + 1u] = ble_bit_reverse_byte((uint8_t)((crc >> 8u) & 0xFFu));
    air[pdu_bytes + 2u] = ble_bit_reverse_byte((uint8_t)(crc & 0xFFu));

    ble_dewhiten(air, pdu_bytes + BLE_CRC_BYTES, channel_index);
    bts_push_bytes(s, air, pdu_bytes + BLE_CRC_BYTES);
    return pdu_bytes + BLE_CRC_BYTES;
}

/* PRNG (xorshift32) so noise is deterministic across runs. */
static inline uint32_t bts_rand32(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    *state = x ? x : 1u;
    return *state;
}

static inline void bts_push_noise(ble_test_stream_t *s,
                                  unsigned int nbits,
                                  uint32_t *prng)
{
    for (unsigned int i = 0; i < nbits; i++)
        bts_push_bit(s, (uint8_t)(bts_rand32(prng) & 1u));
}

/* Feed a stream through a decoder bit-by-bit, collecting emitted frames. */
static inline void bts_feed(ble_bitstream_decoder_t *proc,
                            const ble_test_stream_t *s,
                            ble_test_out_t *out)
{
    memset(out, 0, sizeof(*out));
    for (unsigned int i = 0; i < s->count; i++)
    {
        uint8_t b = (uint8_t)((s->bytes[i / 8u] >> (i % 8u)) & 1u);
        if (ble_bitstream_decoder_push_bit(proc, b) == BLE_VALID_PACKET)
        {
            while (out->count < BTS_MAX_FRAMES &&
                   ble_bitstream_decoder_get_frame(proc,
                        &out->frames[out->count]) == 0)
                out->count++;
        }
    }
    /* Drain any frames still queued after the final bit. */
    while (out->count < BTS_MAX_FRAMES &&
           ble_bitstream_decoder_get_frame(proc, &out->frames[out->count]) == 0)
        out->count++;
}

#endif /* BLE_TEST_STREAM_H */
