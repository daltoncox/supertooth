/**
 * @file bredr_codec.h
 * @brief Reusable BR/EDR codec helpers.
 */

#ifndef BREDR_CODEC_H
#define BREDR_CODEC_H

#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "phy.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BREDR_FHS_INFO_BYTES 18u
#define BREDR_ACL_MAX_USER_PAYLOAD_BYTES 1021u
#define BREDR_SYNC_MAX_PAYLOAD_BYTES 540u

typedef enum
{
    BREDR_DECODE_RAW_ONLY = 0,
    BREDR_DECODE_HEADER_ONLY,
    BREDR_DECODE_FAMILY_ONLY,
    BREDR_DECODE_PARTIAL_PAYLOAD,
    BREDR_DECODE_FULL_PAYLOAD,
} bredr_decode_status_t;

typedef enum
{
    BREDR_DECODE_LIMIT_NONE = 0,
    BREDR_DECODE_LIMIT_NO_HEADER,
    BREDR_DECODE_LIMIT_MISSING_CONTEXT,
    BREDR_DECODE_LIMIT_HEC_FAILED,
    BREDR_DECODE_LIMIT_UNSUPPORTED_PACKET_TYPE,
    BREDR_DECODE_LIMIT_UNSUPPORTED_PAYLOAD_CODING,
    BREDR_DECODE_LIMIT_TRUNCATED_PAYLOAD,
    BREDR_DECODE_LIMIT_IMPOSSIBLE_ACL_LENGTH,
    BREDR_DECODE_LIMIT_INVALID_ACL_HEADER,
    BREDR_DECODE_LIMIT_FEC_UNCORRECTABLE,
    BREDR_DECODE_LIMIT_ESCO_CRC_UNRESOLVED,
} bredr_decode_limit_t;

typedef enum
{
    BREDR_PAYLOAD_FAMILY_NONE = 0,
    BREDR_PAYLOAD_FAMILY_CONTROL,
    BREDR_PAYLOAD_FAMILY_FHS,
    BREDR_PAYLOAD_FAMILY_ACL,
    BREDR_PAYLOAD_FAMILY_SCO,
    BREDR_PAYLOAD_FAMILY_ESCO,
    BREDR_PAYLOAD_FAMILY_HYBRID,
    BREDR_PAYLOAD_FAMILY_UNKNOWN,
} bredr_payload_family_t;

typedef enum
{
    BREDR_FEC_MODE_NONE = 0,
    BREDR_FEC_MODE_1_3,
    BREDR_FEC_MODE_2_3,
} bredr_fec_mode_t;

typedef struct
{
    uint8_t lt_addr;
    uint8_t type;
    uint8_t flow;
    uint8_t arqn;
    uint8_t seqn;
    uint8_t hec;
    uint8_t hec_ok;
} bredr_decoded_header_t;

typedef struct
{
    uint64_t parity_bits;
    uint32_t lap;
    uint8_t eir;
    uint8_t reserved;
    uint8_t sr;
    uint8_t sp;
    uint8_t uap;
    uint16_t nap;
    uint32_t class_of_device;
    uint8_t lt_addr;
    uint32_t clk27_2;
    uint8_t reserved_tail;
} bredr_fhs_payload_t;

typedef struct
{
    uint8_t llid; // 2 bits (0=reserved, 1=continuation, 2=L2CAP start, 3=control)
    uint8_t flow; // ACL flow control bit (0 or 1)
    uint16_t length; // 5 bits in single-slot packets, 10 bits in multi-slot packets. Does not include the header, MIC, or CRC.
    uint8_t user_payload[BREDR_ACL_MAX_USER_PAYLOAD_BYTES];
    uint8_t has_mic;
    uint32_t mic;
    uint8_t mic_ok;
    uint8_t has_crc;
    uint16_t crc;
    uint8_t crc_ok;
} bredr_acl_payload_t;

typedef struct
{
    uint16_t payload_bytes;
    uint8_t payload[BREDR_SYNC_MAX_PAYLOAD_BYTES];
    uint8_t has_crc;
    uint16_t crc;
    uint8_t crc_ok;
    uint8_t is_esco;
} bredr_sync_packet_t;

typedef struct
{
    uint16_t payload_bytes;
} bredr_unknown_payload_t;

typedef struct
{
    /** Physical-layer modulation/coding, propagated from the captured frame. */
    receiver_phy_t phy;
    bredr_decode_status_t status;
    bredr_decode_limit_t limit;
    bredr_payload_family_t family;
    uint16_t air_payload_bytes;
    uint16_t decoded_payload_bytes;
    bredr_decoded_header_t header;
    union
    {
        bredr_fhs_payload_t fhs;
        bredr_acl_payload_t acl;
        bredr_sync_packet_t sync;
        bredr_unknown_payload_t unknown;
    } payload;
} bredr_packet_t;

uint8_t bredr_reverse_byte(uint8_t b);
uint8_t bredr_decode_uap_from_hec(uint16_t data, uint8_t hec);

/**
 * @brief Decode 1/3-rate FEC bits packed LSB-first.
 *
 * The input is processed in 3-bit repetition blocks. Each complete block
 * emits one decoded bit into `output_bits`, also packed LSB-first.
 *
 * @param input_bits         Packed input bits.
 * @param input_bit_count    Number of valid input bits; must be a multiple of 3.
 * @param output_bits        Packed output buffer.
 * @param output_bit_count   Receives the number of decoded bits written.
 * @return                   Number of corrected bit disagreements, or -1 on invalid input.
 */
int bredr_fec_decode_1_3(const uint8_t *input_bits,
                         unsigned int input_bit_count,
                         uint8_t *output_bits,
                         unsigned int *output_bit_count);

/**
 * @brief Decode 2/3-rate FEC bits packed LSB-first.
 *
 * The input is processed in 15-bit `(15,10)` shortened Hamming codewords.
 * Each complete codeword emits 10 decoded bits into `output_bits`, also
 * packed LSB-first.
 *
 * @param input_bits         Packed input bits.
 * @param input_bit_count    Number of valid input bits; must be a multiple of 15.
 * @param output_bits        Packed output buffer.
 * @param output_bit_count   Receives the number of decoded bits written.
 * @return                   Number of corrected/detected FEC errors, or -1 on invalid input.
 */
int bredr_fec_decode_2_3(const uint8_t *input_bits,
                         unsigned int input_bit_count,
                         uint8_t *output_bits,
                         unsigned int *output_bit_count);

/**
 * @brief Compute the BR/EDR payload CRC-16 (BT Core Spec Vol 2, Part B, §7.1.1).
 *
 * The LFSR is initialised with the bit-reversed UAP in the high byte; bits
 * are consumed in air order (LSB-first within each byte of @p data). The
 * returned value is the 16-bit CRC with the low byte being the first CRC
 * byte on air.
 *
 * @param data        Packed input bits (LSB-first within each byte).
 * @param bit_count   Number of valid input bits.
 * @param uap         8-bit Upper Address Part used to seed the register.
 * @return            16-bit CRC.
 */
uint16_t bredr_payload_crc(const uint8_t *data,
                            unsigned int bit_count,
                            uint8_t uap);

/**
 * @brief Dewhiten (or whiten — the operation is symmetric) on-air payload
 *        bits using the BR/EDR whitening sequence for a given CLK1-6.
 *
 * The whitening sequence starts at the bit index that follows
 * @p logical_offset_bits of already-whitened bits (18 for a standard BR/EDR
 * payload, i.e. after the access code and packet header).  Because whitening
 * is an XOR, the same routine is used to reconstruct the payload on receive
 * and to build it on transmit.
 *
 * @param src_air         Packed on-air bits (LSB-first within each byte).
 * @param src_air_bits    Number of valid bits in @p src_air.
 * @param clk6            CLK1-6 whitening key (0-63).
 * @param logical_offset_bits  Whitening start offset, in bits (18 for payload).
 * @param dst             Output buffer.
 * @param dst_capacity    Capacity of @p dst, in bytes.
 * @return                Number of bytes written to @p dst.
 */
unsigned int bredr_dewhiten_air_payload_bytes(const uint8_t *src_air,
                                              unsigned int src_air_bits,
                                              uint8_t clk6,
                                              unsigned int logical_offset_bits,
                                              uint8_t *dst,
                                              unsigned int dst_capacity);



/**
 * @brief Return the FEC mode used by the on-air payload of the given
 *        BR/EDR packet type.
 *
 * 1/3 is used by HV1. 2/3 is used by DM1, DM3, DM5, HV2, and EV4. All
 * other types return `BREDR_FEC_MODE_NONE`.
 *
 * @param type_code  Lower 4 bits are the BR/EDR type code (0x00..0x0F).
 * @return           The FEC mode.
 */
bredr_fec_mode_t bredr_fec_mode_for_type(uint8_t type_code);

/**
 * @brief Count the number of 1/3 FEC blocks in @p input_bits that are valid.
 *
 * Each 3-bit block is considered valid iff all three bits are the same
 * (i.e. 000 or 111). The input is processed in 3-bit groups packed
 * LSB-first, exactly as the on-air bit buffer is laid out for
 * `bredr_fec_decode_1_3()`.
 *
 * @param input_bits        Packed input bits (LSB-first within each byte).
 * @param input_bit_count   Number of valid input bits; must be a non-zero
 *                          multiple of 3.
 * @return                  Number of valid 1/3 blocks, or -1 on invalid input.
 */
int valid_fec_1_3_blocks(const uint8_t *input_bits,
                         unsigned int input_bit_count);

/**
 * @brief Count the number of 2/3 FEC codewords in @p input_bits that are valid.
 *
 * Each 15-bit `(15,10)` shortened Hamming codeword is considered valid iff
 * its syndrome under the BT spec generator (effective polynomial 0x2B due
 * to reciprocal bit ordering, see `bredr_fec_2_3_remainder`) is zero. The
 * input is processed in 15-bit groups packed LSB-first, exactly as the
 * on-air bit buffer is laid out for `bredr_fec_decode_2_3()`.
 *
 * A codeword that contains a single bit error is **not** valid by this
 * definition: only the original (unmodified) transmitted codewords count.
 *
 * @param input_bits        Packed input bits (LSB-first within each byte).
 * @param input_bit_count   Number of valid input bits; must be a non-zero
 *                          multiple of 15.
 * @return                  Number of valid 2/3 codewords, or -1 on invalid input.
 */
int valid_fec_2_3_blocks(const uint8_t *input_bits,
                         unsigned int input_bit_count);

/**
 * @brief Decode a dewhitened 18-bit BR/EDR header into structured fields.
 *
 * The input bits must already be FEC-decoded and dewhitened, in air order:
 *   bits  0- 2  LT_ADDR  (bit 0 = first transmitted)
 *   bits  3- 6  TYPE     (bit 0 = first transmitted)
 *   bit   7     FLOW
 *   bit   8     ARQN
 *   bit   9     SEQN
 *   bits 10-17  HEC      (bit 10 = first transmitted, i.e. LFSR bit 7)
 *
 * @param dewhitened_header  Header bits, one per element (0 or 1).
 * @param out                Decoded header fields.
 */
void bredr_decode_dewhitened_header(const uint8_t dewhitened_header[18],
                                    bredr_decoded_header_t *out);

/**
 * @brief Decode and unwhiten the 18 logical BR/EDR header bits.
 *
 * The result is 18 unwhitened bits in air order:
 *   bits  0- 2  LT_ADDR  (bit 0 = first transmitted)
 *   bits  3- 6  TYPE     (bit 0 = first transmitted)
 *   bit   7     FLOW
 *   bit   8     ARQN
 *   bit   9     SEQN
 *   bits 10-17  HEC      (bit 10 = first transmitted, i.e. LFSR bit 7)
 *
 * @param frame  Frame with a valid `header_raw` field (has_header must be 1).
 * @param clk6   CLK1-6 whitening key (0-63).
 * @param bits   Output array of 18 bits, one per element (0 or 1).
 */
void bredr_decode_header_bits(const bredr_frame_t *frame,
                              uint8_t clk6,
                              uint8_t bits[18]);

unsigned int bredr_on_air_payload_bits(uint8_t type_code);
unsigned int bredr_acl_max_user_payload_bytes(uint8_t type_code);

/**
 * @brief Generate a 64-bit BR/EDR sync word from a 24-bit LAP.
 *
 * Implements the (64,30) linear block code used to encode the LAP into
 * the access code sync word (Bluetooth Core Spec Vol 2, Part B, §6.3.3).
 *
 * @param lap  24-bit Lower Address Part.
 * @return     64-bit sync word in host order (bit 0 = first transmitted).
 */
uint64_t bredr_gen_syncword(uint32_t lap);

/**
 * @brief Verify the HEC of a BR/EDR header for a given CLK1-6 and UAP.
 *
 * Combines FEC majority-vote decode, header unwhitening, and HEC
 * verification in one call.  Returns 0 if `frame` is NULL or has no header.
 *
 * @param frame  Captured frame with a valid `header_raw` field.
 * @param uap    8-bit Upper Address Part.
 * @param clk6   CLK1-6 whitening key (0–63).
 * @return       1 if the HEC matches, 0 otherwise.
 */
int bredr_hec_ok_for_clk6(const bredr_frame_t *frame, uint8_t uap, uint8_t clk6);
const char *bredr_packet_type_name(uint8_t type_code);
const char *bredr_payload_family_name(bredr_payload_family_t family);
const char *bredr_decode_limit_desc(bredr_decode_limit_t limit);
const char *bredr_llid_name(uint8_t llid);
int bredr_decode_frame(const bredr_frame_t *frame,
                       uint8_t uap,
                       uint8_t clk1_6,
                       bredr_packet_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_CODEC_H */