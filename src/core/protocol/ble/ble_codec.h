/**
 * @file ble_codec.h
 * @brief Reusable BLE codec helpers shared by framing and packet decode.
 */

#ifndef BLE_CODEC_H
#define BLE_CODEC_H

#include <stdint.h>

#include "ble_bitstream_decoder.h"
#include "phy.h"

#ifdef __cplusplus
extern "C" {
#endif

void ble_dewhiten(uint8_t *data, unsigned int length_bytes, uint8_t channel_index);
uint8_t ble_bit_reverse_byte(uint8_t b);
uint32_t ble_crc_calc(const uint8_t *data, unsigned int len, uint32_t init);
unsigned int ble_payload_length_from_header(const uint8_t header[2]);
uint32_t ble_extract_crc(const uint8_t *crc_bytes);

/**
 * Initialise the CRCInit-recovery lookup tables (idempotent).
 *
 * The BLE CRC is affine in its init value: crc(M, I) = A(M) ^ T_n*I, where
 * T_n is an invertible 24x24 GF(2) matrix that depends only on the message
 * bit length. The tables built here invert T_16 (2-byte messages, i.e.
 * 0-length LL data PDUs) so a CRCInit candidate can be recovered from any
 * empty data packet. Called by ble_piconet_store_init() and lazily by
 * ble_crc_reverse_init_len2().
 */
void ble_crc_tables_init(void);

/**
 * Recover the CRCInit that produces @p rx_crc over a 2-byte header.
 *
 * @param header  Dewhitened 2-byte LL data PDU header (length must be 0
 *                for the result to be meaningful as a CRCInit candidate).
 * @param rx_crc  Received CRC (already byte-reversed via ble_extract_crc).
 * @return        24-bit CRCInit candidate.
 */
uint32_t ble_crc_reverse_init_len2(const uint8_t header[2], uint32_t rx_crc);

#define BLE_ADDR_LEN 6u
#define BLE_ADV_DATA_MAX_BYTES (BLE_PDU_MAX_BYTES - 2u - BLE_ADDR_LEN)
#define BLE_CONNECT_IND_DATA_MAX_BYTES 22u
#define BLE_RESERVED_PAYLOAD_MAX_BYTES (BLE_PDU_MAX_BYTES - 2u)
#define BLE_LL_PAYLOAD_MAX_BYTES 255u

typedef enum
{
    BLE_ADDR_PUBLIC = 0,
    BLE_ADDR_RANDOM = 1,
} ble_addr_kind_t;

/* Advertising-channel PDU types (4-bit field). Data-channel PDUs are
 * discriminated by ble_packet_t.is_adv_pdu instead of a PDU type. */
typedef enum
{
    BLE_PDU_ADV_IND = 0x00u,
    BLE_PDU_ADV_DIRECT_IND = 0x01u,
    BLE_PDU_ADV_NONCONN_IND = 0x02u,
    BLE_PDU_SCAN_REQ = 0x03u,
    BLE_PDU_SCAN_RSP = 0x04u,
    BLE_PDU_CONNECT_IND = 0x05u,
    BLE_PDU_ADV_SCAN_IND = 0x06u,
} ble_pdu_type_t;

typedef struct
{
    uint8_t addr[BLE_ADDR_LEN];
    ble_addr_kind_t kind;
} ble_address_t;

typedef struct
{
    ble_address_t adv_addr;
    uint8_t adv_data[BLE_ADV_DATA_MAX_BYTES];
    uint8_t adv_data_len;
} ble_adv_data_t;

typedef struct
{
    ble_address_t adv_addr;
    ble_address_t target_addr;
} ble_adv_direct_ind_t;

typedef struct
{
    ble_address_t scanner_addr;
    ble_address_t adv_addr;
} ble_adv_scan_req_t;

typedef struct
{
    ble_address_t init_addr;
    ble_address_t adv_addr;
    uint8_t ll_data[BLE_CONNECT_IND_DATA_MAX_BYTES];
    uint8_t ll_data_len;
} ble_adv_connect_ind_t;

typedef struct
{
    uint8_t payload[BLE_RESERVED_PAYLOAD_MAX_BYTES];
    uint8_t payload_len;
} ble_adv_unknown_t;

/** Decoded advertising-channel PDU: header fields + typed payload. */
typedef struct
{
    uint8_t pdu_type;        /* ble_pdu_type_t (4-bit advertising type) */
    uint8_t payload_len;
    ble_addr_kind_t tx_addr_kind;
    ble_addr_kind_t rx_addr_kind;
    union
    {
        ble_adv_data_t adv_ind;
        ble_adv_direct_ind_t adv_direct_ind;
        ble_adv_data_t adv_nonconn_ind;
        ble_adv_scan_req_t scan_req;
        ble_adv_data_t scan_rsp;
        ble_adv_connect_ind_t connect_ind;
        ble_adv_data_t adv_scan_ind;
        ble_adv_unknown_t unknown;
    } payload;
} ble_adv_pdu_t;

/** Decoded data-channel (LL) PDU: header fields + raw payload. The payload
 * includes the 4-byte MIC when the link is encrypted (not parsed). */
typedef struct
{
    uint8_t llid;
    uint8_t nesn;
    uint8_t sn;
    uint8_t md;
    uint8_t payload[BLE_LL_PAYLOAD_MAX_BYTES];
    uint8_t payload_len;
} ble_data_pdu_t;

/** A general decoded BLE packet: either an advertising or a data PDU. */
typedef struct
{
    /** Physical-layer modulation/coding, propagated from the captured frame. */
    receiver_phy_t phy;
    uint8_t preamble;
    uint32_t access_address;
    uint32_t crc;
    uint8_t crc_ok;
    /** CRCInit used for this packet: the confirmed per-connection value for
     * data PDUs, BLE_CRC_INIT_ADV for advertising PDUs. */
    uint32_t crc_init;
    uint8_t is_adv_pdu;
    union
    {
        ble_adv_pdu_t adv;
        ble_data_pdu_t data;
    } pdu;
} ble_packet_t;

/** Parsed LLData of a CONNECT_IND PDU (connection parameters). */
typedef struct
{
    uint32_t access_address;
    uint32_t crc_init;   /* 24-bit */
    uint8_t window_size;
    uint16_t window_offset;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
    uint8_t channel_map[5];
    uint8_t hop_increment;
    uint8_t sleep_clock_accuracy;
} ble_connect_ind_params_t;

/** Parse the 22-byte LLData of a CONNECT_IND PDU. Returns 0 on success. */
int ble_connect_ind_parse(const uint8_t ll_data[BLE_CONNECT_IND_DATA_MAX_BYTES],
                          ble_connect_ind_params_t *out);

int ble_decode_frame(const ble_frame_t *frame,
                     uint8_t channel_index,
                     ble_packet_t *out);
int ble_verify_crc(const ble_packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CODEC_H */
