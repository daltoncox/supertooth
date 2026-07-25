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

#define BLE_ADDR_LEN 6u
#define BLE_ADV_DATA_MAX_BYTES (BLE_PDU_MAX_BYTES - 2u - BLE_ADDR_LEN)
#define BLE_CONNECT_IND_DATA_MAX_BYTES 22u
#define BLE_RESERVED_PAYLOAD_MAX_BYTES (BLE_PDU_MAX_BYTES - 2u)

typedef enum
{
    BLE_ADDR_PUBLIC = 0,
    BLE_ADDR_RANDOM = 1,
} ble_addr_kind_t;

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
} ble_adv_data_packet_t;

typedef struct
{
    ble_address_t adv_addr;
    ble_address_t target_addr;
} ble_adv_direct_packet_t;

typedef struct
{
    ble_address_t scanner_addr;
    ble_address_t adv_addr;
} ble_scan_req_packet_t;

typedef struct
{
    ble_address_t init_addr;
    ble_address_t adv_addr;
    uint8_t ll_data[BLE_CONNECT_IND_DATA_MAX_BYTES];
    uint8_t ll_data_len;
} ble_connect_ind_packet_t;

typedef struct
{
    uint8_t payload[BLE_RESERVED_PAYLOAD_MAX_BYTES];
    uint8_t payload_len;
} ble_unknown_packet_t;

typedef struct
{
    /** Physical-layer modulation/coding, propagated from the captured frame. */
    receiver_phy_t phy;
    uint8_t preamble;
    uint32_t access_address;
    uint8_t pdu_type;
    uint8_t payload_len;
    ble_addr_kind_t tx_addr_kind;
    ble_addr_kind_t rx_addr_kind;
    uint32_t crc;
    uint8_t crc_ok;
    union
    {
        ble_adv_data_packet_t adv_ind;
        ble_adv_direct_packet_t adv_direct_ind;
        ble_adv_data_packet_t adv_nonconn_ind;
        ble_scan_req_packet_t scan_req;
        ble_adv_data_packet_t scan_rsp;
        ble_connect_ind_packet_t connect_ind;
        ble_adv_data_packet_t adv_scan_ind;
        ble_unknown_packet_t unknown;
    } payload;
} ble_packet_t;

int ble_decode_frame(const ble_frame_t *frame,
                     uint8_t channel_index,
                     ble_packet_t *out);
int ble_verify_crc(const ble_packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CODEC_H */
