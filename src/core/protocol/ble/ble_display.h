#ifndef BLE_DISPLAY_H
#define BLE_DISPLAY_H

#include <stdint.h>

#include "ble_codec.h"
#include "receive_event_models.h"

const char *ble_pdu_type_name(uint8_t pdu_type);
const char *ble_pdu_type_desc(uint8_t pdu_type);
const char *ble_llid_name(uint8_t llid);
int ble_primary_addr(const ble_adv_pdu_t *adv, const ble_address_t **addr_out);
void ble_format_addr(char out[18], const uint8_t addr[BLE_ADDR_LEN]);
void ble_print_packet(const ble_packet_t *pkt);
void ble_print_packet_summary_line(unsigned long packet_no,
                                   const ble_packet_t *pkt,
                                   const rx_metadata_t *meta);

#endif
