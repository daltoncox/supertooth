/**
 * @file rx_packet.h
 * @brief Generic RX packet wrapper for unified logging across PHY types.
 */

#ifndef RX_PACKET_H
#define RX_PACKET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    RX_PACKET_BREDR_INQUIRY = 0,
    RX_PACKET_BREDR_DATA    = 1,
    RX_PACKET_BLE_ADV       = 2,
    RX_PACKET_BLE_DATA      = 3
} rx_packet_type_t;

typedef struct
{
    /** Absolute sample index from the active hardware sample clock. */
    uint64_t sample_index;

    /** Packet family/type discriminator. */
    rx_packet_type_t type;

    /** RF center frequency in Hz. */
    uint32_t frequency_hz;

    /** Measured receive power in dBr (relative dB). */
    float rssi;

    /** Pointer to PHY-specific packet payload (cast by `type`). */
    void *phy_packet;
} rx_packet_t;

#ifdef __cplusplus
}
#endif

#endif /* RX_PACKET_H */
