/**
 * @file phy.h
 * @brief Shared PHY (physical layer) type used by both BLE and BR/EDR frames.
 *
 * The PHY identifies the modulation and coding scheme of a captured packet.
 * This is a property of the physical layer: it is set by the bitstream decoder
 * (which knows the demodulator configuration) and propagated through the
 * frame into the decoded packet.
 */

#ifndef PHY_H
#define PHY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Physical-layer modulation / coding scheme.
 *
 * Values cover the Bluetooth Core Spec PHYs for both BR/EDR and LE:
 *
 * BR/EDR:
 *  - BR      : 1 Mbps GFSK (Basic Rate)
 *  - EDR 2M  : 2 Mbps π/2-DQPSK (Enhanced Data Rate)
 *  - EDR 3M  : 3 Mbps 8DPSK   (Enhanced Data Rate)
 *
 * LE:
 *  - LE 1M        : 1 Mbps GFSK
 *  - LE 2M        : 2 Mbps LFFSK
 *  - LE Coded S=2 : 500 kbps (S=2 coding, 500 kbps data rate)
 *  - LE Coded S=8 : 125 kbps (S=8 coding, 125 kbps data rate)
 */
typedef enum
{
    RECEIVER_PHY_UNKNOWN = 0,

    /* BR/EDR */
    RECEIVER_PHY_BR,        /* 1 Mbps GFSK */
    RECEIVER_PHY_EDR_2M,    /* 2 Mbps π/2-DQPSK */
    RECEIVER_PHY_EDR_3M,    /* 3 Mbps 8DPSK */

    /* LE */
    RECEIVER_PHY_LE_1M,         /* 1 Mbps GFSK */
    RECEIVER_PHY_LE_2M,         /* 2 Mbps LFFSK */
    RECEIVER_PHY_LE_CODED_S2,   /* 500 kbps (S=2) */
    RECEIVER_PHY_LE_CODED_S8,   /* 125 kbps (S=8) */
} receiver_phy_t;

/**
 * @brief Return a short human-readable name for @p phy (e.g. "BR", "LE 1M").
 *
 * Returns "Unknown" for an unrecognized value.
 */
static inline const char *receiver_phy_name(receiver_phy_t phy)
{
    switch (phy)
    {
    case RECEIVER_PHY_BR:           return "BR";
    case RECEIVER_PHY_EDR_2M:       return "EDR 2M";
    case RECEIVER_PHY_EDR_3M:       return "EDR 3M";
    case RECEIVER_PHY_LE_1M:        return "LE 1M";
    case RECEIVER_PHY_LE_2M:        return "LE 2M";
    case RECEIVER_PHY_LE_CODED_S2:  return "LE Coded S=2";
    case RECEIVER_PHY_LE_CODED_S8:  return "LE Coded S=8";
    default:                        return "Unknown";
    }
}

/**
 * @brief Return the symbol rate in symbols/second for @p phy, or 0 if unknown.
 *
 * For coded PHYs this is the on-air symbol rate (not the data rate):
 *  - LE Coded S=2: 500 ksym/s (1 Msps × 1/2)
 *  - LE Coded S=8: 125 ksym/s (1 Msps × 1/8)
 */
static inline unsigned int receiver_phy_symbol_rate(receiver_phy_t phy)
{
    switch (phy)
    {
    case RECEIVER_PHY_BR:           return 1000000u;   /* 1 Msps */
    case RECEIVER_PHY_EDR_2M:       return 2000000u;   /* 2 Msps */
    case RECEIVER_PHY_EDR_3M:       return 3000000u;   /* 3 Msps */
    case RECEIVER_PHY_LE_1M:        return 1000000u;   /* 1 Msps */
    case RECEIVER_PHY_LE_2M:        return 2000000u;   /* 2 Msps */
    case RECEIVER_PHY_LE_CODED_S2:  return 500000u;    /* 500 ksym/s */
    case RECEIVER_PHY_LE_CODED_S8:  return 125000u;    /* 125 ksym/s */
    default:                        return 0u;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* PHY_H */
