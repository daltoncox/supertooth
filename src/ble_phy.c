/**
 * @file ble_phy.c
 * @brief BLE PHY-layer bitstream processor implementation.
 *
 * See ble_phy.h for the public API and design notes.
 */

#include "ble_phy.h"

#include <string.h>   /* memset, memcpy */
#include <stddef.h>   /* NULL */
#include <stdio.h>    /* printf, putchar */
#include <ctype.h>    /* isprint */
#include <inttypes.h> /* PRIX32 */

/* ---------------------------------------------------------------------------
 * Internal constants
 * ---------------------------------------------------------------------------*/

/**
 * Maximum total bits to collect as a safety ceiling:
 * 2-byte header + 255-byte payload + 3-byte CRC = 260 bytes = 2080 bits.
 */
#define MAX_BITS_TO_COLLECT ((BLE_PDU_MAX_BYTES + BLE_CRC_BYTES) * 8u)

/**
 * Width of the sliding detection window in bits.
 * 8-bit preamble + 32-bit access address = 40 bits.
 */
#define DETECTION_WINDOW_BITS 40u

/**
 * Valid preamble + advertising access address patterns.
 *
 * The BLE preamble alternates between 0 and 1, starting opposite to the
 * LSB of the access address.  The advertising AA is 0x8E89BED6 whose LSB
 * is 0, so the preamble must start with 1 → 0x55.  Its bitwise complement
 * 0xAA is also valid (see Core Spec Vol 6, Part B, §2.1.1).
 *
 * The window holds bits oldest→newest as bit0→bit39, so the preamble
 * occupies bits [7:0] and the AA bits [39:8].
 */
#define PATTERN_AA_SHIFT 8u
#define PATTERN_PREAMBLE1 0x55ULL
#define PATTERN_PREAMBLE2 0xAAULL
#define PATTERN1 (PATTERN_PREAMBLE1 | ((uint64_t)BLE_ADVERTISING_AA << PATTERN_AA_SHIFT))
#define PATTERN2 (PATTERN_PREAMBLE2 | ((uint64_t)BLE_ADVERTISING_AA << PATTERN_AA_SHIFT))

/** Mask for the 40-bit sliding window (prevents stale high bits). */
#define WINDOW_MASK ((1ULL << DETECTION_WINDOW_BITS) - 1ULL)

/* ---------------------------------------------------------------------------
 * Internal helpers (not exposed in the header)
 * ---------------------------------------------------------------------------*/

/**
 * @brief Apply BLE data dewhitening in-place.
 *
 * Implements the 7-bit LFSR described in Bluetooth Core Spec
 * Vol 6, Part B, §3.2.  Polynomial: x^7 + x^4 + 1.
 * Initial state: bit6 = 1, bits[5:0] = channel_index[5:0].
 *
 * @param data          Buffer to dewhiten in-place.
 * @param length_bytes  Number of bytes to process.
 * @param channel_index BLE channel index used to seed the LFSR.
 */
static void dewhiten(uint8_t *data, unsigned int length_bytes, uint8_t channel_index)
{
    /* Seed: bit 6 forced to 1, lower 6 bits from channel index. */
    uint8_t lfsr = (channel_index & 0x3Fu) | 0x40u;

    for (unsigned int byte_idx = 0; byte_idx < length_bytes; byte_idx++)
    {
        for (unsigned int bit_idx = 0; bit_idx < 8u; bit_idx++)
        {
            /* Output bit is the LFSR LSB. */
            uint8_t out_bit = lfsr & 0x01u;

            /* XOR the output bit into the corresponding data bit. */
            data[byte_idx] ^= (out_bit << bit_idx);

            /* Advance the LFSR:
             *   1. XOR position 3 with the output bit (tap at x^4).
             *   2. Right-shift by 1 (advance).
             *   3. Feed output bit back into position 6 (tap at x^7). */
            lfsr ^= (out_bit << 3u);
            lfsr >>= 1u;
            lfsr |= (out_bit << 6u);
        }
    }
}

/**
 * @brief Reverse all 8 bits of a byte.
 *
 * Used when packing the received CRC bytes into pkt->crc.  The CRC register
 * is transmitted MSB-first (Core Spec Vol 6, Part B, §3.1.1), so register
 * bit 23 arrives at bit 0 of the first received byte, bit 22 at bit 1, etc.
 * Bit-reversing each received byte (and reordering them highest-first) maps
 * the on-air representation back to the natural CRC register value.
 */
static uint8_t bit_reverse_byte(uint8_t b)
{
    b = (b & 0xF0u) >> 4u | (b & 0x0Fu) << 4u;
    b = (b & 0xCCu) >> 2u | (b & 0x33u) << 2u;
    b = (b & 0xAAu) >> 1u | (b & 0x55u) << 1u;
    return b;
}

/**
 * @brief Compute the BLE 24-bit CRC register value over a data buffer.
 *
 * Implements the 24-stage left-shift LFSR from Bluetooth Core Spec
 * Vol 6, Part B, §3.1.1.  Polynomial: x^24 + x^10 + x^9 + x^6 + x^4 + x^3
 * + x + 1 (non-leading mask 0x65B).  Bits are processed LSB-first within
 * each byte, matching BLE over-the-air ordering.
 *
 * @param data  Buffer to compute CRC over.
 * @param len   Number of bytes to process.
 * @param init  Initial LFSR value (use BLE_CRC_INIT_ADV for advertising).
 * @return      24-bit CRC register value (upper 8 bits are always zero).
 */
static uint32_t crc_calc(const uint8_t *data, unsigned int len, uint32_t init)
{
    uint32_t crc = init & 0xFFFFFFu;

    for (unsigned int i = 0; i < len; i++)
    {
        for (unsigned int bit = 0; bit < 8u; bit++)
        {
            uint8_t d = (data[i] >> bit) & 1u;               /* LSB first */
            uint8_t fb = (uint8_t)(((crc >> 23u) ^ d) & 1u); /* feedback  */
            crc = (crc << 1u) & 0xFFFFFFu;
            if (fb)
                crc ^= 0x65Bu; /* taps: x^10+x^9+x^6+x^4+x^3+x+1 */
        }
    }

    return crc;
}

/**
 * @brief Check whether the current sliding window matches a valid BLE
 *        preamble + advertising access address pattern.
 *
 * @param window  The current 40-bit window value.
 * @return Non-zero if a match is found, zero otherwise.
 */
static int window_matches(uint64_t window)
{
    return (window == PATTERN1) || (window == PATTERN2);
}

/**
 * @brief Reset collection state, ready for the next packet.
 *
 * Does NOT reset channel_index or the sliding bit_window.
 *
 * @param proc  Processor to reset.
 */
static void reset_collection(ble_channel_processor_t *proc)
{
    proc->collecting = 0;
    proc->bits_collected = 0;
    proc->header_decoded = 0;
    proc->bits_to_collect = 0;
    memset(proc->raw_pdu, 0, sizeof(proc->raw_pdu));
}

/* ---------------------------------------------------------------------------
 * Public API implementation
 * ---------------------------------------------------------------------------*/

void ble_processor_init(ble_channel_processor_t *proc, uint8_t channel_index)
{
    if (!proc)
        return;

    memset(proc, 0, sizeof(*proc));
    proc->channel_index = channel_index;
}

ble_status_t ble_push_bit(ble_channel_processor_t *proc, uint8_t bit)
{
    if (!proc)
        return BLE_ERROR;

    /* Normalise: treat any non-zero value as 1. */
    uint8_t b = bit ? 1u : 0u;

    if (proc->collecting)
    {
        /* ------------------------------------------------------------------ *
         * Packet collection state: pack incoming bits into raw_pdu, LSB first
         * within each byte (BLE over-the-air bit ordering).
         * ------------------------------------------------------------------ */
        unsigned int byte_idx = proc->bits_collected / 8u;
        unsigned int bit_idx = proc->bits_collected % 8u;

        /* Guard against buffer overrun. */
        if (byte_idx >= (BLE_PDU_MAX_BYTES + BLE_CRC_BYTES))
        {
            reset_collection(proc);
            return BLE_ERROR;
        }

        if (b)
            proc->raw_pdu[byte_idx] |= (uint8_t)(1u << bit_idx);

        proc->bits_collected++;

        /* ------------------------------------------------------------------ *
         * After the 2-byte PDU header is collected, peek at the dewhitened
         * length field (header[1]) to set the exact collection target:
         *   total = (2 header + payload_len + 3 CRC) bytes.
         * ------------------------------------------------------------------ */
        if (!proc->header_decoded && proc->bits_collected >= 16u)
        {
            uint8_t header[2];
            memcpy(header, proc->raw_pdu, 2u);
            dewhiten(header, 2u, proc->channel_index);

            unsigned int payload_len = header[1];
            /* Cap at the largest payload the buffer can hold. */
            if (payload_len > (BLE_PDU_MAX_BYTES - 2u))
                payload_len = BLE_PDU_MAX_BYTES - 2u;

            proc->bits_to_collect = (2u + payload_len + BLE_CRC_BYTES) * 8u;
            proc->header_decoded = 1;
        }

        /* Use the dynamically determined target, or the safety ceiling if the
         * header has not been decoded yet (should not normally occur). */
        unsigned int target = proc->header_decoded
                                  ? proc->bits_to_collect
                                  : MAX_BITS_TO_COLLECT;

        if (proc->bits_collected >= target)
        {
            /* ---------------------------------------------------------------- *
             * All bits collected — dewhiten PDU+CRC and publish the packet.
             * ---------------------------------------------------------------- */
            unsigned int total_bytes = target / 8u;
            unsigned int pdu_bytes = total_bytes - BLE_CRC_BYTES;

            uint8_t dewhitened[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
            memcpy(dewhitened, proc->raw_pdu, total_bytes);
            dewhiten(dewhitened, total_bytes, proc->channel_index);

            /* Populate the packet structure. */
            ble_packet_t *pkt = &proc->last_packet;
            memset(pkt, 0, sizeof(*pkt));

            pkt->preamble = (uint8_t)PATTERN_PREAMBLE1;
            pkt->access_address = BLE_ADVERTISING_AA;
            memcpy(pkt->pdu, dewhitened, pdu_bytes);

            /*
             * Extract the 24-bit CRC.  The CRC register is transmitted
             * MSB-first (bit 23 → bit 0), but bits are packed LSB-first
             * into bytes, so register bit 23 lands at bit 0 of the first
             * received byte.  Bit-reverse each received byte and reassemble
             * with the highest-order byte first to recover the natural
             * CRC register value directly comparable to crc_calc() output.
             */
            pkt->crc = ((uint32_t)bit_reverse_byte(dewhitened[pdu_bytes]) << 16u) | ((uint32_t)bit_reverse_byte(dewhitened[pdu_bytes + 1u]) << 8u) | (uint32_t)bit_reverse_byte(dewhitened[pdu_bytes + 2u]);

            proc->packet_ready = 1;

            reset_collection(proc);
            return BLE_VALID_PACKET;
        }

        return BLE_COLLECTING;
    }
    else
    {
        /* ------------------------------------------------------------------ *
         * Searching state: maintain a 40-bit sliding window and check for a
         * preamble + access address match on every new bit.
         *
         * Shift oldest bit out of bit 0, push new bit into bit 39.
         * ------------------------------------------------------------------ */
        proc->bit_window = ((proc->bit_window >> 1u) | ((uint64_t)b << 39u)) & WINDOW_MASK;

        if (window_matches(proc->bit_window))
        {
            /* Pattern found — start collecting the PDU. */
            proc->collecting = 1;
            proc->bits_collected = 0;
            proc->bit_window = 0;
            memset(proc->raw_pdu, 0, sizeof(proc->raw_pdu));
            proc->packet_ready = 0;
            return BLE_COLLECTING;
        }

        return BLE_SEARCHING;
    }
}

int ble_get_packet(ble_channel_processor_t *proc, ble_packet_t *out)
{
    if (!proc || !out || !proc->packet_ready)
        return -1;

    memcpy(out, &proc->last_packet, sizeof(*out));
    proc->packet_ready = 0;
    return 0;
}

int ble_verify_crc(const ble_packet_t *pkt)
{
    if (!pkt)
        return 0;

    unsigned int payload_len = pkt->pdu[1];
    if (payload_len > (BLE_PDU_MAX_BYTES - 2u))
        payload_len = BLE_PDU_MAX_BYTES - 2u;

    uint32_t computed = crc_calc(pkt->pdu, 2u + payload_len, BLE_CRC_INIT_ADV);
    return (computed == (pkt->crc & 0xFFFFFFu)) ? 1 : 0;
}

/* ---------------------------------------------------------------------------
 * ble_print_packet — human-readable advertising packet decoder
 * ---------------------------------------------------------------------------*/

/* Advertising PDU type codes (Core Spec Vol 6, Part B, §2.3.1). */
#define PDU_ADV_IND 0x00u
#define PDU_ADV_DIRECT_IND 0x01u
#define PDU_ADV_NONCONN_IND 0x02u
#define PDU_SCAN_REQ 0x03u
#define PDU_SCAN_RSP 0x04u
#define PDU_CONNECT_IND 0x05u
#define PDU_ADV_SCAN_IND 0x06u

/* PDU header byte 0 field masks. */
#define PDU_HDR_TYPE_MASK 0x0Fu
#define PDU_HDR_TXADD (1u << 6)
#define PDU_HDR_RXADD (1u << 7)

/* AD type for Manufacturer Specific Data (Bluetooth Assigned Numbers §2.3). */
#define AD_TYPE_MANUF_SPEC 0xFFu

/* Random address subtype encoded in bits [47:46] (top 2 bits of MSB octet). */
#define RAND_ADDR_STATIC 0x03u /* 11b = Random Static                  */
#define RAND_ADDR_RPA 0x02u    /* 10b = Random Private Resolvable      */
#define RAND_ADDR_NRPA 0x00u   /* 00b = Random Private Non-Resolvable  */

/**
 * Subset of well-known company identifiers from Bluetooth Assigned Numbers §7.
 * Used for Manufacturer Specific Data (AD type 0xFF) lookup.
 */
static const struct
{
    uint16_t id;
    const char *name;
} s_company_ids[] = {
    {0x0000, "Ericsson Technology Licensing"},
    {0x0006, "Microsoft Corporation"},
    {0x004C, "Apple, Inc."},
    {0x0059, "Nordic Semiconductor ASA"},
    {0x0075, "Samsung Electronics Co. Ltd."},
    {0x0087, "Garmin International, Inc."},
    {0x00E0, "Google"},
    {0x0157, "Cypress Semiconductor"},
    {0x02E5, "Espressif Incorporated"},
    {0x0499, "Ruuvi Innovations Ltd."},
};
#define NUM_COMPANY_IDS (sizeof(s_company_ids) / sizeof(s_company_ids[0]))

static const char *lookup_company_id(uint16_t id)
{
    for (unsigned int i = 0; i < NUM_COMPANY_IDS; i++)
        if (s_company_ids[i].id == id)
            return s_company_ids[i].name;
    return NULL;
}

/**
 * Print a 6-byte BLE device address (little-endian, addr[0] = LSB) in
 * standard colon-separated notation with its address type in parentheses.
 */
static void print_ble_addr(const uint8_t *addr, int is_random)
{
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    if (is_random)
    {
        /* Subtype is encoded in the top 2 bits of the most significant octet. */
        uint8_t subtype = (addr[5] >> 6) & 0x03u;
        const char *desc;
        switch (subtype)
        {
        case RAND_ADDR_STATIC:
            desc = "Random Static";
            break;
        case RAND_ADDR_RPA:
            desc = "Random Private Resolvable";
            break;
        case RAND_ADDR_NRPA:
            desc = "Random Private Non-Resolvable";
            break;
        default:
            desc = "Random (Reserved)";
            break;
        }
        printf(" (%s)", desc);
    }
    else
    {
        printf(" (Public)");
    }
}

/**
 * Parse and print AdvData AD structures: raw hex, ASCII attempt, and any
 * Manufacturer Specific Data (AD type 0xFF) with company name lookup.
 */
static void print_adv_data(const uint8_t *data, unsigned int len)
{
    if (len == 0)
    {
        printf("AdvData  : (none)\n");
        return;
    }

    /* Raw hex dump. */
    printf("AdvData  :");
    for (unsigned int i = 0; i < len; i++)
        printf(" %02X", data[i]);
    printf("\n");

    /* Best-effort ASCII display — non-printable bytes shown as '.'. */
    printf("           ASCII: \"");
    for (unsigned int i = 0; i < len; i++)
        putchar(isprint((unsigned char)data[i]) ? data[i] : '.');
    printf("\"\n");

    /* Walk AD structures (length + type + value) looking for type 0xFF. */
    unsigned int i = 0;
    while (i < len)
    {
        uint8_t ad_len = data[i];
        if (ad_len == 0 || (i + 1u + ad_len) > len)
            break;

        uint8_t ad_type = data[i + 1u];
        if (ad_type == AD_TYPE_MANUF_SPEC && ad_len >= 3u)
        {
            /* Company ID is 2 bytes, little-endian. */
            uint16_t cid = (uint16_t)data[i + 2u] | ((uint16_t)data[i + 3u] << 8u);
            const char *name = lookup_company_id(cid);
            printf("Manuf    : %s (0x%04X)", name ? name : "Unknown", cid);

            /* Remaining manufacturer-specific payload bytes. */
            unsigned int val_end = i + 1u + ad_len;
            if (val_end > i + 4u)
            {
                printf(" | Data:");
                for (unsigned int j = i + 4u; j < val_end; j++)
                    printf(" %02X", data[j]);
            }
            printf("\n");
        }

        i += 1u + ad_len;
    }
}

void ble_print_packet(const ble_packet_t *pkt)
{
    if (!pkt)
        return;

    uint8_t hdr0 = pkt->pdu[0];
    uint8_t pdu_type = hdr0 & PDU_HDR_TYPE_MASK;
    int tx_add = (hdr0 & PDU_HDR_TXADD) ? 1 : 0;
    int rx_add = (hdr0 & PDU_HDR_RXADD) ? 1 : 0;
    uint8_t pay_len = pkt->pdu[1];

    /* PDU type name and plain-English description. */
    const char *type_name;
    const char *type_desc;
    switch (pdu_type)
    {
    case PDU_ADV_IND:
        type_name = "ADV_IND";
        type_desc = "Connectable Undirected Advertising";
        break;
    case PDU_ADV_DIRECT_IND:
        type_name = "ADV_DIRECT_IND";
        type_desc = "Connectable Directed Advertising";
        break;
    case PDU_ADV_NONCONN_IND:
        type_name = "ADV_NONCONN_IND";
        type_desc = "Non-Connectable Undirected Advertising";
        break;
    case PDU_SCAN_REQ:
        type_name = "SCAN_REQ";
        type_desc = "Scan Request";
        break;
    case PDU_SCAN_RSP:
        type_name = "SCAN_RSP";
        type_desc = "Scan Response";
        break;
    case PDU_CONNECT_IND:
        type_name = "CONNECT_IND";
        type_desc = "Connect Request";
        break;
    case PDU_ADV_SCAN_IND:
        type_name = "ADV_SCAN_IND";
        type_desc = "Scannable Undirected Advertising";
        break;
    default:
        type_name = "RESERVED";
        type_desc = "Reserved/Unknown";
        break;
    }

    printf("--- BLE Advertising Packet ---\n");
    printf("PDU Type : %s (%s)\n", type_name, type_desc);

    switch (pdu_type)
    {
    case PDU_ADV_IND:
    case PDU_ADV_NONCONN_IND:
    case PDU_ADV_SCAN_IND:
    case PDU_SCAN_RSP:
    {
        /* Payload: AdvA (6 B) + AdvData (pay_len - 6 B) */
        if (pay_len < 6u)
        {
            printf("AdvA     : (payload too short)\n");
            break;
        }
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        print_adv_data(&pkt->pdu[8], pay_len > 6u ? pay_len - 6u : 0u);
        break;
    }

    case PDU_ADV_DIRECT_IND:
    {
        /* Payload: AdvA (6 B, TxAdd) + TargetA (6 B, RxAdd) */
        if (pay_len < 12u)
        {
            printf("AdvA     : (payload too short)\n");
            break;
        }
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("TargetA  : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    case PDU_SCAN_REQ:
    {
        /* Payload: ScanA (6 B, TxAdd) + AdvA (6 B, RxAdd) */
        if (pay_len < 12u)
        {
            printf("ScanA    : (payload too short)\n");
            break;
        }
        printf("ScanA    : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    case PDU_CONNECT_IND:
    {
        /* Payload: InitA (6 B, TxAdd) + AdvA (6 B, RxAdd) + LLData (22 B) */
        if (pay_len < 12u)
        {
            printf("InitA    : (payload too short)\n");
            break;
        }
        printf("InitA    : ");
        print_ble_addr(&pkt->pdu[2], tx_add);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->pdu[8], rx_add);
        printf("\n");
        break;
    }

    default:
    {
        /* Reserved/unknown — dump raw payload bytes. */
        printf("Payload  :");
        for (unsigned int i = 0; i < pay_len && (2u + i) < BLE_PDU_MAX_BYTES; i++)
            printf(" %02X", pkt->pdu[2u + i]);
        printf("\n");
        break;
    }
    }

    printf("CRC      : 0x%06" PRIX32 " [%s]\n",
           pkt->crc, ble_verify_crc(pkt) ? "PASS" : "FAIL");
}