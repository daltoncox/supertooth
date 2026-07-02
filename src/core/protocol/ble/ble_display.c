#include "ble_display.h"

#include "bt_assigned_numbers.h"

#include <inttypes.h>
#include <stdio.h>

#define AD_TYPE_MANUF_SPEC 0xFFu
#define RAND_ADDR_STATIC 0x03u
#define RAND_ADDR_RPA 0x02u
#define RAND_ADDR_NRPA 0x00u

const char *ble_pdu_type_desc(uint8_t pdu_type)
{
    switch (pdu_type & 0x0Fu)
    {
    case BLE_PDU_ADV_IND:
        return "Connectable Undirected Advertising";
    case BLE_PDU_ADV_DIRECT_IND:
        return "Connectable Directed Advertising";
    case BLE_PDU_ADV_NONCONN_IND:
        return "Non-Connectable Undirected Advertising";
    case BLE_PDU_SCAN_REQ:
        return "Scan Request";
    case BLE_PDU_SCAN_RSP:
        return "Scan Response";
    case BLE_PDU_CONNECT_IND:
        return "Connect Request";
    case BLE_PDU_ADV_SCAN_IND:
        return "Scannable Undirected Advertising";
    default:
        return "Reserved/Unknown";
    }
}

static void print_ble_addr(const ble_address_t *addr)
{
    if (!addr)
        return;

    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           addr->addr[5], addr->addr[4], addr->addr[3],
           addr->addr[2], addr->addr[1], addr->addr[0]);

    if (addr->kind == BLE_ADDR_RANDOM)
    {
        uint8_t subtype = (addr->addr[5] >> 6) & 0x03u;
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
        return;
    }

    printf(" (Public)");
}

static void print_adv_data(const uint8_t *data, unsigned int len)
{
    if (len == 0u)
    {
        printf("AdvData  : (none)\n");
        return;
    }

    printf("AdvData  :");
    for (unsigned int i = 0; i < len; i++)
        printf(" %02X", data[i]);
    printf("\n");

    unsigned int i = 0u;
    while (i < len)
    {
        uint8_t ad_len = data[i];
        if (ad_len == 0u || (i + 1u + ad_len) > len)
            break;

        uint8_t ad_type = data[i + 1u];
        unsigned int val_begin = i + 2u;
        unsigned int val_end = i + 1u + ad_len;

        printf("├─ Type  : 0x%02X (%s)\n", ad_type, bt_assigned_ad_type_name(ad_type));
        printf("├─ Data  :");
        if (val_end > val_begin)
        {
            for (unsigned int j = val_begin; j < val_end; j++)
                printf(" %02X", data[j]);
        }
        else
        {
            printf(" (none)");
        }
        printf("\n");

        if (ad_type == AD_TYPE_MANUF_SPEC && ad_len >= 3u)
        {
            uint16_t cid = (uint16_t)data[i + 2u] | ((uint16_t)data[i + 3u] << 8u);
            printf("Manuf    : %s (0x%04X)\n", bt_assigned_company_name(cid), cid);
        }

        i += 1u + ad_len;
    }
}

const char *ble_pdu_type_name(uint8_t pdu_type)
{
    switch (pdu_type & 0x0Fu)
    {
    case BLE_PDU_ADV_IND:
        return "ADV_IND";
    case BLE_PDU_ADV_DIRECT_IND:
        return "ADV_DIRECT_IND";
    case BLE_PDU_ADV_NONCONN_IND:
        return "ADV_NONCONN_IND";
    case BLE_PDU_SCAN_REQ:
        return "SCAN_REQ";
    case BLE_PDU_SCAN_RSP:
        return "SCAN_RSP";
    case BLE_PDU_CONNECT_IND:
        return "CONNECT_IND";
    case BLE_PDU_ADV_SCAN_IND:
        return "ADV_SCAN_IND";
    default:
        return "RESERVED";
    }
}

int ble_primary_addr(const ble_packet_t *pkt, const ble_address_t **addr_out)
{
    if (!pkt || !addr_out)
        return 0;

    switch (pkt->pdu_type & 0x0Fu)
    {
    case BLE_PDU_ADV_IND:
        *addr_out = &pkt->payload.adv_ind.adv_addr;
        return 1;
    case BLE_PDU_ADV_DIRECT_IND:
        *addr_out = &pkt->payload.adv_direct_ind.adv_addr;
        return 1;
    case BLE_PDU_ADV_NONCONN_IND:
        *addr_out = &pkt->payload.adv_nonconn_ind.adv_addr;
        return 1;
    case BLE_PDU_SCAN_REQ:
        *addr_out = &pkt->payload.scan_req.scanner_addr;
        return 1;
    case BLE_PDU_SCAN_RSP:
        *addr_out = &pkt->payload.scan_rsp.adv_addr;
        return 1;
    case BLE_PDU_CONNECT_IND:
        *addr_out = &pkt->payload.connect_ind.init_addr;
        return 1;
    case BLE_PDU_ADV_SCAN_IND:
        *addr_out = &pkt->payload.adv_scan_ind.adv_addr;
        return 1;
    default:
        return 0;
    }
}

void ble_format_addr(char out[18], const uint8_t addr[BLE_ADDR_LEN])
{
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

void ble_print_packet(const ble_packet_t *pkt)
{
    if (!pkt)
        return;

    printf("[BLE Advertising Packet]\n");
    printf("PDU Type : %u [%s] (%s)\n",
           (unsigned int)(pkt->pdu_type & 0x0Fu),
           ble_pdu_type_name(pkt->pdu_type),
           ble_pdu_type_desc(pkt->pdu_type));

    switch (pkt->pdu_type & 0x0Fu)
    {
    case BLE_PDU_ADV_IND:
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.adv_ind.adv_addr);
        printf("\n");
        print_adv_data(pkt->payload.adv_ind.adv_data, pkt->payload.adv_ind.adv_data_len);
        break;
    case BLE_PDU_ADV_DIRECT_IND:
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.adv_direct_ind.adv_addr);
        printf("\n");
        printf("TargetA  : ");
        print_ble_addr(&pkt->payload.adv_direct_ind.target_addr);
        printf("\n");
        break;
    case BLE_PDU_ADV_NONCONN_IND:
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.adv_nonconn_ind.adv_addr);
        printf("\n");
        print_adv_data(pkt->payload.adv_nonconn_ind.adv_data,
                       pkt->payload.adv_nonconn_ind.adv_data_len);
        break;
    case BLE_PDU_SCAN_REQ:
        printf("ScanA    : ");
        print_ble_addr(&pkt->payload.scan_req.scanner_addr);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.scan_req.adv_addr);
        printf("\n");
        break;
    case BLE_PDU_SCAN_RSP:
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.scan_rsp.adv_addr);
        printf("\n");
        print_adv_data(pkt->payload.scan_rsp.adv_data, pkt->payload.scan_rsp.adv_data_len);
        break;
    case BLE_PDU_CONNECT_IND:
        printf("InitA    : ");
        print_ble_addr(&pkt->payload.connect_ind.init_addr);
        printf("\n");
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.connect_ind.adv_addr);
        printf("\n");
        break;
    case BLE_PDU_ADV_SCAN_IND:
        printf("AdvA     : ");
        print_ble_addr(&pkt->payload.adv_scan_ind.adv_addr);
        printf("\n");
        print_adv_data(pkt->payload.adv_scan_ind.adv_data,
                       pkt->payload.adv_scan_ind.adv_data_len);
        break;
    default:
        printf("Payload  :");
        for (unsigned int i = 0; i < pkt->payload.unknown.payload_len; i++)
            printf(" %02X", pkt->payload.unknown.payload[i]);
        printf("\n");
        break;
    }

    printf("CRC      : 0x%06" PRIX32 " [%s]\n",
           pkt->crc,
           ble_verify_crc(pkt) ? "PASS" : "FAIL");
}

void ble_print_packet_summary_line(unsigned long packet_no,
                                   const ble_packet_t *pkt,
                                   const rx_metadata_t *meta)
{
    if (!pkt)
        return;
    const char *pdu_name = ble_pdu_type_name(pkt->pdu_type);
    const ble_address_t *addr = NULL;
    char addr_buf[18];

    if (ble_primary_addr(pkt, &addr))
        ble_format_addr(addr_buf, addr->addr);
    else
        snprintf(addr_buf, sizeof(addr_buf), "--");

    printf("pkt=%-6lu type=BLE pdu=%-14s ch=%02u addr=%s len=%-3u crc=%s rssi=%.1f\n",
           packet_no,
           pdu_name,
           meta->channel_index,
           addr_buf,
           pkt->payload_len,
           ble_verify_crc(pkt) ? "PASS" : "FAIL",
           meta->rssi_dbr);
}
