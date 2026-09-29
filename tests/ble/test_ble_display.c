/* BLE display tests: PDU/LLID names, primary-address selection, formatting.
 *
 * The print functions write to stdout; these tests pin their naming
 * contracts plus NULL-safety (and smoke the printers so format-string
 * regressions crash here, not in the UI). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_codec.h"
#include "ble_display.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static void test_pdu_names(void)
{
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_ADV_IND), "ADV_IND") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_ADV_DIRECT_IND),
                       "ADV_DIRECT_IND") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_ADV_NONCONN_IND),
                       "ADV_NONCONN_IND") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_SCAN_REQ), "SCAN_REQ") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_SCAN_RSP), "SCAN_RSP") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_CONNECT_IND),
                       "CONNECT_IND") == 0);
    TEST_ASSERT(strcmp(ble_pdu_type_name(BLE_PDU_ADV_SCAN_IND),
                       "ADV_SCAN_IND") == 0);
    /* Reserved values still produce a non-NULL label. */
    TEST_ASSERT(ble_pdu_type_name(0x07u) != NULL);
    TEST_ASSERT(ble_pdu_type_name(0x0Fu) != NULL);
    /* Descriptions are non-NULL for every nibble. */
    for (unsigned int t = 0u; t < 16u; t++)
    {
        TEST_ASSERT(ble_pdu_type_desc((uint8_t)t) != NULL);
        TEST_ASSERT(ble_pdu_type_desc((uint8_t)t)[0] != '\0');
    }
    TEST_ASSERT(strstr(ble_pdu_type_desc(BLE_PDU_ADV_IND),
                       "Undirected") != NULL);
}

static void test_llid_names(void)
{
    TEST_ASSERT(ble_llid_name(0u) != NULL);
    TEST_ASSERT(ble_llid_name(1u) != NULL);
    TEST_ASSERT(ble_llid_name(2u) != NULL);
    TEST_ASSERT(ble_llid_name(3u) != NULL);
    /* Masked to 2 bits: 4 aliases 0. */
    TEST_ASSERT(ble_llid_name(4u) == ble_llid_name(0u));
    TEST_ASSERT(strcmp(ble_llid_name(2u), "START") == 0);
    TEST_ASSERT(strcmp(ble_llid_name(3u), "CTRL") == 0);
}

static void test_primary_addr(void)
{
    ble_adv_pdu_t adv;
    const ble_address_t *addr = NULL;

    TEST_ASSERT(ble_primary_addr(NULL, &addr) == 0);
    TEST_ASSERT(ble_primary_addr(&adv, NULL) == 0);

    /* ADV_IND exposes AdvA. */
    memset(&adv, 0, sizeof(adv));
    adv.pdu_type = BLE_PDU_ADV_IND;
    adv.payload.adv_ind.adv_addr.addr[0] = 0x42u;
    TEST_ASSERT(ble_primary_addr(&adv, &addr) == 1);
    TEST_ASSERT(addr == &adv.payload.adv_ind.adv_addr);
    TEST_ASSERT(addr->addr[0] == 0x42u);

    /* SCAN_REQ exposes the scanner address. */
    memset(&adv, 0, sizeof(adv));
    adv.pdu_type = BLE_PDU_SCAN_REQ;
    adv.payload.scan_req.scanner_addr.addr[5] = 0x99u;
    TEST_ASSERT(ble_primary_addr(&adv, &addr) == 1);
    TEST_ASSERT(addr == &adv.payload.scan_req.scanner_addr);

    /* CONNECT_IND exposes the initiator address. */
    memset(&adv, 0, sizeof(adv));
    adv.pdu_type = BLE_PDU_CONNECT_IND;
    TEST_ASSERT(ble_primary_addr(&adv, &addr) == 1);
    TEST_ASSERT(addr == &adv.payload.connect_ind.init_addr);

    /* Reserved type has no primary address. */
    memset(&adv, 0, sizeof(adv));
    adv.pdu_type = 0x07u;
    TEST_ASSERT(ble_primary_addr(&adv, &addr) == 0);
}

static void test_format_addr(void)
{
    const uint8_t a[BLE_ADDR_LEN] = {0x01u, 0x02u, 0x03u,
                                     0x04u, 0x05u, 0x06u};
    char out[18];
    ble_format_addr(out, a);
    TEST_ASSERT(strcmp(out, "06:05:04:03:02:01") == 0);
}

static void test_print_smoke(void)
{
    /* Printers must tolerate NULL and minimal packets without crashing. */
    ble_print_packet(NULL);
    ble_print_packet_summary_line(0u, NULL, NULL);

    ble_packet_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.is_adv_pdu = 1u;
    pkt.pdu.adv.pdu_type = BLE_PDU_ADV_IND;
    pkt.pdu.adv.payload_len = 6u;
    pkt.crc_ok = 1u;
    ble_print_packet(&pkt);

    rx_metadata_t meta;
    memset(&meta, 0, sizeof(meta));
    ble_print_packet_summary_line(7u, &pkt, &meta);

    memset(&pkt, 0, sizeof(pkt));
    pkt.is_adv_pdu = 0u;
    pkt.pdu.data.llid = 2u;
    pkt.pdu.data.payload_len = 3u;
    ble_print_packet(&pkt);
    ble_print_packet_summary_line(8u, &pkt, &meta);
}

int main(void)
{
    test_pdu_names();
    test_llid_names();
    test_primary_addr();
    test_format_addr();
    test_print_smoke();

    if (g_failures)
    {
        printf("test_ble_display: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ble_display: OK\n");
    return 0;
}
