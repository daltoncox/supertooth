/* BLE advertising-info tests: AD parsing, cross-packet service merging,
 * and assigned-numbers spot checks (refreshed 2026-09-15 from SIG YAML). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_codec.h"
#include "bt_assigned_numbers.h"
#include "device_models.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static void test_flags_tx_power(void)
{
    /* Flags=0x06 (GeneralDisc + BR/EDR-NotSupp), TxPower=-12 (0xF4). */
    const uint8_t adv[] = {0x02, 0x01, 0x06, 0x02, 0x0A, 0xF4};
    ble_adv_info_t info;
    ble_adv_parse_info(adv, sizeof(adv), &info);
    TEST_ASSERT(info.has_flags && info.flags == 0x06);
    TEST_ASSERT(info.has_tx_power && info.tx_power == -12);
    char flags[64];
    ble_adv_flags_format(info.flags, flags, sizeof(flags));
    TEST_ASSERT(strstr(flags, "GeneralDisc") != NULL);
    TEST_ASSERT(strstr(flags, "BR/EDR-NotSupp") != NULL);
}

static void test_service_lists_and_merge(void)
{
    /* Packet 1: incomplete 16-bit list {0x180D Heart Rate, 0x180F Battery},
     * solicitation {0x180A Device Information}. */
    const uint8_t p1[] = {
        0x05, 0x02, 0x0D, 0x18, 0x0F, 0x18,
        0x03, 0x14, 0x0A, 0x18,
    };
    /* Packet 2 (e.g. SCAN_RSP): complete list {0x180A}, service data 0x180D,
     * appearance Phone (0x0040), URI "hr". */
    const uint8_t p2[] = {
        0x03, 0x03, 0x0A, 0x18,
        0x04, 0x16, 0x0D, 0x18, 0x01,
        0x03, 0x19, 0x40, 0x00,
        0x03, 0x24, 'h', 'r',
    };
    ble_adv_info_t a, b;
    ble_adv_parse_info(p1, sizeof(p1), &a);
    ble_adv_parse_info(p2, sizeof(p2), &b);
    TEST_ASSERT(a.service_count == 3); /* 180D, 180F, 180A via solicitation */
    TEST_ASSERT(a.has_incomplete_list && a.has_solicitation);
    TEST_ASSERT(b.has_complete_list && b.has_service_data);
    TEST_ASSERT(b.has_appearance && b.appearance == 0x0040);
    TEST_ASSERT(strcmp(b.uri, "hr") == 0);

    ble_adv_info_merge(&a, &b);
    TEST_ASSERT(a.service_count == 3);
    TEST_ASSERT(a.has_complete_list && a.has_incomplete_list);
    TEST_ASSERT(a.has_appearance && a.has_service_data);

    char services[DEVICE_SERVICES_MAX + 64];
    ble_adv_info_format_services(&a, services, sizeof(services));
    TEST_ASSERT(strstr(services, "Heart Rate (0x180D)") != NULL);
    TEST_ASSERT(strstr(services, "Battery (0x180F)") != NULL);
    TEST_ASSERT(strstr(services, "Device Information (0x180A)") != NULL);

    /* Merging the same packet again must not duplicate. */
    ble_adv_info_merge(&a, &b);
    TEST_ASSERT(a.service_count == 3);
}

static void test_uuid128_base_alias(void)
{
    /* 128-bit Heart Rate in BT-base form, LE air order. */
    const uint8_t adv[] = {
        0x11, 0x07,
        0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80,
        0x00, 0x10, 0x00, 0x00, 0x0D, 0x18, 0x00, 0x00,
    };
    ble_adv_info_t info;
    ble_adv_parse_info(adv, sizeof(adv), &info);
    TEST_ASSERT(info.service_count == 1 && info.service_uuids[0] == 0x180D);
    TEST_ASSERT(info.uuid128_total == 0);
}

static void test_uuid128_custom(void)
{
    const uint8_t adv[] = {
        0x11, 0x07,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
    };
    ble_adv_info_t info;
    ble_adv_parse_info(adv, sizeof(adv), &info);
    TEST_ASSERT(info.service_count == 0);
    TEST_ASSERT(info.uuid128_total == 1 && info.uuid128_count == 1);
    char services[160];
    ble_adv_info_format_services(&info, services, sizeof(services));
    TEST_ASSERT(strstr(services, "128-bit") != NULL);
}

static void test_cod(void)
{
    /* CoD 0x200404: service Audio(21), major Audio/Video(4), minor Headset(6). */
    const uint8_t adv[] = {0x04, 0x0D, 0x04, 0x04, 0x20};
    ble_adv_info_t info;
    ble_adv_parse_info(adv, sizeof(adv), &info);
    TEST_ASSERT(info.has_cod && info.cod == 0x200404u);
    char cod[DEVICE_COD_MAX + 32];
    bt_cod_format(info.cod, cod, sizeof(cod));
    TEST_ASSERT(strstr(cod, "Audio/Video") != NULL);
    TEST_ASSERT(strstr(cod, "Headphones") != NULL || strstr(cod, "Headset") != NULL);
}

static void test_assigned_numbers(void)
{
    TEST_ASSERT(strcmp(bt_assigned_company_name(0x004C), "Apple, Inc.") == 0);
    TEST_ASSERT(strcmp(bt_assigned_company_name(0x1121), "Unknown") != 0);
    TEST_ASSERT(strstr(bt_assigned_ad_type_name(0x03), "16-bit") != NULL);
    TEST_ASSERT(strcmp(bt_assigned_service_uuid_name(0x180D), "Heart Rate") == 0);
    TEST_ASSERT(strcmp(bt_assigned_service_uuid_name(0x1108), "Headset") == 0);
    TEST_ASSERT(strcmp(bt_assigned_appearance_category(0x0040), "Phone") == 0);
    char app[48];
    bt_assigned_appearance_format(0x00C2, app, sizeof(app));
    TEST_ASSERT(strstr(app, "Watch") != NULL);
    TEST_ASSERT(strstr(bt_cod_major_device_name(4), "Audio/Video") != NULL);
}

int main(void)
{
    test_flags_tx_power();
    test_service_lists_and_merge();
    test_uuid128_base_alias();
    test_uuid128_custom();
    test_cod();
    test_assigned_numbers();
    if (g_failures == 0)
        printf("PASS test_ble_adv_info\n");
    return g_failures ? 1 : 0;
}
