/* BLE registry tests: ingest gating, CRCInit recovery, eviction, devices.
 *
 * The bitstream-decoder suite already covers the decoder->registry path over
 * the air; these tests drive ble_registry_submit() directly with framed
 * events to pin the registry's own contracts: promote-on-promise,
 * candidate FIFO, confirm/prove, enforce_crc, LRU eviction, pending cap,
 * standalone devices, and the getters. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_codec.h"
#include "ble_registry.h"
#include "device_models.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define CH 4u
#define AA_X 0x9E3C5A77u
#define INIT_X 0x2C4A6Eu

/* Build a frame as the framer would: whitened PDU+CRC in raw_pdu. */
static void make_frame(ble_frame_t *frame, ble_frame_kind_t kind, uint32_t aa,
                       const uint8_t *pdu, unsigned int pdu_bytes,
                       uint8_t ch, uint32_t crc_init)
{
    memset(frame, 0, sizeof(*frame));
    frame->phy = RECEIVER_PHY_LE_1M;
    frame->kind = kind;
    frame->preamble = (aa & 1u) ? 0x55u : 0xAAu;
    frame->access_address = aa;
    frame->crc_init = 0u;
    frame->crc_ok = 0u;

    uint8_t air[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(air, pdu, pdu_bytes);
    uint32_t crc = ble_crc_calc(pdu, pdu_bytes, crc_init) & 0xFFFFFFu;
    air[pdu_bytes + 0u] = ble_bit_reverse_byte((uint8_t)((crc >> 16u) & 0xFFu));
    air[pdu_bytes + 1u] = ble_bit_reverse_byte((uint8_t)((crc >> 8u) & 0xFFu));
    air[pdu_bytes + 2u] = ble_bit_reverse_byte((uint8_t)(crc & 0xFFu));
    ble_dewhiten(air, pdu_bytes + BLE_CRC_BYTES, ch);

    memcpy(frame->raw_pdu, air, pdu_bytes + BLE_CRC_BYTES);
    frame->raw_pdu_bytes = (uint16_t)(pdu_bytes + BLE_CRC_BYTES);
}

static void make_data_event(ble_event_t *ev, uint32_t aa, uint8_t hdr0,
                            const uint8_t *payload, unsigned int payload_len,
                            uint32_t crc_init)
{
    uint8_t pdu[2u + BLE_LL_PAYLOAD_MAX_BYTES];
    pdu[0] = hdr0;
    pdu[1] = (uint8_t)payload_len;
    if (payload_len > 0u)
        memcpy(&pdu[2], payload, payload_len);
    memset(ev, 0, sizeof(*ev));
    ev->meta.channel_index = CH;
    make_frame(&ev->frame, BLE_FRAME_DATA, aa, pdu, 2u + payload_len, CH,
               crc_init);
}

static void make_adv_event(ble_event_t *ev, uint8_t pdu_type,
                           const uint8_t *payload, unsigned int payload_len,
                           uint8_t ch)
{
    uint8_t pdu[2u + BLE_RESERVED_PAYLOAD_MAX_BYTES];
    pdu[0] = pdu_type;
    pdu[1] = (uint8_t)payload_len;
    if (payload_len > 0u)
        memcpy(&pdu[2], payload, payload_len);
    memset(ev, 0, sizeof(*ev));
    ev->meta.channel_index = ch;
    make_frame(&ev->frame, BLE_FRAME_ADVERTISING, BLE_ADVERTISING_AA,
               pdu, 2u + payload_len, ch, BLE_CRC_INIT_ADV);
}

static void test_guards(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);
    ble_event_t ev;
    memset(&ev, 0, sizeof(ev));

    TEST_ASSERT(ble_registry_submit(NULL, &ev, NULL) == 0);
    TEST_ASSERT(ble_registry_submit(&r, NULL, NULL) == 0);
    TEST_ASSERT(ble_registry_add_device(NULL, NULL) == -1);
    TEST_ASSERT(ble_registry_add_device(&r, NULL) == -1);
    ble_registry_set_enforce_crc(NULL, 1);
    ble_registry_confirm_for_test(NULL, AA_X, INIT_X);
    ble_link_t snap;
    TEST_ASSERT(ble_registry_find_link_for_test(NULL, AA_X, &snap) == -1);
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, NULL) == -1);
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) != 0);
    ble_device_snapshot_t devs[4];
    ble_connection_snapshot_t conns[4];
    TEST_ASSERT(ble_registry_get_devices(NULL, devs, 4u) == 0u);
    TEST_ASSERT(ble_registry_get_devices(&r, NULL, 4u) == 0u);
    TEST_ASSERT(ble_registry_get_devices(&r, devs, 0u) == 0u);
    TEST_ASSERT(ble_registry_get_connections(NULL, conns, 4u) == 0u);
    TEST_ASSERT(ble_registry_get_connections(&r, conns, 0u) == 0u);

    ble_registry_free(&r);
    ble_registry_free(NULL); /* must not crash */
}

static void test_adv_ingest_and_device(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);

    /* ADV_IND: AdvA + complete name "TST" + Apple manufacturer. */
    const uint8_t adv_payload[] = {
        0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u,
        0x04, 0x09, 'T', 'S', 'T',
        0x05, 0xFF, 0x4Cu, 0x00u, 0x01u, 0x02u,
    };
    ble_event_t ev;
    make_adv_event(&ev, BLE_PDU_ADV_IND, adv_payload, sizeof(adv_payload),
                   37u);
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 1);

    /* Advertising creates a device row but no data connection. */
    ble_device_snapshot_t devs[8];
    size_t ndev = ble_registry_get_devices(&r, devs, 8u);
    TEST_ASSERT(ndev == 1u);
    if (ndev == 1u)
    {
        uint64_t want = 0u;
        const uint8_t ab[6] = {0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u};
        ble_addr_bytes_to_u64(ab, &want);
        TEST_ASSERT(devs[0].adv_addr == want);
        TEST_ASSERT(strcmp(devs[0].name, "TST") == 0);
        TEST_ASSERT(strstr(devs[0].manufacturer, "Apple") != NULL);
    }
    ble_connection_snapshot_t conns[8];
    TEST_ASSERT(ble_registry_get_connections(&r, conns, 8u) == 0u);

    ble_registry_free(&r);
}

static void test_enforce_crc(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);

    const uint8_t adv_payload[] = {
        0x0Au, 0x0Bu, 0x0Cu, 0x0Du, 0x0Eu, 0x0Fu, 0x02, 0x0A,
    };
    ble_event_t ev;
    make_adv_event(&ev, BLE_PDU_ADV_IND, adv_payload, sizeof(adv_payload),
                   37u);
    /* Corrupt one air bit inside the whitened PDU region. */
    ev.frame.raw_pdu[4] ^= 0x04u;

    /* Without enforcement the frame is still surfaced (crc_ok=0). */
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 1);

    /* With enforcement the CRC-failing adv frame is dropped. */
    ble_registry_set_enforce_crc(&r, 1);
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 0);
    ble_registry_set_enforce_crc(&r, 0);
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 1);

    ble_registry_free(&r);
}

static void test_data_confirm_lifecycle(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);
    ble_event_t ev;
    ble_connection_snapshot_t touched;

    /* Three empty packets promote the AA and seed one candidate. */
    for (int i = 0; i < 3; i++)
    {
        make_data_event(&ev, AA_X, 0x01u, NULL, 0u, INIT_X);
        TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 0);
    }
    ble_link_t snap;
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_CONNECTION_COLLECTING);
    TEST_ASSERT(snap.candidate_count == 1u);
    TEST_ASSERT(snap.candidates[0] == INIT_X);

    /* A non-empty packet under the candidate confirms and surfaces. */
    const uint8_t payload[4] = {0xDEu, 0xADu, 0xBEu, 0xEFu};
    make_data_event(&ev, AA_X, 0x02u, payload, sizeof(payload), INIT_X);
    memset(&touched, 0xA5, sizeof(touched));
    TEST_ASSERT(ble_registry_submit(&r, &ev, &touched) == 1);
    TEST_ASSERT(touched.access_address == AA_X);
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_CONNECTION_CONFIRMED);
    TEST_ASSERT(snap.crc_init == INIT_X);
    TEST_ASSERT(snap.candidate_count == 0u);

    /* One more verified packet bumps the accepted counter. The first two
     * empties only bumped the pending tally (no connection yet), so seen
     * counts the promoting empty plus the two verified data frames. */
    make_data_event(&ev, AA_X, 0x02u, payload, sizeof(payload), INIT_X);
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 1);
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) == 0);
    TEST_ASSERT(snap.packets_seen == 3u);
    TEST_ASSERT(snap.packets_accepted == 2u);

    /* A packet under the wrong CRCInit is rejected on a confirmed link. */
    make_data_event(&ev, AA_X, 0x02u, payload, sizeof(payload), 0x000001u);
    TEST_ASSERT(ble_registry_submit(&r, &ev, NULL) == 0);

    /* Enforced registries hide collecting links but show confirmed ones. */
    ble_registry_set_enforce_crc(&r, 1);
    ble_connection_snapshot_t conns[4];
    TEST_ASSERT(ble_registry_get_connections(&r, conns, 4u) == 1u);
    TEST_ASSERT(conns[0].crc_init_confirmed);
    ble_registry_set_enforce_crc(&r, 0);

    ble_registry_free(&r);
}

static void test_candidate_fifo_cap(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);
    ble_event_t ev;

    /* Three identical empties promote + seed candidate #1. */
    for (int i = 0; i < 3; i++)
    {
        make_data_event(&ev, AA_X, 0x01u, NULL, 0u, INIT_X);
        ble_registry_submit(&r, &ev, NULL);
    }
    /* Four more empties with distinct air CRCs seed distinct candidates. */
    for (unsigned int i = 1u; i <= 4u; i++)
    {
        make_data_event(&ev, AA_X, 0x01u, NULL, 0u, INIT_X);
        ev.frame.raw_pdu[2] ^= (uint8_t)(i * 0x11u);
        ev.frame.raw_pdu[3] ^= (uint8_t)(i * 0x23u);
        ble_registry_submit(&r, &ev, NULL);
    }

    ble_link_t snap;
    TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) == 0);
    TEST_ASSERT(snap.state == BLE_CONNECTION_COLLECTING);
    TEST_ASSERT(snap.candidate_count == BLE_REGISTRY_MAX_CANDIDATES);

    ble_registry_free(&r);
}

static void test_standalone_device(void)
{
    ble_registry_t r;
    ble_registry_init(&r, NULL);

    ble_device_obs_t obs;
    memset(&obs, 0, sizeof(obs));
    const uint8_t ab[6] = {0xAAu, 0xBBu, 0xCCu, 0xDDu, 0xEEu, 0xFFu};
    uint64_t want = 0u;
    ble_addr_bytes_to_u64(ab, &want);
    obs.adv_addr = want;
    obs.addr_type = "random";
    obs.name = "OBO";
    obs.manufacturer = "TestCo";
    obs.rssi_db = -50.0f;
    obs.rssi_valid = 1;
    TEST_ASSERT(ble_registry_add_device(&r, &obs) == 0);

    ble_device_snapshot_t devs[4];
    TEST_ASSERT(ble_registry_get_devices(&r, devs, 4u) == 1u);
    if (ble_registry_get_devices(&r, devs, 4u) == 1u)
    {
        TEST_ASSERT(devs[0].adv_addr == want);
        TEST_ASSERT(strcmp(devs[0].name, "OBO") == 0);
        TEST_ASSERT(devs[0].total_packets == 1u);
    }

    /* Repeat observation merges (packet count grows, still one row). */
    TEST_ASSERT(ble_registry_add_device(&r, &obs) == 0);
    TEST_ASSERT(ble_registry_get_devices(&r, devs, 4u) == 1u);

    ble_registry_free(&r);
}

static void test_pending_cap_and_lru_eviction(void)
{
    /* Junk path: distinct AAs seen once never allocate (pending cap 1024). */
    {
        ble_registry_t r;
        ble_registry_init(&r, NULL);
        ble_event_t ev;
        for (uint32_t i = 0u; i < 1025u; i++)
        {
            make_data_event(&ev, 0x10000000u + i, 0x01u, NULL, 0u, INIT_X);
            ble_registry_submit(&r, &ev, NULL);
        }
        ble_connection_snapshot_t conns[4];
        TEST_ASSERT(ble_registry_get_connections(&r, conns, 4u) == 0u);
        ble_registry_free(&r);
    }

    /* Overflow: 513 promoted AAs exceed the 512 connection slots; the
     * confirmed link must survive victim selection. */
    {
        ble_registry_t r;
        ble_registry_init(&r, NULL);
        ble_registry_confirm_for_test(&r, AA_X, INIT_X);
        ble_event_t ev;
        for (uint32_t i = 0u; i < 513u; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                make_data_event(&ev, 0x20000000u + i, 0x01u, NULL, 0u,
                                INIT_X);
                ble_registry_submit(&r, &ev, NULL);
            }
        }
        ble_connection_snapshot_t conns[600];
        TEST_ASSERT(ble_registry_get_connections(&r, conns, 600u) == 512u);
        ble_link_t snap;
        TEST_ASSERT(ble_registry_find_link_for_test(&r, AA_X, &snap) == 0);
        TEST_ASSERT(snap.state == BLE_CONNECTION_CONFIRMED);
        ble_registry_free(&r);
    }
}

int main(void)
{
    test_guards();
    test_adv_ingest_and_device();
    test_enforce_crc();
    test_data_confirm_lifecycle();
    test_candidate_fifo_cap();
    test_standalone_device();
    test_pending_cap_and_lru_eviction();

    if (g_failures)
    {
        printf("test_ble_registry: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ble_registry: OK\n");
    return 0;
}
