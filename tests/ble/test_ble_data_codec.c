/* BLE codec tests: data-channel PDU decode, advertising decode regression,
 * and CONNECT_IND LLData parsing. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "ble_codec.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

/* Build a frame as the framer would: whitened PDU+CRC in raw_pdu. */
static void make_frame(ble_frame_t *frame,
                       ble_frame_kind_t kind,
                       uint32_t aa,
                       const uint8_t *pdu,
                       unsigned int pdu_bytes,
                       uint8_t channel_index,
                       uint32_t crc_init,
                       uint8_t crc_ok)
{
    memset(frame, 0, sizeof(*frame));
    frame->phy = RECEIVER_PHY_LE_1M;
    frame->kind = kind;
    frame->preamble = (aa & 1u) ? 0xAAu : 0x55u;
    frame->access_address = aa;
    frame->crc_init = crc_init;
    frame->crc_ok = crc_ok;

    uint8_t air[BLE_PDU_MAX_BYTES + BLE_CRC_BYTES];
    memcpy(air, pdu, pdu_bytes);
    uint32_t crc = ble_crc_calc(pdu, pdu_bytes, crc_init) & 0xFFFFFFu;
    air[pdu_bytes + 0u] = ble_bit_reverse_byte((uint8_t)((crc >> 16u) & 0xFFu));
    air[pdu_bytes + 1u] = ble_bit_reverse_byte((uint8_t)((crc >> 8u) & 0xFFu));
    air[pdu_bytes + 2u] = ble_bit_reverse_byte((uint8_t)(crc & 0xFFu));
    ble_dewhiten(air, pdu_bytes + BLE_CRC_BYTES, channel_index);

    memcpy(frame->raw_pdu, air, pdu_bytes + BLE_CRC_BYTES);
    frame->raw_pdu_bytes = (uint16_t)(pdu_bytes + BLE_CRC_BYTES);
}

static void test_data_pdu_decode(void)
{
    const uint8_t ch = 17u;
    const uint32_t aa = 0x12345678u;
    const uint32_t init = 0x2C4A6Eu;
    /* LLID=2 (START), NESN=1, SN=1, MD=1; len=4. */
    const uint8_t pdu[6] = {0x1Eu, 0x04u, 0xDEu, 0xADu, 0xBEu, 0xEFu};

    ble_frame_t frame;
    make_frame(&frame, BLE_FRAME_DATA, aa, pdu, sizeof(pdu), ch, init, 1u);

    ble_packet_t pkt;
    TEST_ASSERT(ble_decode_frame(&frame, ch, &pkt) == 0);
    TEST_ASSERT(pkt.is_adv_pdu == 0u);
    TEST_ASSERT(pkt.access_address == aa);
    TEST_ASSERT(pkt.crc_init == init);
    TEST_ASSERT(pkt.crc_ok == 1u);
    TEST_ASSERT(pkt.pdu.data.llid == 0x02u);
    TEST_ASSERT(pkt.pdu.data.nesn == 1u);
    TEST_ASSERT(pkt.pdu.data.sn == 1u);
    TEST_ASSERT(pkt.pdu.data.md == 1u);
    TEST_ASSERT(pkt.pdu.data.payload_len == 4u);
    TEST_ASSERT(memcmp(pkt.pdu.data.payload, &pdu[2], 4u) == 0);
}

static void test_data_empty_pdu_decode(void)
{
    const uint8_t ch = 0u;
    const uint32_t aa = 0x00000001u;   /* preamble 0xAA */
    const uint32_t init = 0x010203u;
    const uint8_t pdu[2] = {0x01u, 0x00u};   /* LLID=CONT, len=0 */

    ble_frame_t frame;
    make_frame(&frame, BLE_FRAME_DATA, aa, pdu, sizeof(pdu), ch, init, 1u);

    ble_packet_t pkt;
    TEST_ASSERT(ble_decode_frame(&frame, ch, &pkt) == 0);
    TEST_ASSERT(pkt.is_adv_pdu == 0u);
    TEST_ASSERT(pkt.pdu.data.llid == 0x01u);
    TEST_ASSERT(pkt.pdu.data.payload_len == 0u);
    TEST_ASSERT(pkt.preamble == 0xAAu);
}

static void test_adv_pdu_regression(void)
{
    const uint8_t ch = 37u;
    const uint32_t aa = BLE_ADVERTISING_AA;
    /* ADV_IND, TxAdd/RxAdd=0, len=8: AdvA(6) + 2 AdvData bytes. */
    const uint8_t pdu[10] = {
        0x00u, 0x08u,
        0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u,
        0x02u, 0x0Au,
    };

    ble_frame_t frame;
    make_frame(&frame, BLE_FRAME_ADVERTISING, aa, pdu, sizeof(pdu), ch,
               BLE_CRC_INIT_ADV, 0u);

    ble_packet_t pkt;
    TEST_ASSERT(ble_decode_frame(&frame, ch, &pkt) == 0);
    TEST_ASSERT(pkt.is_adv_pdu == 1u);
    TEST_ASSERT(pkt.crc_init == BLE_CRC_INIT_ADV);
    TEST_ASSERT(pkt.crc_ok == 1u);
    TEST_ASSERT(pkt.pdu.adv.pdu_type == BLE_PDU_ADV_IND);
    TEST_ASSERT(pkt.pdu.adv.payload_len == 8u);
    TEST_ASSERT(memcmp(pkt.pdu.adv.payload.adv_ind.adv_addr.addr,
                       &pdu[2], BLE_ADDR_LEN) == 0);
    TEST_ASSERT(pkt.pdu.adv.payload.adv_ind.adv_data_len == 2u);
    TEST_ASSERT(pkt.pdu.adv.payload.adv_ind.adv_data[0] == 0x02u);
}

static void test_connect_ind_parse(void)
{
    uint8_t ll_data[BLE_CONNECT_IND_DATA_MAX_BYTES];
    memset(ll_data, 0, sizeof(ll_data));
    ll_data[0] = 0x78u; ll_data[1] = 0x56u; ll_data[2] = 0x34u; ll_data[3] = 0x12u;
    ll_data[4] = 0x6Eu; ll_data[5] = 0x4Au; ll_data[6] = 0x2Cu;
    ll_data[7] = 0x02u;                       /* WinSize */
    ll_data[8] = 0x05u; ll_data[9] = 0x00u;   /* WinOffset */
    ll_data[10] = 0x18u; ll_data[11] = 0x00u; /* Interval = 24 */
    ll_data[12] = 0x04u; ll_data[13] = 0x00u; /* Latency = 4 */
    ll_data[14] = 0xC8u; ll_data[15] = 0x00u; /* Timeout = 200 */
    ll_data[16] = 0xFFu; ll_data[17] = 0x1Fu; ll_data[18] = 0x00u;
    ll_data[19] = 0x00u; ll_data[20] = 0x00u; /* ChM */
    ll_data[21] = (0x05u << 5u) | 0x0Au;      /* SCA=5, Hop=10 */

    ble_connect_ind_params_t p;
    TEST_ASSERT(ble_connect_ind_parse(ll_data, &p) == 0);
    TEST_ASSERT(p.access_address == 0x12345678u);
    TEST_ASSERT(p.crc_init == 0x2C4A6Eu);
    TEST_ASSERT(p.window_size == 0x02u);
    TEST_ASSERT(p.window_offset == 0x0005u);
    TEST_ASSERT(p.interval == 24u);
    TEST_ASSERT(p.latency == 4u);
    TEST_ASSERT(p.timeout == 200u);
    TEST_ASSERT(p.channel_map[0] == 0xFFu && p.channel_map[1] == 0x1Fu);
    TEST_ASSERT(p.hop_increment == 10u);
    TEST_ASSERT(p.sleep_clock_accuracy == 5u);

    TEST_ASSERT(ble_connect_ind_parse(NULL, &p) != 0);
    TEST_ASSERT(ble_connect_ind_parse(ll_data, NULL) != 0);
}

static void test_connect_ind_frame_roundtrip(void)
{
    const uint8_t ch = 38u;
    const uint32_t aa = BLE_ADVERTISING_AA;
    /* CONNECT_IND: InitA(6) + AdvA(6) + LLData(22) = 34-byte payload. */
    uint8_t pdu[2 + 34];
    pdu[0] = 0x05u;   /* CONNECT_IND, TxAdd/RxAdd = 0 */
    pdu[1] = 34u;
    for (unsigned int i = 0; i < 12u; i++)
        pdu[2u + i] = (uint8_t)(0xA0u + i);
    uint8_t *ll = &pdu[14];
    ll[0] = 0xEFu; ll[1] = 0xBEu; ll[2] = 0xADu; ll[3] = 0xDEu;
    ll[4] = 0x33u; ll[5] = 0x22u; ll[6] = 0x11u;
    ll[21] = 0x07u;   /* hop=7, sca=0 */

    ble_frame_t frame;
    make_frame(&frame, BLE_FRAME_ADVERTISING, aa, pdu, sizeof(pdu), ch,
               BLE_CRC_INIT_ADV, 0u);

    ble_packet_t pkt;
    TEST_ASSERT(ble_decode_frame(&frame, ch, &pkt) == 0);
    TEST_ASSERT(pkt.is_adv_pdu == 1u);
    TEST_ASSERT(pkt.pdu.adv.pdu_type == BLE_PDU_CONNECT_IND);
    TEST_ASSERT(pkt.pdu.adv.payload.connect_ind.ll_data_len == 22u);

    ble_connect_ind_params_t p;
    TEST_ASSERT(ble_connect_ind_parse(pkt.pdu.adv.payload.connect_ind.ll_data,
                                      &p) == 0);
    TEST_ASSERT(p.access_address == 0xDEADBEEFu);
    TEST_ASSERT(p.crc_init == 0x112233u);
    TEST_ASSERT(p.hop_increment == 7u);
}

int main(void)
{
    test_data_pdu_decode();
    test_data_empty_pdu_decode();
    test_adv_pdu_regression();
    test_connect_ind_parse();
    test_connect_ind_frame_roundtrip();

    if (g_failures)
    {
        printf("test_ble_data_codec: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ble_data_codec: OK\n");
    return 0;
}
