/* CRCInit recovery (reverse-CRC) tests.
 *
 * The BLE CRC is affine in its init: crc(M, I) = A(M) ^ T*I. For 2-byte
 * messages (0-length LL data PDUs), ble_crc_reverse_init_len2() must invert
 * the transform exactly for every (header, init) pair.
 */
#include <stdio.h>
#include <stdint.h>

#include "ble_codec.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);            \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

static uint32_t prng_state = 0x12345678u;

static uint32_t prng(void)
{
    uint32_t x = prng_state;
    x ^= x << 13u;
    x ^= x >> 17u;
    x ^= x << 5u;
    prng_state = x ? x : 1u;
    return prng_state;
}

int main(void)
{
    ble_crc_tables_init();

    /* Fixed vectors. */
    {
        const uint8_t header[2] = {0x00u, 0x00u};
        uint32_t crc = ble_crc_calc(header, 2u, 0u);
        TEST_ASSERT(ble_crc_reverse_init_len2(header, crc) == 0u);
    }
    {
        const uint8_t header[2] = {0xA5u, 0x3Cu};
        uint32_t crc = ble_crc_calc(header, 2u, 0xFFFFFFu);
        TEST_ASSERT(ble_crc_reverse_init_len2(header, crc) == 0xFFFFFFu);
    }
    {
        const uint8_t header[2] = {0x11u, 0x22u};
        uint32_t crc = ble_crc_calc(header, 2u, 0x555555u);
        TEST_ASSERT(ble_crc_reverse_init_len2(header, crc) == 0x555555u);
    }

    /* Round-trip over randomized (header, init) pairs. */
    for (unsigned int i = 0; i < 2000u; i++)
    {
        uint8_t header[2];
        header[0] = (uint8_t)(prng() & 0xFFu);
        header[1] = (uint8_t)(prng() & 0xFFu);
        uint32_t init = prng() & 0xFFFFFFu;

        uint32_t crc = ble_crc_calc(header, 2u, init);
        uint32_t recovered = ble_crc_reverse_init_len2(header, crc);
        if (recovered != init)
        {
            printf("FAIL roundtrip hdr=%02X%02X init=%06X got %06X\n",
                   header[0], header[1], init, recovered);
            g_failures++;
        }
    }

    /* Through the air-byte path: CRC as bit-reversed air bytes, extracted,
     * then reversed — the full chain the framer uses. */
    for (unsigned int i = 0; i < 500u; i++)
    {
        uint8_t header[2];
        header[0] = (uint8_t)(prng() & 0xFFu);
        header[1] = (uint8_t)(prng() & 0xFFu);
        uint32_t init = prng() & 0xFFFFFFu;

        uint32_t crc = ble_crc_calc(header, 2u, init);
        uint8_t air_crc[3];
        air_crc[0] = ble_bit_reverse_byte((uint8_t)((crc >> 16u) & 0xFFu));
        air_crc[1] = ble_bit_reverse_byte((uint8_t)((crc >> 8u) & 0xFFu));
        air_crc[2] = ble_bit_reverse_byte((uint8_t)(crc & 0xFFu));

        uint32_t rx_crc = ble_extract_crc(air_crc);
        TEST_ASSERT(rx_crc == (crc & 0xFFFFFFu));
        TEST_ASSERT(ble_crc_reverse_init_len2(header, rx_crc) == init);
    }

    if (g_failures)
    {
        printf("test_ble_crc_reverse: %d FAILURES\n", g_failures);
        return 1;
    }
    printf("test_ble_crc_reverse: OK\n");
    return 0;
}
