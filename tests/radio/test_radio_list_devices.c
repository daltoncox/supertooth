/**
 * @file test_radio_list_devices.c
 * @brief Verify radio_list_devices() / radio_free_device_list() behave.
 *
 * This test runs without guarantees that HackRF hardware is present.
 * When no devices are connected, radio_list_devices() must still report
 * RADIO_SUCCESS with an empty (but non-NULL-semantically valid) result,
 * and radio_free_device_list() must release it without leaking. When
 * devices are present, every returned identifier must be a non-NULL,
 * NUL-terminated string.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "radio_common.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                                              \
    do                                                                                                \
    {                                                                                                 \
        if (!(cond))                                                                                  \
        {                                                                                             \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);                  \
            g_failures++;                                                                             \
        }                                                                                             \
    } while (0)

int main(void)
{
    char **identifiers = NULL;
    size_t count = 0u;

    /* Invalid argument handling: NULL out pointers. */
    TEST_ASSERT(radio_list_devices(RADIO_DEVICE_HACKRF, NULL, &count) != RADIO_SUCCESS);
    TEST_ASSERT(radio_list_devices(RADIO_DEVICE_HACKRF, &identifiers, NULL) != RADIO_SUCCESS);

    int result = radio_list_devices(RADIO_DEVICE_HACKRF, &identifiers, &count);
    TEST_ASSERT(result == RADIO_SUCCESS);

    if (result == RADIO_SUCCESS)
    {
        for (size_t i = 0u; i < count; i++)
        {
            TEST_ASSERT(identifiers[i] != NULL);
            if (identifiers[i])
                TEST_ASSERT(identifiers[i][strlen(identifiers[i])] == '\0');
        }

        radio_free_device_list(&identifiers, count);
        TEST_ASSERT(identifiers == NULL);
    }

    /* Double-free safety: freeing an already-freed list must be a no-op. */
    radio_free_device_list(&identifiers, count);

    /* radio_device_exists(): a bogus id must never be considered present,
     * regardless of whether real hardware is connected. */
    TEST_ASSERT(radio_device_exists(RADIO_DEVICE_HACKRF,
                                    "this_id_does_not_exist") ==
                RADIO_DEVICE_NOT_FOUND);

    if (g_failures)
    {
        fprintf(stderr, "test_radio_list_devices: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("test_radio_list_devices: ok (%zu device(s))\n", count);
    return 0;
}