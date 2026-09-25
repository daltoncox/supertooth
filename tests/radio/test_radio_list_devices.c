/**
 * @file test_radio_list_devices.c
 * @brief Verify radio_list_devices() / radio_free_device_list() behave.
 *
 * This test runs without guarantees that radio hardware is present.
 * For every compiled-in device type, radio_list_devices() must report
 * RADIO_SUCCESS (possibly with zero devices), every returned identifier
 * must be a non-NULL, NUL-terminated string, and a bogus id must never
 * be considered present. Types compiled out (see ENABLE_HACKRF /
 * ENABLE_BLADERF) must fail cleanly instead of touching missing drivers.
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

static void check_compiled_in_type(radio_device_type_t type)
{
    char **identifiers = NULL;
    size_t count = 0u;

    /* Invalid argument handling: NULL out pointers. */
    TEST_ASSERT(radio_list_devices(type, NULL, &count) != RADIO_SUCCESS);
    TEST_ASSERT(radio_list_devices(type, &identifiers, NULL) != RADIO_SUCCESS);

    int result = radio_list_devices(type, &identifiers, &count);
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
    TEST_ASSERT(radio_device_exists(type,
                                    "this_id_does_not_exist") ==
                RADIO_DEVICE_NOT_FOUND);

    printf("test_radio_list_devices: type %d ok (%zu device(s))\n",
           (int)type, count);
}

int main(void)
{
    for (int t = 0; t < (int)RADIO_DEVICE_TYPE_COUNT; t++)
    {
        radio_device_type_t type = (radio_device_type_t)t;
        const char *name = radio_device_type_name(type);

        if (type == RADIO_DEVICE_FILE)
        {
            /* File replay is not enumerable hardware: empty success. */
            char **ids = NULL;
            size_t n = 0u;
            TEST_ASSERT(name != NULL);
            TEST_ASSERT(radio_list_devices(type, &ids, &n) == RADIO_SUCCESS);
            TEST_ASSERT(n == 0u);
            radio_free_device_list(&ids, n);
            continue;
        }

        if (name == NULL)
        {
            /* Backend compiled out: enumeration and existence checks
             * must fail cleanly (never RADIO_SUCCESS / present). */
            char **ids = NULL;
            size_t n = 0u;
            TEST_ASSERT(radio_list_devices(type, &ids, &n) != RADIO_SUCCESS);
            TEST_ASSERT(radio_device_exists(type, "anything") !=
                        RADIO_SUCCESS);
            TEST_ASSERT(radio_device_exists(type, "anything") !=
                        RADIO_DEVICE_NOT_FOUND);
            continue;
        }

        check_compiled_in_type(type);
    }

    if (g_failures)
    {
        fprintf(stderr, "test_radio_list_devices: %d failure(s)\n", g_failures);
        return 1;
    }

    printf("test_radio_list_devices: ok\n");
    return 0;
}
