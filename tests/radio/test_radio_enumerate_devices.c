/**
 * @file test_radio_enumerate_devices.c
 * @brief Verify radio_enumerate_devices() / radio_free_device_entries().
 *
 * This test runs without guarantees that radio hardware is present.
 * Enumeration must always report RADIO_SUCCESS (possibly with zero
 * devices), every entry must carry a known live type with a non-NULL id,
 * and the array must be sorted by "type:id" regardless of backend order.
 * Freeing must null the pointer and tolerate double-free.
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

static void entry_key(const radio_device_entry_t *entry, char *buf, size_t len)
{
    const char *name = radio_device_type_name(entry->type);
    snprintf(buf, len, "%s:%s", name ? name : "", entry->id ? entry->id : "");
}

int main(void)
{
    radio_device_entry_t *entries = NULL;
    size_t count = 0u;

    /* Invalid argument handling: NULL out pointers. */
    TEST_ASSERT(radio_enumerate_devices(NULL, &count) != RADIO_SUCCESS);
    TEST_ASSERT(radio_enumerate_devices(&entries, NULL) != RADIO_SUCCESS);

    TEST_ASSERT(radio_enumerate_devices(&entries, &count) == RADIO_SUCCESS);

    for (size_t i = 0u; i < count; i++)
    {
        TEST_ASSERT(radio_device_type_is_live(entries[i].type));
        TEST_ASSERT(radio_device_type_name(entries[i].type) != NULL);
        TEST_ASSERT(entries[i].id != NULL);
        if (entries[i].id)
            TEST_ASSERT(entries[i].id[0] != '\0');
    }

    /* Sorted by "type:id": every adjacent pair must be ordered. */
    for (size_t i = 1u; i < count; i++)
    {
        char prev[512], cur[512];
        entry_key(&entries[i - 1u], prev, sizeof(prev));
        entry_key(&entries[i], cur, sizeof(cur));
        TEST_ASSERT(strcmp(prev, cur) <= 0);
    }

    /* Repeated enumeration must agree (stable order). */
    {
        radio_device_entry_t *again = NULL;
        size_t again_count = 0u;
        TEST_ASSERT(radio_enumerate_devices(&again, &again_count) == RADIO_SUCCESS);
        TEST_ASSERT(again_count == count);
        for (size_t i = 0u; i < count && i < again_count; i++)
        {
            char first[512], second[512];
            TEST_ASSERT(again[i].type == entries[i].type);
            entry_key(&entries[i], first, sizeof(first));
            entry_key(&again[i], second, sizeof(second));
            TEST_ASSERT(strcmp(first, second) == 0);
        }
        radio_free_device_entries(&again, again_count);
        TEST_ASSERT(again == NULL);
    }

    radio_free_device_entries(&entries, count);
    TEST_ASSERT(entries == NULL);

    /* Double-free safety: freeing an already-freed list must be a no-op. */
    radio_free_device_entries(&entries, count);

    printf("test_radio_enumerate_devices: ok (%zu device(s))\n", count);
    return g_failures == 0 ? 0 : 1;
}
