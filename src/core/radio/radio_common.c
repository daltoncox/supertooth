#include "radio_common.h"

#include "hackrf.h"

#include <stdlib.h>

struct radio_device
{
    radio_device_type_t device_type;
    void *impl;
};

const char *radio_device_type_name(radio_device_type_t type)
{
    switch (type)
    {
    case RADIO_DEVICE_HACKRF:
        return "hackrf";
    default:
        return NULL;
    }
}

int radio_open(radio_device_t **out_device,
               radio_device_type_t device_type,
               const char *device_id,
               sample_dispatcher_t *dispatcher,
               int debug_enabled)
{
    radio_device_t *device = NULL;
    int result = -1;

    if (!out_device || !dispatcher)
        return -1;

    *out_device = NULL;
    device = (radio_device_t *)calloc(1, sizeof(*device));
    if (!device)
        return -1;

    device->device_type = device_type;

    switch (device_type)
    {
    case RADIO_DEVICE_HACKRF:
        result = hackrf_radio_open(&device->impl, device_id, dispatcher,
                                   debug_enabled);
        break;
    default:
        break;
    }

    if (result != RADIO_SUCCESS)
    {
        free(device);
        return result;
    }

    *out_device = device;
    return RADIO_SUCCESS;
}

int radio_configure(radio_device_t *device, const radio_stream_config_t *config)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_configure(device->impl, config);
    default:
        return -1;
    }
}

int radio_start_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_start_rx(device->impl);
    default:
        return -1;
    }
}

int radio_stop_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_stop_rx(device->impl);
    default:
        return -1;
    }
}

void radio_close(radio_device_t *device)
{
    if (!device)
        return;

    switch (device->device_type)
    {
    case RADIO_DEVICE_HACKRF:
        hackrf_radio_close(device->impl);
        break;
    default:
        break;
    }

    free(device);
}

int radio_list_devices(radio_device_type_t device_type,
                       char ***out_identifiers,
                       size_t *out_count)
{
    if (!out_identifiers || !out_count)
        return -1;

    *out_identifiers = NULL;
    *out_count = 0u;

    switch (device_type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_list_devices(out_identifiers, out_count);
    default:
        return -1;
    }
}

void radio_free_device_list(char ***identifiers, size_t count)
{
    if (!identifiers || !*identifiers)
        return;

    char **list = *identifiers;
    for (size_t i = 0u; i < count; i++)
        free(list[i]);
    free(list);
    *identifiers = NULL;
}