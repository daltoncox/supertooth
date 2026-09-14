#include "radio_common.h"

#include "file.h"
#include "hackrf.h"

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

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
    case RADIO_DEVICE_FILE:
        return "file";
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
    case RADIO_DEVICE_FILE:
        /* For file replay the "device id" is the capture path. */
        result = file_radio_open(&device->impl, device_id, dispatcher,
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
    case RADIO_DEVICE_FILE:
        return file_radio_configure(device->impl, config);
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
    case RADIO_DEVICE_FILE:
        return file_radio_start_rx(device->impl);
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
    case RADIO_DEVICE_FILE:
        return file_radio_stop_rx(device->impl);
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
    case RADIO_DEVICE_FILE:
        file_radio_close(device->impl);
        break;
    default:
        break;
    }

    free(device);
}

int radio_is_finished(radio_device_t *device)
{
    if (!device)
        return 0;

    switch (device->device_type)
    {
    case RADIO_DEVICE_FILE:
        return file_radio_is_finished(device->impl);
    case RADIO_DEVICE_HACKRF:
    default:
        return 0;
    }
}

void radio_set_replay_mode(radio_device_t *device, int exhaustive)
{
    if (!device)
        return;

    switch (device->device_type)
    {
    case RADIO_DEVICE_FILE:
        file_radio_set_exhaustive(device->impl, exhaustive);
        break;
    case RADIO_DEVICE_HACKRF:
    default:
        break;
    }
}

int radio_get_max_sample_rate_for_type(radio_device_type_t type,
                                       uint32_t *out_rate_hz)
{
    if (!out_rate_hz)
        return -1;

    switch (type)
    {
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_get_max_sample_rate(NULL, out_rate_hz);
    case RADIO_DEVICE_FILE:
        /* Replay of an existing capture is not bound by ADC limits. */
        *out_rate_hz = RADIO_MAX_SAMPLE_RATE_HZ;
        return RADIO_SUCCESS;
    default:
        return -1;
    }
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
    case RADIO_DEVICE_FILE:
        /* Captures are user paths, not enumerable hardware. */
        return RADIO_SUCCESS;
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

int radio_device_exists(radio_device_type_t device_type, const char *device_id)
{
    /* For file replay "existence" is just path readability. */
    if (device_type == RADIO_DEVICE_FILE)
    {
        struct stat st;
        if (!device_id || device_id[0] == '\0')
            return RADIO_DEVICE_NOT_FOUND;
        if (stat(device_id, &st) == 0 && S_ISREG(st.st_mode) &&
            access(device_id, R_OK) == 0)
            return RADIO_SUCCESS;
        return RADIO_DEVICE_NOT_FOUND;
    }

    char **identifiers = NULL;
    size_t count = 0u;
    int result = radio_list_devices(device_type, &identifiers, &count);
    if (result != RADIO_SUCCESS)
        return result;

    result = RADIO_DEVICE_NOT_FOUND;
    for (size_t i = 0u; i < count; i++)
    {
        if (identifiers[i] && strcmp(identifiers[i], device_id) == 0)
        {
            result = RADIO_SUCCESS;
            break;
        }
    }

    radio_free_device_list(&identifiers, count);
    return result;
}