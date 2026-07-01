#include "hackrf.h"

#include <libhackrf/hackrf.h>

#include <complex.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The device id for a HackRF is the unique trailing portion of the LPC43xx
 * factory-programmed die ID (read_partid_serialno_t::serial_no[2..3]),
 * rendered as 16 lowercase hex chars. It is unique per chip and stable
 * across replugs. The leading words serial_no[0..1] read as zero on
 * every board we have seen, so they are not part of the id. */
static int hackrf_read_id(hackrf_device *device, char out[HACKRF_ID_LEN + 1u])
{
    read_partid_serialno_t info;
    int result = hackrf_board_partid_serialno_read(device, &info);
    if (result != HACKRF_SUCCESS)
        return result;
    snprintf(out, HACKRF_ID_LEN + 1u, "%08x%08x",
             info.serial_no[2], info.serial_no[3]);
    return HACKRF_SUCCESS;
}

typedef struct
{
    hackrf_device *device;
    sample_dispatcher_t *dispatcher;
    int debug_enabled;
    uint64_t samples_received;
} hackrf_radio_t;

static inline float complex hackrf_iq_to_complex(const int8_t *samples,
                                                 unsigned int sample_index)
{
    return samples[2u * sample_index] / 128.0f +
           (samples[2u * sample_index + 1u] / 128.0f) * _Complex_I;
}

static int hackrf_rx_cb(hackrf_transfer *transfer)
{
    hackrf_radio_t *radio = transfer ? (hackrf_radio_t *)transfer->rx_ctx : NULL;
    sample_block_t *block;
    const int8_t *samples;
    unsigned int num_samples;

    if (!radio || !radio->dispatcher)
        return -1;
    if (!transfer->buffer)
        return -1;

    num_samples = (unsigned int)(transfer->valid_length / 2u);
    if (num_samples > SAMPLE_BLOCK_SAMPLE_CAPACITY)
        num_samples = SAMPLE_BLOCK_SAMPLE_CAPACITY;

    block = sample_dispatcher_acquire_block(radio->dispatcher);
    if (!block)
    {
        sample_dispatcher_note_drop(radio->dispatcher, radio->debug_enabled);
        return 0;
    }

    block->num_samples = num_samples;
    block->block_base_sample = radio->samples_received;
    radio->samples_received += num_samples;

    samples = (const int8_t *)transfer->buffer;
    for (unsigned int i = 0; i < num_samples; i++)
        block->samples[i] = hackrf_iq_to_complex(samples, i);

    __atomic_thread_fence(__ATOMIC_RELEASE);
    sample_dispatcher_push_block(radio->dispatcher, block);
    sample_block_release(block);

    return 0;
}

int hackrf_radio_open(void **out_device,
                      const char *device_id,
                      sample_dispatcher_t *dispatcher,
                      int debug_enabled)
{
    hackrf_radio_t *radio = NULL;
    hackrf_device_list_t *list = NULL;
    int result;

    if (!out_device || !dispatcher)
        return HACKRF_ERROR_INVALID_PARAM;

    *out_device = NULL;
    radio = (hackrf_radio_t *)calloc(1, sizeof(*radio));
    if (!radio)
        return -1;

    radio->dispatcher = dispatcher;
    radio->debug_enabled = debug_enabled;

    result = hackrf_init();
    if (result != HACKRF_SUCCESS)
        goto fail;

    if (device_id && device_id[0] != '\0')
    {
        /* Open a specific device by reading each connected HackRF's id and
         * keeping the first match. */
        list = hackrf_device_list();
        if (!list)
        {
            result = HACKRF_ERROR_NOT_FOUND;
            goto fail_with_exit;
        }

        result = HACKRF_ERROR_NOT_FOUND;
        for (int i = 0; i < list->devicecount; i++)
        {
            hackrf_device *dev = NULL;
            if (hackrf_device_list_open(list, i, &dev) != HACKRF_SUCCESS)
                continue;

            char id[HACKRF_ID_LEN + 1u] = {0};
            if (hackrf_read_id(dev, id) == HACKRF_SUCCESS &&
                strcmp(id, device_id) == 0)
            {
                radio->device = dev;
                result = HACKRF_SUCCESS;
                break;
            }

            hackrf_close(dev);
        }

        hackrf_device_list_free(list);
        list = NULL;
        if (result != HACKRF_SUCCESS)
            goto fail_with_exit;
    }
    else
    {
        result = hackrf_open(&radio->device);
        if (result != HACKRF_SUCCESS)
            goto fail_with_exit;
    }

    *out_device = radio;
    return HACKRF_SUCCESS;

fail_with_exit:
    hackrf_exit();
    if (list)
        hackrf_device_list_free(list);
fail:
    free(radio);
    return result;
}

int hackrf_radio_configure(void *device, const radio_stream_config_t *config)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    int result;

    if (!radio || !radio->device || !config)
        return HACKRF_ERROR_INVALID_PARAM;

    result = hackrf_set_lna_gain(radio->device, config->lna_gain);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_vga_gain(radio->device, config->vga_gain);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_freq(radio->device, config->lo_freq_hz);
    if (result != HACKRF_SUCCESS)
        return result;

    result = hackrf_set_sample_rate(radio->device, config->sample_rate);
    if (result != HACKRF_SUCCESS)
        return result;

    return HACKRF_SUCCESS;
}

int hackrf_radio_start_rx(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;

    if (!radio || !radio->device || !radio->dispatcher)
        return HACKRF_ERROR_INVALID_PARAM;

    radio->samples_received = 0ULL;
    return hackrf_start_rx(radio->device, hackrf_rx_cb, radio);
}

int hackrf_radio_stop_rx(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    if (!radio || !radio->device)
        return HACKRF_ERROR_INVALID_PARAM;

    return hackrf_stop_rx(radio->device);
}

void hackrf_radio_close(void *device)
{
    hackrf_radio_t *radio = (hackrf_radio_t *)device;
    if (!radio)
        return;

    if (radio->device)
        hackrf_close(radio->device);
    hackrf_exit();
    free(radio);
}

int hackrf_list_devices(char ***out_identifiers, size_t *out_count)
{
    char **identifiers = NULL;
    int result;
    int device_count;
    size_t copied = 0u;

    if (!out_identifiers || !out_count)
        return -1;

    *out_identifiers = NULL;
    *out_count = 0u;

    result = hackrf_init();
    if (result != HACKRF_SUCCESS)
        return result;

    hackrf_device_list_t *list = hackrf_device_list();
    if (!list)
    {
        result = HACKRF_ERROR_NOT_FOUND;
        goto cleanup;
    }

    device_count = list->devicecount;
    if (device_count <= 0)
    {
        hackrf_device_list_free(list);
        result = HACKRF_SUCCESS;
        goto cleanup;
    }

    identifiers = (char **)calloc((size_t)device_count, sizeof(char *));
    if (!identifiers)
    {
        hackrf_device_list_free(list);
        result = -1;
        goto cleanup;
    }

    for (int i = 0; i < device_count; i++)
    {
        hackrf_device *dev = NULL;
        char id[HACKRF_ID_LEN + 1u] = {0};

        result = hackrf_device_list_open(list, i, &dev);
        if (result != HACKRF_SUCCESS)
            continue;  /* leave this slot NULL; skip below */

        result = hackrf_read_id(dev, id);
        hackrf_close(dev);

        if (result != HACKRF_SUCCESS)
            continue;

        identifiers[copied] = strdup(id);
        if (!identifiers[copied])
        {
            for (size_t j = 0u; j < copied; j++)
                free(identifiers[j]);
            free(identifiers);
            identifiers = NULL;
            hackrf_device_list_free(list);
            result = -1;
            goto cleanup;
        }
        copied++;
        result = HACKRF_SUCCESS;
    }

    hackrf_device_list_free(list);

    *out_identifiers = identifiers;
    *out_count = copied;
    result = (copied > 0u) ? HACKRF_SUCCESS : result;

cleanup:
    hackrf_exit();
    return result;
}