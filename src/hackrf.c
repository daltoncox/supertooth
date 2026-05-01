#include "hackrf.h"
#include <stdlib.h>

int hackrf_connect(hackrf_device **device)
{
    int result;

    if (!device)
    {
        return HACKRF_ERROR_INVALID_PARAM;
    }

    // Initialize HackRF library
    result = hackrf_init();
    if (result != HACKRF_SUCCESS)
    {
        return result;
    }

    // Open HackRF device
    result = hackrf_open(device);
    if (result != HACKRF_SUCCESS)
    {
        hackrf_exit();
        return result;
    }

    return HACKRF_SUCCESS;
}

int hackrf_configure(hackrf_device *device, const hackrf_config_t *config)
{
    int result;

    if (!device || !config)
    {
        return HACKRF_ERROR_INVALID_PARAM;
    }

    // Set LNA gain
    result = hackrf_set_lna_gain(device, config->lna_gain);
    if (result != HACKRF_SUCCESS)
    {
        return result;
    }

    // Set VGA gain
    result = hackrf_set_vga_gain(device, config->vga_gain);
    if (result != HACKRF_SUCCESS)
    {
        return result;
    }

    // Set frequency
    result = hackrf_set_freq(device, config->lo_freq_hz);
    if (result != HACKRF_SUCCESS)
    {
        return result;
    }

    // Set sample rate
    result = hackrf_set_sample_rate(device, config->sample_rate);
    if (result != HACKRF_SUCCESS)
    {
        return result;
    }

    return HACKRF_SUCCESS;
}

void hackrf_disconnect(hackrf_device *device)
{
    if (device)
    {
        hackrf_close(device);
    }
    hackrf_exit();
}