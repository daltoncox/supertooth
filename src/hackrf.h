#ifndef HACKRF_WRAPPER_H
#define HACKRF_WRAPPER_H

#include <libhackrf/hackrf.h>

/**
 * Configuration structure for HackRF parameters
 */
typedef struct
{
    uint64_t lo_freq_hz;  // Local Oscillator Frequency in Hz
    uint32_t sample_rate; // Sample Rate in Hz
    uint32_t lna_gain;    // LNA Gain (0-40 dB, 8 dB steps)
    uint32_t vga_gain;    // VGA Gain (0-62 dB, 2 dB steps)
} hackrf_config_t;

/**
 * Connect to and initialize the HackRF device
 * @param device Pointer to hackrf_device pointer that will be set
 * @return HACKRF_SUCCESS on success, error code on failure
 */
int hackrf_connect(hackrf_device **device);

/**
 * Configure the HackRF device with specified parameters
 * @param device HackRF device handle
 * @param config Configuration parameters
 * @return HACKRF_SUCCESS on success, error code on failure
 */
int hackrf_configure(hackrf_device *device, const hackrf_config_t *config);

/**
 * Cleanup and disconnect from HackRF device
 * @param device HackRF device handle
 */
void hackrf_disconnect(hackrf_device *device);

#endif // HACKRF_WRAPPER_H