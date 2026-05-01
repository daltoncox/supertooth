#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <libhackrf/hackrf.h>
#include <unistd.h>
#include <complex.h>
#include "wav.h"

#define FILE_NAME "raw_2403lo_20msps.wav"
#define LO_FREQ_HZ 2403e6                   // 2.440 GHz, somewhere in Bluetooth Classic band
#define SAMPLE_RATE 20e6                    // 20 MHz, change as needed
#define SAMPLES_TO_ACQUIRE 10 * SAMPLE_RATE // 10 seconds worth of samples

float complex *iq_buffer;
uint64_t samples_received = 0;

int rx_callback(hackrf_transfer *transfer)
{
    if (samples_received >= SAMPLES_TO_ACQUIRE)
    {
        return -1; // Stop RX
    }
    size_t samples_in_buffer = transfer->valid_length / 2; // 1 I + 1 Q = 2 bytes per sample
    if (samples_received + samples_in_buffer > SAMPLES_TO_ACQUIRE)
    {
        samples_in_buffer = SAMPLES_TO_ACQUIRE - samples_received;
    }

    // Convert int8_t IQ pairs to complex float
    int8_t *buf = (int8_t *)transfer->buffer;
    for (size_t i = 0; i < samples_in_buffer; i++)
    {
        float i_val = buf[i * 2 + 0] / 128.0f; // Normalize to [-1, 1)
        float q_val = buf[i * 2 + 1] / 128.0f;
        iq_buffer[samples_received + i] = i_val + q_val * I;
    }

    samples_received += samples_in_buffer;
    return 0;
}

int main()
{
    int result;
    hackrf_device *device = NULL;

    result = hackrf_init();
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_init() failed: %s\n", hackrf_error_name(result));
        return EXIT_FAILURE;
    }

    // Scan and open the first available HackRF device
    result = hackrf_open(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_open() failed: %s\n", hackrf_error_name(result));
        hackrf_exit();
        return EXIT_FAILURE;
    }

    // Set LNA gain (0-40 dB)
    result = hackrf_set_lna_gain(device, 32);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_set_lna_gain() failed: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    // Set VGA gain (0-62 dB)
    result = hackrf_set_vga_gain(device, 32);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_set_vga_gain() failed: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    // Set local oscillator frequency
    result = hackrf_set_freq(device, LO_FREQ_HZ);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_set_freq() failed: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    // Set sampling Rate
    result = hackrf_set_sample_rate(device, SAMPLE_RATE);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_set_sample_rate() failed: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    // Allocate buffer for complex samples
    iq_buffer = (float complex *)malloc(SAMPLES_TO_ACQUIRE * sizeof(float complex));
    if (!iq_buffer)
    {
        fprintf(stderr, "Unable to allocate memory for IQ buffer\n");
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    samples_received = 0;
    result = hackrf_start_rx(device, rx_callback, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        free(iq_buffer);
        hackrf_close(device);
        hackrf_exit();
        return EXIT_FAILURE;
    }

    usleep(15000000); // Sleep for 15s

    hackrf_stop_rx(device);
    hackrf_close(device);
    hackrf_exit();

    printf("Recording complete. %llu samples captured.\n", samples_received);
    printf("Writing WAV file '%s'...\n", FILE_NAME);

    // Write complex samples to WAV file
    result = cf2wav(iq_buffer, samples_received, (unsigned int)SAMPLE_RATE, FILE_NAME);
    free(iq_buffer);

    if (result != 0)
    {
        fprintf(stderr, "Failed to write WAV file (error code: %d)\n", result);
        return EXIT_FAILURE;
    }

    printf("Done. %llu samples written to '%s'\n", samples_received, FILE_NAME);
    return EXIT_SUCCESS;
}
