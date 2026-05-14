#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>
#include <btbb.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include "../src/hackrf.h"

#define SAMPLE_RATE 20e6
#define LO_FREQ_HZ 2460e6
#define OFFSET_FREQ_HZ 6e6f
#define BUFFER_SIZE 262144 // HackRF transfer buffer size

nco_crcf nco;
firdecim_crcf firdec;
cpfskdem demod;

float complex raw[BUFFER_SIZE];
float complex mixed[BUFFER_SIZE];
float complex decimated[BUFFER_SIZE / 4];

uint8_t bits[BUFFER_SIZE / 4];

int bredr_channel_cb(hackrf_transfer *transfer)
{
    // Convert all transfer samples to complex floats
    // HackRF provides interleaved I/Q samples as int8_t
    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_samples = transfer->valid_length / 2; // Divide by 2 for I/Q pairs
    for (unsigned int i = 0; i < num_samples; i++)
    {
        // Convert int8_t samples to float range [-1.0, 1.0]
        float i_sample = samples[2 * i] / 128.0f;
        float q_sample = samples[2 * i + 1] / 128.0f;
        raw[i] = i_sample + q_sample * _Complex_I;
    }

    // Mix samples to desired frequency
    nco_crcf_mix_block_down(nco, raw, mixed, num_samples);

    // Filter and decimate
    firdecim_crcf_execute_block(firdec, mixed, num_samples / 10, decimated);

    // Demodulate Bluetooth signal
    for (unsigned int i = 0; i < num_samples / 10; i += 2)
    {
        unsigned int bit = cpfskdem_demodulate(demod, &decimated[i]);
        uint8_t outbit = (uint8_t)(bit & 1);
        bits[i / 2] = outbit;
    }

    // Procees bits
    btbb_packet *pkt = NULL;
    int offset = btbb_find_ac((char *)bits, num_samples / 20 - 100, LAP_ANY, 2, &pkt);
    if (offset >= 0 && pkt != NULL)
    {
        uint32_t lap = btbb_packet_get_lap(pkt);

        // Calculate power for each IQ sample and find maximum
        float max_power = 0.0f;
        for (unsigned int i = offset * 2; i < offset * 2 + 144; i++)
        {
            float power = crealf(decimated[i]) * crealf(decimated[i]) +
                          cimagf(decimated[i]) * cimagf(decimated[i]);
            if (power > max_power)
                max_power = power;
        }

        // Convert power to relative RSSI in dBr
        float rssi_dbr = 0.0f;
        if (max_power > 0.0f)
        {
            rssi_dbr = 10.0f * log10f(max_power);
        }
        printf("LAP: %06x, AC errors: %d, RSSI: %f dBr\n", lap, btbb_packet_get_ac_errors(pkt), rssi_dbr);
        fflush(stdout);

        btbb_packet_unref(pkt);
    }
    return 0;
}

int main()
{
    // Create NCO for frequency shifting
    nco = nco_crcf_create(LIQUID_NCO);
    float normalized_freq = 2.0f * M_PI * OFFSET_FREQ_HZ / SAMPLE_RATE;
    nco_crcf_set_frequency(nco, normalized_freq);

    // Create firdecim filter for decimation by 2
    unsigned int decimation_factor = 10; // Example decimation factor
    unsigned int filter_delay = 4;       // Filter delay in symbols
    float As = 60.0f;                    // Stopband attenuation (dB)
    firdec = firdecim_crcf_create_kaiser(decimation_factor, filter_delay, As);

    // Create Bluetooth BR/EDR demodulator
    unsigned int bps = 1;                // number of bits/symbol
    float h = 0.3f;                      // modulation index (h=1/2 for MSK)
    unsigned int k = 2;                  // filter samples/symbol
    unsigned int m = 3;                  // filter delay (symbols)
    float beta = 0.5f;                   // filter bandwidth-time product
    int filter_type = LIQUID_CPFSK_GMSK; // Gaussian filter
    demod = cpfskdem_create(bps, h, k, m, beta, filter_type);

    // HackRF setup and start RX
    int result;
    hackrf_device *device = NULL;
    // Connect to HackRF device
    result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        return EXIT_FAILURE;
    }

    // Configure HackRF device
    hackrf_config_t config = {
        .lo_freq_hz = LO_FREQ_HZ,
        .sample_rate = SAMPLE_RATE,
        .lna_gain = 32,
        .vga_gain = 32};

    result = hackrf_configure(device, &config);
    if (result != HACKRF_SUCCESS)
    {
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, bredr_channel_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        return EXIT_FAILURE;
    }
    printf("Receiving Bluetooth BR/EDR packets...\n");
    // Run indefinitely
    while (1)
    {
        sleep(1);
    }

    return 0;
}