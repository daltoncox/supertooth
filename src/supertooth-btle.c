#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <sys/time.h>
#include "hackrf.h"
#include "ble_phy.h"

// BTLE Advertising Channel 37 constants
#define BTLE_CH37_INDEX 37         // BLE channel index for 2.402 GHz
#define BTLE_CH37_FREQ 2402e6      // Channel 37 frequency (2.402 GHz)
#define ADVERTISING_AA 0x8E89BED6U // BTLE advertising access address

#define BTLE_SYMBOL_RATE 1e6 // 1 Mbps BTLE symbol rate
#define SAMPLES_PER_SYMBOL 2 // 2 MHz / 1 Mbps = 2 samples/symbol

// HackRF parameters
#define LNA_GAIN 32        // LNA gain
#define VGA_GAIN 32        // VGA gain
#define BUFFER_SIZE 262144 // HackRF transfer buffer size (bytes)
#define SAMPLE_RATE 2e6    // 2 MHz sample rate

// BTLE demodulation objects
static cpfskdem demod;

// BLE PHY processor
static ble_channel_processor_t ble_proc;

// Processing buffers
static float complex raw[BUFFER_SIZE / 2]; // complex samples

// RSSI tracking state
static unsigned long long total_samples = 0;
static long long pkt_start_abs = -1;
static ble_status_t prev_status = BLE_SEARCHING;

// --- HackRF RX Callback ------------------------------------------------------

int btle_rx_cb(hackrf_transfer *transfer)
{
    // HackRF provides interleaved I/Q samples as int8_t.
    int8_t *samples = (int8_t *)transfer->buffer;
    unsigned int num_iq_bytes = transfer->valid_length;
    unsigned int num_complex = num_iq_bytes / 2; // I/Q pair per complex sample

    if (num_complex > (BUFFER_SIZE / 2))
        num_complex = BUFFER_SIZE / 2;

    // Record the absolute sample index of this buffer's first sample.
    unsigned long long buf_start = total_samples;
    total_samples += num_complex;

    // Convert int8_t samples to complex float range [-1.0, 1.0].
    for (unsigned int i = 0; i < num_complex; i++)
    {
        float i_sample = samples[2 * i] / 128.0f;
        float q_sample = samples[2 * i + 1] / 128.0f;
        raw[i] = i_sample + q_sample * _Complex_I;
    }

    // Demodulate BTLE signal and push bits to the BLE PHY processor
    unsigned int num_bits = num_complex / SAMPLES_PER_SYMBOL;
    for (unsigned int s = 0; s < num_bits; s++)
    {
        unsigned int sample_index = s * SAMPLES_PER_SYMBOL;
        unsigned int sym = cpfskdem_demodulate(demod, &raw[sample_index]);
        uint8_t bit = (uint8_t)(sym & 0x01);

        // Push the bit to the BLE PHY processor
        ble_status_t status = ble_push_bit(&ble_proc, bit);

        // Record the sample index when preamble+AA is first matched.
        if (prev_status == BLE_SEARCHING && status == BLE_COLLECTING)
            pkt_start_abs = (long long)(buf_start + sample_index);
        prev_status = status;

        if (status == BLE_VALID_PACKET)
        {
            // Compute RSSI: max instantaneous power over the packet samples.
            float max_power = 0.0f;
            if (pkt_start_abs >= 0)
            {
                long long rel_start = pkt_start_abs - (long long)buf_start;
                unsigned int i_start = (rel_start < 0) ? 0 : (unsigned int)rel_start;
                unsigned int i_end = (s + 1) * SAMPLES_PER_SYMBOL;
                for (unsigned int i = i_start; i < i_end && i < num_complex; i++)
                {
                    float power = crealf(raw[i]) * crealf(raw[i]) +
                                  cimagf(raw[i]) * cimagf(raw[i]);
                    if (power > max_power)
                        max_power = power;
                }
            }
            float rssi_dbm = (max_power > 0.0f) ? 10.0f * log10f(max_power) - 30.0f : 0.0f;

            // Retrieve and print the completed packet
            ble_packet_t pkt;
            if (ble_get_packet(&ble_proc, &pkt) == 0)
            {
                printf("\n");
                ble_print_packet(&pkt);
                printf("RSSI     : %.1f dBm\n", rssi_dbm);
                fflush(stdout);
            }
        }
    }

    return 0;
}

// --- Main --------------------------------------------------------------------

int main(void)
{
    printf("BTLE Advertising Packet Detector - Channel 37\n");
    printf("==============================================\n");

    // Initialize BLE PHY processor for channel 37
    ble_processor_init(&ble_proc, BTLE_CH37_INDEX);

    // Create BTLE GFSK demodulator (1 bit/symbol, h ~ 0.5, etc.).
    unsigned int bps = 1;                                         // bits/symbol
    float h = 0.5f;                                               // modulation index
    unsigned int k = SAMPLES_PER_SYMBOL;                          // samples/symbol
    unsigned int m = 3;                                           // filter delay (symbols)
    float BT = 0.5f;                                              // BT product
    demod = cpfskdem_create(bps, h, k, m, BT, LIQUID_CPFSK_GMSK); // generic CPFSK GMSK

    if (!demod)
    {
        fprintf(stderr, "Error: Failed to create cpfskdem object.\n");
        return EXIT_FAILURE;
    }

    // HackRF setup and start RX.
    int result;
    hackrf_device *device = NULL;

    result = hackrf_connect(&device);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_connect() failed: %s\n", hackrf_error_name(result));
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    hackrf_config_t config = {
        .lo_freq_hz = BTLE_CH37_FREQ,
        .sample_rate = SAMPLE_RATE,
        .lna_gain = LNA_GAIN,
        .vga_gain = VGA_GAIN,
    };

    result = hackrf_configure(device, &config);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_configure() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    result = hackrf_start_rx(device, btle_rx_cb, NULL);
    if (result != HACKRF_SUCCESS)
    {
        fprintf(stderr, "hackrf_start_rx() failed: %s\n", hackrf_error_name(result));
        hackrf_disconnect(device);
        cpfskdem_destroy(demod);
        return EXIT_FAILURE;
    }

    printf("Monitoring BTLE Channel 37 (2.402 GHz) for advertising packets...\n");
    printf("Press Ctrl+C to exit\n\n");

    // Run indefinitely; in a real application, handle signals for graceful exit.
    while (1)
    {
        sleep(1);
    }

    // Cleanup (not reached in this simple example).
    hackrf_stop_rx(device);
    hackrf_disconnect(device);
    cpfskdem_destroy(demod);

    return 0;
}
