#include <liquid/liquid.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <math.h>
#include "wav.h"

int main(int argc, char *argv[])
{
    // -------------------- Parse args & read WAV file --------------------
    // Check command line arguments
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <wav_file>\n", argv[0]);
        return 1;
    }

    // Read input WAV file
    float complex *input_samples = NULL;
    size_t num_input_samples = 0;
    unsigned int sample_rate = 0;

    const char *wav_file = argv[1];

    int result = wav2cf(wav_file, &input_samples, &num_input_samples, &sample_rate);
    if (result != 0)
    {
        fprintf(stderr, "Failed to read WAV file '%s' (error code: %d)\n", wav_file, result);
        return 1;
    }

    printf("Successfully loaded %zu samples at %u Hz from '%s'\n",
           num_input_samples, sample_rate, wav_file);

    // Print first few samples as a test
    printf("\nFirst 3 samples:\n");
    for (size_t i = 0; i < 3 && i < num_input_samples; i++)
    {
        printf("  [%zu]: %.6f + %.6fi\n", i,
               crealf(input_samples[i]),
               cimagf(input_samples[i]));
    }

    // -------------------- Mix data to desired frequency --------------------

    float complex *mix_out = malloc(num_input_samples * sizeof(float complex));
    if (mix_out == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for output buffer\n");
        free(input_samples);
        return 1;
    }

    // Desired frequency shift (Hz)
    float freq_shift_hz = 0e6f; // Example: shift up

    // Create NCO for frequency shifting
    nco_crcf nco = nco_crcf_create(LIQUID_NCO);
    float normalized_freq = 2.0f * M_PI * freq_shift_hz / (float)sample_rate;
    nco_crcf_set_frequency(nco, normalized_freq);

    nco_crcf_mix_block_down(nco, input_samples, mix_out, num_input_samples);

    printf("Mixed signal by %.2f Hz\n", freq_shift_hz);

    // Write Mixed Data
    const char *output_wav_file = "../data/mixed_output.wav";
    cf2wav(mix_out, num_input_samples, sample_rate, output_wav_file);

    // -------------------- Filter and Decimate --------------------

    unsigned int decimation_factor = 10; // Example decimation factor
    unsigned int filter_delay = 4;       // Filter delay in symbols
    float As = 60.0f;                    // Stopband attenuation (dB)

    firdecim_crcf firdec = firdecim_crcf_create_kaiser(decimation_factor, filter_delay, As);

    float complex *decimated_out = malloc((num_input_samples / decimation_factor + 1) * sizeof(float complex));
    if (decimated_out == NULL)
    {
        fprintf(stderr, "Failed to allocate memory for decimated output buffer\n");
        nco_crcf_destroy(nco);
        free(mix_out);
        free(input_samples);
        return 1;
    }

    firdecim_crcf_execute_block(firdec, mix_out, num_input_samples / decimation_factor, decimated_out);

    printf("Filtered and decimated signal by factor of %u\n", decimation_factor);

    // Write Decimated Data
    const char *decimated_wav_file = "../data/decimated_output.wav";
    cf2wav(decimated_out, num_input_samples / decimation_factor, sample_rate / decimation_factor, decimated_wav_file);

    // Clean up
    firdecim_crcf_destroy(firdec);
    nco_crcf_destroy(nco);
    free(mix_out);
    free(input_samples);

    return 0;
}
