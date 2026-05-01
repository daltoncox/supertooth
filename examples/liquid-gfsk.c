#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <liquid/liquid.h>
#include "wav.h"

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        fprintf(stderr, "Usage: %s <wav_file>\n", argv[0]);
        return 1;
    }

    const char *in_filename = argv[1];
    const char *out_filename = "bitstream.raw";

    // --- Read WAV file ---
    float complex *samples = NULL;
    size_t num_samples = 0;
    unsigned int sample_rate = 0;

    int result = wav2cf(in_filename, &samples, &num_samples, &sample_rate);
    if (result != 0)
    {
        fprintf(stderr, "Failed to read WAV file (error code: %d)\n", result);
        return 1;
    }

    printf("Read %zu samples at %u Hz from '%s'\n", num_samples, sample_rate, in_filename);

    // --- Initialize GFSK Demodulator for Bluetooth Classic ---
    unsigned int k = 2; // samples/symbol (1 Msps, 1 Mbps, so k = 1)
    unsigned int m = 4; // GMSK filter delay (symbols)
    float BT = 0.5f;    // Bluetooth Classic BT
    cpfskdem demod = cpfskdem_create_gmsk(k, m, BT);
    if (!demod)
    {
        fprintf(stderr, "Error: Failed to create cpfskdem object.\n");
        free(samples);
        return 1;
    }

    // --- Open output bitstream file ---
    FILE *bitfile = fopen(out_filename, "wb");
    if (!bitfile)
    {
        perror("Failed to create output bitstream file");
        cpfskdem_destroy(demod);
        free(samples);
        return 1;
    }

    // --- Demodulate and write bits ---
    for (size_t i = 0; i < num_samples; i += k)
    {
        unsigned int bit = cpfskdem_demodulate(demod, &samples[i]);
        uint8_t outbit = (uint8_t)(bit & 1);
        fwrite(&outbit, 1, 1, bitfile);
    }

    fclose(bitfile);
    cpfskdem_destroy(demod);
    free(samples);

    printf("Bitstream extraction complete. Output in '%s'.\n", out_filename);
    return 0;
}
