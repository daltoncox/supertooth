#include <liquid/liquid.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <complex.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "wav.h"

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Usage: %s <wavfile>\n", argv[0]);
        return 1;
    }

    // Read input WAV file
    float complex *input_samples = NULL;
    size_t num_input_samples = 0;
    unsigned int sample_rate = 0;

    int result = wav2cf(argv[1], &input_samples, &num_input_samples, &sample_rate);
    if (result != 0)
    {
        fprintf(stderr, "Failed to read WAV file (error code: %d)\n", result);
        return 1;
    }

    printf("Read %zu samples at %u Hz from '%s'\n", num_input_samples, sample_rate, argv[1]);

    // Calculate number of channels based on sample rate (1 channel per MHz)
    unsigned int NUM_CHANNELS = sample_rate / 1000000;
    if (NUM_CHANNELS < 2 || NUM_CHANNELS % 2 != 0)
    {
        fprintf(stderr, "Sample rate incorrect: need at least 2 MHz, got %u Hz\n", sample_rate);
        free(input_samples);
        return 1;
    }
    printf("Using %u channels (sample_rate / 1 MHz)\n", NUM_CHANNELS);

    // Polyphase channelizer2 setup
    unsigned int m = 4; // filter delay per channel
    float As = 60.0f;   // Stopband attenuation
    firpfbch2_crcf channelizer = firpfbch2_crcf_create_kaiser(
        LIQUID_ANALYZER,
        NUM_CHANNELS,
        m,
        As);

    firpfbch2_crcf_print(channelizer);

    // Buffers: OVERSAMPLING output per channel per input
    float complex x_buf[NUM_CHANNELS / 2];
    float complex y[NUM_CHANNELS];

    // Allocate output buffers for each channel
    float complex **channel_buffers = (float complex **)malloc(NUM_CHANNELS * sizeof(float complex *));
    size_t *channel_sample_counts = (size_t *)calloc(NUM_CHANNELS, sizeof(size_t));
    size_t max_output_samples = num_input_samples * 2; // Conservative estimate

    for (unsigned c = 0; c < NUM_CHANNELS; c++)
    {
        channel_buffers[c] = (float complex *)malloc(max_output_samples * sizeof(float complex));
        if (!channel_buffers[c])
        {
            fprintf(stderr, "Failed to allocate buffer for channel %u\n", c);
            exit(1);
        }
    }

    // Process input samples
    for (size_t i = 0; i < num_input_samples; i += NUM_CHANNELS / 2)
    {
        // Fill input buffer
        for (unsigned j = 0; j < NUM_CHANNELS / 2 && (i + j) < num_input_samples; j++)
        {
            x_buf[j] = input_samples[i + j];
        }

        // Run polyphase analyzer: produce outputs per channel
        firpfbch2_crcf_execute(channelizer, x_buf, y);

        // Store output for each channel
        for (unsigned c = 0; c < NUM_CHANNELS; c++)
        {
            channel_buffers[c][channel_sample_counts[c]++] = y[c];
        }
    }

    // Create channels directory if it doesn't exist
    mkdir("channels", 0755);

    // Write each channel to its own WAV file
    char fname[64];
    unsigned int channel_rate = sample_rate / NUM_CHANNELS * 2; // Adjust for channelizer output rate

    for (unsigned c = 0; c < NUM_CHANNELS; c++)
    {
        snprintf(fname, sizeof(fname), "channels/channel%u.wav", c);
        result = cf2wav(channel_buffers[c], channel_sample_counts[c], channel_rate, fname);
        if (result != 0)
        {
            fprintf(stderr, "Failed to write channel %u WAV file (error code: %d)\n", c, result);
        }
        else
        {
            printf("Wrote %zu samples to '%s'\n", channel_sample_counts[c], fname);
        }
        free(channel_buffers[c]);
    }

    free(channel_buffers);
    free(channel_sample_counts);
    free(input_samples);
    firpfbch2_crcf_destroy(channelizer);
    return 0;
}
