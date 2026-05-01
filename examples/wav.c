#include "wav.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <complex.h>

#pragma pack(push, 1)
typedef struct
{
    char riff_id[4]; // "RIFF"
    uint32_t riff_size;
    char wave_id[4]; // "WAVE"

    char fmt_id[4];        // "fmt "
    uint32_t fmt_size;     // 16 for PCM
    uint16_t audio_format; // 1 = PCM
    uint16_t num_channels; // 2 = stereo (left=real, right=imaginary)
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;

    char data_id[4]; // "data"
    uint32_t data_size;
} wav_header_t;
#pragma pack(pop)

/* ---- internal helpers ---- */

static void fill_wav_header(wav_header_t *h,
                            uint32_t sample_rate,
                            uint32_t num_samples)
{
    memcpy(h->riff_id, "RIFF", 4);
    memcpy(h->wave_id, "WAVE", 4);
    memcpy(h->fmt_id, "fmt ", 4);
    memcpy(h->data_id, "data", 4);

    h->fmt_size = 16;
    h->audio_format = 1; // PCM
    h->num_channels = 2; // stereo: left=real, right=imaginary
    h->sample_rate = sample_rate;
    /* Use 16-bit integer PCM samples */
    h->bits_per_sample = 16;
    h->block_align = h->num_channels * (h->bits_per_sample / 8);
    h->byte_rate = h->sample_rate * h->block_align;

    h->data_size = num_samples * h->block_align;
    h->riff_size = 4 + (8 + h->fmt_size) + (8 + h->data_size);
}

/* ---- public: complex -> WAV ---- */

int cf2wav(const float complex *buf,
           size_t num_samples,
           unsigned int sample_rate_hz,
           const char *filename)
{
    if (!buf || !filename || num_samples == 0 || sample_rate_hz == 0)
    {
        return -1;
    }

    FILE *f = fopen(filename, "wb");
    if (!f)
    {
        return -2;
    }

    /* Allocate stereo buffer: 2 samples per complex number */
    int16_t *pcm = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));
    if (!pcm)
    {
        fclose(f);
        return -3;
    }

    /* Convert normalized float [-1, 1] to signed 16-bit int range */
    for (size_t i = 0; i < num_samples; ++i)
    {
        float real = crealf(buf[i]) * 32767.0f;
        float imag = cimagf(buf[i]) * 32767.0f;

        pcm[i * 2 + 0] = (int16_t)lrintf(real); // left = real
        pcm[i * 2 + 1] = (int16_t)lrintf(imag); // right = imaginary
    }

    wav_header_t h;
    fill_wav_header(&h, (uint32_t)sample_rate_hz, (uint32_t)num_samples);

    if (fwrite(&h, sizeof(h), 1, f) != 1)
    {
        free(pcm);
        fclose(f);
        return -4;
    }

    /* Write stereo interleaved data */
    if (fwrite(pcm, sizeof(int16_t), num_samples * 2, f) != num_samples * 2)
    {
        free(pcm);
        fclose(f);
        return -5;
    }

    free(pcm);
    fclose(f);
    return 0;
}

/* ---- public: WAV -> complex ---- */

int wav2cf(const char *filename,
           float complex **out_buf,
           size_t *out_num_samples,
           unsigned int *out_sample_rate)
{
    if (!filename || !out_buf || !out_num_samples || !out_sample_rate)
    {
        return -1;
    }

    *out_buf = NULL;
    *out_num_samples = 0;
    *out_sample_rate = 0;

    FILE *f = fopen(filename, "rb");
    if (!f)
    {
        return -2;
    }

    wav_header_t h;
    if (fread(&h, sizeof(h), 1, f) != 1)
    {
        fclose(f);
        return -3;
    }

    /* Basic validation for simple PCM mono 16-bit WAV. */
    if (memcmp(h.riff_id, "RIFF", 4) != 0 ||
        memcmp(h.wave_id, "WAVE", 4) != 0)
    {
        fclose(f);
        return -4; // not a RIFF/WAVE file
    }

    if (memcmp(h.fmt_id, "fmt ", 4) != 0 ||
        h.audio_format != 1 || // 1 = PCM
        h.num_channels != 2 || // stereo: left=real, right=imaginary
        !(h.bits_per_sample == 16 || h.bits_per_sample == 32))
    {
        fclose(f);
        return -5; // unsupported format
    }

    if (memcmp(h.data_id, "data", 4) != 0)
    {
        fclose(f);
        return -6; // unsupported layout (no simple "data" chunk)
    }

    uint32_t num_samples = h.data_size / h.block_align;
    if (num_samples == 0)
    {
        fclose(f);
        return -7;
    }

    float _Complex *buf = (float _Complex *)malloc(num_samples * sizeof(float _Complex));
    if (!buf)
    {
        fclose(f);
        return -10;
    }

    if (h.bits_per_sample == 16)
    {
        /* Read stereo interleaved: 2 samples per complex number */
        int16_t *pcm16 = (int16_t *)malloc(num_samples * 2 * sizeof(int16_t));
        if (!pcm16)
        {
            free(buf);
            fclose(f);
            return -8;
        }

        size_t read_count = fread(pcm16, sizeof(int16_t), num_samples * 2, f);
        fclose(f);

        if (read_count != num_samples * 2)
        {
            free(pcm16);
            free(buf);
            return -9;
        }

        const float inv_scale = 1.0f / 32768.0f;
        for (uint32_t i = 0; i < num_samples; ++i)
        {
            float real = (float)pcm16[i * 2 + 0] * inv_scale; // left channel
            float imag = (float)pcm16[i * 2 + 1] * inv_scale; // right channel
            buf[i] = real + imag * I;
        }

        free(pcm16);
    }
    else /* 32-bit */
    {
        /* Read stereo interleaved: 2 samples per complex number */
        int32_t *pcm32 = (int32_t *)malloc(num_samples * 2 * sizeof(int32_t));
        if (!pcm32)
        {
            free(buf);
            fclose(f);
            return -8;
        }

        size_t read_count = fread(pcm32, sizeof(int32_t), num_samples * 2, f);
        fclose(f);

        if (read_count != num_samples * 2)
        {
            free(pcm32);
            free(buf);
            return -9;
        }

        /* scale down from signed 32-bit integer range */
        const float inv_scale = 1.0f / 2147483648.0f;
        for (uint32_t i = 0; i < num_samples; ++i)
        {
            float real = (float)pcm32[i * 2 + 0] * inv_scale; // left channel
            float imag = (float)pcm32[i * 2 + 1] * inv_scale; // right channel
            buf[i] = real + imag * I;
        }

        free(pcm32);
    }
    *out_buf = buf;
    *out_num_samples = num_samples;
    *out_sample_rate = h.sample_rate;

    return 0;
}
