/**
 * @file test_wav_io.c
 * @brief Round-trip tests for the SDR++-compatible WAV IQ reader/writer.
 *
 * Covers: all four sample types, auxi center-frequency carriage, auxi-less
 * files (SDR++ style: center reports 0), unknown-chunk skipping, and the
 * self-describing recording filename build/parse pair.
 */

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wav.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                              \
    do                                                                                 \
    {                                                                                  \
        if (!(cond))                                                                   \
        {                                                                              \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);   \
            g_failures++;                                                              \
        }                                                                              \
    } while (0)

static void make_tone(float complex *out, size_t n)
{
    for (size_t i = 0u; i < n; i++)
    {
        float t = (float)i / (float)n;
        out[i] = (0.5f * sinf(6.2831853f * t)) +
                 (0.25f * cosf(6.2831853f * 2.0f * t)) * _Complex_I;
    }
}

static void round_trip(wav_sample_type_t type, const char *path)
{
    static float complex tx[4096];
    static float complex rx[4096];
    wav_writer_t w;
    wav_info_t info;
    FILE *fp = NULL;

    make_tone(tx, 4096u);

    TEST_ASSERT(wav_write_open(path, 2000000u, type, 2406500000ull, &w) == 0);
    TEST_ASSERT(wav_write_frames(&w, tx, 4096u) == 0);
    TEST_ASSERT(wav_write_close(&w) == 0);

    TEST_ASSERT(wav_read_open(path, &info, &fp) == 0);
    TEST_ASSERT(info.sample_rate_hz == 2000000u);
    TEST_ASSERT(info.num_channels == 2u);
    TEST_ASSERT(info.sample_type == type);
    TEST_ASSERT(info.center_freq_hz == 2406500000ull);
    TEST_ASSERT(info.total_frames == 4096u);

    {
        size_t got = 4096u;
        TEST_ASSERT(wav_read_frames(fp, &info, rx, &got) == 0);
        TEST_ASSERT(got == 4096u);
        /* Quantization tolerance: u8 is the coarsest (~1/127 step). */
        float tol = (type == WAV_SAMP_U8) ? 0.02f : 0.002f;
        for (size_t i = 0u; i < got; i++)
        {
            TEST_ASSERT(fabsf(crealf(rx[i]) - crealf(tx[i])) < tol);
            TEST_ASSERT(fabsf(cimagf(rx[i]) - cimagf(tx[i])) < tol);
        }
        got = 4096u;
        TEST_ASSERT(wav_read_frames(fp, &info, rx, &got) == 0);
        TEST_ASSERT(got == 0u); /* EOF */
    }
    wav_read_close(fp);
    unlink(path);
}

/* Minimal fmt+data WAV with no auxi chunk (what SDR++ writes): center must
 * report 0 and samples must still decode. */
static void no_auxi_file(const char *path)
{
    FILE *fp = fopen(path, "wb");
    TEST_ASSERT(fp != NULL);
    if (!fp)
        return;
    {
        /* RIFF/WAVE + fmt(16) + data(2 frames stereo s16). */
        uint8_t h[44] = {
            'R','I','F','F', 52,0,0,0, 'W','A','V','E',
            'f','m','t',' ', 16,0,0,0, 1,0, 2,0,
            0x44,0xAC,0,0, 0x10,0xB1,0x2C,0, 4,0, 16,0,
            'd','a','t','a', 8,0,0,0,
        };
        /* frame0: I=32767 Q=0; frame1: I=0 Q=-32768 */
        uint8_t d[8] = {0xFF,0x7F, 0,0, 0,0, 0,0x80};
        TEST_ASSERT(fwrite(h, 1, sizeof(h), fp) == sizeof(h));
        TEST_ASSERT(fwrite(d, 1, sizeof(d), fp) == sizeof(d));
        fclose(fp);
    }

    {
        wav_info_t info;
        FILE *rfp = NULL;
        float complex out[4];
        size_t got = 4u;
        TEST_ASSERT(wav_read_open(path, &info, &rfp) == 0);
        TEST_ASSERT(info.sample_type == WAV_SAMP_S16);
        TEST_ASSERT(info.center_freq_hz == 0u);
        TEST_ASSERT(info.total_frames == 2u);
        TEST_ASSERT(wav_read_frames(rfp, &info, out, &got) == 0);
        TEST_ASSERT(got == 2u);
        TEST_ASSERT(fabsf(crealf(out[0]) - 1.0f) < 0.001f);
        TEST_ASSERT(fabsf(cimagf(out[0])) < 0.001f);
        TEST_ASSERT(fabsf(cimagf(out[1]) + 1.0f) < 0.01f);
        wav_read_close(rfp);
    }
    unlink(path);
}

/* Write a minimal hand-crafted WAV: fmt(codec, ch, rate, bits) + optional
 * extra chunks + 8-byte stereo s16 data payload. */
static void write_mini_wav(const char *path, uint16_t codec, uint16_t ch,
                           uint32_t rate, uint16_t bits,
                           const uint8_t *pre_chunks, size_t pre_len,
                           uint64_t auxi_freq, int with_auxi)
{
    FILE *fp = fopen(path, "wb");
    TEST_ASSERT(fp != NULL);
    if (!fp)
        return;
    /* RIFF header (size patched loosely; the reader scans chunks). */
    uint8_t riff[12] = {'R','I','F','F', 0xFF,0xFF,0xFF,0x7F, 'W','A','V','E'};
    TEST_ASSERT(fwrite(riff, 1, sizeof(riff), fp) == sizeof(riff));
    if (pre_len)
        TEST_ASSERT(fwrite(pre_chunks, 1, pre_len, fp) == pre_len);
    {
        uint8_t fh[8] = {'f','m','t',' ', 16,0,0,0};
        uint8_t fmt[16];
        memset(fmt, 0, sizeof(fmt));
        fmt[0] = (uint8_t)(codec & 0xFFu);
        fmt[1] = (uint8_t)((codec >> 8) & 0xFFu);
        fmt[2] = (uint8_t)(ch & 0xFFu);
        fmt[3] = (uint8_t)((ch >> 8) & 0xFFu);
        fmt[4] = (uint8_t)(rate & 0xFFu);
        fmt[5] = (uint8_t)((rate >> 8) & 0xFFu);
        fmt[6] = (uint8_t)((rate >> 16) & 0xFFu);
        fmt[7] = (uint8_t)((rate >> 24) & 0xFFu);
        {
            uint32_t br = rate * ch * (bits / 8u);
            fmt[8] = (uint8_t)(br & 0xFFu);
            fmt[9] = (uint8_t)((br >> 8) & 0xFFu);
            fmt[10] = (uint8_t)((br >> 16) & 0xFFu);
            fmt[11] = (uint8_t)((br >> 24) & 0xFFu);
        }
        fmt[12] = (uint8_t)((ch * (bits / 8u)) & 0xFFu);
        fmt[13] = (uint8_t)(((ch * (bits / 8u)) >> 8) & 0xFFu);
        fmt[14] = (uint8_t)(bits & 0xFFu);
        fmt[15] = (uint8_t)((bits >> 8) & 0xFFu);
        TEST_ASSERT(fwrite(fh, 1, sizeof(fh), fp) == sizeof(fh));
        TEST_ASSERT(fwrite(fmt, 1, sizeof(fmt), fp) == sizeof(fmt));
    }
    if (with_auxi)
    {
        uint8_t ah[8] = {'a','u','x','i', 8,0,0,0};
        uint8_t ab[8];
        for (unsigned int i = 0u; i < 8u; i++)
            ab[i] = (uint8_t)((auxi_freq >> (8u * i)) & 0xFFu);
        TEST_ASSERT(fwrite(ah, 1, sizeof(ah), fp) == sizeof(ah));
        TEST_ASSERT(fwrite(ab, 1, sizeof(ab), fp) == sizeof(ab));
    }
    {
        uint8_t dh[8] = {'d','a','t','a', 8,0,0,0};
        uint8_t d[8] = {0xFF,0x7F, 0,0, 0,0, 0,0x80};
        TEST_ASSERT(fwrite(dh, 1, sizeof(dh), fp) == sizeof(dh));
        TEST_ASSERT(fwrite(d, 1, sizeof(d), fp) == sizeof(d));
    }
    fclose(fp);
}

static void test_reject_paths(const char *path)
{
    wav_info_t info;
    FILE *fp = NULL;

    /* Mono files are rejected (stereo required). */
    write_mini_wav(path, 1u, 1u, 2000000u, 16u, NULL, 0u, 0u, 0);
    TEST_ASSERT(wav_read_open(path, &info, &fp) != 0);

    /* Unknown codec (0x11) is rejected. */
    write_mini_wav(path, 0x11u, 2u, 2000000u, 16u, NULL, 0u, 0u, 0);
    TEST_ASSERT(wav_read_open(path, &info, &fp) != 0);

    /* Truncated garbage with no fmt/data is rejected. */
    {
        FILE *g = fopen(path, "wb");
        TEST_ASSERT(g != NULL);
        if (g)
        {
            TEST_ASSERT(fwrite("RIFFxxxx", 1, 8, g) == 8u);
            fclose(g);
        }
    }
    TEST_ASSERT(wav_read_open(path, &info, &fp) != 0);
    unlink(path);
}

static void test_chunk_skipping(const char *path)
{
    /* Unknown JUNK chunk (odd size 5 + pad) before fmt must be skipped. */
    uint8_t junk[8 + 5 + 1];
    memcpy(junk, "JUNK", 4);
    junk[4] = 5; junk[5] = 0; junk[6] = 0; junk[7] = 0;
    memset(junk + 8, 0xA5, 6u);
    write_mini_wav(path, 1u, 2u, 2000000u, 16u, junk, sizeof(junk),
                   2406500000ull, 1);

    wav_info_t info;
    FILE *fp = NULL;
    float complex out[4];
    size_t got = 4u;
    TEST_ASSERT(wav_read_open(path, &info, &fp) == 0);
    TEST_ASSERT(info.center_freq_hz == 2406500000ull);
    TEST_ASSERT(info.total_frames == 2u);
    TEST_ASSERT(wav_read_frames(fp, &info, out, &got) == 0);
    TEST_ASSERT(got == 2u);
    wav_read_close(fp);

    /* Implausible auxi content (100 Hz) is ignored, center reports 0. */
    write_mini_wav(path, 1u, 2u, 2000000u, 16u, NULL, 0u, 100u, 1);
    TEST_ASSERT(wav_read_open(path, &info, &fp) == 0);
    TEST_ASSERT(info.center_freq_hz == 0u);
    wav_read_close(fp);
    unlink(path);
}

int main(void)
{
    char tmp[256];

    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_s16_%d.wav", (int)getpid());
    round_trip(WAV_SAMP_S16, tmp);
    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_f32_%d.wav", (int)getpid());
    round_trip(WAV_SAMP_F32, tmp);
    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_u8_%d.wav", (int)getpid());
    round_trip(WAV_SAMP_U8, tmp);
    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_s32_%d.wav", (int)getpid());
    round_trip(WAV_SAMP_S32, tmp);

    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_noauxi_%d.wav", (int)getpid());
    no_auxi_file(tmp);

    /* Self-describing filename round trip. */
    {
        char name[256];
        uint64_t lo = 0u;
        uint32_t rate = 0u;
        TEST_ASSERT(wav_build_recording_filename(2406500000ull, 20000000u,
                                                 name, sizeof(name)) == 0);
        TEST_ASSERT(strstr(name, "supertooth_baseband_2406500000Hz") != NULL);
        TEST_ASSERT(strstr(name, "20Msps") != NULL);
        TEST_ASSERT(wav_parse_recording_filename(name, &lo, &rate) == 2);
        TEST_ASSERT(lo == 2406500000ull);
        TEST_ASSERT(rate == 20000000u);
    }

    /* Garbage names parse to zero fields without touching outputs. */
    {
        uint64_t lo = 0u;
        uint32_t rate = 0u;
        TEST_ASSERT(wav_parse_recording_filename("capture.wav", &lo,
                                                 &rate) == 0);
        TEST_ASSERT(lo == 0u && rate == 0u);
    }

    /* Bad input rejected. */
    {
        wav_info_t info;
        FILE *fp = NULL;
        TEST_ASSERT(wav_read_open("/nonexistent/path.wav", &info, &fp) != 0);
        TEST_ASSERT(wav_read_open(NULL, &info, &fp) != 0);
    }

    /* Tiny filename buffer is rejected. */
    {
        char tiny[8];
        TEST_ASSERT(wav_build_recording_filename(2406500000ull, 20000000u,
                                                 tiny, sizeof(tiny)) != 0);
        TEST_ASSERT(wav_build_recording_filename(2406500000ull, 20000000u,
                                                 NULL, 0u) != 0);
    }

    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_reject_%d.wav", (int)getpid());
    test_reject_paths(tmp);
    snprintf(tmp, sizeof(tmp), "/tmp/test_wav_chunks_%d.wav", (int)getpid());
    test_chunk_skipping(tmp);

    if (g_failures)
    {
        fprintf(stderr, "test_wav_io: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_wav_io: ok\n");
    return 0;
}
