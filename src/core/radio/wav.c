#include "wav.h"

#include "sample_dispatcher.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define WAV_CODEC_PCM 1u
#define WAV_CODEC_FLOAT 3u

/* Accept auxi center frequencies only in a plausible RF range so a foreign
 * auxi payload (SDR# variants, timestamps, …) never tunes us to garbage. */
#define WAV_AUXI_MIN_HZ 100000ull
#define WAV_AUXI_MAX_HZ 6000000000ull

static uint16_t read_u16le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint64_t read_u64le(const uint8_t *p)
{
    uint64_t lo = read_u32le(p);
    uint64_t hi = read_u32le(p + 4);
    return lo | (hi << 32);
}

static void write_u16le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void write_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void write_u64le(uint8_t *p, uint64_t v)
{
    write_u32le(p, (uint32_t)(v & 0xFFFFFFFFull));
    write_u32le(p + 4, (uint32_t)((v >> 32) & 0xFFFFFFFFull));
}

int wav_read_open(const char *path, wav_info_t *info, FILE **out_fp)
{
    uint8_t hdr[12];
    uint8_t chunk_hdr[8];
    FILE *fp = NULL;

    if (!path || !info || !out_fp)
        return -1;
    *out_fp = NULL;
    memset(info, 0, sizeof(*info));

    fp = fopen(path, "rb");
    if (!fp)
        return -1;

    if (fread(hdr, 1, sizeof(hdr), fp) != sizeof(hdr) ||
        memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0)
    {
        fclose(fp);
        return -1;
    }

    uint16_t codec = 0u, channels = 0u, bits = 0u;
    uint32_t rate = 0u;
    int have_fmt = 0;
    uint64_t auxi_freq = 0u;
    long data_off = 0L;
    uint64_t data_bytes = 0u;
    size_t bytes_per_frame = 0u;

    /* Walk chunks: fmt (required), data (required), auxi (optional), rest
     * skipped. Chunk payloads are padded to even sizes. */
    while (fread(chunk_hdr, 1, sizeof(chunk_hdr), fp) == sizeof(chunk_hdr))
    {
        uint32_t size = read_u32le(chunk_hdr + 4);
        long payload_off = ftell(fp);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0)
        {
            uint8_t fmt[40];
            size_t want = size < sizeof(fmt) ? size : sizeof(fmt);
            if (size < 16u || fread(fmt, 1, want, fp) != want)
            {
                fclose(fp);
                return -1;
            }
            codec = read_u16le(fmt + 0);
            channels = read_u16le(fmt + 2);
            rate = read_u32le(fmt + 4);
            bits = read_u16le(fmt + 14);
            have_fmt = 1;
        }
        else if (memcmp(chunk_hdr, "auxi", 4) == 0)
        {
            /* Our auxi payload is an 8-byte LE center freq in Hz; anything
             * else (other tools' auxi layouts) is validated by range and
             * ignored when implausible. */
            uint8_t aux[32];
            size_t want = size < sizeof(aux) ? size : sizeof(aux);
            if (want >= 8u && fread(aux, 1, want, fp) == want)
            {
                uint64_t candidate = read_u64le(aux);
                if (candidate >= WAV_AUXI_MIN_HZ && candidate <= WAV_AUXI_MAX_HZ)
                    auxi_freq = candidate;
            }
        }
        else if (memcmp(chunk_hdr, "data", 4) == 0)
        {
            data_off = payload_off;
            data_bytes = size;
            /* Keep scanning: a later auxi chunk must still be honored, and
             * the data payload is skipped via seek below. */
        }

        if (fseek(fp, payload_off + (long)(size + (size & 1u)), SEEK_SET) != 0)
        {
            /* Tolerate truncated or unfinalized files (e.g. a recording
             * killed before sizes were patched, where the data size still
             * reads all-ones): keep whatever chunks were found and let the
             * frame reader stop at real EOF. */
            if (data_off != 0L)
                break;
            fclose(fp);
            return -1;
        }
    }

    if (!have_fmt || data_off == 0L || channels != 2u || rate == 0u)
    {
        fclose(fp);
        return -1;
    }

    wav_sample_type_t st;
    if (codec == WAV_CODEC_PCM && bits == 8u)
        st = WAV_SAMP_U8;
    else if (codec == WAV_CODEC_PCM && bits == 16u)
        st = WAV_SAMP_S16;
    else if (codec == WAV_CODEC_PCM && bits == 32u)
        st = WAV_SAMP_S32;
    else if (codec == WAV_CODEC_FLOAT && bits == 32u)
        st = WAV_SAMP_F32;
    else
    {
        fclose(fp);
        return -1;
    }

    bytes_per_frame = 2u * (bits / 8u);
    if (bytes_per_frame == 0u)
    {
        fclose(fp);
        return -1;
    }
    /* Floor (don't reject): unfinalized recordings keep the 0xFFFFFFFF
     * placeholder size, which is rarely frame-aligned; the frame reader
     * stops at real EOF regardless. */
    if (data_bytes / bytes_per_frame == 0u)
    {
        fclose(fp);
        return -1;
    }

    info->sample_rate_hz = rate;
    info->num_channels = channels;
    info->sample_type = st;
    info->center_freq_hz = auxi_freq;
    info->total_frames = data_bytes / bytes_per_frame;
    info->data_offset = data_off;
    info->bytes_per_frame = bytes_per_frame;

    if (fseek(fp, data_off, SEEK_SET) != 0)
    {
        fclose(fp);
        return -1;
    }

    *out_fp = fp;
    return 0;
}

/* Decode one interleaved stereo frame at p into complex (I=left, Q=right). */
static float complex wav_decode_frame(const uint8_t *p, wav_sample_type_t st)
{
    float i = 0.0f, q = 0.0f;
    switch (st)
    {
    case WAV_SAMP_U8:
        i = ((float)p[0] - 128.0f) / 127.0f;
        q = ((float)p[1] - 128.0f) / 127.0f;
        break;
    case WAV_SAMP_S16:
    {
        int16_t vi = (int16_t)read_u16le(p);
        int16_t vq = (int16_t)read_u16le(p + 2);
        i = (float)vi / 32767.0f;
        q = (float)vq / 32767.0f;
        break;
    }
    case WAV_SAMP_S32:
    {
        int32_t vi = (int32_t)read_u32le(p);
        int32_t vq = (int32_t)read_u32le(p + 4);
        i = (float)vi / 2147483647.0f;
        q = (float)vq / 2147483647.0f;
        break;
    }
    case WAV_SAMP_F32:
    {
        float vi, vq;
        memcpy(&vi, p, sizeof(vi));
        memcpy(&vq, p + 4, sizeof(vq));
        i = vi;
        q = vq;
        break;
    }
    }
    return i + q * _Complex_I;
}

int wav_read_frames(FILE *fp, const wav_info_t *info,
                    float complex *out, size_t *inout_count)
{
    static uint8_t buf[32768u * 8u]; /* 32768 stereo f32 frames max */
    size_t want, got, frames;

    if (!fp || !info || !out || !inout_count)
        return -1;

    want = *inout_count;
    *inout_count = 0u;
    while (want > 0u)
    {
        size_t chunk = want > 32768u ? 32768u : want;
        size_t bytes = chunk * info->bytes_per_frame;
        got = fread(buf, 1, bytes, fp);
        frames = got / info->bytes_per_frame;
        for (size_t n = 0u; n < frames; n++)
            out[*inout_count + n] =
                wav_decode_frame(buf + n * info->bytes_per_frame,
                                 info->sample_type);
        *inout_count += frames;
        want -= frames;
        if (frames < chunk)
            break; /* EOF or short read */
        if (got != bytes && ferror(fp))
            return -1;
    }
    return 0;
}

void wav_read_close(FILE *fp)
{
    if (fp)
        fclose(fp);
}

static unsigned int wav_bits_for_type(wav_sample_type_t st)
{
    switch (st)
    {
    case WAV_SAMP_U8:
        return 8u;
    case WAV_SAMP_S16:
        return 16u;
    case WAV_SAMP_S32:
    case WAV_SAMP_F32:
        return 32u;
    }
    return 16u;
}

static int wav_codec_for_type(wav_sample_type_t st)
{
    return (st == WAV_SAMP_F32) ? WAV_CODEC_FLOAT : WAV_CODEC_PCM;
}

int wav_write_open(const char *path, uint32_t sample_rate_hz,
                   wav_sample_type_t sample_type, uint64_t center_freq_hz,
                   wav_writer_t *writer)
{
    uint8_t riff[12], fmt_hdr[8], fmt[16], auxi_hdr[8], auxi[8], data_hdr[8];
    unsigned int bits;
    FILE *fp;

    if (!path || sample_rate_hz == 0u || !writer)
        return -1;
    memset(writer, 0, sizeof(*writer));

    fp = fopen(path, "wb");
    if (!fp)
        return -1;

    bits = wav_bits_for_type(sample_type);

    memcpy(riff, "RIFF", 4);
    write_u32le(riff + 4, 0xFFFFFFFFu); /* patched on close */
    memcpy(riff + 8, "WAVE", 4);

    memcpy(fmt_hdr, "fmt ", 4);
    write_u32le(fmt_hdr + 4, 16u);
    write_u16le(fmt + 0, (uint16_t)wav_codec_for_type(sample_type));
    write_u16le(fmt + 2, 2u);
    write_u32le(fmt + 4, sample_rate_hz);
    write_u32le(fmt + 8, sample_rate_hz * 2u * (bits / 8u));
    write_u16le(fmt + 12, (uint16_t)(2u * (bits / 8u)));
    write_u16le(fmt + 14, (uint16_t)bits);

    memcpy(auxi_hdr, "auxi", 4);
    write_u32le(auxi_hdr + 4, 8u);
    write_u64le(auxi, center_freq_hz);

    memcpy(data_hdr, "data", 4);
    write_u32le(data_hdr + 4, 0xFFFFFFFFu); /* patched on close */

    if (fwrite(riff, 1, sizeof(riff), fp) != sizeof(riff) ||
        fwrite(fmt_hdr, 1, sizeof(fmt_hdr), fp) != sizeof(fmt_hdr) ||
        fwrite(fmt, 1, sizeof(fmt), fp) != sizeof(fmt) ||
        fwrite(auxi_hdr, 1, sizeof(auxi_hdr), fp) != sizeof(auxi_hdr) ||
        fwrite(auxi, 1, sizeof(auxi), fp) != sizeof(auxi) ||
        fwrite(data_hdr, 1, sizeof(data_hdr), fp) != sizeof(data_hdr))
    {
        fclose(fp);
        return -1;
    }

    writer->fp = fp;
    writer->sample_rate_hz = sample_rate_hz;
    writer->sample_type = sample_type;
    writer->center_freq_hz = center_freq_hz;
    writer->bytes_per_frame = 2u * (bits / 8u);
    writer->riff_size_off = 4L;
    writer->data_size_off = (long)(sizeof(riff) + sizeof(fmt_hdr) +
                                   sizeof(fmt) + sizeof(auxi_hdr) +
                                   sizeof(auxi) + 4L);
    writer->frames_written = 0u;
    return 0;
}

static float wav_clamp(float v)
{
    if (v > 1.0f)
        return 1.0f;
    if (v < -1.0f)
        return -1.0f;
    return v;
}

int wav_write_frames(wav_writer_t *writer, const float complex *samples,
                     size_t count)
{
    /* Stack scratch for one dispatcher block max, at the widest depth
     * (stereo float32). Heap-free steady state. */
    static uint8_t buf[SAMPLE_BLOCK_SAMPLE_CAPACITY * 8u];
    uint8_t *p = buf;

    if (!writer || !writer->fp || (!samples && count > 0u))
        return -1;

    for (size_t n = 0u; n < count; n++)
    {
        float i = wav_clamp(crealf(samples[n]));
        float q = wav_clamp(cimagf(samples[n]));
        switch (writer->sample_type)
        {
        case WAV_SAMP_U8:
            /* Inverse of the SDR++ reader mapping. */
            p[0] = (uint8_t)(i * 127.0f + 128.0f);
            p[1] = (uint8_t)(q * 127.0f + 128.0f);
            p += 2;
            break;
        case WAV_SAMP_S16:
        {
            int16_t vi = (int16_t)(i * 32767.0f);
            int16_t vq = (int16_t)(q * 32767.0f);
            write_u16le(p + 0, (uint16_t)vi);
            write_u16le(p + 2, (uint16_t)vq);
            p += 4;
            break;
        }
        case WAV_SAMP_S32:
        {
            int32_t vi = (int32_t)(i * 2147483647.0f);
            int32_t vq = (int32_t)(q * 2147483647.0f);
            write_u32le(p + 0, (uint32_t)vi);
            write_u32le(p + 4, (uint32_t)vq);
            p += 8;
            break;
        }
        case WAV_SAMP_F32:
            memcpy(p + 0, &i, sizeof(i));
            memcpy(p + 4, &q, sizeof(q));
            p += 8;
            break;
        }
    }

    size_t bytes = count * writer->bytes_per_frame;
    if (bytes > 0u && fwrite(buf, 1, bytes, writer->fp) != bytes)
        return -1;
    writer->frames_written += (uint64_t)count;
    return 0;
}

int wav_write_close(wav_writer_t *writer)
{
    uint8_t size_le[4];
    uint64_t data_bytes;
    long end;
    int result = 0;

    if (!writer || !writer->fp)
        return -1;

    data_bytes = writer->frames_written * writer->bytes_per_frame;
    end = ftell(writer->fp);

    /* Patch data size, then RIFF size (file - 8). Saturate at 32 bits:
     * classic WAV ceiling, same tradeoff as SDR++'s default container. */
    uint32_t data_size =
        data_bytes > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)data_bytes;
    uint64_t riff_size =
        end > 8L ? (uint64_t)(end - 8L) : 0u;
    uint32_t riff_field =
        riff_size > 0xFFFFFFFFull ? 0xFFFFFFFFu : (uint32_t)riff_size;

    write_u32le(size_le, data_size);
    if (fseek(writer->fp, writer->data_size_off, SEEK_SET) != 0)
        result = -1;
    else if (fwrite(size_le, 1, sizeof(size_le), writer->fp) != sizeof(size_le))
        result = -1;
    else
    {
        write_u32le(size_le, riff_field);
        if (fseek(writer->fp, writer->riff_size_off, SEEK_SET) != 0)
            result = -1;
        else if (fwrite(size_le, 1, sizeof(size_le), writer->fp) != sizeof(size_le))
            result = -1;
    }

    fclose(writer->fp);
    memset(writer, 0, sizeof(*writer));
    return result;
}

int wav_build_recording_filename(uint64_t lo_hz, uint32_t sample_rate_hz,
                                 char *out, size_t out_cap)
{
    time_t now = time(NULL);
    struct tm tm_utc;
    unsigned int rate_mhz;

    if (!out || out_cap == 0u)
        return -1;

#if defined(_WIN32)
    gmtime_s(&tm_utc, &now);
#else
    gmtime_r(&now, &tm_utc);
#endif

    rate_mhz = sample_rate_hz / 1000000u;
    int needed = snprintf(out, out_cap,
                          "supertooth_baseband_%lluHz_%uMsps_%04d%02d%02d_%02d%02d%02dZ.wav",
                          (unsigned long long)lo_hz, rate_mhz,
                          tm_utc.tm_year + 1900, tm_utc.tm_mon + 1,
                          tm_utc.tm_mday, tm_utc.tm_hour, tm_utc.tm_min,
                          tm_utc.tm_sec);
    if (needed < 0 || (size_t)needed >= out_cap)
    {
        if (out_cap > 0u)
            out[out_cap - 1u] = '\0';
        return -1;
    }
    return 0;
}

/* Scan for decimal "<digits>Hz" (LO) and "<digits>Msps" (rate) tokens in the
 * basename. First plausible hit wins for each field. */
int wav_parse_recording_filename(const char *path, uint64_t *out_lo_hz,
                                 uint32_t *out_rate_hz)
{
    const char *base;
    int found = 0;

    if (!path)
        return 0;
    base = strrchr(path, '/');
    base = base ? base + 1 : path;
#if defined(_WIN32)
    {
        const char *bs = strrchr(base, '\\');
        if (bs)
            base = bs + 1;
    }
#endif

    for (const char *p = base; *p; p++)
    {
        if (!isdigit((unsigned char)*p))
            continue;
        char *end = NULL;
        unsigned long long val = strtoull(p, &end, 10);
        if (end == p)
            continue;
        if (strncmp(end, "Hz", 2) == 0 && out_lo_hz && *out_lo_hz == 0u)
        {
            if (val >= WAV_AUXI_MIN_HZ && val <= WAV_AUXI_MAX_HZ)
            {
                *out_lo_hz = (uint64_t)val;
                found++;
            }
        }
        else if (strncmp(end, "Msps", 4) == 0 && out_rate_hz && *out_rate_hz == 0u)
        {
            if (val >= 1ull && val <= 200ull)
            {
                *out_rate_hz = (uint32_t)(val * 1000000ull);
                found++;
            }
        }
    }
    return found;
}
