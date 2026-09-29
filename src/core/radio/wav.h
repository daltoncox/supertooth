#ifndef WAV_H
#define WAV_H

#include <complex.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* Minimal RIFF/WAVE reader + writer for baseband IQ captures.
 *
 * Wire format matches SDR++ baseband recordings (and SDR#/SDRangel/SDR-Console
 * stereo IQ WAVs): 2-channel interleaved PCM, I = left, Q = right, with the
 * sample rate in the `fmt ` chunk. Sample-type mapping follows SDR++:
 *
 *   8-bit PCM  (codec 1) : unsigned 8-bit,  f = (v - 128) / 127
 *   16-bit PCM (codec 1) : signed 16-bit,   f = v / 32767        (default)
 *   32-bit PCM (codec 1) : signed 32-bit,   f = v / 2147483647
 *   32-bitIEEE (codec 3) : float32,         f = v (raw)
 *
 * Center frequency is NOT part of plain WAV, so (like SDR#) we use an `auxi`
 * chunk whose payload is an 8-byte little-endian uint64 center frequency in
 * Hz. Writers emit it; readers accept it when present and range-plausible
 * (100 kHz .. 6 GHz) and otherwise ignore the chunk, so files from SDR++ (no
 * auxi, freq only in the filename) still read. Unknown chunks (`fact`,
 * `JUNK`, `LIST`, …) are skipped, and odd-sized chunks honor RIFF padding.
 *
 * Classic WAV (32-bit sizes) caps files near 4 GB (~50 s at 20 Msps Int16);
 * RF64 is out of scope for now, same as SDR++'s default WAV container. */

typedef enum
{
    WAV_SAMP_U8 = 0,
    WAV_SAMP_S16 = 1,
    WAV_SAMP_S32 = 2,
    WAV_SAMP_F32 = 3,
} wav_sample_type_t;

typedef struct
{
    uint32_t sample_rate_hz;
    uint16_t num_channels; /* must be 2 for IQ */
    wav_sample_type_t sample_type;
    uint64_t center_freq_hz; /* from auxi, or 0 when absent/implausible */
    uint64_t total_frames;   /* complex samples in data chunk */
    long data_offset;        /* file offset of first data byte */
    size_t bytes_per_frame;  /* num_channels * bits/8 */
} wav_info_t;

/* Open + parse the header of a WAV IQ file. Returns 0 on success with *info
 * filled; non-zero on I/O error, bad RIFF/WAVE magic, unsupported format
 * (not 2ch, unknown codec/depth), or missing data chunk. */
int wav_read_open(const char *path, wav_info_t *info, FILE **out_fp);

/* Read up to *inout_count complex samples (I=left, Q=right, normalized to
 * roughly ±1.0) from an open reader. On return *inout_count holds the frames
 * actually read (0 at EOF). Returns 0 on success/EOF, non-zero on I/O error. */
int wav_read_frames(FILE *fp, const wav_info_t *info,
                    float complex *out, size_t *inout_count);

/* Close a reader opened with wav_read_open(). */
void wav_read_close(FILE *fp);

typedef struct
{
    FILE *fp;
    uint32_t sample_rate_hz;
    wav_sample_type_t sample_type;
    uint64_t center_freq_hz; /* written to auxi (0 = still writes auxi w/ 0) */
    size_t bytes_per_frame;
    long riff_size_off; /* offsets to patch on close */
    long data_size_off;
    uint64_t frames_written;
} wav_writer_t;

/* Open a WAV IQ file for writing (2ch, given rate/type, auxi=center Hz).
 * Returns 0 on success, non-zero when the file cannot be created. */
int wav_write_open(const char *path, uint32_t sample_rate_hz,
                   wav_sample_type_t sample_type, uint64_t center_freq_hz,
                   wav_writer_t *writer);

/* Append count complex samples (clamped to ±1.0, scaled per sample type).
 * Returns 0 on success, non-zero on I/O error. */
int wav_write_frames(wav_writer_t *writer, const float complex *samples,
                     size_t count);

/* Finalize sizes and close. Returns 0 on success, non-zero on I/O error. */
int wav_write_close(wav_writer_t *writer);

/* Self-describing recording filename:
 *   supertooth_baseband_<LO>Hz_<RATE>Msps_<YYYYMMDD>_<HHMMSS>Z.wav
 * Timestamp is UTC at call time. Always NUL-terminates; truncates on tiny
 * buffers. Returns 0 on success, -1 when out_cap is too small. */
int wav_build_recording_filename(uint64_t lo_hz, uint32_t sample_rate_hz,
                                 char *out, size_t out_cap);

/* Parse LO (Hz) and sample rate (Hz) back out of a filename of the above
 * form (matches anywhere in the basename, so renamed files with the tokens
 * intact still work). Returns the number of fields recovered (0..2); missing
 * fields leave their out-params untouched (pass 0-initialized). */
int wav_parse_recording_filename(const char *path, uint64_t *out_lo_hz,
                                 uint32_t *out_rate_hz);

#endif /* WAV_H */
