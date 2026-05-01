#ifndef WAV_H
#define WAV_H

#include <stddef.h>
#include <complex.h>

int cf2wav(const float complex *buf,
           size_t num_samples,
           unsigned int sample_rate_hz,
           const char *filename);

/* Read mono 16-bit PCM WAV into complex float buffer.
 *
 * filename       : path to .wav file
 * out_buf        : on success, *out_buf points to heap-allocated buffer
 * out_num_samples: on success, number of complex samples
 * out_sample_rate: on success, sample rate in Hz
 *
 * Returns 0 on success, non-zero on error.
 */
int wav2cf(const char *filename,
           float complex **out_buf,
           size_t *out_num_samples,
           unsigned int *out_sample_rate);

#endif /* WAV_H */
