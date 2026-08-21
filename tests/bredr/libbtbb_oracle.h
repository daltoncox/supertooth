/* libbtbb_oracle.h -- test-only oracle, not linked into product */
#pragma once
#include <stdint.h>
uint8_t  oracle_reverse(uint8_t b);
uint8_t  oracle_uap_from_hec(uint16_t data, uint8_t hec);
uint16_t oracle_crcgen(char *payload, int length, int UAP);
/* expose for header-path parity checks */
int oracle_unfec13(char *input, char *output, int length);
char* oracle_unfec23(char *input, int length);
void oracle_unwhiten(char *input, char *output, int clock, int length, int skip);
uint8_t oracle_air_to_host8(char *air, int bits);
uint16_t oracle_air_to_host16(char *air, int bits);
/* FEC 2/3 ENCODER (libbtbb fec23): encodes `length` data bits (multiple of
 * 10) into a malloc'd char array of length/10*15 symbols. Test-only helper
 * used to build reference codewords for the decoder parity tests. */
char* oracle_fec23_encode(const char *data_bits, int length);
