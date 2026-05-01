#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <btbb.h>

#define MAX_AC_ERRORS 2
#define BLOCK_SIZE_BYTES 4096
#define AC_LENGTH_BITS 72
#define AC_LENGTH_BYTES ((AC_LENGTH_BITS + 7) / 8)

typedef struct
{
    uint32_t *laps;
    size_t count;
    size_t cap;
} LapSet;

void lapset_init(LapSet *set)
{
    set->cap = 32;
    set->count = 0;
    set->laps = malloc(set->cap * sizeof(uint32_t));
}

void lapset_free(LapSet *set)
{
    free(set->laps);
}

int lapset_contains(LapSet *set, uint32_t lap)
{
    for (size_t i = 0; i < set->count; i++)
    {
        if (set->laps[i] == lap)
            return 1;
    }
    return 0;
}

void lapset_add(LapSet *set, uint32_t lap)
{
    if (lapset_contains(set, lap))
        return;
    if (set->count == set->cap)
    {
        set->cap *= 2;
        set->laps = realloc(set->laps, set->cap * sizeof(uint32_t));
    }
    set->laps[set->count++] = lap;
}

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s <input_file>\n", argv[0]);
        return 1;
    }

    const char *input_filename = argv[1];
    FILE *f = fopen(input_filename, "rb");
    if (!f)
    {
        perror("Unable to open input file");
        return 1;
    }

    uint8_t *buffer = malloc(BLOCK_SIZE_BYTES + AC_LENGTH_BYTES);
    if (!buffer)
    {
        fprintf(stderr, "Memory allocation failed for buffer\n");
        fclose(f);
        return 1;
    }

    btbb_init(MAX_AC_ERRORS);

    btbb_packet *pkt = NULL;
    int found = 0;
    long total_offset = 0;
    size_t prev_overlap = 0;

    LapSet laps;
    lapset_init(&laps);

    printf("Searching for LAPs in %s...\n", input_filename);

    while (1)
    {
        if (prev_overlap > 0)
            memmove(buffer, buffer + BLOCK_SIZE_BYTES, prev_overlap);

        size_t bytes_read = fread(buffer + prev_overlap, 1, BLOCK_SIZE_BYTES, f);
        if (bytes_read == 0)
            break;

        size_t valid_bytes = bytes_read + prev_overlap;
        size_t search_bits = valid_bytes * 8;
        int offset = 0;

        while (offset < search_bits)
        {
            int found_offset = btbb_find_ac((char *)buffer + (offset / 8), search_bits - offset, LAP_ANY, MAX_AC_ERRORS, &pkt);
            if (found_offset < 0 || pkt == NULL)
                break;

            int ac_bit_offset = offset + found_offset;
            uint32_t lap = btbb_packet_get_lap(pkt);
            printf("LAP: %06x, AC errors: %d, Bit Offset: %ld\n", lap, btbb_packet_get_ac_errors(pkt), total_offset + ac_bit_offset);

            lapset_add(&laps, lap);

            found++;
            offset = ac_bit_offset + 72;

            btbb_packet_unref(pkt);
            pkt = NULL;
        }

        prev_overlap = (valid_bytes > AC_LENGTH_BYTES) ? AC_LENGTH_BYTES : valid_bytes;
        total_offset += bytes_read * 8;
    }

    if (found == 0)
    {
        printf("No access code found in %s\n", input_filename);
    }
    else
    {
        printf("\nUnique LAPs found:\n");
        for (size_t i = 0; i < laps.count; i++)
        {
            printf("%06x\n", laps.laps[i]);
        }
    }

    lapset_free(&laps);
    free(buffer);
    fclose(f);
    return 0;
}
