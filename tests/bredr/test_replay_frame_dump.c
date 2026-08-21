/**
 * @file test_replay_frame_dump.c
 * @brief Offline replay of a real-capture frame dump through the recovery backend.
 *
 * This is the Phase 3 harness: with no IQ/file replay path in the radio stack,
 * a real capture is recorded via bredr_piconet_store_set_frame_dump() and
 * replayed here through bredr_recovery_process_packet() to confirm the
 * aligned recovery produces a stable, expected UAP.
 *
 * Usage:
 *   SUPERTOTH_REPLAY_FILE=capture.bin ./test_replay_frame_dump
 *   SUPERTOTH_REPLAY_FILE=capture.bin SUPERTOTH_REPLAY_UAP=0x15 ./test_replay_frame_dump
 *
 * If SUPERTOTH_REPLAY_FILE is unset the test is a no-op SKIP (passes) so it is
 * safe to run in CI without a capture present.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_clock_recovery.h"
#include "bredr_piconet_store.h"

#define FRAME_DUMP_MAGIC   0x53544C44u
#define FRAME_DUMP_VERSION 1u

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t lap;
    int32_t channel;
    uint32_t clkn;
    uint64_t header_raw;
    uint32_t air_payload_bits;
    uint8_t air_payload[BR_MAX_AIR_PAYLOAD_BYTES];
} frame_dump_rec_t;

int main(void)
{
    const char *path = getenv("SUPERTOTH_REPLAY_FILE");
    if (!path)
    {
        printf("test_replay_frame_dump: SKIP (set SUPERTOTH_REPLAY_FILE to replay a capture)\n");
        return 0;
    }

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        fprintf(stderr, "test_replay_frame_dump: cannot open %s\n", path);
        return 1;
    }

    bredr_recovery_state_t *st = NULL;
    int records = 0;
    int recovered = 0;
    uint8_t got_uap = 0;
    uint8_t got_clk = 0;
    uint32_t last_lap = 0u;

    for (;;)
    {
        frame_dump_rec_t rec;
        size_t n = fread(&rec, sizeof(rec), 1u, f);
        if (n != 1u)
            break;
        if (rec.magic != FRAME_DUMP_MAGIC || rec.version != FRAME_DUMP_VERSION)
        {
            fprintf(stderr, "test_replay_frame_dump: bad record magic/version at #%d\n", records);
            fclose(f);
            return 1;
        }

        if (st == NULL || rec.lap != last_lap)
        {
            if (st)
                bredr_recovery_state_destroy(st);
            st = bredr_recovery_state_create(rec.lap);
            last_lap = rec.lap;
        }

        bredr_frame_t fr;
        memset(&fr, 0, sizeof(fr));
        fr.has_header = 1;
        fr.lap = rec.lap;
        fr.header_raw = rec.header_raw;
        fr.air_payload_bits = rec.air_payload_bits > BR_MAX_AIR_PAYLOAD_BITS
                                  ? BR_MAX_AIR_PAYLOAD_BITS
                                  : rec.air_payload_bits;
        unsigned int nbytes = (fr.air_payload_bits + 7u) / 8u;
        if (nbytes > sizeof(fr.air_payload))
            nbytes = sizeof(fr.air_payload);
        memcpy(fr.air_payload, rec.air_payload, nbytes);

        bredr_recovery_result_t r;
        if (bredr_recovery_process_packet(st, &fr, (int)rec.channel, rec.clkn, &r))
        {
            recovered = 1;
            got_uap = r.uap;
            got_clk = r.clk6_hint;
            break;
        }
        records++;
    }

    fclose(f);
    if (st)
        bredr_recovery_state_destroy(st);

    printf("test_replay_frame_dump: replayed %d records from %s\n", records, path);

    if (!recovered)
    {
        fprintf(stderr, "test_replay_frame_dump: no UAP recovered from capture\n");
        return 1;
    }

    printf("test_replay_frame_dump: recovered UAP=0x%02X clk6=%u\n", got_uap, got_clk);

    const char *expect = getenv("SUPERTOTH_REPLAY_UAP");
    if (expect)
    {
        unsigned long exp = strtoul(expect, NULL, 0);
        if ((exp & 0xFFu) != got_uap)
        {
            fprintf(stderr,
                    "test_replay_frame_dump: UAP 0x%02X != expected 0x%02lX\n",
                    got_uap, exp);
            return 1;
        }
    }

    return 0;
}
