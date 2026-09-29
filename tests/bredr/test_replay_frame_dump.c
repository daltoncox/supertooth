/**
 * @file test_replay_frame_dump.c
 * @brief Offline replay of a real-capture frame dump through the recovery backend.
 *
 * Two modes:
 *   1. Default (CI): replays the checked-in LAP 0x1FC475 capture fixture
 *      (tests/bredr/capture_151FC475.h) through bredr_recovery_process()
 *      and asserts the expected UAP 0x15 is recovered. No env needed.
 *   2. Manual: with SUPERTOOTH_REPLAY_FILE=capture.bin, replays a
 *      real capture recorded via bredr_registry_set_frame_dump() instead.
 *      SUPERTOOTH_REPLAY_UAP=0x15 optionally asserts the expected UAP.
 *
 * Usage:
 *   ./test_replay_frame_dump
 *   SUPERTOOTH_REPLAY_FILE=capture.bin ./test_replay_frame_dump
 *   SUPERTOOTH_REPLAY_FILE=capture.bin SUPERTOOTH_REPLAY_UAP=0x15 ./test_replay_frame_dump
 *
 * NOTE: the legacy SUPERTOTH_REPLAY_* spelling (missing 'O') is still
 * accepted as a fallback for existing scripts, but the SUPERTOOTH_*
 * spelling takes precedence.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_bitstream_decoder.h"
#include "bredr_clock_recovery.h"
#include "receive_event_models.h"
#include "bredr_link.h"
#include "capture_151FC475.h"

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

static const char *pick_env(const char *primary, const char *legacy)
{
    const char *v = getenv(primary);
    if (v && *v)
        return v;
    return getenv(legacy);
}

static int feed_frame(bredr_link_t **link, uint32_t *last_lap,
                      uint32_t lap, int32_t channel, uint32_t clkn,
                      uint64_t header_raw, uint32_t air_payload_bits,
                      const uint8_t *air_payload)
{
    if (!*link || lap != *last_lap)
    {
        free(*link);
        *link = malloc(sizeof(**link));
        if (!*link)
            return -1;
        bredr_link_init(*link, lap);
        *last_lap = lap;
    }

    bredr_frame_t fr;
    memset(&fr, 0, sizeof(fr));
    fr.has_header = 1;
    fr.lap = lap;
    fr.header_raw = header_raw;
    fr.air_payload_bits = air_payload_bits > BR_MAX_AIR_PAYLOAD_BITS
                               ? BR_MAX_AIR_PAYLOAD_BITS
                               : air_payload_bits;
    unsigned int nbytes = (fr.air_payload_bits + 7u) / 8u;
    if (nbytes > sizeof(fr.air_payload))
        nbytes = sizeof(fr.air_payload);
    if (nbytes && air_payload)
        memcpy(fr.air_payload, air_payload, nbytes);

    bredr_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ev.meta.radio_sample_rate_hz = 3200u;
    ev.meta.radio_start_sample_index = clkn;
    ev.meta.channel_index = (uint16_t)channel;
    ev.frame = fr;

    return bredr_recovery_process(*link, &ev);
}

static int replay_builtin_fixture(void)
{
    bredr_link_t *link = NULL;
    uint32_t last_lap = 0u;
    int records = 0;
    int recovered = 0;
    uint8_t got_uap = 0;
    uint8_t got_clk = 0;

    for (int i = 0; i < cap_151FC475_n; i++)
    {
        const cap_pkt_t *p = &cap_151FC475[i];
        int rc = feed_frame(&link, &last_lap, CAP_LAP, p->channel, p->clkn,
                            p->header_raw, p->air_payload_bits, p->air_payload);
        if (rc < 0)
        {
            fprintf(stderr, "test_replay_frame_dump: out of memory\n");
            free(link);
            return 1;
        }
        records++;
        if (rc)
        {
            recovered = 1;
            got_uap = link->uap;
            got_clk = (uint8_t)link->clock_offset;
            break;
        }
    }

    free(link);

    printf("test_replay_frame_dump: replayed %d fixture records (LAP 0x%08X)\n",
           records, CAP_LAP);
    if (!recovered)
    {
        fprintf(stderr, "test_replay_frame_dump: no UAP recovered from fixture\n");
        return 1;
    }
    printf("test_replay_frame_dump: recovered UAP=0x%02X clk6=%u\n", got_uap, got_clk);
    if (got_uap != CAP_TRUE_UAP)
    {
        fprintf(stderr, "test_replay_frame_dump: UAP 0x%02X != expected 0x%02X\n",
                got_uap, CAP_TRUE_UAP);
        return 1;
    }
    return 0;
}

int main(void)
{
    const char *path = pick_env("SUPERTOOTH_REPLAY_FILE", "SUPERTOTH_REPLAY_FILE");
    if (!path)
        return replay_builtin_fixture();

    FILE *f = fopen(path, "rb");
    if (!f)
    {
        fprintf(stderr, "test_replay_frame_dump: cannot open %s\n", path);
        return 1;
    }

    bredr_link_t *link = NULL;
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
            free(link);
            return 1;
        }

        int rc = feed_frame(&link, &last_lap, rec.lap, rec.channel, rec.clkn,
                            rec.header_raw, rec.air_payload_bits, rec.air_payload);
        if (rc < 0)
        {
            fclose(f);
            free(link);
            return 1;
        }
        if (rc)
        {
            recovered = 1;
            got_uap = link->uap;
            got_clk = (uint8_t)link->clock_offset;
            break;
        }
        records++;
    }

    free(link);

    fclose(f);

    printf("test_replay_frame_dump: replayed %d records from %s\n", records, path);

    if (!recovered)
    {
        fprintf(stderr, "test_replay_frame_dump: no UAP recovered from capture\n");
        return 1;
    }

    printf("test_replay_frame_dump: recovered UAP=0x%02X clk6=%u\n", got_uap, got_clk);

    const char *expect = pick_env("SUPERTOOTH_REPLAY_UAP", "SUPERTOTH_REPLAY_UAP");
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
