/**
 * @file test_bredr_registry.c
 * @brief Coverage for the BR/EDR link + registry layer (previously untested).
 *
 * Covers: link init (LAP masking, GIAC/LIAC DCI preset), set_uap_only vs
 * set_uap + central-clock math, queue overflow, registry NULL/rate
 * guards, pending-gate promotion, full UAP acquisition through
 * registry_submit (golden vectors), idle-reset on LAP silence, standalone
 * add_device rows, and the get_connections/get_devices getters.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bredr_link.h"
#include "bredr_registry.h"
#include "device_models.h"
#include "receive_event_models.h"
#include "golden_vectors.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                     \
    do {                                                                      \
        if (!(cond)) {                                                        \
            fprintf(stderr, "ASSERT FAILED %s:%d: %s\n", __FILE__, __LINE__,  \
                    #cond);                                                   \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

#define TEST_LAP 0xABCDEFu

static void make_event(bredr_event_t *ev, uint32_t lap, uint32_t clkn,
                       int has_header, uint64_t header_raw)
{
    memset(ev, 0, sizeof(*ev));
    ev->meta.radio_sample_rate_hz = 3200u;
    ev->meta.radio_start_sample_index = clkn; /* 3200 Hz: index == clkn */
    ev->frame.lap = lap & 0xFFFFFFu;
    ev->frame.has_header = (uint8_t)(has_header ? 1 : 0);
    ev->frame.header_raw = header_raw;
}

static void test_link_init(void)
{
    bredr_link_t *link = malloc(sizeof(*link));
    TEST_ASSERT(link != NULL);
    if (!link)
        return;

    bredr_link_init(link, TEST_LAP);
    TEST_ASSERT((link->lap & 0xFFFFFFu) == TEST_LAP);
    TEST_ASSERT(link->tracking_state == -1);
    TEST_ASSERT(link->uap_valid == 0);

    /* 24-bit masking. */
    bredr_link_init(link, 0x1FFFFFFu);
    TEST_ASSERT(link->lap == 0xFFFFFFu);

    /* Inquiry LAPs preset the well-known DCI UAP. */
    bredr_link_init(link, BREDR_LAP_GIAC);
    TEST_ASSERT(link->uap == BREDR_DCI && link->uap_valid == 1);
    bredr_link_init(link, BREDR_LAP_LIAC);
    TEST_ASSERT(link->uap == BREDR_DCI && link->uap_valid == 1);

    bredr_link_init(NULL, TEST_LAP); /* must not crash */
    free(link);
}

static void test_link_uap_clock(void)
{
    bredr_link_t *link = malloc(sizeof(*link));
    TEST_ASSERT(link != NULL);
    if (!link)
        return;
    bredr_link_init(link, TEST_LAP);

    /* UAP-only: valid UAP, clock still unknown. */
    bredr_link_set_uap_only(link, 0x47u);
    TEST_ASSERT(link->uap == 0x47u && link->uap_valid == 1);
    TEST_ASSERT(link->clk_known == 0);

    /* Full solve: central_clk(rx) must round-trip the solved clock. */
    bredr_link_set_uap(link, 0x47u, 10u, 100u);
    TEST_ASSERT(link->clk_known == 1);
    TEST_ASSERT(bredr_link_central_clk_1_6(link, 100u) == 10u);
    TEST_ASSERT(bredr_link_central_clk_1_6(link, 101u) == 11u);

    /* NULL guards. */
    bredr_link_set_uap(NULL, 0u, 0u, 0u);
    bredr_link_set_uap_only(NULL, 0u);
    TEST_ASSERT(bredr_link_central_clk_1_6(NULL, 0u) == 0u);
    TEST_ASSERT(bredr_link_add_packet(NULL, NULL) == 0);

    bredr_event_t ev;
    make_event(&ev, TEST_LAP, 0u, 0, 0u);
    TEST_ASSERT(bredr_link_add_packet(NULL, &ev) == 0);
    TEST_ASSERT(bredr_link_add_packet(link, NULL) == 0);

    free(link);
}

static void test_link_queue_overflow(void)
{
    bredr_link_t *link = malloc(sizeof(*link));
    TEST_ASSERT(link != NULL);
    if (!link)
        return;
    bredr_link_init(link, TEST_LAP);

    bredr_event_t ev;
    for (uint32_t i = 0u; i < BREDR_LINK_QUEUE_SIZE + 2u; i++)
    {
        make_event(&ev, TEST_LAP, i * 2u, 0, 0u);
        bredr_link_add_packet(link, &ev);
    }
    TEST_ASSERT(link->queue_fill == BREDR_LINK_QUEUE_SIZE);
    TEST_ASSERT(link->total_packets == BREDR_LINK_QUEUE_SIZE + 2u);
    free(link);
}

static void test_registry_guards_and_gate(void)
{
    bredr_registry_t r;
    bredr_registry_config_t cfg;
    bredr_event_t ev;

    memset(&cfg, 0, sizeof(cfg));
    cfg.idle_reset_clkn = 0u; /* disable idle reset for this subtest */
    cfg.promote_threshold = 2u;
    bredr_registry_init(&r, &cfg);

    /* NULL / bad-input guards. */
    TEST_ASSERT(bredr_registry_submit(NULL, &ev, NULL, NULL) == -1);
    TEST_ASSERT(bredr_registry_submit(&r, NULL, NULL, NULL) == -1);
    make_event(&ev, TEST_LAP, 0u, 0, 0u);
    ev.meta.radio_sample_rate_hz = 0u;
    TEST_ASSERT(bredr_registry_submit(&r, &ev, NULL, NULL) == -1);
    ev.meta.radio_sample_rate_hz = 3200u;

    /* Pending gate: first sighting tallies but creates nothing. */
    TEST_ASSERT(bredr_registry_submit(&r, &ev, NULL, NULL) == 0);
    bredr_connection_snapshot_t conns[4];
    TEST_ASSERT(bredr_registry_get_connections(&r, conns, 4u) == 0u);

    /* Second sighting promotes to a real connection. */
    ev.meta.radio_start_sample_index = 2u;
    TEST_ASSERT(bredr_registry_submit(&r, &ev, NULL, NULL) == 0);
    TEST_ASSERT(bredr_registry_get_connections(&r, conns, 4u) == 1u);
    TEST_ASSERT(conns[0].lap == TEST_LAP);

    /* Getter guards. */
    TEST_ASSERT(bredr_registry_get_connections(&r, NULL, 4u) == 0u);
    TEST_ASSERT(bredr_registry_get_connections(&r, conns, 0u) == 0u);
    bredr_device_snapshot_t devs[4];
    TEST_ASSERT(bredr_registry_get_devices(&r, NULL, 4u) == 0u);
    TEST_ASSERT(bredr_registry_get_devices(&r, devs, 0u) == 0u);

    bredr_registry_free(&r);
}

/* Find the first golden case with the given UAP and n. */
static const golden_case_t *find_case(uint8_t uap, int n)
{
    size_t count = sizeof(golden_cases) / sizeof(golden_cases[0]);
    for (size_t i = 0u; i < count; i++)
        if (golden_cases[i].n == n && golden_cases[i].uap == uap)
            return &golden_cases[i];
    return NULL;
}

static void test_registry_acquire_and_idle_reset(void)
{
    const golden_case_t *c = find_case(0x47u, 12);
    TEST_ASSERT(c != NULL);
    if (!c)
        return;

    bredr_registry_t r;
    bredr_registry_init(&r, NULL); /* defaults: threshold 1, idle 16384 */

    bredr_event_t ev;
    for (int k = 0; k < c->n; k++)
    {
        const golden_packet_t *p = &c->pkts[k];
        make_event(&ev, TEST_LAP, p->clkn, 1, p->header_raw);
        unsigned int nb = (p->air_payload_bits + 7u) / 8u;
        if (nb > sizeof(ev.frame.air_payload))
            nb = sizeof(ev.frame.air_payload);
        memcpy(ev.frame.air_payload, p->air_payload, nb);
        ev.frame.air_payload_bits = p->air_payload_bits;
        TEST_ASSERT(bredr_registry_submit(&r, &ev, NULL, NULL) == 0);
    }

    /* Acquisition through the registry must converge on the golden UAP. */
    bredr_connection_snapshot_t conns[4];
    TEST_ASSERT(bredr_registry_get_connections(&r, conns, 4u) == 1u);
    TEST_ASSERT(conns[0].uap_valid == 1);
    TEST_ASSERT(conns[0].uap == 0x47u);
    TEST_ASSERT(conns[0].clk_known == 1);

    /* LAP silence beyond the idle window resets recovery (uap_valid clears;
     * the sticky display UAP in `uap` is preserved by design). */
    uint32_t last_clkn = c->pkts[c->n - 1].clkn;
    make_event(&ev, TEST_LAP, last_clkn + 100000u, 0, 0u);
    TEST_ASSERT(bredr_registry_submit(&r, &ev, NULL, NULL) == 0);
    TEST_ASSERT(bredr_registry_get_connections(&r, conns, 4u) == 1u);
    TEST_ASSERT(conns[0].uap_valid == 0);
    TEST_ASSERT(conns[0].uap == 0x47u);

    bredr_registry_free(&r);
}

static void test_registry_standalone_device(void)
{
    bredr_registry_t r;
    bredr_registry_init(&r, NULL);

    bredr_device_obs_t obs;
    memset(&obs, 0, sizeof(obs));
    obs.lap = 0x123456u;
    obs.uap = 0xABu;
    obs.uap_valid = 1;
    obs.rssi_db = -42.0f;
    obs.rssi_valid = 1;

    TEST_ASSERT(bredr_registry_add_device(NULL, &obs) == -1);
    TEST_ASSERT(bredr_registry_add_device(&r, NULL) == -1);
    TEST_ASSERT(bredr_registry_add_device(&r, &obs) == 0);

    bredr_device_snapshot_t devs[4];
    TEST_ASSERT(bredr_registry_get_devices(&r, devs, 4u) == 1u);
    TEST_ASSERT(devs[0].connection_id == 0u);
    TEST_ASSERT(devs[0].lap == 0x123456u);
    TEST_ASSERT(devs[0].uap == 0xABu && devs[0].uap_valid == 1);

    /* Repeat observation merges into the same standalone row. */
    TEST_ASSERT(bredr_registry_add_device(&r, &obs) == 0);
    TEST_ASSERT(bredr_registry_get_devices(&r, devs, 4u) == 1u);
    TEST_ASSERT(devs[0].total_packets == 2u);

    bredr_registry_free(&r);
}

int main(void)
{
    test_link_init();
    test_link_uap_clock();
    test_link_queue_overflow();
    test_registry_guards_and_gate();
    test_registry_acquire_and_idle_reset();
    test_registry_standalone_device();

    if (g_failures != 0)
    {
        fprintf(stderr, "test_bredr_registry: %d assertion(s) failed\n",
                g_failures);
        return 1;
    }
    printf("test_bredr_registry: all checks passed\n");
    return 0;
}
