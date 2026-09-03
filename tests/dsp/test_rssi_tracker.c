/* RSSI tracker unit tests (no RF hardware required).
 *
 * Regression cover for the sample-ring RSSI tracker: empty returns false,
 * single sample, the 1 s time-window cutoff, source_id filtering, and ring
 * wrap past MAX_SAMPLES. */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "rssi_tracker.h"

static int g_failures = 0;

#define CHECK(cond, fmt, ...)                                                 \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d " fmt "\n", __FILE__, __LINE__, __VA_ARGS__);  \
            g_failures++;                                                     \
        }                                                                     \
    } while (0)

/** Build a metadata frame at a given receive time (seconds) with an RSSI. */
static rx_metadata_t make_frame(uint32_t source_id, double t_sec, float rssi)
{
    rx_metadata_t m;
    memset(&m, 0, sizeof(m));
    m.source_id = source_id;
    m.radio_sample_rate_hz = 1000u;                 /* index == milliseconds */
    m.radio_start_sample_index = (uint64_t)(t_sec * 1000.0);
    m.rssi_dbr = rssi;
    return m;
}

static void test_empty(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);
    float avg = -999.0f;
    CHECK(rssi_tracker_average(&t, &avg) == false, "empty tracker not false %d", 0);
}

static void test_single(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);
    rx_metadata_t f = make_frame(0u, 0.0, -42.0f);
    rssi_tracker_add(&t, &f);

    float avg = 0.0f;
    CHECK(rssi_tracker_average(&t, &avg), "single sample not valid %d", 0);
    CHECK(fabsf(avg - (-42.0f)) < 1e-3f, "single avg %.3f", avg);
}

static void test_window_cutoff(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);

    /* Two frames 1.5 s apart: only the newer one is in the 1 s window. */
    rx_metadata_t old = make_frame(0u, 0.0, -10.0f);
    rx_metadata_t recent = make_frame(0u, 1.5, -50.0f);
    rssi_tracker_add(&t, &old);
    rssi_tracker_add(&t, &recent);

    float avg = 0.0f;
    CHECK(rssi_tracker_average(&t, &avg), "window avg not valid %d", 0);
    CHECK(fabsf(avg - (-50.0f)) < 1e-3f, "cutoff avg %.3f want -50", avg);
}

static void test_window_average(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);

    /* Three frames within the window average to -30. */
    rx_metadata_t a = make_frame(0u, 0.0, -20.0f);
    rx_metadata_t b = make_frame(0u, 0.1, -40.0f);
    rx_metadata_t c = make_frame(0u, 0.2, -30.0f);
    rssi_tracker_add(&t, &a);
    rssi_tracker_add(&t, &b);
    rssi_tracker_add(&t, &c);

    float avg = 0.0f;
    CHECK(rssi_tracker_average(&t, &avg), "window avg not valid %d", 0);
    CHECK(fabsf(avg - (-30.0f)) < 1e-3f, "mean avg %.3f want -30", avg);
}

static void test_source_id_filter(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);

    /* Different sources interleaved; average must ignore the other source. */
    rx_metadata_t s1a = make_frame(1u, 0.0, -10.0f);
    rx_metadata_t s2  = make_frame(2u, 0.0, -90.0f);
    rx_metadata_t s1b = make_frame(1u, 0.1, -20.0f);
    rssi_tracker_add(&t, &s1a);
    rssi_tracker_add(&t, &s2);
    rssi_tracker_add(&t, &s1b);

    float avg = 0.0f;
    CHECK(rssi_tracker_average(&t, &avg), "source avg not valid %d", 0);
    CHECK(fabsf(avg - (-15.0f)) < 1e-3f, "source avg %.3f want -15", avg);
}

static void test_ring_wrap(void)
{
    rssi_tracker_t t;
    rssi_tracker_init(&t);

    /* Exceed MAX_SAMPLES; only the last MAX_SAMPLES within 1 s count. */
    for (size_t i = 0; i < RSSI_TRACKER_MAX_SAMPLES + 50u; i++)
    {
        double t_sec = (double)i / 1000.0;   /* 1 ms apart, all in window */
        float rssi = (float)(i % 2 == 0 ? -40.0 : -20.0);
        rx_metadata_t f = make_frame(0u, t_sec, rssi);
        rssi_tracker_add(&t, &f);
    }

    float avg = 0.0f;
    CHECK(rssi_tracker_average(&t, &avg), "wrap avg not valid %d", 0);
    /* Last MAX_SAMPLES alternate -40/-20 => mean near -30. */
    CHECK(fabsf(avg - (-30.0f)) < 2.0f, "wrap avg %.3f near -30", avg);
}

int main(void)
{
    test_empty();
    test_single();
    test_window_cutoff();
    test_window_average();
    test_source_id_filter();
    test_ring_wrap();

    if (g_failures == 0)
        printf("test_rssi_tracker: all checks passed\n");
    else
        printf("test_rssi_tracker: %d failure(s)\n", g_failures);
    return g_failures;
}
