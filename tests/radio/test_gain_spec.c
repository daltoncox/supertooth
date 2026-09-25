/* Unit tests for the unified -g/--gain parser (radio_parse_gain_spec).
 *
 * Pure logic, no hardware required: safe in CI in every flag combination.
 * Compiled-out backends still parse (the parser is backend-independent);
 * only radio_device_type_name() gates on HAVE_*.
 */
#include <stdio.h>
#include <string.h>

#include "radio_common.h"

static int g_failures = 0;

#define TEST_ASSERT(cond)                                                   \
    do {                                                                    \
        if (!(cond))                                                        \
        {                                                                   \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            g_failures++;                                                   \
        }                                                                   \
    } while (0)

int main(void)
{
    radio_gain_spec_t out;
    char err[128];

    /* HackRF: full triple. */
    memset(&out, 0xA5, sizeof(out));
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24,18,0", &out,
                                      err, sizeof(err)) == 0);
    TEST_ASSERT(out.present == 1);
    TEST_ASSERT(out.hackrf_lna == 24);
    TEST_ASSERT(out.hackrf_vga == 18);
    TEST_ASSERT(out.hackrf_amp == 0);

    /* HackRF: AMP defaults to off when omitted. */
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "32,28", &out,
                                      NULL, 0) == 0);
    TEST_ASSERT(out.hackrf_lna == 32 && out.hackrf_vga == 28 &&
                out.hackrf_amp == 0);

    /* HackRF: AMP on. */
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24,18,1", &out,
                                      NULL, 0) == 0);
    TEST_ASSERT(out.hackrf_amp == 1);

    /* HackRF: malformed / out of range. */
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24", &out, err,
                                      sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "41,18", &out,
                                      err, sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24,63", &out,
                                      err, sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24,18,2", &out,
                                      err, sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "24,18,0,5",
                                      &out, err, sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_HACKRF, "xx,18", &out,
                                      err, sizeof(err)) != 0);
    TEST_ASSERT(err[0] != '\0');

    /* bladeRF: single dB value. */
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "30", &out,
                                      err, sizeof(err)) == 0);
    TEST_ASSERT(out.present == 1 && out.bladerf_gain_db == 30);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "0", &out, NULL,
                                      0) == 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "60", &out,
                                      NULL, 0) == 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "61", &out, err,
                                      sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "-1", &out, err,
                                      sizeof(err)) != 0);
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_BLADERF, "24,18", &out,
                                      err, sizeof(err)) != 0);

    /* File replay: anything accepted, ignored. */
    TEST_ASSERT(radio_parse_gain_spec(RADIO_DEVICE_FILE, "whatever", &out,
                                      NULL, 0) == 0);
    TEST_ASSERT(out.present == 1);

    /* Defaults. */
    radio_gain_default(RADIO_DEVICE_HACKRF, &out);
    TEST_ASSERT(out.present == 0 && out.hackrf_lna == 24 &&
                out.hackrf_vga == 18 && out.hackrf_amp == 0);
    radio_gain_default(RADIO_DEVICE_BLADERF, &out);
    TEST_ASSERT(out.present == 0 && out.bladerf_gain_db == 30);

    /* Live-type predicate + type names respect compiled-in backends. */
#if HAVE_HACKRF
    TEST_ASSERT(radio_device_type_is_live(RADIO_DEVICE_HACKRF) == 1);
    TEST_ASSERT(radio_device_type_name(RADIO_DEVICE_HACKRF) != NULL);
#else
    TEST_ASSERT(radio_device_type_is_live(RADIO_DEVICE_HACKRF) == 0);
    TEST_ASSERT(radio_device_type_name(RADIO_DEVICE_HACKRF) == NULL);
#endif
#if HAVE_BLADERF
    TEST_ASSERT(radio_device_type_is_live(RADIO_DEVICE_BLADERF) == 1);
    TEST_ASSERT(radio_device_type_name(RADIO_DEVICE_BLADERF) != NULL);
#else
    TEST_ASSERT(radio_device_type_is_live(RADIO_DEVICE_BLADERF) == 0);
    TEST_ASSERT(radio_device_type_name(RADIO_DEVICE_BLADERF) == NULL);
#endif
    TEST_ASSERT(radio_device_type_is_live(RADIO_DEVICE_FILE) == 0);

    if (g_failures)
    {
        fprintf(stderr, "test_gain_spec: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_gain_spec: ok\n");
    return 0;
}
