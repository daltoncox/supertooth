#include "radio_common.h"

#include "file.h"

#if HAVE_HACKRF
#include "hackrf.h"
#endif
#if HAVE_BLADERF
#include "bladerf.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

struct radio_device
{
    radio_device_type_t device_type;
    void *impl;
};

const char *radio_device_type_name(radio_device_type_t type)
{
    switch (type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return "hackrf";
#endif
    case RADIO_DEVICE_FILE:
        return "file";
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return "bladerf";
#endif
    default:
        return NULL;
    }
}

int radio_device_type_is_live(radio_device_type_t type)
{
    switch (type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return 1;
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return 1;
#endif
    default:
        return 0;
    }
}

void radio_gain_default(radio_device_type_t type, radio_gain_spec_t *out)
{
    if (!out)
        return;
    memset(out, 0, sizeof(*out));
    out->present = 0;
    out->hackrf_lna = RADIO_HACKRF_LNA_DEFAULT;
    out->hackrf_vga = RADIO_HACKRF_VGA_DEFAULT;
    out->hackrf_amp = 0;
    out->bladerf_gain_db = RADIO_BLADERF_GAIN_DEFAULT;
    (void)type;
}

/* Strict integer parse: no trailing junk, no overflow. */
static int parse_int_strict(const char *s, long *out)
{
    char *end = NULL;
    long v;

    if (!s || !*s || !out)
        return -1;
    v = strtol(s, &end, 10);
    if (end == s || *end != '\0')
        return -1;
    *out = v;
    return 0;
}

int radio_parse_gain_spec(radio_device_type_t type, const char *str,
                          radio_gain_spec_t *out,
                          char *err, size_t err_len)
{
#define GAIN_FAIL(fmt, ...)                                             \
    do {                                                                \
        if (err && err_len > 0u)                                        \
            snprintf(err, err_len, fmt, ##__VA_ARGS__);                 \
        return -1;                                                      \
    } while (0)

    radio_gain_spec_t tmp;

    if (!str || !out)
        GAIN_FAIL("empty gain spec");
    radio_gain_default(type, &tmp);

    switch (type)
    {
    case RADIO_DEVICE_HACKRF:
    {
        /* "LNA,VGA[,AMP]": split on commas without mutating the input. */
        const char *c1 = strchr(str, ',');
        const char *c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        char part[32];
        long lna = 0, vga = 0, amp = 0;
        size_t len;

        if (!c1)
            GAIN_FAIL("expected LNA,VGA[,AMP] (e.g. 24,18,0)");
        if (c2 && strchr(c2 + 1, ','))
            GAIN_FAIL("expected LNA,VGA[,AMP] (e.g. 24,18,0)");

        len = (size_t)(c1 - str);
        if (len == 0u || len >= sizeof(part))
            GAIN_FAIL("bad LNA value");
        memcpy(part, str, len);
        part[len] = '\0';
        if (parse_int_strict(part, &lna) != 0)
            GAIN_FAIL("bad LNA value '%s'", part);

        len = c2 ? (size_t)(c2 - (c1 + 1)) : strlen(c1 + 1);
        if (len == 0u || len >= sizeof(part))
            GAIN_FAIL("bad VGA value");
        memcpy(part, c1 + 1, len);
        part[len] = '\0';
        if (parse_int_strict(part, &vga) != 0)
            GAIN_FAIL("bad VGA value '%s'", part);

        if (c2)
        {
            if (parse_int_strict(c2 + 1, &amp) != 0)
                GAIN_FAIL("bad AMP value '%s'", c2 + 1);
        }

        if (lna < RADIO_HACKRF_LNA_MIN || lna > RADIO_HACKRF_LNA_MAX)
            GAIN_FAIL("LNA %ld out of range (%d-%d)", lna,
                      RADIO_HACKRF_LNA_MIN, RADIO_HACKRF_LNA_MAX);
        if (vga < RADIO_HACKRF_VGA_MIN || vga > RADIO_HACKRF_VGA_MAX)
            GAIN_FAIL("VGA %ld out of range (%d-%d)", vga,
                      RADIO_HACKRF_VGA_MIN, RADIO_HACKRF_VGA_MAX);
        if (amp != 0 && amp != 1)
            GAIN_FAIL("AMP %ld out of range (0=off, 1=on)", amp);

        tmp.present = 1;
        tmp.hackrf_lna = (int)lna;
        tmp.hackrf_vga = (int)vga;
        tmp.hackrf_amp = (int)amp;
        *out = tmp;
        return 0;
    }
    case RADIO_DEVICE_BLADERF:
    {
        long g = 0;
        if (parse_int_strict(str, &g) != 0)
            GAIN_FAIL("expected GAIN_DB (e.g. 30)");
        if (g < RADIO_BLADERF_GAIN_MIN || g > RADIO_BLADERF_GAIN_MAX)
            GAIN_FAIL("gain %ld out of range (%d-%d dB)", g,
                      RADIO_BLADERF_GAIN_MIN, RADIO_BLADERF_GAIN_MAX);
        tmp.present = 1;
        tmp.bladerf_gain_db = (int)g;
        *out = tmp;
        return 0;
    }
    case RADIO_DEVICE_FILE:
        /* Gains are meaningless for replay; accept anything, ignore it. */
        tmp.present = 1;
        *out = tmp;
        return 0;
    default:
        GAIN_FAIL("unknown device type");
    }
#undef GAIN_FAIL
}

void radio_print_gain_usage(void)
{
    fprintf(stderr, "  %-30s RX gain: hackrf LNA,VGA[,AMP] | bladerf GAIN_DB (default: unset = device default)\n",
            "-g, --gain SPEC");
}

void radio_print_gain_help(radio_device_type_t type)
{
    switch (type)
    {
    case RADIO_DEVICE_HACKRF:
        fprintf(stderr,
                "HackRF gain (-g LNA,VGA[,AMP]):\n"
                "  LNA  %d-%d dB in 8 dB steps (default %d)\n"
                "  VGA  %d-%d dB in 2 dB steps (default %d)\n"
                "  AMP  0=off (default), 1=on (+14 dB)\n"
                "  Examples: -g 24,18  -g 32,28,1\n",
                RADIO_HACKRF_LNA_MIN, RADIO_HACKRF_LNA_MAX,
                RADIO_HACKRF_LNA_DEFAULT,
                RADIO_HACKRF_VGA_MIN, RADIO_HACKRF_VGA_MAX,
                RADIO_HACKRF_VGA_DEFAULT);
        break;
    case RADIO_DEVICE_BLADERF:
        fprintf(stderr,
                "bladeRF gain (-g GAIN_DB):\n"
                "  Overall RX gain %d-%d dB, manual gain control (default %d)\n"
                "  Example: -g 30\n",
                RADIO_BLADERF_GAIN_MIN, RADIO_BLADERF_GAIN_MAX,
                RADIO_BLADERF_GAIN_DEFAULT);
        break;
    case RADIO_DEVICE_FILE:
        fprintf(stderr,
                "File replay ignores -g (gains are meaningless for a capture).\n");
        break;
    default:
        fprintf(stderr, "Unknown device type for -g.\n");
        break;
    }
}

int radio_open(radio_device_t **out_device,
               radio_device_type_t device_type,
               const char *device_id,
               sample_dispatcher_t *dispatcher,
               int debug_enabled)
{
    radio_device_t *device = NULL;
    int result = -1;

    if (!out_device || !dispatcher)
        return -1;

    *out_device = NULL;
    device = (radio_device_t *)calloc(1, sizeof(*device));
    if (!device)
        return -1;

    device->device_type = device_type;

    switch (device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        result = hackrf_radio_open(&device->impl, device_id, dispatcher,
                                   debug_enabled);
        break;
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        result = bladerf_radio_open(&device->impl, device_id, dispatcher,
                                    debug_enabled);
        break;
#endif
    case RADIO_DEVICE_FILE:
        /* For file replay the "device id" is the capture path. */
        result = file_radio_open(&device->impl, device_id, dispatcher,
                                 debug_enabled);
        break;
    default:
        break;
    }

    if (result != RADIO_SUCCESS)
    {
        free(device);
        return result;
    }

    *out_device = device;
    return RADIO_SUCCESS;
}

int radio_configure(radio_device_t *device, const radio_stream_config_t *config)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_configure(device->impl, config);
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return bladerf_radio_configure(device->impl, config);
#endif
    case RADIO_DEVICE_FILE:
        return file_radio_configure(device->impl, config);
    default:
        return -1;
    }
}

int radio_start_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_start_rx(device->impl);
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return bladerf_radio_start_rx(device->impl);
#endif
    case RADIO_DEVICE_FILE:
        return file_radio_start_rx(device->impl);
    default:
        return -1;
    }
}

int radio_stop_rx(radio_device_t *device)
{
    if (!device)
        return -1;

    switch (device->device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_stop_rx(device->impl);
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return bladerf_radio_stop_rx(device->impl);
#endif
    case RADIO_DEVICE_FILE:
        return file_radio_stop_rx(device->impl);
    default:
        return -1;
    }
}

void radio_close(radio_device_t *device)
{
    if (!device)
        return;

    switch (device->device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        hackrf_radio_close(device->impl);
        break;
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        bladerf_radio_close(device->impl);
        break;
#endif
    case RADIO_DEVICE_FILE:
        file_radio_close(device->impl);
        break;
    default:
        break;
    }

    free(device);
}

int radio_is_finished(radio_device_t *device)
{
    if (!device)
        return 0;

    switch (device->device_type)
    {
    case RADIO_DEVICE_FILE:
        return file_radio_is_finished(device->impl);
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
#endif
    default:
        return 0;
    }
}

void radio_set_replay_mode(radio_device_t *device, int exhaustive)
{
    if (!device)
        return;

    switch (device->device_type)
    {
    case RADIO_DEVICE_FILE:
        file_radio_set_exhaustive(device->impl, exhaustive);
        break;
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
#endif
    default:
        break;
    }
}

int radio_get_max_sample_rate_for_type(radio_device_type_t type,
                                       uint32_t *out_rate_hz)
{
    if (!out_rate_hz)
        return -1;

    switch (type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return hackrf_radio_get_max_sample_rate(NULL, out_rate_hz);
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return bladerf_radio_get_max_sample_rate(NULL, out_rate_hz);
#endif
    case RADIO_DEVICE_FILE:
        /* Replay of an existing capture is not bound by ADC limits. */
        *out_rate_hz = RADIO_MAX_SAMPLE_RATE_HZ;
        return RADIO_SUCCESS;
    default:
        return -1;
    }
}

int radio_list_devices(radio_device_type_t device_type,
                       char ***out_identifiers,
                       size_t *out_count)
{
    if (!out_identifiers || !out_count)
        return -1;

    *out_identifiers = NULL;
    *out_count = 0u;

    switch (device_type)
    {
#if HAVE_HACKRF
    case RADIO_DEVICE_HACKRF:
        return hackrf_list_devices(out_identifiers, out_count);
#endif
#if HAVE_BLADERF
    case RADIO_DEVICE_BLADERF:
        return bladerf_list_devices(out_identifiers, out_count);
#endif
    case RADIO_DEVICE_FILE:
        /* Captures are user paths, not enumerable hardware. */
        return RADIO_SUCCESS;
    default:
        return -1;
    }
}

void radio_free_device_list(char ***identifiers, size_t count)
{
    if (!identifiers || !*identifiers)
        return;

    char **list = *identifiers;
    for (size_t i = 0u; i < count; i++)
        free(list[i]);
    free(list);
    *identifiers = NULL;
}

int radio_device_exists(radio_device_type_t device_type, const char *device_id)
{
    /* For file replay "existence" is just path readability. */
    if (device_type == RADIO_DEVICE_FILE)
    {
        struct stat st;
        if (!device_id || device_id[0] == '\0')
            return RADIO_DEVICE_NOT_FOUND;
        if (stat(device_id, &st) == 0 && S_ISREG(st.st_mode) &&
            access(device_id, R_OK) == 0)
            return RADIO_SUCCESS;
        return RADIO_DEVICE_NOT_FOUND;
    }

    char **identifiers = NULL;
    size_t count = 0u;
    int result = radio_list_devices(device_type, &identifiers, &count);
    if (result != RADIO_SUCCESS)
        return result;

    result = RADIO_DEVICE_NOT_FOUND;
    for (size_t i = 0u; i < count; i++)
    {
        if (identifiers[i] && strcmp(identifiers[i], device_id) == 0)
        {
            result = RADIO_SUCCESS;
            break;
        }
    }

    radio_free_device_list(&identifiers, count);
    return result;
}
