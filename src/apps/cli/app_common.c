#include "app_common.h"

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static session_t *g_session_slot = NULL;
static pthread_mutex_t g_output_mutex = PTHREAD_MUTEX_INITIALIZER;

static void app_handle_sigint(int sig)
{
    (void)sig;
    if (g_session_slot)
        session_request_stop(g_session_slot);
}

int app_parse_output_mode(const char *arg,
                          const app_output_mode_option_t *options,
                          size_t option_count,
                          app_output_mode_t *out_mode)
{
    if (!arg || !options || option_count == 0u || !out_mode)
        return -1;

    for (size_t i = 0; i < option_count; i++)
    {
        if (strcmp(arg, options[i].name) == 0)
        {
            *out_mode = options[i].mode;
            return 0;
        }
    }

    return -1;
}

const app_output_mode_option_t *app_output_mode_option(app_output_mode_t mode,
                                                       const app_output_mode_option_t *options,
                                                       size_t option_count)
{
    if (!options || option_count == 0u)
        return NULL;

    for (size_t i = 0; i < option_count; i++)
    {
        if (options[i].mode == mode)
            return &options[i];
    }

    return &options[0];
}

const char *app_output_mode_name(app_output_mode_t mode,
                                 const app_output_mode_option_t *options,
                                 size_t option_count)
{
    const app_output_mode_option_t *option =
        app_output_mode_option(mode, options, option_count);
    return option ? option->name : "";
}

void app_output_lock(void)
{
    pthread_mutex_lock(&g_output_mutex);
}

void app_output_unlock(void)
{
    pthread_mutex_unlock(&g_output_mutex);
}

void app_install_sigint_handler(session_t *session_slot)
{
    g_session_slot = session_slot;
    signal(SIGINT, app_handle_sigint);
}

int app_parse_device_spec(const char *spec, app_device_spec_t *out)
{
    if (!spec || !out)
        return -1;

    const char *colon = strchr(spec, ':');
    if (!colon)
        return -1;

    size_t type_len = (size_t)(colon - spec);
    const char *id = colon + 1;
    if (!id[0])
        return -1;

    for (int t = 0; t < (int)RADIO_DEVICE_TYPE_COUNT; t++)
    {
        const char *name = radio_device_type_name((radio_device_type_t)t);
        if (!name)
            continue;
        if (strlen(name) == type_len && strncmp(spec, name, type_len) == 0)
        {
            out->type = (radio_device_type_t)t;
            out->id = id;
            return 0;
        }
    }
    return -1;
}

int app_print_available_devices(const char *argv0)
{
    radio_device_entry_t *entries = NULL;
    size_t count = 0u;

    if (radio_enumerate_devices(&entries, &count) != RADIO_SUCCESS)
    {
        fprintf(stderr, "Device listing failed.\n");
        return EXIT_FAILURE;
    }

    printf("Found available devices:\n");
    for (size_t i = 0u; i < count; i++)
        printf("%zu. %s:%s\n", i + 1u,
               radio_device_type_name(entries[i].type), entries[i].id);

    if (count == 0u)
        printf("(no devices found)\n");

    radio_free_device_entries(&entries, count);
    (void)argv0;
    return EXIT_SUCCESS;
}

int app_validate_device_spec(const char *argv0, const app_device_spec_t *spec)
{
    if (!spec || !spec->id)
        return -1;

    int result = radio_device_exists(spec->type, spec->id);
    if (result == RADIO_SUCCESS)
        return 0;

    if (result == RADIO_DEVICE_NOT_FOUND)
    {
        fprintf(stderr,
                "Device not found. Run the following to list detected devices:\n\n"
                "%s -d\n",
                argv0);
        return -1;
    }

    fprintf(stderr,
            "Device check failed for %s:%s (error %d). Run the following "
            "to list detected devices:\n\n%s -d\n",
            radio_device_type_name(spec->type), spec->id, result, argv0);
    return -1;
}

static void app_print_list_hint(const char *argv0)
{
    fprintf(stderr, "Run the following to list detected devices:\n\n%s -d\n",
            argv0 ? argv0 : "supertooth");
}

/* Look up a live device type by name ("hackrf"); case-sensitive, matching
 * app_parse_device_spec(). FILE is a replay path, not a device. */
static int app_find_live_type(const char *name, radio_device_type_t *out)
{
    size_t len;

    if (!name || !out)
        return -1;
    len = strlen(name);
    if (len == 0u)
        return -1;
    for (int t = 0; t < (int)RADIO_DEVICE_TYPE_COUNT; t++)
    {
        radio_device_type_t type = (radio_device_type_t)t;
        const char *type_name;
        if (!radio_device_type_is_live(type))
            continue;
        type_name = radio_device_type_name(type);
        if (type_name && strcmp(type_name, name) == 0)
        {
            *out = type;
            return 0;
        }
    }
    return -1;
}

/* Adopt an inventory entry into @p out with a process-lifetime id copy. */
static int app_adopt_entry(const radio_device_entry_t *entry,
                           app_device_spec_t *out)
{
    size_t len;

    if (!entry || !entry->id || !out)
        return -1;
    len = strlen(entry->id);
    out->id = (const char *)malloc(len + 1u);
    if (!out->id)
        return -1;
    memcpy((char *)out->id, entry->id, len + 1u);
    out->type = entry->type;
    return 0;
}

int app_resolve_device_arg(const char *argv0, const char *arg,
                           app_device_spec_t *out)
{
    radio_device_entry_t *entries = NULL;
    size_t count = 0u;
    app_device_spec_t picked = { .type = RADIO_DEVICE_HACKRF, .id = NULL };
    int result = -1;

    if (!arg || !arg[0] || !out)
    {
        fprintf(stderr, "Invalid device selection.\n");
        app_print_list_hint(argv0);
        return -1;
    }

    /* Pure type:id specs (file replay included) keep the existing path. */
    if (strchr(arg, ':') != NULL)
    {
        if (app_parse_device_spec(arg, out) != 0)
        {
            fprintf(stderr, "Invalid device spec: %s (expected <n>, <type> or <type>:<id>)\n",
                    arg);
            app_print_list_hint(argv0);
            return -1;
        }
        return app_validate_device_spec(argv0, out);
    }

    if (radio_enumerate_devices(&entries, &count) != RADIO_SUCCESS)
    {
        fprintf(stderr, "Device listing failed.\n");
        return -1;
    }

    /* All-digits: 1-based index into the numbered listing. */
    {
        char *end = NULL;
        unsigned long index = strtoul(arg, &end, 10);
        if (end != arg && *end == '\0')
        {
            if (index < 1ul || index > (unsigned long)count)
            {
                fprintf(stderr, "Invalid device index: %s (expected 1-%lu).\n",
                        arg, (unsigned long)count);
                radio_free_device_entries(&entries, count);
                app_print_list_hint(argv0);
                return -1;
            }
            picked.type = entries[index - 1ul].type;
            if (app_adopt_entry(&entries[index - 1ul], &picked) != 0)
            {
                radio_free_device_entries(&entries, count);
                return -1;
            }
            radio_free_device_entries(&entries, count);
            if (app_validate_device_spec(argv0, &picked) != 0)
            {
                free((void *)picked.id);
                return -1;
            }
            *out = picked;
            return 0;
        }
    }

    /* Otherwise a device type name: first device of that type. */
    {
        radio_device_type_t type;
        if (app_find_live_type(arg, &type) != 0)
        {
            if (strcmp(arg, "file") == 0)
                fprintf(stderr, "No file devices to select: replay a capture with -d file:<path>.\n");
            else
                fprintf(stderr, "Unknown device type: %s.\n", arg);
            radio_free_device_entries(&entries, count);
            app_print_list_hint(argv0);
            return -1;
        }
        for (size_t i = 0u; i < count; i++)
        {
            if (entries[i].type == type)
            {
                if (app_adopt_entry(&entries[i], &picked) != 0)
                {
                    radio_free_device_entries(&entries, count);
                    return -1;
                }
                result = 0;
                break;
            }
        }
        if (result != 0)
        {
            fprintf(stderr, "No %s devices found.\n", arg);
            radio_free_device_entries(&entries, count);
            app_print_list_hint(argv0);
            return -1;
        }
        radio_free_device_entries(&entries, count);
        if (app_validate_device_spec(argv0, &picked) != 0)
        {
            free((void *)picked.id);
            return -1;
        }
        *out = picked;
        return 0;
    }
}

int app_pick_default_device(const char *argv0, app_device_spec_t *out)
{
    radio_device_entry_t *entries = NULL;
    size_t count = 0u;

    if (!out)
        return -1;
    if (radio_enumerate_devices(&entries, &count) != RADIO_SUCCESS)
    {
        fprintf(stderr, "Device listing failed.\n");
        return -1;
    }
    if (count == 0u)
    {
        radio_free_device_entries(&entries, count);
        fprintf(stderr, "No devices found. ");
        app_print_list_hint(argv0);
        return -1;
    }
    if (app_adopt_entry(&entries[0], out) != 0)
    {
        radio_free_device_entries(&entries, count);
        return -1;
    }
    radio_free_device_entries(&entries, count);
    return app_validate_device_spec(argv0, out);
}

void app_print_device_usage_line(void)
{
    fprintf(stderr, "  %-30s List available devices, or select one by number, type, or <type>:<id>\n",
            "-d, --device <selection>");
}

void app_print_version_usage_line(void)
{
    fprintf(stderr, "  %-30s Print version and exit\n", "-V, --version");
}

void app_print_debug_usage_line(void)
{
    fprintf(stderr, "  %-30s Print block-drop diagnostics\n", "--debug");
}

void app_print_help_usage_line(void)
{
    fprintf(stderr, "  %-30s Print this help and exit\n", "-h, --help");
}

void app_print_other_usage_lines(void)
{
    fprintf(stderr, "\nOther Options:\n");
    app_print_version_usage_line();
    app_print_help_usage_line();
}

void app_print_ac_errors_usage_line(void)
{
    fprintf(stderr, "  %-30s Max access-code bit errors (default: 0, strict)\n",
            "--ac-errors N");
}

void app_print_enforce_crc_usage_line(void)
{
    fprintf(stderr, "  %-30s Drop LE frames whose CRC fails (default: on)\n",
            "--enforce-crc on|off");
}

void app_print_exhaustive_usage_line(void)
{
    fprintf(stderr, "  %-30s Replay as fast as consumers allow, never drop (default: realtime)\n",
            "--exhaustive");
}

void app_print_gain_usage_line(void)
{
    radio_print_gain_usage();
}

radio_device_type_t app_default_device_type(void)
{
#if HAVE_HACKRF
    return RADIO_DEVICE_HACKRF;
#elif HAVE_BLADERF
    return RADIO_DEVICE_BLADERF;
#else
    return RADIO_DEVICE_FILE;
#endif
}

void app_print_gain_summary(radio_device_type_t type,
                            const radio_gain_spec_t *spec)
{
    int present = spec && spec->present;
    switch (type)
    {
    case RADIO_DEVICE_HACKRF:
        printf("Gain        : LNA %d dB, VGA %d dB, AMP %s%s\n",
               spec ? spec->hackrf_lna : RADIO_HACKRF_LNA_DEFAULT,
               spec ? spec->hackrf_vga : RADIO_HACKRF_VGA_DEFAULT,
               (spec && spec->hackrf_amp) ? "on" : "off",
               present ? "" : " (default)");
        break;
    case RADIO_DEVICE_BLADERF:
        printf("Gain        : %d dB%s\n",
               spec ? spec->bladerf_gain_db : RADIO_BLADERF_GAIN_DEFAULT,
               present ? "" : " (default)");
        break;
    default:
        printf("Gain        : (n/a - replay)\n");
        break;
    }
}

int app_resolve_gain_spec(const char *argv0, radio_device_type_t type,
                          const char *raw, radio_gain_spec_t *out)
{
    char err[128];

    if (!out)
        return -1;
    if (!raw)
    {
        radio_gain_default(type, out);
        return 0;
    }
    if (radio_parse_gain_spec(type, raw, out, err, sizeof(err)) != 0)
    {
        const char *type_name = radio_device_type_name(type);
        fprintf(stderr, "Invalid --gain '%s'%s%s%s%s.\n", raw,
                err[0] ? ": " : "", err,
                type_name ? " for " : "", type_name ? type_name : "");
        radio_print_gain_help(type);
        (void)argv0;
        return -1;
    }
    return 0;
}

void app_print_drop_breakdown(const session_drop_breakdown_t *b)
{
    if (!b)
        return;

    /* Each stage reports two sub-reasons:
     *   pool_exhausted : a producer could not allocate a block.
     *   consumer_full  : a reader's queue was full (a consumer fell behind). */
    unsigned long total = b->rf_pool_exhausted + b->rf_consumer_full +
        b->sub_pool_exhausted + b->sub_consumer_full +
        b->out_pool_exhausted + b->out_consumer_full;
    printf("  Dropped blocks by stage (total %lu, %u lane%s):\n",
           total, b->lane_count, b->lane_count == 1u ? "" : "s");
    printf("    rf (radio -> DDC lanes):\n");
    printf("        pool exhausted : %lu   (radio could not allocate an RF block)\n",
           b->rf_pool_exhausted);
    printf("        consumer full  : %lu   (a DDC lane fell behind reading RF)\n",
           b->rf_consumer_full);
    printf("    sub (DDC lane -> PFB lane, intermediate):\n");
    printf("        pool exhausted : %lu   (a DDC lane could not allocate a sub block)\n",
           b->sub_pool_exhausted);
    printf("        consumer full  : %lu   (a PFB lane fell behind reading its sub lane)\n",
           b->sub_consumer_full);
    printf("    out (PFB lane -> channel workers):\n");
    printf("        pool exhausted : %lu   (a PFB lane could not allocate a frame block)\n",
           b->out_pool_exhausted);
    printf("        consumer full  : %lu   (a channel worker fell behind)\n",
           b->out_consumer_full);
    if (b->lane_count > 1u)
    {
        unsigned int n = b->lane_count;
        if (n > CHANNELIZER_SERVICE_MAX_LANES)
            n = CHANNELIZER_SERVICE_MAX_LANES;
        printf("    per-lane (sub pool / sub full / out pool / out full):\n");
        for (unsigned int k = 0u; k < n; k++)
            printf("        lane %u : %lu / %lu / %lu / %lu\n", k,
                   b->sub_pool_exhausted_lane[k],
                   b->sub_consumer_full_lane[k],
                   b->out_pool_exhausted_lane[k],
                   b->out_consumer_full_lane[k]);
    }
}
