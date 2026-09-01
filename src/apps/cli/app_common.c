#include "app_common.h"

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
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
    int enumerated = 0;

    printf("Found Available Devices:\n");
    for (int type = 0; type < (int)RADIO_DEVICE_TYPE_COUNT; type++)
    {
        const char *type_name = radio_device_type_name((radio_device_type_t)type);
        if (!type_name)
            continue;

        char **identifiers = NULL;
        size_t count = 0u;
        int result = radio_list_devices((radio_device_type_t)type,
                                        &identifiers, &count);
        if (result != RADIO_SUCCESS || count == 0u)
        {
            radio_free_device_list(&identifiers, count);
            continue;
        }

        for (size_t i = 0u; i < count; i++)
            printf("%s:%s\n", type_name, identifiers[i] ? identifiers[i] : "");

        radio_free_device_list(&identifiers, count);
        enumerated += (int)count;
    }

    if (enumerated == 0)
        printf("(no devices found)\n");

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

void app_print_device_usage_line(void)
{
    fprintf(stderr, "  %-30s List available devices, or open a specific one\n",
            "-d, --device [<type>:<id>]");
}

void app_print_drop_breakdown(const session_drop_breakdown_t *b)
{
    if (!b)
        return;

    /* Each pool reports two sub-reasons:
     *   pool_exhausted : a producer could not allocate a block.
     *   consumer_full  : a reader's queue was full (a consumer fell behind). */
    printf("  Dropped blocks by pool:\n");
    printf("    rf (radio input, shared by both channelizers):\n");
    printf("        pool exhausted : %lu   (radio could not allocate an RF block)\n",
           b->rf_pool_exhausted);
    printf("        consumer full  : %lu   (a channelizer RF queue fell behind)\n",
           b->rf_consumer_full);
    printf("    bredr_out (BR/EDR channelizer -> channel workers):\n");
    printf("        pool exhausted : %lu   (BR/EDR channelizer could not allocate a frame block)\n",
           b->bredr_out_pool_exhausted);
    printf("        consumer full  : %lu   (a BR/EDR channel worker fell behind)\n",
           b->bredr_out_consumer_full);
    printf("    ble_out (BLE channelizer -> channel workers):\n");
    printf("        pool exhausted : %lu   (BLE channelizer could not allocate a frame block)\n",
           b->ble_out_pool_exhausted);
    printf("        consumer full  : %lu   (a BLE channel worker fell behind)\n",
           b->ble_out_consumer_full);
}
