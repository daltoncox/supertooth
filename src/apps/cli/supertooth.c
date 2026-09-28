#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cli_modes.h"
#include "version.h"

static void print_top_level_help(const char *prog)
{
    printf("Supertooth - software-defined Bluetooth receiver\n");
    printf("please select an operating mode\n\n");
    printf("Usage: %s <mode> [options]\n\n", prog);
    printf("Primary modes:\n");
    printf("  %-10s simultaneous BR/EDR multichannel + BLE processing from a shared stream\n",
           "hybrid");
    printf("  %-10s BLE capture/decoder over a window of LE channels\n",
           "ble");
    printf("  %-10s BR/EDR multichannel receiver with piconet tracking\n",
           "bredr");
    printf("Other modes:\n");
    printf("  %-10s record raw IQ to a WAV file in the current directory (no decoding)\n",
           "record");
    printf("\nRun '%s <mode> -h' for mode-specific options.\n", prog);
    printf("Run '%s --version' to print the version.\n", prog);
}

int main(int argc, char *argv[])
{
    const char *prog = (argc > 0 && argv[0] != NULL) ? argv[0] : "supertooth";

    if (argc < 2)
    {
        print_top_level_help(prog);
        return EXIT_SUCCESS;
    }

    const char *mode = argv[1];

    if (strcmp(mode, "-h") == 0 || strcmp(mode, "--help") == 0 ||
        strcmp(mode, "help") == 0)
    {
        print_top_level_help(prog);
        return EXIT_SUCCESS;
    }

    if (strcmp(mode, "-V") == 0 || strcmp(mode, "--version") == 0)
    {
        printf("supertooth %s\n", supertooth_get_version());
        return EXIT_SUCCESS;
    }

    /* Strip the mode word: the mode entry point sees its options in
     * argv[1..] with argv[0] still pointing at the mode word. Each mode
     * overwrites argv[0] with its "supertooth <mode>" display name, so
     * getopt parsing is unaffected. */
    if (strcmp(mode, "hybrid") == 0)
        return hybrid_main(argc - 1, argv + 1);
    if (strcmp(mode, "ble") == 0)
        return ble_main(argc - 1, argv + 1);
    if (strcmp(mode, "bredr") == 0)
        return bredr_main(argc - 1, argv + 1);
    if (strcmp(mode, "record") == 0)
        return record_main(argc - 1, argv + 1);

    fprintf(stderr, "Unknown mode: %s\n\n", mode);
    fprintf(stderr, "please select an operating mode: hybrid, ble, bredr, record\n");
    fprintf(stderr, "Run '%s -h' to list operating modes.\n", prog);
    return EXIT_FAILURE;
}
