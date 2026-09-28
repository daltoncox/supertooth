#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "version.h"

/* Mode entry points (cli_modes.c). Each mode owns argv[1..] after the
 * dispatcher strips the mode word. */
int ble_main(int argc, char *argv[]);
int bredr_main(int argc, char *argv[]);
int hybrid_main(int argc, char *argv[]);
int record_main(int argc, char *argv[]);

static void print_top_level_help(const char *prog)
{
    printf("Supertooth - software-defined Bluetooth receiver\n");
    printf("please select an operating mode\n\n");
    printf("Usage: %s <mode> [options]\n\n", prog);
    printf("Primary modes:\n");
    printf("  %-16s simultaneous BR/EDR multichannel + LE processing from a shared stream\n",
           "hybrid");
    printf("  %-16s LE capture/decoder over a window of LE channels\n",
           "le");
    printf("  %-16s BR/EDR multichannel receiver with piconet tracking\n",
           "bredr");
    printf("\nOther modes:\n");
    printf("  %-16s record raw IQ to a WAV file in the current directory (no decoding)\n",
           "record");
    printf("\nOther Options:\n");
    printf("  %-16s Print version and exit\n",
           "-V, --version");
    printf("  %-16s Print this help and exit\n",
           "-h, --help");
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

    if (strcmp(mode, "-V") == 0 || strcmp(mode, "--version") == 0 ||
        strcmp(mode, "version") == 0)
    {
        printf("supertooth %s\n", supertooth_get_version());
        return EXIT_SUCCESS;
    }

    /* Strip the mode word: the mode entry point sees its options in
     * argv[1..] with argv[0] still pointing at the mode word. Each mode
     * overwrites argv[0] with its "supertooth <mode>" display name, so
     * getopt parsing is unaffected. "ble" remains a hidden alias for "le",
     * and "classic" remains a hidden alias for "bredr": both work but are
     * intentionally omitted from the help above. */
    if (strcmp(mode, "hybrid") == 0)
        return hybrid_main(argc - 1, argv + 1);
    if (strcmp(mode, "le") == 0 || strcmp(mode, "ble") == 0)
        return ble_main(argc - 1, argv + 1);
    if (strcmp(mode, "bredr") == 0 || strcmp(mode, "classic") == 0)
        return bredr_main(argc - 1, argv + 1);
    if (strcmp(mode, "record") == 0)
        return record_main(argc - 1, argv + 1);

    fprintf(stderr, "Unknown mode: %s\n\n", mode);
    fprintf(stderr, "please select an operating mode: hybrid, le, bredr, record\n");
    fprintf(stderr, "Run '%s -h' to list operating modes.\n", prog);
    return EXIT_FAILURE;
}
