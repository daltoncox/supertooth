#ifndef CLI_MODES_H
#define CLI_MODES_H

/* Entry points for the multiplexed `supertooth` CLI. Each mode owns
 * argv[1..] after the dispatcher strips the mode word; each one sets its
 * own "supertooth <mode>" display name for usage/diagnostic messages.
 * Return process exit status. */
int ble_main(int argc, char *argv[]);
int bredr_main(int argc, char *argv[]);
int hybrid_main(int argc, char *argv[]);
int record_main(int argc, char *argv[]);

#endif /* CLI_MODES_H */
