/**
 * @file backend_api.h
 * @brief Opaque C facade between the Qt/QML GUI and the C core.
 *
 * The core headers pull in C99 `_Complex` and <liquid/liquid.h>, which are not
 * valid C++. This facade is the only surface the GUI's C++ side is allowed to
 * include. It exposes only primitive types and opaque pointers; all session
 * lifecycle, frame decoding, and string formatting happens on the C side
 * (compiled as C) and is delivered to the GUI as ready-to-display POD rows.
 *
 * Note: the data crossing this boundary is decoded events/frames, not raw IQ
 * samples. HackRF streaming stays entirely inside the session.
 */
#ifndef BACKEND_API_H
#define BACKEND_API_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Input-type indices mirror Header.qml inputTypeSelector. */
#define BACKEND_INPUT_HACKRF 0
#define BACKEND_INPUT_FILE   1

/* BLE advertising channel defaults (mirrors CLI). */
#define BACKEND_BLE_CH37 37u
#define BACKEND_BLE_CH38 38u
#define BACKEND_BLE_CH39 39u

#define BACKEND_ADDR_TEXT_LEN   32
#define BACKEND_TYPE_TEXT_LEN   32
#define BACKEND_INFO_TEXT_LEN   160
#define BACKEND_TIME_TEXT_LEN   32
#define BACKEND_RSSI_TEXT_LEN   16
#define BACKEND_PROTO_TEXT_LEN  8
#define BACKEND_DETAIL_KEY_LEN  40
#define BACKEND_DETAIL_VAL_LEN  192
#define BACKEND_DETAIL_MAX      32
#define BACKEND_RAW_MAX_BYTES   512

/**
 * A fully decoded, display-ready BLE row. All strings are NUL-terminated.
 * The callback receives a pointer to a stack-local instance; the receiver must
 * copy out anything it needs before returning.
 */
typedef struct
{
    unsigned long no;
    char time[BACKEND_TIME_TEXT_LEN];
    char rssi[BACKEND_RSSI_TEXT_LEN];
    char proto[BACKEND_PROTO_TEXT_LEN];
    unsigned int ch_idx;
    char addr[BACKEND_ADDR_TEXT_LEN];
    char src[BACKEND_ADDR_TEXT_LEN];
    char dst[BACKEND_ADDR_TEXT_LEN];
    char type[BACKEND_TYPE_TEXT_LEN];
    char info[BACKEND_INFO_TEXT_LEN];

    /* Raw captured bytes: preamble + access address + PDU (incl. CRC). */
    uint8_t  raw[BACKEND_RAW_MAX_BYTES];
    unsigned int raw_len;

    /* Key/value pairs for the Frame Info pane (selected-row detail). */
    unsigned int detail_count;
    char detail_keys[BACKEND_DETAIL_MAX][BACKEND_DETAIL_KEY_LEN];
    char detail_vals[BACKEND_DETAIL_MAX][BACKEND_DETAIL_VAL_LEN];
} backend_ble_row_t;

typedef void (*backend_ble_row_fn)(const backend_ble_row_t *row, void *user);

/** Opaque session handle (hides receiver_session_t). */
typedef struct backend_session backend_session_t;

backend_session_t *backend_session_create(void);
void backend_session_destroy(backend_session_t *session);

/**
 * Start a blocking BLE receive session on the advertising channel @p ble_channel.
 * Blocks until backend_session_request_stop() is called (call from a worker
 * thread). For each decoded frame, @p on_row is invoked on the session worker
 * thread with a populated row.
 *
 * @param input_type  BACKEND_INPUT_HACKRF or BACKEND_INPUT_FILE.
 * @param device_id   HackRF identifier (NULL = default). Ignored for FILE.
 * @return 0 on clean stop, negative on failure to start.
 */
int backend_session_run_ble(backend_session_t *session,
                            uint8_t ble_channel,
                            int input_type,
                            const char *device_id,
                            backend_ble_row_fn on_row,
                            void *user);

/** Request a running session to stop (safe to call from another thread). */
void backend_session_request_stop(backend_session_t *session);

#ifdef __cplusplus
}
#endif

#endif /* BACKEND_API_H */
