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

/* Session-type indices mirror CaptureView.qml sessionTypeSelector. */
#define BACKEND_SESSION_HYBRID 0
#define BACKEND_SESSION_BLE    1
#define BACKEND_SESSION_BREDR  2

/* BLE advertising channel defaults (mirrors CLI). */
#define BACKEND_BLE_CH37 37u
#define BACKEND_BLE_CH38 38u
#define BACKEND_BLE_CH39 39u

/* Channel-layout grids (mirror RECEIVER_BREDR_GRID_* in the core). */
#define BACKEND_GRID_BREDR 0
#define BACKEND_GRID_LE    1

#define BACKEND_ADDR_TEXT_LEN   32
#define BACKEND_TYPE_TEXT_LEN   32
#define BACKEND_INFO_TEXT_LEN   160
#define BACKEND_TIME_TEXT_LEN   32
#define BACKEND_PROTO_TEXT_LEN  8
#define BACKEND_NAME_TEXT_LEN   72
#define BACKEND_MANUF_TEXT_LEN  40
#define BACKEND_DETAIL_KEY_LEN  40
#define BACKEND_DETAIL_VAL_LEN  192
#define BACKEND_DETAIL_MAX      32
#define BACKEND_RAW_MAX_BYTES   512

/**
 * A fully decoded, display-ready row for either a BLE or BR/EDR frame. All
 * strings are NUL-terminated. The callback receives a pointer to a
 * stack-local instance; the receiver must copy out anything it needs before
 * returning.
 */
typedef struct
{
    unsigned long no;
    char time[BACKEND_TIME_TEXT_LEN];
    float rssi_db;            /* Per-frame RSSI in dBr (NaN if invalid). */
    char proto[BACKEND_PROTO_TEXT_LEN];
    unsigned int ch_idx;
    char addr[BACKEND_ADDR_TEXT_LEN];
    char src[BACKEND_ADDR_TEXT_LEN];
    char dst[BACKEND_ADDR_TEXT_LEN];
    char type[BACKEND_TYPE_TEXT_LEN];
    /* BLE advertising address subtype: one of "PUBLIC", "STATIC",
     * "RESOLVABLE", "NONRESOLVABLE", "RESERVED"; "" for non-BLE-adv rows
     * (data-channel PDUs, BR/EDR frames, decode failures). */
    char addr_type[BACKEND_TYPE_TEXT_LEN];
    char info[BACKEND_INFO_TEXT_LEN];

    /* Raw captured on-air bytes (BLE: preamble+AA+PDU; BR/EDR: header FEC + payload). */
    uint8_t  raw[BACKEND_RAW_MAX_BYTES];
    unsigned int raw_len;

    /* Key/value pairs for the Frame Info pane (selected-row detail). */
    unsigned int detail_count;
    char detail_keys[BACKEND_DETAIL_MAX][BACKEND_DETAIL_KEY_LEN];
    char detail_vals[BACKEND_DETAIL_MAX][BACKEND_DETAIL_VAL_LEN];
} backend_row_t;

typedef void (*backend_row_fn)(const backend_row_t *row, void *user);

/**
 * A polled, display-ready device/piconet entity (one row in the device list).
 * Produced by backend_session_poll_entities() from the core trackers. All
 * strings are NUL-terminated. The callback receives a pointer valid only for
 * the duration of the call; copy out anything needed.
 */
typedef struct
{
    uint64_t id;                       /**< stable entity id (matches core) */
    int      kind;                    /**< entity_kind_t: 0=BR/EDR dev,
                                          1=BR/EDR piconet, 2=BLE dev,
                                          3=BLE piconet */
    char     proto[BACKEND_PROTO_TEXT_LEN];   /**< "BR/EDR" or "LE" */
    char     addr[BACKEND_ADDR_TEXT_LEN];     /**< core addr_str */
    char     device[BACKEND_TYPE_TEXT_LEN];   /**< core label */
    char     addr_type[BACKEND_TYPE_TEXT_LEN];/**< BLE address subtype */
    char     name[BACKEND_NAME_TEXT_LEN];     /**< BLE local name (if any) */
    char     manufacturer[BACKEND_MANUF_TEXT_LEN]; /**< BLE manufacturer */
    float    rssi_db;                  /**< 1 s average RSSI (NaN if invalid) */
    int      rssi_valid;              /**< 0 => no signal yet */
    uint64_t first_seen_ms;
    uint64_t last_seen_ms;
    unsigned long total_packets;
    unsigned int   packet_rate;
    uint32_t crc_init;              /**< BLE connection CRCInit (0 if not found) */
    int      crc_init_confirmed;    /**< 0 while CRCInit is still unconfirmed */
    unsigned int crc_init_candidates; /**< distinct CRCInit candidates accumulated */
} backend_entity_t;

/**
 * Invoked (on the session worker thread) the moment the capture loop ends,
 * BEFORE the blocking radio teardown. Lets the GUI flip to "stopped"
 * immediately rather than waiting on device shutdown.
 */
typedef void (*backend_stopped_fn)(void *user);

/** Opaque session handle (hides session_t). */
typedef struct backend_session backend_session_t;

backend_session_t *backend_session_create(void);
void backend_session_destroy(backend_session_t *session);

/** Enable verbose backend/session debug logging (channel-processor frame
 *  emits, etc.). Must be called before starting a session. */
void backend_set_debug(int on);

/**
 * Register the capture-stopped callback. @p on_stopped is invoked on the
 * session worker thread just before the blocking radio teardown begins.
 */
void backend_session_set_stopped_callback(backend_session_t *session,
                                          backend_stopped_fn on_stopped,
                                          void *user);

/**
 * Start a blocking BLE receive session on the given LE channel window:
 * le_channel_count consecutive LE RF channels (1..10) starting at
 * bottom_le_rf (0..39). The radio tunes a whole-MHz LO at the window
 * center; only advertising RF channels (0/12/39 -> LE 37/38/39) are
 * decoded, data channels stay idle. Values are clamped defensively.
 * Blocks until backend_session_request_stop() is called (call from a
 * worker thread). For each decoded frame, @p on_row is invoked on the
 * session worker thread with a populated row.
 *
 * @param input_type  BACKEND_INPUT_HACKRF or BACKEND_INPUT_FILE.
 * @param device_id   HackRF identifier (NULL = default). Ignored for FILE.
 * @return 0 on clean stop, negative on failure to start.
 */
int backend_session_run_ble(backend_session_t *session,
                            unsigned int bottom_le_rf,
                            unsigned int le_channel_count,
                            int input_type,
                            const char *device_id,
                            int enforce_crc,
                            backend_row_fn on_row,
                            void *user);

/**
 * Start a blocking BR/EDR (Classic Bluetooth) receive session on the given
 * channel window: channel_count consecutive BR/EDR channels (even, 2..20)
 * starting at bottom_channel (0..78, bottom+count-1 <= 78). Values are
 * clamped defensively. Blocks until backend_session_request_stop() is
 * called (call from a worker thread). For each decoded frame, @p on_row is
 * invoked on the session worker thread with a populated row.
 *
 * @param input_type  BACKEND_INPUT_HACKRF or BACKEND_INPUT_FILE.
 * @param device_id   HackRF identifier (NULL = default). Ignored for FILE.
 * @return 0 on clean stop, negative on failure to start.
 */
int backend_session_run_bredr(backend_session_t *session,
                              unsigned int channel_count,
                              unsigned int bottom_channel,
                              int input_type,
                              const char *device_id,
                              backend_row_fn on_row,
                              void *user);

/**
 * Start a blocking hybrid (BR/EDR + BLE) receive session from a single
 * stream covering the configured channel window. The grid selects the LO
 * alignment: BACKEND_GRID_BREDR = window is channel_count MHz starting at
 * bottom_channel (LO at a half-MHz); BACKEND_GRID_LE = window is
 * channel_count+1 MHz starting at bottom_channel (LO at a whole MHz;
 * channel_count is odd and bottom_channel even, since the two BR/EDR
 * channels centered on the Nyquist edges are not processed). ble_channel
 * is the advertising channel (37/38/39) whose center lies inside the
 * window, or 0 to leave the BLE worker idle. Values are clamped/validated
 * defensively. Blocks until backend_session_request_stop() is called
 * (call from a worker thread). Both BR/EDR and BLE decoded frames are
 * delivered through @p on_row.
 *
 * @param input_type  BACKEND_INPUT_HACKRF or BACKEND_INPUT_FILE.
 * @param device_id   HackRF identifier (NULL = default). Ignored for FILE.
 * @return 0 on clean stop, negative on failure to start.
 */
int backend_session_run_hybrid(backend_session_t *session,
                               unsigned int channel_count,
                               unsigned int bottom_channel,
                               int le_grid,
                               uint8_t ble_channel,
                               int input_type,
                               const char *device_id,
                               int enforce_crc,
                               backend_row_fn on_row,
                               void *user);

/** Request a running session to stop (safe to call from another thread). */
void backend_session_request_stop(backend_session_t *session);

/**
 * Snapshot the current device/piconet entities from the core trackers into
 * @p out (capacity @p max). Returns the number of entities written. Caller
 * provides the buffer; safe to call from the GUI thread while a capture runs
 * (the core getters lock internally).
 */
size_t backend_session_poll_entities(backend_session_t *session,
                                     backend_entity_t *out, size_t max);

#ifdef __cplusplus
}
#endif

#endif /* BACKEND_API_H */
