#ifndef RECEIVER_SESSION_H
#define RECEIVER_SESSION_H

#include <complex.h>
#include <pthread.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>

#include <liquid/liquid.h>

#include "bredr_display.h"
#include "bredr_bitstream_decoder.h"
#include "ble_bitstream_decoder.h"
#include "ble_piconet.h"
#include "radio_common.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"
#include "bredr_piconet.h"
#include "bredr_piconet_store.h"

#define RECEIVER_BLE_SAMPLE_RATE 2000000u
#define RECEIVER_BLE_LNA_GAIN 24u
#define RECEIVER_BLE_VGA_GAIN 18u
#define RECEIVER_BLE_SAMPLES_PER_SYMBOL 2u
/* Max LE channel processors in a BLE session: the largest capture window
 * (RECEIVER_BREDR_MAX_CHANNELS MHz) spans 10 LE channels at 2 MHz spacing. */
#define RECEIVER_BLE_MAX_CHANNELS (RECEIVER_BREDR_MAX_CHANNELS / 2u)

#define RECEIVER_BREDR_CHANNEL_0_FREQ 2402000000.0
#define RECEIVER_BREDR_CHANNEL_BW 1000000.0
#define RECEIVER_BREDR_BLOCK_POOL_SIZE 64u
#define RECEIVER_BREDR_CHANNEL_RING_SIZE 8u
#define RECEIVER_BREDR_MAX_CHANNELS 20u
#define RECEIVER_BREDR_DEFAULT_RSSI_AVERAGING_WINDOW 16u
#define RECEIVER_BREDR_LNA_GAIN 24u
#define RECEIVER_BREDR_VGA_GAIN 18u
#define RECEIVER_BREDR_SYMBOL_STEP 2u
#define RECEIVER_BREDR_BUFFER_SIZE SAMPLE_BLOCK_SAMPLE_CAPACITY

#define RECEIVER_HYBRID_DECIMATION 10u
#define RECEIVER_HYBRID_BLE_FREQ_OFFSET_HZ (-9500000.0f)
#define RECEIVER_HYBRID_LO_FREQ_HZ 2411500000ULL
#define RECEIVER_HYBRID_SAMPLE_RATE 20000000u

/* Channel-layout grids for the BR/EDR channelizer. The layout is always N
 * channels at 1 MHz offsets j-(N-1)/2 around the window-center LO; the grid
 * only selects the window width and hence the LO alignment:
 *   - BREDR grid: window = N MHz, LO at a half-MHz frequency.
 *   - LE grid:    window = N+1 MHz, LO at a whole-MHz frequency; the two
 *                 BR/EDR channels that would be centered exactly on the
 *                 Nyquist edges (half cut) get no processor.
 */
#define RECEIVER_BREDR_GRID_BREDR 0u
#define RECEIVER_BREDR_GRID_LE    1u

#define RECEIVER_SOURCE_ID_DEFAULT 0u

typedef struct receiver_session receiver_session_t;

typedef enum
{
    RECEIVER_BLE_PIPELINE_SESSION = 0,  /* wideband channelized, BLE session */
    RECEIVER_BLE_PIPELINE_HYBRID = 1,   /* wideband channelized, hybrid session */
} receiver_ble_pipeline_t;

typedef struct
{
    unsigned int bredr_channel;
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;
    bredr_bitstream_decoder_t proc;
    float complex mixed[RECEIVER_BREDR_BUFFER_SIZE];
    float complex decimated[RECEIVER_BREDR_BUFFER_SIZE];
    windowcf rssi_history;
    unsigned int rssi_history_capacity;
    unsigned int rssi_history_valid;
    uint64_t rssi_history_next_sample;
    float pending_rssi_dbr;
    int pending_rssi_valid;
    bredr_status_t prev_status;
    sample_reader_t reader;
    struct receiver_session *session;
} bredr_channel_processor_t;

typedef struct
{
    nco_crcf nco;
    firdecim_crcf firdec;
    cpfskdem demod;
    ble_bitstream_decoder_t proc;
    float complex mixed[RECEIVER_BREDR_BUFFER_SIZE];
    /* Decimation varies with the capture span (2..10), so size for the
     * worst case (no decimation) rather than RECEIVER_HYBRID_DECIMATION. */
    float complex decimated[RECEIVER_BREDR_BUFFER_SIZE];
    receiver_ble_pipeline_t pipeline;
    unsigned int input_decimation;
    uint32_t input_sample_rate_hz;  /* wideband rate feeding this channel */
    uint32_t center_frequency_hz;
    uint16_t channel_index;
    /* 1 = decode this channel (all channels in the window are decoded;
     * the unified framer handles advertising and data PDUs). */
    unsigned int active;
    unsigned int sample_scale;
    ble_status_t prev_status;
    long long pkt_start_sample;
    sample_reader_t reader;
    unsigned int reader_initialized;
    struct receiver_session *session;
} ble_channel_processor_t;

typedef struct
{
    /* Capture window in LE RF channels: le_channel_count consecutive RF
     * channels (1..RECEIVER_BLE_MAX_CHANNELS) starting at bottom_le_channel
     * (0..39). The radio tunes to the window center (a whole-MHz LO); every
     * channel in the window is decoded (advertising and data alike). */
    unsigned int bottom_le_channel;
    unsigned int le_channel_count;
    radio_device_type_t device_type;
    const char *device_id;
    int debug;
    int enforce_crc;   /* 1 = drop frames whose BLE CRC fails (default on) */
} receiver_ble_config_t;

typedef struct
{
    unsigned int channel_count;
    unsigned int bottom_channel;
    /* RECEIVER_BREDR_GRID_*: BR/EDR grid (default, LO at a half-MHz) or LE
     * grid (window widened by 1 MHz, LO at a whole MHz, edge channels not
     * processed). */
    unsigned int le_grid;
    unsigned int rssi_averaging_window;
    uint32_t lap_filter;
    int lap_filter_enabled;
    radio_device_type_t device_type;
    const char *device_id;
    int debug;
} receiver_bredr_config_t;

typedef struct
{
    unsigned long long total_bits;
    unsigned long total_packets;
    unsigned long header_packets;
    unsigned long id_packets;
    unsigned int channel_count;
} receiver_bredr_stats_t;

typedef struct
{
    /* BR/EDR side: channel_count processors covering a window of
     * channel_count (BR/EDR grid) or channel_count+1 (LE grid) MHz starting
     * at bottom_channel. BLE side: when ble_enabled is nonzero, every LE
     * channel whose center is fully inside the capture window gets a
     * processor (advertising and data alike, via the unified decoder);
     * 0 disables BLE processing. */
    unsigned int channel_count;
    unsigned int bottom_channel;
    unsigned int le_grid;      /* RECEIVER_BREDR_GRID_* */
    int ble_enabled;
    radio_device_type_t device_type;
    const char *device_id;
    int debug;
    int enforce_crc;   /* 1 = drop frames whose BLE CRC fails (default on) */
} receiver_hybrid_config_t;

typedef struct
{
    unsigned long total_packets;
    unsigned int bredr_channel_count;
} receiver_hybrid_stats_t;

typedef bredr_piconet_snapshot_t receiver_bredr_piconet_snapshot_t;

typedef void (*receiver_ble_packet_fn)(const ble_event_t *event,
                                       void *user);

typedef struct
{
    receiver_ble_packet_fn on_packet;
    void *user;
} receiver_ble_callbacks_t;

typedef void (*receiver_bredr_packet_fn)(const bredr_event_t *event,
                                         const receiver_bredr_piconet_snapshot_t *piconet,
                                         void *user);

typedef struct
{
    receiver_bredr_packet_fn on_packet;
    void *user;
} receiver_bredr_callbacks_t;

typedef void (*receiver_hybrid_bredr_packet_fn)(const bredr_event_t *event,
                                                const receiver_bredr_piconet_snapshot_t *piconet,
                                                void *user);

typedef void (*receiver_hybrid_ble_packet_fn)(const ble_event_t *event,
                                              void *user);

typedef struct
{
    receiver_hybrid_bredr_packet_fn on_bredr_packet;
    receiver_hybrid_ble_packet_fn on_ble_packet;
    void *user;
} receiver_hybrid_callbacks_t;

struct receiver_session
{
    volatile sig_atomic_t stop_requested;
    pthread_mutex_t stop_mutex;
    pthread_cond_t stop_cv;
    int debug;

    receiver_ble_config_t ble_config;
    receiver_ble_callbacks_t ble_callbacks;
    ble_piconet_store_t ble_store;
    ble_channel_processor_t *ble_ctx;
    unsigned int ble_ctx_count;
    pthread_t *ble_worker_threads;
    unsigned int ble_worker_count;
    unsigned int ble_worker_running;
    unsigned int ble_shutdown_requested;
    uint64_t ble_lo_freq_hz;
    unsigned int ble_sample_rate;
    unsigned int ble_decim_factor;

    receiver_bredr_config_t bredr_config;
    receiver_bredr_callbacks_t bredr_callbacks;
    bredr_piconet_store_t bredr_store;
    bredr_channel_processor_t *bredr_ctx;
    sample_dispatcher_t sample_dispatcher;
    pthread_t *bredr_worker_threads;
    unsigned int bredr_worker_count;
    unsigned int bredr_sample_rate;
    unsigned int bredr_decim_factor;
    unsigned int bredr_raw_samps_per_bit;
    uint64_t bredr_lo_freq_hz;
    unsigned int bredr_shutdown_requested;
    unsigned long long bredr_total_bits;
    unsigned long bredr_total_packets;
    unsigned long bredr_header_packets;
    unsigned long bredr_id_packets;
    pthread_mutex_t decoded_packet_mutex;

    receiver_hybrid_config_t hybrid_config;
    receiver_hybrid_callbacks_t hybrid_callbacks;
    pthread_t *hybrid_worker_threads;
    unsigned int hybrid_worker_count;
    unsigned int hybrid_ble_enabled;
    unsigned int hybrid_shutdown_requested;
    unsigned long hybrid_total_packets;
};

rx_metadata_t receiver_make_metadata(uint64_t radio_start_sample_index,
                                     uint32_t radio_sample_rate_hz,
                                     uint32_t center_frequency_hz,
                                     uint16_t channel_index,
                                     float rssi_dbr);
void receiver_fill_bredr_piconet_snapshot(const bredr_piconet_t *pnet,
                                          receiver_bredr_piconet_snapshot_t *out);
void receiver_bredr_session_init(receiver_session_t *session,
                                 const receiver_bredr_config_t *config,
                                 const receiver_bredr_callbacks_t *callbacks);

receiver_session_t *receiver_session_create(void);
void receiver_session_destroy(receiver_session_t *session);
void receiver_session_request_stop(receiver_session_t *session);

int receiver_session_run_ble(receiver_session_t *session,
                             const receiver_ble_config_t *config,
                             const receiver_ble_callbacks_t *callbacks);

int receiver_session_run_bredr(receiver_session_t *session,
                               const receiver_bredr_config_t *config,
                               const receiver_bredr_callbacks_t *callbacks,
                               receiver_bredr_stats_t *stats_out);

int receiver_session_run_hybrid(receiver_session_t *session,
                                const receiver_hybrid_config_t *config,
                                const receiver_hybrid_callbacks_t *callbacks,
                                receiver_hybrid_stats_t *stats_out);

size_t receiver_session_bredr_piconet_count(receiver_session_t *session);
int receiver_session_bredr_piconet_snapshot(receiver_session_t *session,
                                            size_t index,
                                            receiver_bredr_piconet_snapshot_t *out);
unsigned long receiver_session_dispatcher_dropped_blocks(receiver_session_t *session);

#endif
