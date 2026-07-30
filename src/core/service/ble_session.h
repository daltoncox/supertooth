#ifndef BLE_SESSION_H
#define BLE_SESSION_H

#include <pthread.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ble_channel_processor_new.h"
#include "ble_piconet.h"
#include "receive_event_models.h"
#include "sample_dispatcher.h"
#include "radio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t lo_frequency_hz;
    uint32_t sample_rate_hz;

    unsigned int bottom_le_channel;
    unsigned int le_channel_count;

    struct {
        void (*cb)(const ble_event_t *event, void *user);
        void *user;
    } packet_cb;

    sample_dispatcher_t *dispatcher;

    radio_device_type_t device_type;
    const char *device_id;
    int debug;

    uint32_t lna_gain;
    uint32_t vga_gain;
} ble_session_config_t;

typedef struct ble_session {
    uint32_t lo_frequency_hz;
    uint32_t sample_rate_hz;
    unsigned int bottom_le_channel;
    unsigned int le_channel_count;

    ble_piconet_store_t piconet_store;

    sample_dispatcher_t *dispatcher;

    ble_channel_processor_t *channels;
    size_t channel_count;

    pthread_t *worker_threads;
    _Bool workers_running;

    unsigned int shutdown_requested;

    ble_session_config_t config;

    radio_device_t *device;
} ble_session_t;

int  ble_session_init(ble_session_t *session, const ble_session_config_t *cfg);
int  ble_session_run(ble_session_t *session);
void ble_session_request_stop(ble_session_t *session);
int  ble_session_destroy(ble_session_t *session);

void ble_session_process_ble_event(ble_session_t *session, const ble_event_t *event);

#ifdef __cplusplus
}
#endif

#endif