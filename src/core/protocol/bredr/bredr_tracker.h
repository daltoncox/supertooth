/**
 * @file bredr_tracker.h
 * @brief BR/EDR tracker: owns UAP/clock recovery (via the piconet store) and
 *        exposes protocol-agnostic device + piconet snapshots for polling.
 *
 * A bredr_tracker_t embeds a bredr_piconet_store_t for the proven recovery and
 * clock-tracking logic, and adds:
 *   - stable, monotonic entity ids (piconet_id and per-member device_id),
 *   - epoch-ms timestamp conversion (from the piconet's slot clock),
 *   - two poll functions: get_piconets() and get_devices() (the latter
 *     derives a master device + one device per observed slave LT_ADDR).
 *
 * All access is serialized by an internal mutex, so the getters are safe to
 * call from the application's poll thread while packets are ingested from the
 * session/decoder threads.
 */

#ifndef BREDR_TRACKER_H
#define BREDR_TRACKER_H

#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#include "bredr_piconet_store.h"
#include "receive_event_models.h"
#include "device_models.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Per-piconet auxiliary state owned by the tracker (ids + linkage). */
typedef struct {
    uint64_t piconet_id;          /**< stable id for the piconet */
    uint64_t master_device_id;    /**< stable id for the master member */
    uint64_t slave_device_id[8];  /**< stable id per slave LT_ADDR */
} bredr_tracker_piconet_aux_t;

typedef struct bredr_tracker_t {
    bredr_piconet_store_t store;       /**< recovery / clock tracking */
    bredr_tracker_piconet_aux_t *aux; /**< parallel to store.entries[] */
    size_t aux_cap;
    uint64_t next_id;                 /**< monotonic id allocator */
    uint64_t epoch_base_ms;           /**< wall-clock at first packet */
    uint32_t clk_base_1600;           /**< rx_clk_1600 at first packet */
    int      has_epoch_base;
    pthread_mutex_t lock;
} bredr_tracker_t;

void bredr_tracker_init(bredr_tracker_t *t);
void bredr_tracker_free(bredr_tracker_t *t);

/** Ingest a BR/EDR event; drives recovery + member accounting. Returns the
 *  piconet that received it, or NULL on error. */
bredr_piconet_t *bredr_tracker_add_packet(bredr_tracker_t *t,
                                          const bredr_event_t *event,
                                          int *packet_is_newest_out);

/** Fill out[] with at most max BR/EDR piconet snapshots; returns count. */
size_t bredr_tracker_get_piconets(const bredr_tracker_t *t,
                                  bredr_piconet_snapshot_t *out,
                                  size_t max);

/** Fill out[] with at most max BR/EDR member-device snapshots; returns count. */
size_t bredr_tracker_get_devices(const bredr_tracker_t *t,
                                 bredr_device_snapshot_t *out,
                                 size_t max);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_TRACKER_H */
