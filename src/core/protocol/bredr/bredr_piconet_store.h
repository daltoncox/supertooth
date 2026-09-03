/**
 * @file bredr_piconet_store.h
 * @brief Dynamic collection of BR/EDR piconets with integrated recovery state.
 *
 * Overview
 * --------
 * A `bredr_piconet_store_t` manages a dynamic, LAP-keyed collection of
 * `bredr_piconet_t` objects. Callers feed it BR/EDR events via
 * `bredr_piconet_store_add_packet()`; the store routes each event to the
 * matching piconet (creating one on first sight) and internally drives a
 * repository-owned BR/EDR recovery backend.
 *
 * Recovery backend integration
 * ----------------------------
 * Each entry in the store owns recovery backend state used to accumulate
 * header packets for UAP recovery.  The backend is repository-owned
 * (src/core/protocol/bredr/bredr_clock_recovery.c); the dependency on
 * external libraries is intentionally hidden behind repository-owned wrappers.
 * Once the backend reports a valid UAP, the store scans all 64 CLK1-6
 * candidates using HEC verification and calls `bredr_piconet_set_uap()` to
 * put the `bredr_piconet_t` into clock-tracking mode.
 *
 * Idle reset
 * ----------
 * If a LAP is silent for more than ~5.1 seconds (16384 CLKN half-slot ticks),
 * the recovery state is reset so that UAP recovery restarts cleanly
 * when the piconet reappears.  The `bredr_piconet_t` ring buffer and
 * statistics are preserved.
 */

#ifndef BREDR_PICONET_STORE_H
#define BREDR_PICONET_STORE_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "bredr_piconet.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ---------------------------------------------------------------------------
 * Constants
 * ---------------------------------------------------------------------------*/

/** Initial capacity of the store's entry array. */
#define BREDR_PICONET_STORE_INIT_CAP 8u

/* ---------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------------*/

/**
 * Internal per-piconet entry; opaque to callers.
 * Defined in bredr_piconet_store.c.
 */
typedef struct bredr_piconet_store_entry bredr_piconet_store_entry_t;

/**
 * @brief A dynamic collection of observed piconets, keyed by LAP.
 *
 * Always initialise with bredr_piconet_store_init() before use.
 */
typedef struct
{
    /** Heap-allocated array of per-piconet entries. */
    bredr_piconet_store_entry_t *entries;

    /** Number of piconets currently tracked. */
    size_t count;

    /** Allocated capacity of entries[]. */
    size_t capacity;

} bredr_piconet_store_t;

/* ---------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------*/

/**
 * @brief Initialise an empty piconet store.
 *
 * Initialises the currently configured BR/EDR recovery backend and the store.
 * Must be called exactly once before any other store function.
 *
 * @param store  Must not be NULL.
 */
void bredr_piconet_store_init(bredr_piconet_store_t *store);

/**
 * @brief Release all heap memory owned by the store and its piconets.
 *
 * The store itself is zeroed; it may be re-initialised with
 * bredr_piconet_store_init() after this call.
 *
 * @param store  Must not be NULL.
 */
void bredr_piconet_store_free(bredr_piconet_store_t *store);

/**
 * @brief Route an event to its piconet, driving UAP/clock recovery.
 *
 * Searches the store for a piconet matching event->frame.lap. If not found, a new
 * piconet is heap-allocated, initialised, and appended to the store.
 *
 * After adding the event, if the frame has a decoded header (has_header != 0)
 * and the piconet does not yet have both UAP and CLK1-6, the frame is fed to
 * the active BR/EDR recovery backend. Access-code acceptance (and its error
 * tolerance) is enforced solely by the bitstream decoder, so any frame that
 * reaches this layer with a header has already passed that gate. Once a UAP is
 * reported, the store scans all 64 CLK1-6 candidates using HEC verification
 * and calls bredr_piconet_set_uap() on the piconet.
 *
 * @param store    Initialised store.  Must not be NULL.
 * @param event    BR/EDR event to add. Must not be NULL.
 * @return         Pointer to the piconet that received the packet, or NULL on
 *                 allocation failure.
 */
bredr_piconet_t *bredr_piconet_store_add_packet(bredr_piconet_store_t *store,
                                                const bredr_event_t *event,
                                                int *packet_is_newest);

/**
 * @brief Return the number of piconets currently tracked by the store.
 *
 * @param store  Must not be NULL.
 * @return       Number of piconets, or 0 if store is NULL.
 */
size_t bredr_piconet_store_count(const bredr_piconet_store_t *store);

/**
 * @brief Get a piconet by index (0..count-1) from the store.
 *
 * The returned pointer is owned by the store and remains valid until the
 * store is modified or freed.
 *
 * @param store  Must not be NULL.
 * @param index  Piconet index in store insertion order.
 * @return       Piconet pointer, or NULL if index is out of range.
 */
const bredr_piconet_t *bredr_piconet_store_get(const bredr_piconet_store_t *store,
                                                size_t index);

/**
 * @brief Enable recording of every recovered frame to a binary dump file.
 *
 * When @p file is non-NULL, each frame fed to UAP acquisition is appended to
 * it in a self-describing binary format (magic + version + lap + channel +
 * clkn + header_raw + air_payload) so it can be replayed offline through the
 * recovery backend for regression testing on a real capture.  Pass NULL to
 * disable.  The file is owned by the caller, who is responsible for opening it
 * (e.g. "wb"/"ab") and closing it; the store never closes it.
 *
 * @param store  Initialised store.  May be NULL (no-op).
 * @param file   Opened FILE for writing, or NULL to disable.
 */
void bredr_piconet_store_set_frame_dump(bredr_piconet_store_t *store, FILE *file);

#ifdef __cplusplus
}
#endif

#endif /* BREDR_PICONET_STORE_H */
