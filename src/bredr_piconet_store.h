/**
 * @file bredr_piconet_store.h
 * @brief Dynamic collection of BR/EDR piconets with integrated libbtbb UAP
 *        and clock recovery.
 *
 * Overview
 * --------
 * A `bredr_piconet_store_t` manages a dynamic, LAP-keyed collection of
 * `bredr_piconet_t` objects.  Callers feed it raw `bredr_packet_t` objects via
 * `bredr_piconet_store_add_packet()`; the store routes each packet to the
 * matching piconet (creating one on first sight) and internally drives libbtbb
 * UAP and clock-offset recovery.
 *
 * libbtbb integration
 * -------------------
 * Each entry in the store owns a libbtbb piconet object that is used to
 * accumulate header packets for UAP recovery.  Once libbtbb reports a valid
 * UAP, the store immediately scans all 64 possible CLK1-6 values using HEC
 * verification to find the clock, then calls `bredr_piconet_set_uap()` to put
 * the `bredr_piconet_t` into clock-tracking mode.
 *
 * libbtbb is the only external dependency added here.  No other file in
 * supertooth needs to include or call btbb directly.
 *
 * Idle reset
 * ----------
 * If a LAP is silent for more than ~5.1 seconds (16384 CLKN half-slot ticks),
 * the libbtbb piconet state is reset so that UAP recovery restarts cleanly
 * when the piconet reappears.  The `bredr_piconet_t` ring buffer and
 * statistics are preserved.
 */

#ifndef BREDR_PICONET_STORE_H
#define BREDR_PICONET_STORE_H

#include <stddef.h>
#include <stdint.h>
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
 * Calls btbb_init() internally.  Must be called exactly once before any
 * other store function.
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
 * @brief Route a packet to its piconet, driving UAP/clock recovery.
 *
 * Searches the store for a piconet matching pkt->lap.  If not found, a new
 * piconet is heap-allocated, initialised, and appended to the store.
 *
 * After adding the packet, if the packet has a clean decoded header
 * (has_header && ac_errors == 0) and the piconet does not yet have both UAP
 * and CLK1-6, the packet is fed to libbtbb.  Once libbtbb reports a valid
 * UAP, the store scans all 64 CLK1-6 candidates using HEC verification and
 * calls bredr_piconet_set_uap() on the piconet.
 *
 * @param store    Initialised store.  Must not be NULL.
 * @param pkt      Packet to add.  Must not be NULL.
 * @param channel  BR/EDR channel (0–79) on which the packet was received.
 *                 Required for btbb packet construction.
 * @param clkn     Packet timestamp in CLKN half-slot ticks (312.5 us).
 * @return         Pointer to the piconet that received the packet, or NULL on
 *                 allocation failure.
 */
bredr_piconet_t *bredr_piconet_store_add_packet(bredr_piconet_store_t *store,
                                                  const bredr_packet_t *pkt,
                                                  int channel,
                                                  uint32_t clkn);

/**
 * @brief Print a summary of every piconet in the store to stdout.
 *
 * @param store  Must not be NULL.
 */
void bredr_piconet_store_print(const bredr_piconet_store_t *store);

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

#ifdef __cplusplus
}
#endif

#endif /* BREDR_PICONET_STORE_H */
