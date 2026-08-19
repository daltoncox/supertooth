/**
 * @file rssi_window.h
 * @brief Rolling 1-second RSSI averaging window.
 *
 * Maintained per observed entity (piconet/connection, or a piconet member).
 * On each packet with a valid RSSI sample the window is topped up; the window
 * resets every 1000 ms so the reported average always reflects roughly the
 * last second of traffic. No heap allocation; safe to embed by value.
 */

#ifndef RSSI_WINDOW_H
#define RSSI_WINDOW_H

#include <stdint.h>

typedef struct
{
    double    sum;          /**< sum of samples in the current window */
    unsigned long count;    /**< sample count in the current window */
    uint64_t  win_start_ms; /**< epoch ms the current window opened */
} rssi_window_state_t;

/** Feed one RSSI sample (dB) into the window at wall-clock time @p now_ms. */
static inline void rssi_window_add(rssi_window_state_t *w,
                                   double rssi, uint64_t now_ms)
{
    if (w->win_start_ms == 0u)
        w->win_start_ms = now_ms;
    if (now_ms - w->win_start_ms >= 1000u)
    {
        w->sum = 0.0;
        w->count = 0u;
        w->win_start_ms = now_ms;
    }
    w->sum += rssi;
    w->count++;
}

/** Current 1-second average (live partial while a window is in progress).
 *  Returns 0.0 when no sample has ever been fed. */
static inline double rssi_window_avg(const rssi_window_state_t *w)
{
    if (w->count == 0u)
        return 0.0;
    return w->sum / (double)w->count;
}

/** Non-zero once at least one sample has been observed. */
static inline int rssi_window_has_data(const rssi_window_state_t *w)
{
    return w->count > 0u ? 1 : 0;
}

#endif /* RSSI_WINDOW_H */
