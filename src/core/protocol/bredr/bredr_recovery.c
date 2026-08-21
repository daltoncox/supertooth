/**
 * @file bredr_recovery.c
 * @brief Default BR/EDR recovery interface backed by the native backend.
 */

#include "bredr_recovery.h"

#include <stdlib.h>

#include "bredr_recovery_backend.h"
#include "bredr_recovery_native.h"

struct bredr_recovery_state
{
    const bredr_recovery_backend_ops_t *ops;
    bredr_recovery_backend_state_t *backend_state;
};

void bredr_recovery_global_init(uint8_t max_ac_errors)
{
    const bredr_recovery_backend_ops_t *ops = bredr_recovery_default_backend();
    if (!ops || !ops->global_init)
        return;
    ops->global_init(max_ac_errors);
}

bredr_recovery_state_t *bredr_recovery_state_create(uint32_t lap)
{
    bredr_recovery_state_t *state =
        (bredr_recovery_state_t *)calloc(1, sizeof(*state));
    if (!state)
        return NULL;

    state->ops = bredr_recovery_default_backend();
    if (!state->ops || !state->ops->state_create)
    {
        free(state);
        return NULL;
    }

    state->backend_state = state->ops->state_create(lap);
    return state;
}

void bredr_recovery_state_destroy(bredr_recovery_state_t *state)
{
    if (!state)
        return;
    if (state->ops && state->ops->state_destroy)
        state->ops->state_destroy(state->backend_state);
    free(state);
}

void bredr_recovery_state_reset(bredr_recovery_state_t *state, uint32_t lap)
{
    if (!state)
        return;
    if (state->ops && state->ops->state_reset)
        state->ops->state_reset(state->backend_state, lap);
}

int bredr_recovery_process_packet(bredr_recovery_state_t *state,
                                  const bredr_frame_t *frame,
                                  int channel,
                                  uint32_t clkn,
                                  bredr_recovery_result_t *out)
{
    if (!state || !out)
        return 0;
    if (!state->ops || !state->ops->process_packet)
        return 0;

    return state->ops->process_packet(state->backend_state, frame, channel, clkn, out);
}

const bredr_recovery_backend_ops_t *bredr_recovery_default_backend(void)
{
    return bredr_recovery_native_backend();
}
