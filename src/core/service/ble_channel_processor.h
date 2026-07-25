#ifndef BLE_CHANNEL_PROCESSOR_H
#define BLE_CHANNEL_PROCESSOR_H

#include "receiver_session.h"

/** Compute the BLE session layout (LO, sample rate, decimation) from
 * session->ble_config's channel window. */
void receiver_ble_update_layout(receiver_session_t *session);

/** Set up session->ble_ctx channels for the given pipeline:
 *  - RECEIVER_BLE_PIPELINE_SESSION: le_channel_count channels from
 *    ble_config.bottom_le_channel. Advertising RF channels get full DSP
 *    (NCO + decimation + demod + decoder); data channels stay idle (reader
 *    only, no work).
 *  - RECEIVER_BLE_PIPELINE_HYBRID: a single channel from
 *    hybrid_config.ble_channel, mixed from the bredr wideband layout.
 */
int receiver_ble_channel_processor_setup(receiver_session_t *session,
                                         receiver_ble_pipeline_t pipeline);
void receiver_ble_channel_processor_destroy(receiver_session_t *session);
void receiver_ble_channel_processor_process(ble_channel_processor_t *ble,
                                            sample_block_t *blk);

#endif
