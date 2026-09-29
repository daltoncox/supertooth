#ifndef BREDR_DISPLAY_H
#define BREDR_DISPLAY_H

#include <stddef.h>
#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "receive_event_models.h"
#include "device_models.h"

void bredr_print_packet_details(const bredr_frame_t *frame,
                                const bredr_connection_snapshot_t *connection,
                                const rx_metadata_t *meta);
void bredr_print_packet_summary_line(unsigned long packet_no,
                                     const bredr_frame_t *frame,
                                     const bredr_connection_snapshot_t *connection,
                                     const rx_metadata_t *meta);
void bredr_print_connection_snapshot(const bredr_connection_snapshot_t *connection);
void bredr_print_rssi_snapshot(unsigned long packet_no,
                               const bredr_frame_t *frame,
                               const rx_metadata_t *meta,
                               const bredr_connection_snapshot_t *const *connections,
                               size_t count,
                               unsigned int master_clock_mhz);

#endif
