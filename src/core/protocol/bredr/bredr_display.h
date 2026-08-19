#ifndef BREDR_DISPLAY_H
#define BREDR_DISPLAY_H

#include <stddef.h>
#include <stdint.h>

#include "bredr_bitstream_decoder.h"
#include "receive_event_models.h"
#include "device_models.h"

void bredr_print_packet_details(const bredr_frame_t *frame,
                                const bredr_piconet_snapshot_t *pnet,
                                const rx_metadata_t *meta);
void bredr_print_packet_summary_line(unsigned long packet_no,
                                     const bredr_frame_t *frame,
                                     const bredr_piconet_snapshot_t *pnet,
                                     const rx_metadata_t *meta);
void bredr_print_piconet_snapshot(const bredr_piconet_snapshot_t *pnet);
void bredr_print_rssi_snapshot(unsigned long packet_no,
                               const bredr_frame_t *frame,
                               const rx_metadata_t *meta,
                               const bredr_piconet_snapshot_t *const *piconets,
                               size_t count,
                               unsigned int master_clock_mhz);

#endif
