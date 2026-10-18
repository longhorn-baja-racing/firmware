#pragma once

#include <stdint.h>

/*
 * Node identity scheme. A node id is the 8-bit source field of every CAN ID
 * this node transmits, so it must be unique on the bus. Ids are part of the
 * shared protocol: APPEND ONLY, never renumber. 0x00 and 0xFF are reserved.
 */
typedef enum {
    LBR_NODE_INVALID       = 0x00,
    LBR_NODE_GATEWAY       = 0x01, /* logger, time master, telemetry */
    LBR_NODE_IMU_GPS       = 0x02,
    LBR_NODE_SUSPENSION    = 0x03,
    LBR_NODE_DRIVER_INPUTS = 0x04,
    LBR_NODE_TEMPERATURE   = 0x05,
    LBR_NODE_FUEL_GAUGE    = 0x06,
    LBR_NODE_STRESS        = 0x07,
    LBR_NODE_DASHBOARD     = 0x08, /* in-cockpit driver display (not lbr-dashboard) */
    LBR_NODE_RESERVED      = 0xFF,
} lbr_node_id_t;

/* Short lowercase name ("imu_gps"), or "?" for an unknown id. */
const char *lbr_node_name(uint8_t node_id);
