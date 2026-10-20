#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Node identity scheme (DAQ-WIRING.md sections 4 and 7). Ids 1..6 are the
 * wired CAN controllers and match their "CAN node N" number, which is also
 * the low nibble of their heartbeat id (0x700 + N). Ids 7.. are the wireless
 * nodes; they never transmit on CAN, but samples they took carry their id as
 * the origin. Part of the shared protocol: never renumber or reuse an id.
 * 0x00 and 0xFF are reserved.
 */
typedef enum {
    LBR_NODE_INVALID      = 0x00,
    LBR_NODE_IMU          = 0x01, /* IMU Controller (ESP32-C5): chassis IMU, corner receiver */
    LBR_NODE_POWERTRAIN   = 0x02, /* fuel level, engine speed, CVT speed */
    LBR_NODE_DRIVER_INPUT = 0x03, /* brake pedal, gas pedal, steering angle */
    LBR_NODE_TEMPERATURE  = 0x04, /* four brake temperatures, CVT temperature */
    LBR_NODE_STRESS       = 0x05, /* stress sensors 1..8 */
    LBR_NODE_CENTRAL   = 0x06, /* Central Controller (ESP32-C5): logger, time master, LoRa, GPS */
    LBR_NODE_DASHBOARD = 0x07, /* in-cockpit display over WiFi (not lbr-dashboard) */
    LBR_NODE_CORNER_FL = 0x08, /* Mini ESP32-C5 suspension nodes over WiFi */
    LBR_NODE_CORNER_FR = 0x09,
    LBR_NODE_CORNER_BL = 0x0A,
    LBR_NODE_CORNER_BR = 0x0B,
    LBR_NODE_RESERVED  = 0xFF,
} lbr_node_id_t;

/* Highest wired CAN controller id. */
#define LBR_NODE_CAN_MAX 0x06u

/* Short lowercase name ("powertrain"), or "?" for an unknown id. */
const char *lbr_node_name(uint8_t node_id);

/* True for the wired controllers that transmit on CAN (ids 1..6). */
static inline bool lbr_node_is_can(uint8_t node_id)
{
    return node_id >= LBR_NODE_IMU && node_id <= LBR_NODE_CAN_MAX;
}

/* Corner index 0..3 (FL, FR, BL, BR) of a corner node, or -1. */
static inline int lbr_node_corner_index(uint8_t node_id)
{
    return (node_id >= LBR_NODE_CORNER_FL && node_id <= LBR_NODE_CORNER_BR)
               ? (int)(node_id - LBR_NODE_CORNER_FL)
               : -1;
}
