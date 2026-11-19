#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Node identity scheme (DAQ-WIRING.md sections 4 and 7). Ids 1..10 are the
 * CAN nodes and match their "CAN node N" number, which is also the low
 * nibble of their heartbeat id (0x700 + N). The cockpit is the only node
 * off the bus; it never transmits on CAN. Part of the shared protocol:
 * never renumber or reuse an id. 0x00 and 0xFF are reserved.
 */
typedef enum {
    LBR_NODE_INVALID      = 0x00,
    LBR_NODE_IMU          = 0x01, /* IMU Controller (ESP32-C5): chassis IMU */
    LBR_NODE_POWERTRAIN   = 0x02, /* fuel level, engine speed, CVT speed */
    LBR_NODE_DRIVER_INPUT = 0x03, /* brake pedal, gas pedal, steering angle */
    LBR_NODE_TEMPERATURE  = 0x04, /* four brake temperatures, CVT temperature */
    LBR_NODE_STRESS       = 0x05, /* stress sensors 1..8 */
    LBR_NODE_CENTRAL   = 0x06, /* Central Controller (ESP32-C5): logger, time master, LoRa, GPS */
    LBR_NODE_CORNER_FL = 0x07, /* wheel hub nodes (Mini ESP32-C5): one suspension IMU each */
    LBR_NODE_CORNER_FR = 0x08,
    LBR_NODE_CORNER_BL = 0x09,
    LBR_NODE_CORNER_BR = 0x0A,
    LBR_NODE_COCKPIT  = 0x0B, /* driver display in the cockpit, over WiFi (not the dashboard app) */
    LBR_NODE_RESERVED = 0xFF,
} lbr_node_id_t;

/* Highest CAN node id. */
#define LBR_NODE_CAN_MAX 0x0Au

/* Highest assigned node id. */
#define LBR_NODE_LAST LBR_NODE_COCKPIT

/* Short lowercase name ("powertrain"), or "?" for an unknown id. */
const char *lbr_node_name(uint8_t node_id);

/* True for the nodes that transmit on CAN (ids 1..10). */
static inline bool lbr_node_is_can(uint8_t node_id)
{
    return node_id >= LBR_NODE_IMU && node_id <= LBR_NODE_CAN_MAX;
}
