#pragma once

#include <stddef.h>
#include <stdint.h>

#include "lbr/health.h"
#include "lbr/node_table.h"

/*
 * Human-readable bus status: one line for the gateway, then one per node.
 * Shared by the BLE status characteristic, the WiFi status page, and the
 * USB STAT command so all three show the same thing. ASCII only.
 *
 *   central RUN sync sd up 1234s log=00000012.lbrlog can tec0 boff0 bus 45.9%
 *   imu RUN sync up 1200s sens 1 cal 1 fw 0.2.0 SIM
 *   stress LOST 12s
 *   temperature MISSING (never heard)
 *
 * bus_load_permille: the measured bus load (LBR_STATUS_LOAD_UNKNOWN: not shown).
 */
#define LBR_STATUS_LOAD_UNKNOWN 0xFFFFu

size_t lbr_status_format(const lbr_health_record_t *gw, const char *log_name,
                         uint16_t bus_load_permille, const lbr_node_table_t *nodes, uint64_t now_us,
                         char *buf, size_t cap);

const char *lbr_node_state_name(uint8_t state);
