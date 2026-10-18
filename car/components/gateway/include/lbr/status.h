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
 *   gw RUN sync sd log=00000012.LBR up 1234s can tec0 boff0
 *   imu_gps RUN sync up 1200s sens 1f/1f cal 1f fw 0.2.0 SIM
 *   stress LOST 12s
 */
size_t lbr_status_format(const lbr_health_record_t *gw, const char *log_name,
                         const lbr_node_table_t *nodes, uint64_t now_us, char *buf, size_t cap);

const char *lbr_node_state_name(uint8_t state);
