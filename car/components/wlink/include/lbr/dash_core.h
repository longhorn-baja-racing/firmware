#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/acq.h"
#include "lbr/node.h"
#include "lbr/node_core.h"
#include "lbr/value_table.h"

/*
 * Cockpit dashboard logic (ESP32-C5, own battery), independent of the
 * platform. It has no sensors on the car and no CAN: it shows what the
 * Central sends over WiFi (lbr/dash_link.h) and reports its own battery
 * back. Values older than LBR_DASH_STALE_US read as stale, never as numbers.
 * Times are the dashboard's local clock (it only needs ages).
 */

#define LBR_DASH_STALE_US  2000000u
#define LBR_DASH_MAX_NODES 16u

typedef struct {
    uint8_t  node_id; /* 0 = free */
    uint8_t  state;   /* lbr_node_state_t */
    uint16_t flags;   /* LBR_HF_* */
    uint32_t sensor_status;
    uint64_t rx_us;
} lbr_dash_node_t;

typedef struct {
    void *ctx;
    uint64_t (*mono_us)(void *ctx);
    /* One datagram to the Central; false if it was not sent. */
    bool (*send)(void *ctx, const uint8_t *pkt, size_t len);
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
    void (*log)(void *ctx, lbr_log_level_t level, const char *text);
} lbr_dash_hal_t;

typedef struct {
    const lbr_node_config_t *cfg;
    lbr_dash_hal_t           hal;
    lbr_acq_slot_t           slot[LBR_NODE_MAX_SENSORS];

    /* hal.lock */
    lbr_value_table_t values;
    lbr_dash_node_t   nodes[LBR_DASH_MAX_NODES];
    bool              have_seq;
    uint32_t          last_seq;
    uint64_t          last_rx_us;

    uint32_t    tx_seq;
    atomic_uint packets;
    atomic_uint packet_gaps;
    atomic_uint rejected;
    atomic_uint sent;
    atomic_uint send_failed;
} lbr_dash_core_t;

void lbr_dash_core_init(lbr_dash_core_t *c, const lbr_node_config_t *cfg,
                        const lbr_dash_hal_t *hal);

/* A datagram from the Central, at local time `rx_us`. */
void lbr_dash_core_on_packet(lbr_dash_core_t *c, const uint8_t *buf, size_t len, uint64_t rx_us);

/* Battery (and any other) slot: sample and report to the Central. The
 * dashboard clock is not synced, so its grid is local. */
void     lbr_dash_core_slot_start(lbr_dash_core_t *c, size_t i);
void     lbr_dash_core_slot_tick(lbr_dash_core_t *c, size_t i);
uint64_t lbr_dash_core_slot_next_due(lbr_dash_core_t *c, size_t i);

/* True while the Central's datagrams keep arriving. */
bool lbr_dash_core_link_up(lbr_dash_core_t *c, uint64_t now_us);

/* Copy the display state under the lock (for rendering). */
void lbr_dash_core_snapshot(lbr_dash_core_t *c, lbr_value_table_t *values,
                            lbr_dash_node_t nodes[LBR_DASH_MAX_NODES]);
