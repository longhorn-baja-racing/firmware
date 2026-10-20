#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/node_table.h"
#include "lbr/protocol.h"
#include "lbr/sensor.h"
#include "lbr/value_table.h"

/*
 * Central side of the cockpit dashboard link (pure; host-tested).
 *
 * Down (Central -> dashboard, LBR_LINK_DASH, every dash_period_ms): one
 * datagram with a SAMPLE record for each display signal (lbr_dash_signals,
 * the latest value from the bus; a stale one goes as NO_DATA) and a
 * HEALTH_LITE record per CAN node.
 *
 * Up (dashboard -> Central): SAMPLE records, today only DASH_BATTERY, once a
 * second. The dashboard is up while they keep arriving (within
 * LBR_WLINK_DASH_TIMEOUT_US); the DASH_STATUS frame
 * (0x60A) puts its battery and the link state on CAN, and so in the log, via
 * an ordinary 1 Hz slot whose driver reads this state (lbr_sensor_dash_link).
 *
 * Threading: the caller serializes access (the Central's state lock); the
 * slot driver takes the same lock through the functions it was given.
 */

/* What the cockpit display shows (any source node). Shared with the
 * dashboard app and the simulator. */
extern const lbr_signal_id_t lbr_dash_signals[];
extern const size_t          lbr_dash_signal_count;

typedef struct {
    bool         seen;
    bool         up;
    uint64_t     last_rx_us;
    bool         have_battery;
    lbr_sample_t battery;

    bool     have_seq;
    uint32_t last_seq;
    uint32_t packets;
    uint32_t packet_gaps; /* datagrams from the dashboard that never arrived */
    uint32_t rejected;
    uint32_t tx_seq;

    /* Set by lbr_sensor_dash_link: how its driver locks this state. */
    void (*lock)(void *);
    void (*unlock)(void *);
    void *lock_ctx;
} lbr_dash_link_t;

void lbr_dash_link_init(lbr_dash_link_t *d);

/* A datagram from the dashboard, received at shared time `rx_us`. */
void lbr_dash_link_on_packet(lbr_dash_link_t *d, const uint8_t *buf, size_t len, uint64_t rx_us);

/* Link timeout check: +1 the dashboard came up, -1 it went down, 0 no change. */
int lbr_dash_link_tick(lbr_dash_link_t *d, uint64_t now_us);

/* Build the next display datagram. Returns its length (0 if nothing). */
size_t lbr_dash_link_build(lbr_dash_link_t *d, uint64_t now_us, uint8_t source,
                           const lbr_value_table_t *values, const lbr_node_table_t *nodes,
                           uint8_t *buf, size_t cap);

/* Driver for the DASH_STATUS slot: channels DASH_BATTERY, DASH_LINK. Reads
 * `d` under the caller's lock functions (may be NULL). */
void lbr_sensor_dash_link(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                          size_t n_signals, lbr_dash_link_t *d, void (*lock)(void *),
                          void (*unlock)(void *), void      *lock_ctx);
