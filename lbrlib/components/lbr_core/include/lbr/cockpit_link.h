#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/node_table.h"
#include "lbr/protocol.h"
#include "lbr/sensor.h"
#include "lbr/value_table.h"
#include "lbr/wlink.h"

/*
 * Central side of the cockpit display link (pure; host-tested).
 *
 * Down (Central -> cockpit, LBR_LINK_COCKPIT, every cockpit_period_ms): one
 * datagram with a SAMPLE record for each display signal (lbr_cockpit_signals,
 * the latest value from the bus; a stale one goes as NO_DATA) and a
 * HEALTH_LITE record per CAN node.
 *
 * Up (cockpit -> Central): SAMPLE records, today only COCKPIT_BATTERY, once a
 * second. The cockpit is up while they keep arriving (within
 * LBR_WLINK_COCKPIT_TIMEOUT_US); the COCKPIT_STATUS frame
 * (0x60A) puts its battery and the link state on CAN, and so in the log, via
 * an ordinary 1 Hz slot whose driver reads this state (lbr_sensor_cockpit_link).
 *
 * Threading: the caller serializes access (the Central's state lock); the
 * slot driver takes the same lock through the functions it was given.
 */

/* What the cockpit display shows (any source node). Shared with the
 * cockpit app and the simulator. */
extern const lbr_signal_id_t lbr_cockpit_signals[];
extern const size_t          lbr_cockpit_signal_count;

typedef struct {
    bool         seen;
    bool         up;
    uint64_t     last_rx_us;
    bool         have_battery;
    lbr_sample_t battery;

    lbr_seq_window_t seq;
    uint32_t         packets;
    uint32_t         packet_gaps; /* datagrams from the cockpit that never arrived */
    uint32_t         rejected;
    uint32_t         tx_seq;

    /* Set by lbr_sensor_cockpit_link: how its driver locks this state. */
    void (*lock)(void *);
    void (*unlock)(void *);
    void *lock_ctx;
} lbr_cockpit_link_t;

void lbr_cockpit_link_init(lbr_cockpit_link_t *d);

/* A datagram from the cockpit, received at shared time `rx_us`. */
void lbr_cockpit_link_on_packet(lbr_cockpit_link_t *d, const uint8_t *buf, size_t len,
                                uint64_t rx_us);

/* Link timeout check: +1 the cockpit came up, -1 it went down, 0 no change. */
int lbr_cockpit_link_tick(lbr_cockpit_link_t *d, uint64_t now_us);

/* Build the next display datagram. Returns its length (0 if nothing). */
size_t lbr_cockpit_link_build(lbr_cockpit_link_t *d, uint64_t now_us, uint8_t source,
                              const lbr_value_table_t *values, const lbr_node_table_t *nodes,
                              uint8_t *buf, size_t cap);

/* Driver for the COCKPIT_STATUS slot: channels COCKPIT_BATTERY, COCKPIT_LINK. Reads
 * `d` under the caller's lock functions (may be NULL). */
void lbr_sensor_cockpit_link(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                             size_t n_signals, lbr_cockpit_link_t *d, void (*lock)(void *),
                             void (*unlock)(void *), void         *lock_ctx);
