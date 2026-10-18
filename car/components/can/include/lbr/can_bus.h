#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"

/*
 * TWAI (on-chip CAN controller) adapter. Hardware-facing; not built on host.
 *
 * Send is non-blocking: a full TX queue or bus-off drops the frame and counts
 * it. Received frames are timestamped in the RX ISR with the local monotonic
 * clock and passed through a bounded queue; overflow is counted. Bus-off is
 * recovered automatically and counted.
 */

typedef struct {
    uint32_t bitrate;      /* bit/s; must match every node on the bus */
    int      tx_gpio;      /* -1 = not configured (pins TBD) */
    int      rx_gpio;      /* -1 = not configured (pins TBD) */
    size_t   rx_queue_len; /* frames */
    size_t   tx_queue_len; /* frames */
    bool     accept_all;   /* false: hardware filter passes only gateway system frames */
} lbr_can_bus_config_t;

typedef struct {
    lbr_can_frame_t frame;
    uint64_t        rx_local_us; /* local monotonic clock in the RX ISR */
} lbr_can_rx_t;

typedef enum {
    LBR_CAN_OK             = 0,
    LBR_CAN_ERR_PINS_UNSET = -1, /* Kconfig GPIOs are still -1 (TBD) */
    LBR_CAN_ERR_DRIVER     = -2,
    LBR_CAN_ERR_NO_MEM     = -3,
} lbr_can_err_t;

/* Defaults from Kconfig (CONFIG_LBR_CAN_*). */
lbr_can_bus_config_t lbr_can_bus_default_config(void);

lbr_can_err_t lbr_can_bus_start(const lbr_can_bus_config_t *cfg);

/* Non-blocking. Returns false (and counts a drop) if the frame was not queued. */
bool lbr_can_bus_send(const lbr_can_frame_t *f);

/* Wait up to `timeout_ms` for a received frame. */
bool lbr_can_bus_receive(lbr_can_rx_t *out, uint32_t timeout_ms);

/* Gateway time sync: call arm from the sync task before sending TIME_SYNC,
 * then wait returns the local monotonic time at which the TIME_SYNC with
 * `sync_seq` finished transmitting (captured in the TX-done ISR), for the
 * FOLLOW_UP. Completions of other (older, retried) sync frames are ignored. */
void lbr_can_bus_arm_sync_tx(void);
bool lbr_can_bus_wait_sync_tx_done(uint8_t sync_seq, uint64_t *tx_local_us, uint32_t timeout_ms);

void lbr_can_bus_get_stats(lbr_can_stats_t *out);
