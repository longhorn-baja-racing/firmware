#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/node_core.h"
#include "lbr/sensor.h"
#include "lbr/wlink.h"

/*
 * IMU Controller side of the corner links (pure; host-tested): receives the
 * four corner nodes' datagrams, answers their clock requests, and forwards
 * every IMU sample onto CAN exactly once in its frame-table frames (counter
 * = grid index), whatever order or how many times it arrives. A sample that
 * never arrives is counted as lost once it falls out of the 64-sample
 * window. A corner silent for LBR_WLINK_LINK_TIMEOUT_US is down
 * (NODE_LOST); its next datagram brings it back (NODE_FOUND).
 *
 * The CORNER_STATUS frame (0x30A) is built by an ordinary 1 Hz slot whose
 * driver reads this state (lbr_sensor_corner_status).
 *
 * Threading: on_packet runs in the UDP task, tick and the status driver in
 * others; hal.lock guards the state.
 */

#define LBR_CORNER_COUNT 4u

typedef struct {
    void *ctx;
    uint64_t (*now_us)(void *ctx); /* shared time */
    bool (*synced)(void *ctx);     /* this node's clock is on the shared base */
    uint16_t (*epoch)(void *ctx);
    /* Forward one frame onto CAN (lbr_node_core_send). */
    bool (*send_frame)(void *ctx, const lbr_can_frame_t *f);
    /* Datagram back to corner `node` (its clock reply). */
    bool (*reply)(void *ctx, uint8_t node, const uint8_t *pkt, size_t len);
    void (*event)(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault);
    void (*lock)(void *ctx);
    void (*unlock)(void *ctx);
} lbr_corner_rx_hal_t;

typedef struct {
    bool     seen;
    bool     up;
    uint64_t last_rx_us;
    uint8_t  battery_raw;
    uint8_t  flags; /* LBR_WLINK_F_* from the latest batch */

    bool     have_k;
    uint64_t next_k; /* one past the newest sample forwarded */
    uint64_t window; /* bit i: sample next_k - 1 - i was forwarded */

    bool     have_seq;
    uint32_t last_seq;

    uint32_t packets;
    uint32_t packet_gaps; /* datagrams that never arrived (seq gaps) */
    uint32_t forwarded;   /* samples put on CAN */
    uint32_t duplicates;  /* samples received again (the redundant copy) */
    uint32_t lost;        /* samples that never arrived */
    uint32_t late;        /* arrived after leaving the window: not forwarded, already lost */
    uint32_t tx_failed;   /* forwards the CAN controller did not take */

    const lbr_frame_def_t *frames[4];
    size_t                 n_frames;
    size_t                 sample_size;
} lbr_corner_peer_t;

typedef struct {
    lbr_corner_rx_hal_t hal;
    lbr_corner_peer_t   peer[LBR_CORNER_COUNT];
    uint32_t            reply_seq;
    uint32_t            rejected; /* malformed or foreign datagrams */
    uint32_t            reported_loss;
} lbr_corner_rx_t;

void lbr_corner_rx_init(lbr_corner_rx_t *rx, const lbr_corner_rx_hal_t *hal);

/* A datagram from a corner node, received at shared time `rx_shared_us`. */
void lbr_corner_rx_on_packet(lbr_corner_rx_t *rx, const uint8_t *buf, size_t len,
                             uint64_t rx_shared_us);

/* Periodic (every ~100 ms): link timeouts. Returns the heartbeat error flags
 * the corners raise (LBR_HB_ERR_LINK, LBR_HB_ERR_SENSOR) and adds the
 * samples newly lost since the last call to *new_loss. */
uint8_t lbr_corner_rx_tick(lbr_corner_rx_t *rx, uint32_t *new_loss);

/* What the IMU Controller app does every LBR_CORNER_SERVICE_MS: run
 * lbr_corner_rx_tick and hand its flags and new losses to the node core, so
 * they reach the heartbeat (LINK, SENSOR, LOSS) and transport_error_count.
 * The simulator calls the same function. */
#define LBR_CORNER_SERVICE_MS 100u
void lbr_corner_rx_service(lbr_corner_rx_t *rx, lbr_node_core_t *core);

/* Bitmasks, bit i = corner i (FL, FR, BL, BR). */
uint8_t lbr_corner_rx_link_mask(lbr_corner_rx_t *rx);
uint8_t lbr_corner_rx_synced_mask(lbr_corner_rx_t *rx);

/* Driver for the CORNER_STATUS slot: channels CORNER_xx_BATTERY, CORNER_LINK,
 * CORNER_SYNCED in any order. A corner that is down reads NO_DATA. */
void lbr_sensor_corner_status(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                              size_t n_signals, lbr_corner_rx_t *rx);
