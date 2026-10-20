#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/acq.h"
#include "lbr/node.h"
#include "lbr/node_core.h"
#include "lbr/timebase.h"
#include "lbr/wlink.h"

/*
 * Corner suspension node logic (Mini ESP32-C5, own battery), independent of
 * the platform. It samples its IMU on the shared time grid and its battery,
 * sends the samples to the IMU Controller in IMU_BATCH datagrams (each
 * carrying the previous batch again), and disciplines its clock to the IMU
 * Controller over WiFi. It has no CAN and no SD card: everything it measures
 * reaches the Central's log through the IMU Controller.
 *
 * Plan: one slot whose signals are the corner's IMU frames (lbr/frames.h,
 * origin = this node) and one with its CORNER_xx_BATTERY signal.
 */

typedef struct {
    void *ctx;
    uint64_t (*mono_us)(void *ctx);
    uint64_t (*now_us)(void *ctx); /* disciplined shared time */
    void (*time_servo)(void *ctx, lbr_time_servo_t *out);
    lbr_time_update_t (*time_apply)(void *ctx, uint64_t shared_us, uint64_t local_rx_us,
                                    uint16_t epoch);
    /* One datagram to the IMU Controller; false if it was not sent. */
    bool (*send)(void *ctx, const uint8_t *pkt, size_t len);
    void (*log)(void *ctx, lbr_log_level_t level, const char *text);
} lbr_corner_hal_t;

#define LBR_CORNER_DEFAULT_BATCH  5u
#define LBR_CORNER_SYNC_FAST_MS   250u /* clock requests until synced */
#define LBR_CORNER_SYNC_PERIOD_MS 1000u

typedef struct {
    const lbr_node_config_t *cfg;
    lbr_corner_hal_t         hal;
    uint8_t                  batch; /* samples per datagram, 1..LBR_WLINK_MAX_BATCH */
    bool                     sim;

    lbr_acq_slot_t slot[LBR_NODE_MAX_SENSORS];
    int            imu_slot;     /* -1 if none */
    int            battery_slot; /* -1 if none */

    const lbr_frame_def_t *frames[4]; /* this corner's IMU frames */
    uint8_t                chan[4][LBR_FRAME_MAX_FIELDS];
    size_t                 n_frames;
    size_t                 sample_size;
    lbr_field_def_t        battery_field;
    uint8_t                battery_raw;

    /* A batch carries the flags its samples were taken with: a sample taken
     * before the clock synced must never be labelled synced. */
    uint8_t  cur[LBR_WLINK_MAX_BATCH * LBR_WLINK_MAX_SAMPLE];
    uint32_t cur_k0;
    uint8_t  cur_n;
    uint8_t  cur_flags;
    uint8_t  prev[LBR_WLINK_MAX_BATCH * LBR_WLINK_MAX_SAMPLE];
    uint32_t prev_k0;
    uint8_t  prev_n;
    uint8_t  prev_flags;

    atomic_uint  pkt_seq; /* both the IMU slot and the clock task send */
    lbr_wclock_t clock;

    /* IMU slot only: the batch timeline (LBR_WLINK_F_TL_*). */
    bool     have_timeline;
    uint32_t last_timeline;
    uint8_t  tl_seq;

    atomic_uint acquired;
    atomic_uint overruns;
    atomic_uint out_of_range;
    atomic_uint packets_sent;
    atomic_uint packets_dropped;
    atomic_uint rejected; /* malformed datagrams received */
} lbr_corner_core_t;

/* Returns false (and logs why) if the configuration is unusable; the node
 * then only keeps its clock and reports nothing. */
bool lbr_corner_core_init(lbr_corner_core_t *c, const lbr_node_config_t *cfg,
                          const lbr_corner_hal_t *hal, uint8_t batch);

void     lbr_corner_core_slot_start(lbr_corner_core_t *c, size_t i);
void     lbr_corner_core_slot_tick(lbr_corner_core_t *c, size_t i);
uint64_t lbr_corner_core_slot_next_due(lbr_corner_core_t *c, size_t i);

/* Send a clock request; returns the delay (ms) until the next one. */
uint32_t lbr_corner_core_clock_tick(lbr_corner_core_t *c);

/* A datagram from the IMU Controller, with its local receive time. */
void lbr_corner_core_on_packet(lbr_corner_core_t *c, const uint8_t *buf, size_t len,
                               uint64_t rx_local_us);

/* Sensor state flags as sent in IMU_BATCH (LBR_WLINK_F_*). */
uint8_t lbr_corner_core_flags(lbr_corner_core_t *c);
