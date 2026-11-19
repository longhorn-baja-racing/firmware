#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Shared, CAN-distributed monotonic time base.
 *
 * The gateway is the time master: shared time is its local monotonic clock,
 * epoch = gateway boot, unit = microseconds. Every other node disciplines its
 * local clock to the gateway's two-step sync (TIME_SYNC + FOLLOW_UP frames,
 * see can.h) so all logs share one timeline. A new gateway boot starts a new
 * epoch (the 16-bit epoch id in TIME_SYNC changes).
 *
 * Guarantees: lbr_time_now_us() never goes backwards within an epoch. Small
 * corrections are slewed (offset + rate); errors above
 * LBR_TIME_STEP_THRESHOLD_US are stepped and counted. A backwards step holds
 * the output still until the disciplined clock catches up.
 */

#define LBR_TIME_STEP_THRESHOLD_US 2000
#define LBR_TIME_MAX_RATE_PPB      500000 /* +/- 500 ppm crystal error bound */

/* ------------------------------------------------------------------------ */
/* Pure servo (host-tested)                                                  */

typedef enum {
    LBR_TIME_UPD_FIRST = 0,    /* first sync: clock stepped onto the shared base */
    LBR_TIME_UPD_EPOCH_CHANGE, /* gateway rebooted: re-anchored to the new epoch */
    LBR_TIME_UPD_STEP,         /* error too large to slew: stepped */
    LBR_TIME_UPD_SLEW,         /* normal: offset/rate corrected smoothly */
    LBR_TIME_UPD_IGNORED,      /* master clock, or sample older than the last update */
} lbr_time_update_t;

typedef struct {
    bool     synced;
    bool     master;
    uint16_t epoch;
    uint64_t anchor_local_us;
    uint64_t anchor_shared_us;
    int32_t  rate_ppb; /* shared runs this much faster than local */
    uint64_t last_update_local_us;
    uint64_t last_output_us;

    uint32_t sync_count;
    uint32_t step_count;
    uint32_t epoch_changes;
    int32_t  last_error_us;
} lbr_time_servo_t;

void lbr_time_servo_init(lbr_time_servo_t *s);

/* Make this servo the master: shared time == local time, always synced. */
void lbr_time_servo_set_master(lbr_time_servo_t *s);

/* Apply one sync measurement: the gateway's time `gateway_us` corresponds to
 * this node's local clock `local_rx_us`. */
lbr_time_update_t lbr_time_servo_update(lbr_time_servo_t *s, uint64_t gateway_us,
                                        uint64_t local_rx_us, uint16_t epoch);

/* Shared time at local time `local_us`, without the monotonic guard (use for
 * converting captured timestamps such as CAN RX times). */
uint64_t lbr_time_servo_map(const lbr_time_servo_t *s, uint64_t local_us);

/* Shared time now, monotonic within an epoch. Mutates the guard state. */
uint64_t lbr_time_servo_now(lbr_time_servo_t *s, uint64_t local_us);

/* Pairs TIME_SYNC receptions with their FOLLOW_UP. */
typedef struct {
    bool     pending;
    uint8_t  seq;
    uint16_t epoch;
    uint64_t local_rx_us;
    uint32_t unmatched; /* follow-ups without a matching sync (counted, not used) */
} lbr_time_sync_rx_t;

void lbr_time_sync_on_sync(lbr_time_sync_rx_t *rx, uint8_t seq, uint16_t epoch,
                           uint64_t local_rx_us);
/* True when the follow-up completes a measurement (outputs filled). */
bool lbr_time_sync_on_follow_up(lbr_time_sync_rx_t *rx, uint8_t seq, uint64_t gateway_tx_us,
                                uint64_t *gateway_us, uint64_t *local_rx_us, uint16_t *epoch);

/* ------------------------------------------------------------------------ */
/* A clock: one servo behind the board's time source and lock                */

/*
 * The board's shared clock, as a pure object: the servo plus the local
 * monotonic time source and the lock the platform gives it. LBRLib keeps one
 * per board (lbr/board.h, lbr_time_now_us() and friends); tests and the
 * simulator make their own with a fake time source.
 */
typedef struct {
    lbr_time_servo_t servo;
    uint64_t (*mono_us)(void); /* local monotonic microseconds */
    void (*lock)(void);        /* the servo is shared between tasks */
    void (*unlock)(void);
} lbr_clock_t;

/* Set up a clock on a time source and lock (both lock functions may be NULL). */
void lbr_clock_init(lbr_clock_t *c, uint64_t (*mono_us)(void), void (*lock)(void),
                    void (*unlock)(void));

/* Become the time master: shared time is this clock's local time. */
void lbr_clock_set_master(lbr_clock_t *c);

/* Discipline the clock with one sync measurement. */
lbr_time_update_t lbr_clock_sync_apply(lbr_clock_t *c, uint64_t gateway_us, uint64_t local_rx_us,
                                       uint16_t epoch);

/* Disciplined shared time (never goes backwards); a captured local time mapped
 * to shared time; whether a sync (or master) has set it; a servo snapshot. */
uint64_t lbr_clock_now_us(lbr_clock_t *c);
uint64_t lbr_clock_local_to_shared(lbr_clock_t *c, uint64_t local_us);
bool     lbr_clock_is_synced(lbr_clock_t *c);
void     lbr_clock_get_servo(lbr_clock_t *c, lbr_time_servo_t *out);
