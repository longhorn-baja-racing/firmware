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
/* Node-wide time base (thread-safe wrapper around one servo)                */

/* Raw local monotonic microseconds (esp_timer on hardware). */
uint64_t lbr_time_mono_us(void);

/* Become the time master (gateway). */
void lbr_time_set_master(void);

/* Discipline the local clock with one sync measurement. */
lbr_time_update_t lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us, uint16_t epoch);

/* Disciplined shared time base - use this to stamp samples. */
uint64_t lbr_time_now_us(void);

/* Convert a captured local timestamp to shared time. */
uint64_t lbr_time_local_to_shared(uint64_t local_us);

/* True once at least one sync frame has disciplined this node (or master). */
bool lbr_time_is_synced(void);

/* Snapshot of the servo state for health reporting. */
void lbr_time_get_servo(lbr_time_servo_t *out);

/* ------------------------------------------------------------------------ */
/* Platform port (timebase_port_esp.c on target; host tests provide a fake)  */

uint64_t lbr_time_port_mono_us(void);
void     lbr_time_port_lock(void);
void     lbr_time_port_unlock(void);
