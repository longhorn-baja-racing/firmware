#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/protocol.h"
#include "lbr/sensor.h"

/*
 * Acquisition engine for one slot (pure; host-tested). Shared by every node
 * that samples sensors: the CAN sensor nodes (node_core.c), the Central's
 * GPS (gw_core.c), the corner nodes and the dashboard (wlink).
 *
 * Sampling is locked to the shared time grid: sample k of a slot is due at
 * shared time k * period_us, and the platform wakes the slot at each grid
 * point (lbr_acq_next_due). The tick takes the grid point nearest the wake
 * time as its k, so a receiver can rebuild the sample time from k alone
 * (lbr/frames.h). Grid points skipped because the slot ran late are counted
 * as overruns; a jump of the shared clock (first sync, step, new epoch) is a
 * new timeline, not an overrun.
 *
 * Sensor failures: init failure or a failed read marks every channel
 * SENSOR_FAULT, raises SENSOR_INIT_FAIL, and retries init every
 * LBR_ACQ_RETRY_US; success raises SENSOR_RECOVERED.
 */

#define LBR_ACQ_RETRY_US 2000000u

typedef void (*lbr_acq_event_fn)(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault);

typedef struct {
    lbr_sensor_t *sensor;
    uint32_t      period_us;
    uint16_t      index; /* slot index: the event argument */

    atomic_bool healthy;
    atomic_bool calibrated;
    uint64_t    next_retry_us; /* local clock */

    bool     have_k;
    uint64_t last_k;
    uint32_t timeline;
} lbr_acq_slot_t;

typedef struct {
    uint64_t k;      /* grid index of this sample */
    uint32_t missed; /* grid points skipped since the previous sample */
} lbr_acq_tick_t;

void lbr_acq_slot_init(lbr_acq_slot_t *a, lbr_sensor_t *sensor, uint16_t rate_hz, uint16_t index);

/* First bring-up of the sensor. `mono_us` is the local clock. */
void lbr_acq_slot_start(lbr_acq_slot_t *a, uint64_t mono_us, lbr_acq_event_fn ev, void *ctx);

/*
 * One wake-up. `timeline` is any counter that changes when the shared clock
 * jumps (lbr_acq_timeline). Fills out[0..n_signals-1] and *tick, and returns
 * true; returns false (nothing read) when this grid point was already
 * sampled.
 */
bool lbr_acq_slot_tick(lbr_acq_slot_t *a, uint64_t now_shared_us, uint64_t mono_us,
                       uint32_t timeline, lbr_reading_t *out, lbr_acq_tick_t *tick,
                       lbr_acq_event_fn ev, void *ctx);

/* Shared time of the first grid point after `now_shared_us`. */
static inline uint64_t lbr_acq_next_due(uint64_t now_shared_us, uint32_t period_us)
{
    return (now_shared_us / period_us + 1u) * period_us;
}

/* Timeline counter from a time servo's state: changes on the first sync,
 * on every step, and on every new epoch. */
static inline uint32_t lbr_acq_timeline(bool synced, uint32_t step_count, uint32_t epoch_changes)
{
    return (synced ? 1u : 0u) + 2u * (step_count + epoch_changes);
}
