#include "lbr/acq.h"

#include <string.h>

/* Pure acquisition engine; see lbr/acq.h. No platform headers here. */

void lbr_acq_slot_init(lbr_acq_slot_t *a, lbr_sensor_t *sensor, uint16_t rate_hz, uint16_t index)
{
    memset(a, 0, sizeof *a);
    a->sensor    = sensor;
    a->period_us = 1000000u / (rate_hz ? rate_hz : 1u);
    a->index     = index;
    atomic_store(&a->calibrated, true);
}

static void emit(lbr_acq_event_fn ev, void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    if (ev != NULL) {
        ev(ctx, code, arg, fault);
    }
}

void lbr_acq_slot_start(lbr_acq_slot_t *a, uint64_t mono_us, lbr_acq_event_fn ev, void *ctx)
{
    bool ok = a->sensor->init(a->sensor) == 0;
    atomic_store(&a->healthy, ok);
    if (!ok) {
        emit(ev, ctx, LBR_EVT_SENSOR_INIT_FAIL, a->index, true);
        a->next_retry_us = mono_us + LBR_ACQ_RETRY_US;
    }
}

static void mark_all(lbr_reading_t *out, size_t n, lbr_unavail_reason_t why)
{
    for (size_t i = 0; i < n; i++) {
        out[i].valid  = false;
        out[i].reason = why;
        out[i].v.u32  = 0;
    }
}

bool lbr_acq_slot_tick(lbr_acq_slot_t *a, uint64_t now_shared_us, uint64_t mono_us,
                       uint32_t timeline, lbr_reading_t *out, lbr_acq_tick_t *tick,
                       lbr_acq_event_fn ev, void *ctx)
{
    uint64_t k      = (now_shared_us + a->period_us / 2u) / a->period_us;
    uint32_t missed = 0;
    if (a->have_k && k == a->last_k) {
        /* Woke early or twice for one grid point, or a backward step that
         * the monotonic guard holds at the point just sampled. */
        return false;
    }
    if (a->have_k && timeline == a->timeline) {
        if (k < a->last_k) {
            return false;
        }
        uint64_t gap = k - a->last_k - 1u;
        missed       = gap > UINT32_MAX ? UINT32_MAX : (uint32_t)gap;
    }
    a->have_k   = true;
    a->last_k   = k;
    a->timeline = timeline;

    lbr_sensor_t *sensor = a->sensor;
    if (atomic_load(&a->healthy)) {
        if (sensor->read(sensor, k * a->period_us, out) != 0) {
            atomic_store(&a->healthy, false);
            emit(ev, ctx, LBR_EVT_SENSOR_INIT_FAIL, a->index, true);
            a->next_retry_us = mono_us + LBR_ACQ_RETRY_US;
            mark_all(out, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
        }
    } else {
        mark_all(out, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
        if (mono_us >= a->next_retry_us) {
            if (sensor->init(sensor) == 0) {
                atomic_store(&a->healthy, true);
                emit(ev, ctx, LBR_EVT_SENSOR_RECOVERED, a->index, false);
            } else {
                a->next_retry_us = mono_us + LBR_ACQ_RETRY_US;
            }
        }
    }
    atomic_store(&a->calibrated, sensor->is_calibrated == NULL || sensor->is_calibrated(sensor));

    tick->k      = k;
    tick->missed = missed;
    return true;
}
