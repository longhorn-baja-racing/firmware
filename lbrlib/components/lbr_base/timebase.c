#include "lbr/timebase.h"

#include <string.h>

/*
 * Servo: shared(local) = anchor_shared + elapsed + elapsed * rate_ppb / 1e9,
 * with elapsed = local - anchor_local. Each slew update re-anchors halfway
 * toward the measured gateway time (proportional term) and nudges the rate by
 * an eighth of the observed frequency error (integral term).
 */

#define KP_DIV 2
#define KI_DIV 8

static int64_t rate_correction(int64_t elapsed_us, int32_t rate_ppb)
{
    return elapsed_us * (int64_t)rate_ppb / 1000000000;
}

void lbr_time_servo_init(lbr_time_servo_t *s)
{
    memset(s, 0, sizeof *s);
}

void lbr_time_servo_set_master(lbr_time_servo_t *s)
{
    lbr_time_servo_init(s);
    s->master = true;
    s->synced = true;
}

uint64_t lbr_time_servo_map(const lbr_time_servo_t *s, uint64_t local_us)
{
    if (s->master || !s->synced) {
        return local_us;
    }
    int64_t elapsed = (int64_t)(local_us - s->anchor_local_us);
    int64_t shared = (int64_t)s->anchor_shared_us + elapsed + rate_correction(elapsed, s->rate_ppb);
    return shared < 0 ? 0u : (uint64_t)shared;
}

static void anchor(lbr_time_servo_t *s, uint64_t local_us, uint64_t shared_us)
{
    s->anchor_local_us  = local_us;
    s->anchor_shared_us = shared_us;
}

lbr_time_update_t lbr_time_servo_update(lbr_time_servo_t *s, uint64_t gateway_us,
                                        uint64_t local_rx_us, uint16_t epoch)
{
    if (s->master) {
        return LBR_TIME_UPD_IGNORED;
    }

    if (!s->synced || epoch != s->epoch) {
        lbr_time_update_t kind = s->synced ? LBR_TIME_UPD_EPOCH_CHANGE : LBR_TIME_UPD_FIRST;
        if (kind == LBR_TIME_UPD_EPOCH_CHANGE) {
            s->epoch_changes++;
        }
        /* New timeline (first sync or new epoch): the guard restarts, or a
         * node that ran unsynced longer than the Central has been up would
         * hold its clock still until the Central's time caught up. */
        s->last_output_us = 0;
        anchor(s, local_rx_us, gateway_us);
        s->synced               = true;
        s->epoch                = epoch;
        s->last_update_local_us = local_rx_us;
        s->last_error_us        = 0;
        s->sync_count++;
        return kind;
    }

    if (local_rx_us <= s->last_update_local_us) {
        return LBR_TIME_UPD_IGNORED;
    }

    uint64_t predicted = lbr_time_servo_map(s, local_rx_us);
    int64_t  err       = (int64_t)gateway_us - (int64_t)predicted;
    int64_t  dt        = (int64_t)(local_rx_us - s->last_update_local_us);

    s->last_error_us = err > INT32_MAX ? INT32_MAX : err < INT32_MIN ? INT32_MIN : (int32_t)err;
    s->last_update_local_us = local_rx_us;
    s->sync_count++;

    if (err > LBR_TIME_STEP_THRESHOLD_US || err < -LBR_TIME_STEP_THRESHOLD_US) {
        anchor(s, local_rx_us, gateway_us);
        s->step_count++;
        return LBR_TIME_UPD_STEP;
    }

    anchor(s, local_rx_us, (uint64_t)((int64_t)predicted + err / KP_DIV));
    int64_t rate = (int64_t)s->rate_ppb + (err * 1000000000 / dt) / KI_DIV;
    if (rate > LBR_TIME_MAX_RATE_PPB) {
        rate = LBR_TIME_MAX_RATE_PPB;
    } else if (rate < -LBR_TIME_MAX_RATE_PPB) {
        rate = -LBR_TIME_MAX_RATE_PPB;
    }
    s->rate_ppb = (int32_t)rate;
    return LBR_TIME_UPD_SLEW;
}

uint64_t lbr_time_servo_now(lbr_time_servo_t *s, uint64_t local_us)
{
    uint64_t t = lbr_time_servo_map(s, local_us);
    if (t < s->last_output_us) {
        return s->last_output_us;
    }
    s->last_output_us = t;
    return t;
}

void lbr_time_sync_on_sync(lbr_time_sync_rx_t *rx, uint8_t seq, uint16_t epoch,
                           uint64_t local_rx_us)
{
    rx->pending     = true;
    rx->seq         = seq;
    rx->epoch       = epoch;
    rx->local_rx_us = local_rx_us;
}

bool lbr_time_sync_on_follow_up(lbr_time_sync_rx_t *rx, uint8_t seq, uint64_t gateway_tx_us,
                                uint64_t *gateway_us, uint64_t *local_rx_us, uint16_t *epoch)
{
    if (!rx->pending || rx->seq != seq) {
        rx->unmatched++;
        return false;
    }
    rx->pending  = false;
    *gateway_us  = gateway_tx_us;
    *local_rx_us = rx->local_rx_us;
    *epoch       = rx->epoch;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Clock: a servo behind a time source and a lock                            */

static void clock_lock(const lbr_clock_t *c)
{
    if (c->lock != NULL) {
        c->lock();
    }
}

static void clock_unlock(const lbr_clock_t *c)
{
    if (c->unlock != NULL) {
        c->unlock();
    }
}

void lbr_clock_init(lbr_clock_t *c, uint64_t (*mono_us)(void), void (*lock)(void),
                    void (*unlock)(void))
{
    *c = (lbr_clock_t){.mono_us = mono_us, .lock = lock, .unlock = unlock};
    lbr_time_servo_init(&c->servo);
}

void lbr_clock_set_master(lbr_clock_t *c)
{
    clock_lock(c);
    lbr_time_servo_set_master(&c->servo);
    clock_unlock(c);
}

lbr_time_update_t lbr_clock_sync_apply(lbr_clock_t *c, uint64_t gateway_us, uint64_t local_rx_us,
                                       uint16_t epoch)
{
    clock_lock(c);
    lbr_time_update_t r = lbr_time_servo_update(&c->servo, gateway_us, local_rx_us, epoch);
    clock_unlock(c);
    return r;
}

uint64_t lbr_clock_now_us(lbr_clock_t *c)
{
    clock_lock(c);
    uint64_t t = lbr_time_servo_now(&c->servo, c->mono_us());
    clock_unlock(c);
    return t;
}

uint64_t lbr_clock_local_to_shared(lbr_clock_t *c, uint64_t local_us)
{
    clock_lock(c);
    uint64_t t = lbr_time_servo_map(&c->servo, local_us);
    clock_unlock(c);
    return t;
}

bool lbr_clock_is_synced(lbr_clock_t *c)
{
    clock_lock(c);
    bool synced = c->servo.synced;
    clock_unlock(c);
    return synced;
}

void lbr_clock_get_servo(lbr_clock_t *c, lbr_time_servo_t *out)
{
    clock_lock(c);
    *out = c->servo;
    clock_unlock(c);
}
