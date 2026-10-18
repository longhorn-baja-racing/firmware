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
            s->last_output_us = 0; /* new timeline: the guard restarts */
        }
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
/* Node-wide wrapper                                                         */

static lbr_time_servo_t s_servo;

uint64_t lbr_time_mono_us(void)
{
    return lbr_time_port_mono_us();
}

void lbr_time_set_master(void)
{
    lbr_time_port_lock();
    lbr_time_servo_set_master(&s_servo);
    lbr_time_port_unlock();
}

lbr_time_update_t lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us, uint16_t epoch)
{
    lbr_time_port_lock();
    lbr_time_update_t r = lbr_time_servo_update(&s_servo, gateway_us, local_rx_us, epoch);
    lbr_time_port_unlock();
    return r;
}

uint64_t lbr_time_now_us(void)
{
    lbr_time_port_lock();
    uint64_t t = lbr_time_servo_now(&s_servo, lbr_time_port_mono_us());
    lbr_time_port_unlock();
    return t;
}

uint64_t lbr_time_local_to_shared(uint64_t local_us)
{
    lbr_time_port_lock();
    uint64_t t = lbr_time_servo_map(&s_servo, local_us);
    lbr_time_port_unlock();
    return t;
}

bool lbr_time_is_synced(void)
{
    lbr_time_port_lock();
    bool synced = s_servo.synced;
    lbr_time_port_unlock();
    return synced;
}

void lbr_time_get_servo(lbr_time_servo_t *out)
{
    lbr_time_port_lock();
    *out = s_servo;
    lbr_time_port_unlock();
}
