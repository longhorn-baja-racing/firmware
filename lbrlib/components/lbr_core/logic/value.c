#include "lbr/value.h"

#include <math.h>

/* Pure; see lbr/value.h. */

const char *lbr_status_name(lbr_status st)
{
    static const char *const names[LBR_STATUS_COUNT_] = {
        [LBR_OK]             = "OK",
        [LBR_UNSPECIFIED]    = "UNSPECIFIED",
        [LBR_NOT_FITTED]     = "NOT_FITTED",
        [LBR_NO_DATA]        = "NO_DATA",
        [LBR_SENSOR_FAULT]   = "SENSOR_FAULT",
        [LBR_OUT_OF_RANGE]   = "OUT_OF_RANGE",
        [LBR_NOT_CALIBRATED] = "NOT_CALIBRATED",
        [LBR_BUS_ERROR]      = "BUS_ERROR",
        [LBR_STALE]          = "STALE",
        [LBR_NODE_LOST]      = "NODE_LOST",
        [LBR_NODE_MISSING]   = "NODE_MISSING",
        [LBR_BAD_MATH]       = "BAD_MATH",
    };
    return ((unsigned)st < LBR_STATUS_COUNT_) ? names[st] : "?";
}

lbr_status lbr_status_from_reason(lbr_unavail_reason_t reason)
{
    switch (reason) {
        case LBR_UNAVAIL_NOT_FITTED:
            return LBR_NOT_FITTED;
        case LBR_UNAVAIL_NO_DATA:
            return LBR_NO_DATA;
        case LBR_UNAVAIL_SENSOR_FAULT:
            return LBR_SENSOR_FAULT;
        case LBR_UNAVAIL_OUT_OF_RANGE:
            return LBR_OUT_OF_RANGE;
        case LBR_UNAVAIL_NOT_CALIBRATED:
            return LBR_NOT_CALIBRATED;
        case LBR_UNAVAIL_BUS_ERROR:
            return LBR_BUS_ERROR;
        default:
            return LBR_UNSPECIFIED;
    }
}

static uint32_t older(uint32_t a, uint32_t b)
{
    return a > b ? a : b;
}

/* The result of an operation on a and b: a's problem first, then b's, then
 * the value itself (never NaN or infinite). */
static lbr_f32 result(float v, lbr_status a, lbr_status b, uint32_t age_ms)
{
    lbr_status st = a != LBR_OK ? a : b;
    if (st == LBR_OK && !isfinite(v)) {
        st = LBR_BAD_MATH;
    }
    return (lbr_f32){.v = st == LBR_OK ? v : 0.0f, .st = st, .age_ms = age_ms};
}

lbr_f32 lbr_add(lbr_f32 a, lbr_f32 b)
{
    return result(a.v + b.v, a.st, b.st, older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_sub(lbr_f32 a, lbr_f32 b)
{
    return result(a.v - b.v, a.st, b.st, older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_mul(lbr_f32 a, lbr_f32 b)
{
    return result(a.v * b.v, a.st, b.st, older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_div(lbr_f32 a, lbr_f32 b)
{
    if (a.st == LBR_OK && b.st == LBR_OK && b.v == 0.0f) {
        return result(0.0f, LBR_BAD_MATH, LBR_OK, older(a.age_ms, b.age_ms));
    }
    return result(a.st == LBR_OK && b.st == LBR_OK ? a.v / b.v : 0.0f, a.st, b.st,
                  older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_scale(lbr_f32 a, float k)
{
    return result(a.v * k, a.st, LBR_OK, a.age_ms);
}

lbr_f32 lbr_offset(lbr_f32 a, float k)
{
    return result(a.v + k, a.st, LBR_OK, a.age_ms);
}

lbr_f32 lbr_abs(lbr_f32 a)
{
    return result(fabsf(a.v), a.st, LBR_OK, a.age_ms);
}

/* An OK value that is not a number is bad math (a comparison would skip it). */
static lbr_status checked(lbr_f32 a)
{
    return a.st == LBR_OK && !isfinite(a.v) ? LBR_BAD_MATH : a.st;
}

lbr_f32 lbr_min(lbr_f32 a, lbr_f32 b)
{
    return result(a.v < b.v ? a.v : b.v, checked(a), checked(b), older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_max(lbr_f32 a, lbr_f32 b)
{
    return result(a.v > b.v ? a.v : b.v, checked(a), checked(b), older(a.age_ms, b.age_ms));
}

lbr_f32 lbr_clamp(lbr_f32 a, float lo, float hi)
{
    float v = a.v < lo ? lo : (a.v > hi ? hi : a.v);
    return result(v, a.st, LBR_OK, a.age_ms);
}

static lbr_bool compare(bool v, lbr_f32 a)
{
    lbr_status st = a.st;
    if (st == LBR_OK && !isfinite(a.v)) {
        st = LBR_BAD_MATH;
    }
    return (lbr_bool){.v = st == LBR_OK && v, .st = st, .age_ms = a.age_ms};
}

lbr_bool lbr_gt(lbr_f32 a, float limit)
{
    return compare(a.v > limit, a);
}

lbr_bool lbr_lt(lbr_f32 a, float limit)
{
    return compare(a.v < limit, a);
}

lbr_f32 lbr_i32_to_f32(lbr_i32 a)
{
    return result((float)a.v, a.st, LBR_OK, a.age_ms);
}

static lbr_bool alarm(bool *state, lbr_f32 x, bool turn_on, bool turn_off)
{
    lbr_bool r = compare(*state, x);
    if (r.st != LBR_OK) {
        r.v = *state; /* keep the alarm as it was; the status says why */
        return r;
    }
    if (turn_on) {
        *state = true;
    } else if (turn_off) {
        *state = false;
    }
    r.v = *state;
    return r;
}

lbr_bool lbr_above(bool *state, lbr_f32 x, float on, float off)
{
    return alarm(state, x, x.v > on, x.v < off);
}

lbr_bool lbr_below(bool *state, lbr_f32 x, float on, float off)
{
    return alarm(state, x, x.v<on, x.v> off);
}

static uint32_t older_b(lbr_bool a, lbr_bool b)
{
    return older(a.age_ms, b.age_ms);
}

lbr_bool lbr_and(lbr_bool a, lbr_bool b)
{
    if ((lbr_ok(a) && !a.v) || (lbr_ok(b) && !b.v)) {
        return (lbr_bool){.v = false, .st = LBR_OK, .age_ms = older_b(a, b)};
    }
    lbr_status st = a.st != LBR_OK ? a.st : b.st;
    return (lbr_bool){.v = st == LBR_OK, .st = st, .age_ms = older_b(a, b)};
}

lbr_bool lbr_or(lbr_bool a, lbr_bool b)
{
    if ((lbr_ok(a) && a.v) || (lbr_ok(b) && b.v)) {
        return (lbr_bool){.v = true, .st = LBR_OK, .age_ms = older_b(a, b)};
    }
    lbr_status st = a.st != LBR_OK ? a.st : b.st;
    return (lbr_bool){.v = false, .st = st, .age_ms = older_b(a, b)};
}

lbr_bool lbr_not(lbr_bool a)
{
    return (lbr_bool){.v = lbr_ok(a) && !a.v, .st = a.st, .age_ms = a.age_ms};
}

lbr_f32 lbr_lowpass(lbr_filter *f, lbr_f32 x, float alpha)
{
    if (!lbr_ok(x) || !isfinite(x.v)) {
        f->primed = false; /* restart on the next good value */
        return result(0.0f, x.st == LBR_OK ? LBR_BAD_MATH : x.st, LBR_OK, x.age_ms);
    }
    if (!f->primed) {
        f->v      = x.v;
        f->primed = true;
    } else {
        f->v += alpha * (x.v - f->v);
    }
    if (!isfinite(f->v)) {
        f->primed = false; /* overflowed: restart instead of staying bad for good */
    }
    return result(f->v, LBR_OK, LBR_OK, x.age_ms);
}
