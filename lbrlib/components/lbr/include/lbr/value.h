#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/protocol.h"

/*
 * Values that cannot silently become zero.
 *
 * Every measurement an app reads is an lbr_f32 / lbr_i32 / lbr_bool: the
 * number plus its status and age. A sensor that is not fitted, a node that
 * dropped off the bus, a value that stopped updating, a division by zero:
 * each shows up as a status, never as a plausible-looking number.
 *
 *     lbr_f32 ratio = lbr_div(engine_rpm, cvt_rpm);   // status flows through
 *     if (lbr_ok(ratio)) { ... ratio.v ... }
 *     float shown = lbr_or(ratio, 0.0f);              // a default, said out loud
 *
 * Arithmetic propagates the first operand's problem (or the second's), like
 * NaN, but you can ask why. Results that would be NaN or infinite become
 * LBR_BAD_MATH. The age of a result is the age of its oldest input.
 * Pure; host-tested.
 */

typedef enum : uint8_t {
    LBR_OK = 0,

    /* The source node says it has no value (lbr_unavail_reason_t, same meaning). */
    LBR_UNSPECIFIED,
    LBR_NOT_FITTED,     /* no part/driver on that channel */
    LBR_NO_DATA,        /* driver alive, nothing fresh this tick */
    LBR_SENSOR_FAULT,   /* the part failed or will not start */
    LBR_OUT_OF_RANGE,   /* reading outside its valid range, or does not fit the frame */
    LBR_NOT_CALIBRATED, /* raw reading, no calibration */
    LBR_BUS_ERROR,      /* I2C/SPI/ADC transaction failed */

    /* The receiver found a problem. */
    LBR_STALE,        /* nothing fresh for 3 periods */
    LBR_NODE_LOST,    /* the source node went silent */
    LBR_NODE_MISSING, /* the source node was never heard this session */

    /* Arithmetic. */
    LBR_BAD_MATH, /* division by zero, or a NaN / infinite result */

    LBR_STATUS_COUNT_
} lbr_status;

typedef struct {
    float      v;
    lbr_status st;
    uint32_t   age_ms; /* how old the newest measurement behind it is */
} lbr_f32;

typedef struct {
    int32_t    v;
    lbr_status st;
    uint32_t   age_ms;
} lbr_i32;

typedef struct {
    bool       v;
    lbr_status st;
    uint32_t   age_ms;
} lbr_bool;

/* "OK", "NOT_FITTED", ... ("?" if unknown). */
const char *lbr_status_name(lbr_status st);

/* The status for a node's unavailability reason (protocol.h). */
lbr_status lbr_status_from_reason(lbr_unavail_reason_t reason);

/* Constructors. */
static inline lbr_f32 lbr_f32_of(float v)
{
    return (lbr_f32){.v = v, .st = LBR_OK, .age_ms = 0};
}
static inline lbr_i32 lbr_i32_of(int32_t v)
{
    return (lbr_i32){.v = v, .st = LBR_OK, .age_ms = 0};
}
static inline lbr_bool lbr_bool_of(bool v)
{
    return (lbr_bool){.v = v, .st = LBR_OK, .age_ms = 0};
}
static inline lbr_f32 lbr_f32_missing(lbr_status st)
{
    return (lbr_f32){.v = 0.0f, .st = st == LBR_OK ? LBR_UNSPECIFIED : st, .age_ms = 0};
}

/* True when x holds a usable value. Works on lbr_f32, lbr_i32 and lbr_bool. */
#define lbr_ok(x) ((x).st == LBR_OK)

/* x's value, or `fallback` when x is unavailable. The fallback is explicit on
 * purpose: choosing what "unknown" means is the caller's decision. */
#define lbr_or(x, fallback) (lbr_ok(x) ? (x).v : (fallback))

/* Arithmetic on lbr_f32. */
[[nodiscard]] lbr_f32 lbr_add(lbr_f32 a, lbr_f32 b);
[[nodiscard]] lbr_f32 lbr_sub(lbr_f32 a, lbr_f32 b);
[[nodiscard]] lbr_f32 lbr_mul(lbr_f32 a, lbr_f32 b);
[[nodiscard]] lbr_f32 lbr_div(lbr_f32 a, lbr_f32 b); /* b == 0: LBR_BAD_MATH */
[[nodiscard]] lbr_f32 lbr_scale(lbr_f32 a, float k);
[[nodiscard]] lbr_f32 lbr_offset(lbr_f32 a, float k);
[[nodiscard]] lbr_f32 lbr_abs(lbr_f32 a);
[[nodiscard]] lbr_f32 lbr_min(lbr_f32 a, lbr_f32 b);
[[nodiscard]] lbr_f32 lbr_max(lbr_f32 a, lbr_f32 b);
[[nodiscard]] lbr_f32 lbr_clamp(lbr_f32 a, float lo, float hi);

/* Comparisons give an lbr_bool that is unavailable when an input is. */
[[nodiscard]] lbr_bool lbr_gt(lbr_f32 a, float limit);
[[nodiscard]] lbr_bool lbr_lt(lbr_f32 a, float limit);

/* An lbr_i32 as an lbr_f32 (same status and age). */
[[nodiscard]] lbr_f32 lbr_i32_to_f32(lbr_i32 a);
