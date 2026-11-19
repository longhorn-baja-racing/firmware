#include <math.h>

#include "lbr/value.h"
#include "lbr_test.h"

/* Values with status (lbr/value.h): unavailability and bad math always show,
 * never as a number. */

static lbr_f32 aged(float v, uint32_t age_ms)
{
    lbr_f32 x = lbr_f32_of(v);
    x.age_ms  = age_ms;
    return x;
}

static void test_ok_arithmetic(void)
{
    lbr_f32 r = lbr_div(aged(3000.0f, 5), aged(1500.0f, 12));
    CHECK(lbr_ok(r));
    CHECK(r.v == 2.0f);
    CHECK_EQ_U(r.age_ms, 12); /* the oldest input */
    CHECK(lbr_add(lbr_f32_of(1.0f), lbr_f32_of(2.0f)).v == 3.0f);
    CHECK(lbr_sub(lbr_f32_of(1.0f), lbr_f32_of(2.0f)).v == -1.0f);
    CHECK(lbr_mul(lbr_f32_of(1.5f), lbr_f32_of(2.0f)).v == 3.0f);
    CHECK(lbr_scale(lbr_f32_of(2.0f), 0.5f).v == 1.0f);
    CHECK(lbr_offset(lbr_f32_of(2.0f), -0.5f).v == 1.5f);
    CHECK(lbr_abs(lbr_f32_of(-4.0f)).v == 4.0f);
    CHECK(lbr_min(lbr_f32_of(1.0f), lbr_f32_of(2.0f)).v == 1.0f);
    CHECK(lbr_max(lbr_f32_of(1.0f), lbr_f32_of(2.0f)).v == 2.0f);
    CHECK(lbr_clamp(lbr_f32_of(7.0f), 0.0f, 5.0f).v == 5.0f);
    CHECK(lbr_gt(lbr_f32_of(7.0f), 5.0f).v);
    CHECK(!lbr_lt(lbr_f32_of(7.0f), 5.0f).v);
    CHECK(lbr_i32_to_f32(lbr_i32_of(-3)).v == -3.0f);
}

static void test_unavailable_propagates(void)
{
    lbr_f32 missing = lbr_f32_missing(LBR_NODE_LOST);
    lbr_f32 fitted  = lbr_f32_missing(LBR_NOT_FITTED);
    lbr_f32 one     = lbr_f32_of(1.0f);

    CHECK_EQ_U(lbr_add(missing, one).st, LBR_NODE_LOST);
    CHECK_EQ_U(lbr_add(one, missing).st, LBR_NODE_LOST);
    CHECK_EQ_U(lbr_div(fitted, missing).st, LBR_NOT_FITTED); /* first operand's reason */
    CHECK_EQ_U(lbr_scale(missing, 2.0f).st, LBR_NODE_LOST);
    CHECK(lbr_add(missing, one).v == 0.0f); /* no stale number leaks out */

    /* A comparison on an unavailable value is unavailable, and never true. */
    lbr_bool hot = lbr_gt(missing, -1000.0f);
    CHECK(!lbr_ok(hot));
    CHECK(!hot.v);

    CHECK(lbr_or(missing, -1.0f) == -1.0f);
    CHECK(lbr_or(one, -1.0f) == 1.0f);
    CHECK_EQ_U(lbr_f32_missing(LBR_OK).st, LBR_UNSPECIFIED); /* "missing but OK" is not allowed */
}

static void test_bad_math(void)
{
    CHECK_EQ_U(lbr_div(lbr_f32_of(1.0f), lbr_f32_of(0.0f)).st, LBR_BAD_MATH);
    CHECK_EQ_U(lbr_div(lbr_f32_of(0.0f), lbr_f32_of(0.0f)).st, LBR_BAD_MATH);
    CHECK_EQ_U(lbr_mul(lbr_f32_of(3e38f), lbr_f32_of(3e38f)).st, LBR_BAD_MATH); /* inf */
    CHECK_EQ_U(lbr_add(lbr_f32_of(NAN), lbr_f32_of(1.0f)).st, LBR_BAD_MATH);
    CHECK_EQ_U(lbr_gt(lbr_f32_of(NAN), 0.0f).st, LBR_BAD_MATH);
    /* A real problem upstream wins over the math it causes. */
    CHECK_EQ_U(lbr_div(lbr_f32_missing(LBR_STALE), lbr_f32_of(0.0f)).st, LBR_STALE);
}

static void test_names_and_reasons(void)
{
    CHECK(strcmp(lbr_status_name(LBR_OK), "OK") == 0);
    CHECK(strcmp(lbr_status_name(LBR_NODE_MISSING), "NODE_MISSING") == 0);
    CHECK(strcmp(lbr_status_name(LBR_STATUS_COUNT_), "?") == 0);
    for (unsigned s = 0; s < LBR_STATUS_COUNT_; s++) {
        CHECK(strcmp(lbr_status_name((lbr_status)s), "?") != 0); /* every status named */
    }
    CHECK_EQ_U(lbr_status_from_reason(LBR_UNAVAIL_NOT_FITTED), LBR_NOT_FITTED);
    CHECK_EQ_U(lbr_status_from_reason(LBR_UNAVAIL_BUS_ERROR), LBR_BUS_ERROR);
    CHECK_EQ_U(lbr_status_from_reason(LBR_UNAVAIL_UNSPECIFIED), LBR_UNSPECIFIED);
}

int main(void)
{
    test_ok_arithmetic();
    test_unavailable_propagates();
    test_bad_math();
    test_names_and_reasons();
    return lbr_test_failures;
}
