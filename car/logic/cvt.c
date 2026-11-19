#include "car_logic.h"

/*
 * CVT ratio: engine speed over CVT (secondary) speed, smoothed. Unavailable,
 * with the reason, whenever either speed is (sensor not fitted, powertrain
 * node lost, stale data) or the CVT is not turning.
 */

LBR_OUTPUT(ratio, "CVT/Ratio", F32, NONE);

static lbr_filter s_ratio;

static void cvt_periodic(lbr_cycle *c)
{
    lbr_f32 raw = lbr_div(lbr_get(c, ENGINE_RPM), lbr_get(c, CVT_RPM));
    lbr_out(c, &ratio, lbr_lowpass(&s_ratio, raw, 0.3f));
}

LBR_TASK(cvt, .rate_hz = 50, .periodic = cvt_periodic);
