#include "car_logic.h"

/*
 * Overheat alarms for the four brakes and the CVT. Limits are provisional
 * (TBD with the drivetrain and brake leads). An alarm turns on above its "on"
 * limit and off below its "off" limit, so it does not flicker. While its
 * sensor is unavailable the alarm is logged as unknown, with the reason,
 * never as "off".
 */

#define BRAKE_ON_C  300.0f
#define BRAKE_OFF_C 270.0f
#define CVT_ON_C    110.0f
#define CVT_OFF_C   100.0f

LBR_OUTPUT(hottest_brake, "Brakes/HottestC", F32, DEG_C);
LBR_OUTPUT(brake_alarm, "Alarms/BrakeOverheat", BOOL, NONE);
LBR_OUTPUT(cvt_alarm, "Alarms/CvtOverheat", BOOL, NONE);

static bool s_brake_hot, s_cvt_hot;

static void brakes_periodic(lbr_cycle *c)
{
    lbr_f32 front = lbr_max(lbr_get(c, TEMP_BRAKE_FL), lbr_get(c, TEMP_BRAKE_FR));
    lbr_f32 back  = lbr_max(lbr_get(c, TEMP_BRAKE_BL), lbr_get(c, TEMP_BRAKE_BR));
    lbr_f32 hot   = lbr_max(front, back);

    lbr_out(c, &hottest_brake, hot);
    lbr_out_bool(c, &brake_alarm, lbr_above(&s_brake_hot, hot, BRAKE_ON_C, BRAKE_OFF_C));
    lbr_out_bool(c, &cvt_alarm, lbr_above(&s_cvt_hot, lbr_get(c, TEMP_CVT), CVT_ON_C, CVT_OFF_C));
}

LBR_TASK(brakes, .rate_hz = 10, .periodic = brakes_periodic);
