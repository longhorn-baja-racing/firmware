#include "car_logic.h"

/* Vehicle state the driver and the pit read at a glance. */

#define ENGINE_RUNNING_RPM 1000.0f

LBR_OUTPUT(speed_kph, "Vehicle/SpeedKph", F32, NONE);
LBR_OUTPUT(engine_running, "Engine/Running", BOOL, NONE);

static void vehicle_periodic(lbr_cycle *c)
{
    lbr_out(c, &speed_kph, lbr_scale(lbr_get(c, GPS_SPEED), 3.6f));
    lbr_out_bool(c, &engine_running, lbr_gt(lbr_get(c, ENGINE_RPM), ENGINE_RUNNING_RPM));
}

LBR_TASK(vehicle, .rate_hz = 10, .periodic = vehicle_periodic);
