#pragma once

#include <stddef.h>

#include "lbr.h"

/*
 * The 2026 car's logic: periodic tasks on the Central (lbr/logic.h). Each
 * task lives in its own file in car/logic/ and is listed in tasks.c, which
 * also fixes the order they run in.
 */

extern const lbr_task cvt;     /* cvt.c: CVT ratio */
extern const lbr_task brakes;  /* brakes.c: brake and CVT overheat alarms */
extern const lbr_task vehicle; /* vehicle.c: speed in km/h, engine running */

/* Every task, in run order. */
extern const lbr_task *const car_tasks[];
extern const size_t          car_task_count;
