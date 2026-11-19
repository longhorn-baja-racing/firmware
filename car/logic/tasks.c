#include "car_logic.h"

/* The order tasks run in each tick. A new task: write it in its own file,
 * declare it in car_logic.h, and add it here. */
const lbr_task *const car_tasks[] = {
    &vehicle,
    &cvt,
    &brakes,
};

const size_t car_task_count = sizeof car_tasks / sizeof car_tasks[0];
