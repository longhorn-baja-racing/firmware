#include "car_logic.h"

/* The car's tasks, in the order they run each tick. A new task: write it in its
 * own file, declare it in car_logic.h, and add it here. */
const lbr_task_list car_logic = LBR_TASK_LIST(&vehicle, &cvt, &brakes);
