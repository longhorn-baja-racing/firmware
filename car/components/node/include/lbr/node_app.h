#pragma once

#include "lbr/node.h"
#include "sdkconfig.h"

/*
 * Node apps (car/nodes/<node>/main/main.c) only. The driver every slot gets
 * until a real part is bound: NOT_FITTED on race builds, the simulator on
 * bench builds (CONFIG_LBR_SENSOR_SIM=y, flagged in every heartbeat). The
 * simulator is not even compiled into a race build.
 */
#if CONFIG_LBR_SENSOR_SIM
#define LBR_NODE_UNBOUND_DRIVER lbr_sensor_sim
#else
#define LBR_NODE_UNBOUND_DRIVER lbr_sensor_unconfigured
#endif
