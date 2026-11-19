#include "cockpit_plan.h"
#include "display.h"
#include "lbr.h"

/*
 * Cockpit Controller (ESP32-C5, own battery): the driver's display, over WiFi
 * from the Central. Everything is the cockpit role (lbrlib/components/
 * lbr_cockpit); this car picks the display driver. The battery slot binds from
 * menuconfig (main/Kconfig, generated from the catalog).
 */

LBR_COCKPIT_MAIN(&lbr_plan_cockpit, .display = &cockpit_console);
