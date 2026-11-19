#include "corner_plan.h"
#include "lbr.h"
#include "sdkconfig.h"

/*
 * Wheel hub node (Mini ESP32-C5, CAN nodes 7..10): the suspension IMU and the
 * hub's own battery level. The same firmware runs on all four;
 * CONFIG_LBR_CORNER_POSITION picks the plan.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

static void init(lbr_node *n)
{
    /* The battery is an ADC input configured in menuconfig (CONFIG_LBR_IO_BATTERY);
     * empty leaves it unbound. The IMU waits for a part-specific driver (issue #10):
     * lbr_bind(n, CORNER_SLOT_IMU, &<part>, &state). */
    lbr_bind_io(n, CORNER_SLOT_BATTERY, CONFIG_LBR_IO_BATTERY);
}

LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION], .init = init);
