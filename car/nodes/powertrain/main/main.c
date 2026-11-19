#include "powertrain_plan.h"
#include "lbr/node_app.h"
#include "sdkconfig.h"

/*
 * Powertrain Controller (ESP32, CAN node 2): engine and CVT speed, fuel level.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

static void init(lbr_node *n)
{
    /* Inputs wired to on-chip ADC / pulse / digital pins are set in menuconfig
     * (CONFIG_LBR_IO_*); an empty setting leaves the slot unbound. A part driver
     * (issue #10) binds a slot as lbr_sensor_<part>(lbr_slot(n, SLOT), ...). */
    lbr_bind_io(n, POWERTRAIN_SLOT_SPEEDS, CONFIG_LBR_IO_SPEEDS);
    lbr_bind_io(n, POWERTRAIN_SLOT_FUEL, CONFIG_LBR_IO_FUEL);
}

LBR_NODE_MAIN(&lbr_plan_powertrain, .init = init);
