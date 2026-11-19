#include "temperature_plan.h"
#include "lbr.h"
#include "sdkconfig.h"

/*
 * Temperature Controller (ESP32, CAN node 4): brake and CVT temperatures.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

static void init(lbr_node *n)
{
    /* Inputs wired to on-chip ADC / pulse / digital pins are set in menuconfig
     * (CONFIG_LBR_IO_*); an empty setting leaves the slot unbound. A part driver
     * (issue #10) binds a slot as lbr_sensor_<part>(lbr_slot(n, SLOT), ...). */
    lbr_bind_io(n, TEMPERATURE_SLOT_BRAKES, CONFIG_LBR_IO_BRAKES);
    lbr_bind_io(n, TEMPERATURE_SLOT_CVT, CONFIG_LBR_IO_CVT);
}

LBR_NODE_MAIN(&lbr_plan_temperature, .init = init);
