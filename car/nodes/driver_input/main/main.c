#include "driver_input_plan.h"
#include "lbr/node_app.h"
#include "sdkconfig.h"

/*
 * Driver Input Controller (ESP32, CAN node 3): pedals and steering.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

static void init(lbr_node *n)
{
    /* Inputs wired to on-chip ADC / pulse / digital pins are set in menuconfig
     * (CONFIG_LBR_IO_*); an empty setting leaves the slot unbound. A part driver
     * (issue #10) binds a slot as lbr_sensor_<part>(lbr_slot(n, SLOT), ...). */
    lbr_bind_io(n, DRIVER_INPUT_SLOT_INPUTS, CONFIG_LBR_IO_INPUTS);
}

LBR_NODE_MAIN(&lbr_plan_driver_input, .init = init);
