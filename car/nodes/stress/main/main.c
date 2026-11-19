#include "stress_plan.h"
#include "lbr/node_app.h"
#include "sdkconfig.h"

/*
 * Stress Controller (ESP32, CAN node 5): stress sensors 1..8.
 *
 * Everything but binding parts to slots is the shared node runtime
 * (lbrlib/components/node). Slots left unbound report NOT_FITTED.
 */

static void init(lbr_node *n)
{
    /* Inputs wired to on-chip ADC / pulse / digital pins are set in menuconfig
     * (CONFIG_LBR_IO_*); an empty setting leaves the slot unbound. A part driver
     * (issue #10) binds a slot as lbr_sensor_<part>(lbr_slot(n, SLOT), ...). */
    lbr_bind_io(n, STRESS_SLOT_STRESS, CONFIG_LBR_IO_STRESS);
}

LBR_NODE_MAIN(&lbr_plan_stress, .init = init);
