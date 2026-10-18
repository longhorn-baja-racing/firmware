#include "driver_inputs_plan.h"
#include "lbr/node_app.h"
#include "sdkconfig.h"

/*
 * driver_inputs node app. Everything except driver binding lives in the shared node
 * runtime (car/components/node).
 */

static lbr_sensor_t      s_sensors[DRIVER_INPUTS_SLOT_COUNT];
static lbr_acq_t         s_acq[DRIVER_INPUTS_SLOT_COUNT];
static lbr_node_config_t s_cfg;

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    lbr_node_build(&lbr_plan_driver_inputs, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);

    /*
     * Slots wired to on-chip ADC / pulse / digital inputs are configured in
     * menuconfig (CONFIG_LBR_IO_*); empty settings leave them not fitted.
     * Part-specific drivers (issue #10) replace a slot the same way:
     *
     *     lbr_sensor_<part>(&s_sensors[DRIVER_INPUTS_SLOT_...], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */
    lbr_node_bind_io(&lbr_plan_driver_inputs, DRIVER_INPUTS_SLOT_PEDALS, s_sensors,
                     CONFIG_LBR_IO_PEDALS);
    lbr_node_bind_io(&lbr_plan_driver_inputs, DRIVER_INPUTS_SLOT_STEERING, s_sensors,
                     CONFIG_LBR_IO_STEERING);
    lbr_node_bind_io(&lbr_plan_driver_inputs, DRIVER_INPUTS_SLOT_BRAKE_SWITCH, s_sensors,
                     CONFIG_LBR_IO_BRAKE_SWITCH);

    lbr_node_run(&s_cfg);
}
