#include "suspension_plan.h"
#include "sdkconfig.h"

/*
 * suspension node app. Everything except driver binding lives in the shared node
 * runtime (components/node).
 */

static lbr_sensor_t      s_sensors[SUSPENSION_SLOT_COUNT];
static lbr_acq_t         s_acq[SUSPENSION_SLOT_COUNT];
static lbr_node_config_t s_cfg;

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
#if CONFIG_LBR_SENSOR_SIM
    const bool sim = true;
#else
    const bool sim = false;
#endif
    lbr_node_build(&lbr_plan_suspension, sim, s_sensors, s_acq, &s_cfg);

    /*
     * Bind real drivers here as parts are chosen (issue #10), e.g.:
     *
     *     lbr_sensor_<part>(&s_sensors[SUSPENSION_SLOT_TRAVEL], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */

    lbr_node_run(&s_cfg);
}
