#include "imu_plan.h"
#include "lbr/node_app.h"

/*
 * IMU Controller (ESP32-C5, CAN node 1) app: the chassis IMU. Everything
 * except driver binding lives in the shared node runtime (car/components/node).
 */

static lbr_sensor_t      s_sensors[IMU_SLOT_COUNT];
static lbr_acq_t         s_acq[IMU_SLOT_COUNT];
static lbr_node_config_t s_cfg;

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    lbr_node_build(&lbr_plan_imu, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);

    /*
     * The chassis IMU waits for a part-specific driver (issue #10):
     *
     *     lbr_sensor_<part>(&s_sensors[IMU_SLOT_CHASSIS], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */

    lbr_node_run(&s_cfg);
}
