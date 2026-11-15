#include "corner_plan.h"
#include "lbr/node_app.h"
#include "lbr/sensor_io.h"
#include "sdkconfig.h"

/*
 * Wheel hub node app (Mini ESP32-C5, CAN nodes 7..10): suspension IMU and
 * local battery percentage acquisition.
 * The same firmware runs on all four; CONFIG_LBR_CORNER_POSITION picks the
 * plan. Everything except driver binding lives in the shared node runtime
 * (car/components/node).
 */

static lbr_sensor_t      s_sensors[CORNER_SLOT_COUNT];
static lbr_acq_t         s_acq[CORNER_SLOT_COUNT];
static lbr_node_config_t s_cfg;

/* Entry point called by the ESP-IDF startup code. */
void app_main(void);

void app_main(void)
{
    const lbr_node_plan_t *plan = &lbr_plan_corner[CONFIG_LBR_CORNER_POSITION];
    lbr_node_build(plan, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_cfg);

    if (CONFIG_LBR_IO_BATTERY[0] != '\0') {
        const lbr_slot_t *b = &plan->slots[CORNER_SLOT_BATTERY];
        lbr_sensor_io(&s_sensors[CORNER_SLOT_BATTERY], b->name, b->signals, b->n_signals,
                      CONFIG_LBR_IO_BATTERY);
    }

    /*
     * The suspension IMU waits for a part-specific driver (issue #10):
     *
     *     lbr_sensor_<part>(&s_sensors[CORNER_SLOT_IMU], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */

    lbr_node_run(&s_cfg);
}
