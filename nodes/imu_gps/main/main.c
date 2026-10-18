#include "imu_gps_plan.h"
#include "lbr/nmea.h"
#include "sdkconfig.h"

/*
 * imu_gps node app. Everything except driver binding lives in the shared node
 * runtime (components/node).
 */

static lbr_sensor_t      s_sensors[IMU_GPS_SLOT_COUNT];
static lbr_acq_t         s_acq[IMU_GPS_SLOT_COUNT];
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
    lbr_node_build(&lbr_plan_imu_gps, sim, s_sensors, s_acq, &s_cfg);

    /*
     * GPS: any NMEA 0183 module on a UART (CONFIG_LBR_GPS_*). The IMU slots
     * wait for a part-specific driver (issue #10), bound the same way:
     *
     *     lbr_sensor_<part>(&s_sensors[IMU_GPS_SLOT_IMU], ...);
     *
     * A bound driver must produce exactly the slot's signals, in order.
     */
#if CONFIG_LBR_GPS_RX_GPIO >= 0
    static const lbr_gps_uart_cfg_t gps = {
        .uart_num = CONFIG_LBR_GPS_UART_NUM,
        .rx_gpio  = CONFIG_LBR_GPS_RX_GPIO,
        .tx_gpio  = CONFIG_LBR_GPS_TX_GPIO,
        .baud     = CONFIG_LBR_GPS_BAUD,
    };
    const lbr_slot_t *slot = &lbr_plan_imu_gps.slots[IMU_GPS_SLOT_GPS];
    lbr_sensor_gps_nmea(&s_sensors[IMU_GPS_SLOT_GPS], "gps_nmea", slot->signals, slot->n_signals,
                        &gps);
#endif

    lbr_node_run(&s_cfg);
}
