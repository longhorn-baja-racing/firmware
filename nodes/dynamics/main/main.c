#include "lbr/protocol.h"
#include "lbr/sensor.h"
#include "lbr/timebase.h"
#include "lbr/transport.h"

/*
 * Dynamics sensor node.
 *
 * Owns (parts TBD): IMU (accel/gyro/orientation) and vehicle/wheel speed.
 * Samples each sensor at its configured rate, timestamps against the shared
 * time base, and publishes schema-driven records on CAN. Reports sensor-frame
 * values faithfully — the mounting transform is applied by the dashboard.
 *
 * Signals: LBR_SIG_ACCEL_*, LBR_SIG_GYRO_*, LBR_SIG_ORIENT_QUAT_*,
 *          LBR_SIG_VEHICLE_SPEED, LBR_SIG_WHEEL_SPEED_* (see signals.h).
 */

#define LBR_NODE_ID 0x03u

void app_main(void)
{
    /* TODO: init TWAI (CAN) + external transceiver. */
    /* TODO: register sensor drivers (lbr_sensor_t) for IMU / speed. */
    /* TODO: discipline clock from gateway time-sync frames (lbr_time_sync_apply). */
    /* TODO: acquisition loop -> timestamp -> publish CAN records; emit heartbeats. */
    (void)LBR_NODE_ID;
}
