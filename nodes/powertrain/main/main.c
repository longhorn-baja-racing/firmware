#include "lbr/protocol.h"
#include "lbr/sensor.h"
#include "lbr/timebase.h"
#include "lbr/transport.h"

/*
 * Powertrain sensor node.
 *
 * Owns (parts TBD): engine RPM, engine/CVT temperatures, fuel level.
 * Samples each sensor at its configured rate, timestamps against the shared
 * time base, and publishes schema-driven records on CAN. Local SD logging only
 * if this node is deemed high-rate/safety-critical (see README "redundancy").
 *
 * Signals: LBR_SIG_ENGINE_RPM, LBR_SIG_TEMP_ENGINE, LBR_SIG_TEMP_CVT,
 *          LBR_SIG_FUEL_LEVEL (see components/protocol/include/lbr/signals.h).
 */

#define LBR_NODE_ID 0x02u

void app_main(void)
{
    /* TODO: init TWAI (CAN) + external transceiver. */
    /* TODO: register sensor drivers (lbr_sensor_t) for RPM / temps / fuel. */
    /* TODO: discipline clock from gateway time-sync frames (lbr_time_sync_apply). */
    /* TODO: acquisition loop -> timestamp -> publish CAN records; emit heartbeats. */
    (void)LBR_NODE_ID;
}
