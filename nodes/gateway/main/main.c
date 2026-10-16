#include "lbr/health.h"
#include "lbr/logging.h"
#include "lbr/protocol.h"
#include "lbr/timebase.h"
#include "lbr/transport.h"

/*
 * Gateway / logger node.
 *
 * Responsibilities (see README "Gateway / logger node"):
 *   - sniff the entire CAN (TWAI) bus
 *   - write the authoritative, time-aligned, full-fidelity microSD log
 *   - broadcast the CAN time-sync frame that disciplines every node
 *   - drive the live transports: LoRa in-race; USB/WiFi/BLE for pit/bench
 *
 * A slow or absent live link must NEVER reduce the fidelity of the SD log.
 * CAN rx -> log append (authoritative) and -> transport send (downsampled) run
 * through independent bounded queues.
 */

#define LBR_NODE_ID 0x01u

void app_main(void)
{
    /* TODO: init TWAI (CAN) + external transceiver. */
    /* TODO: mount microSD; open authoritative log session (lbr_log_open). */
    /* TODO: init transports (LoRa/USB/WiFi/BLE) behind lbr_transport_t. */
    /* TODO: start periodic CAN time-sync broadcast (lbr_time_now_us). */
    /* TODO: run the bounded-queue pipeline and emit heartbeats. */
    (void)LBR_NODE_ID;
}
