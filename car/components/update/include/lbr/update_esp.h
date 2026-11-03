#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/update.h"

/*
 * ESP32 side of firmware updates for sensor nodes (ESP only; not built on
 * host). The image goes into the spare OTA slot; the bootloader runs it once
 * after ACTIVATE and rolls it back unless it confirms itself.
 */

/* Flash access for lbr_upd_node_t. */
const lbr_upd_ota_t *lbr_update_esp_ota(void);

/*
 * Call first thing at boot, before CAN starts. On a normal boot it records
 * which node this board is (NVS). On the first boot of an image that arrived
 * over CAN, an image for a different node (e.g. corner_fr firmware on the
 * corner_fl board) is rejected at once: the board rolls back and reboots.
 */
void lbr_update_esp_boot(uint8_t node_id);

/*
 * Call once per heartbeat. `healthy` = on the Central's shared clock with CAN
 * running, which proves the new image can receive and send on the bus. A new
 * image confirms itself the first time it is healthy; if it is not healthy
 * within LBR_UPDATE_CONFIRM_TIMEOUT_S it rolls back to the previous one.
 */
#define LBR_UPDATE_CONFIRM_TIMEOUT_S 60u
void lbr_update_esp_confirm_tick(bool healthy);
