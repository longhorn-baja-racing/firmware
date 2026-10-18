#pragma once

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/transport.h"

/*
 * Concrete gateway links (ESP only). Each returns a ready lbr_transport_t,
 * or NULL if it cannot run (pins TBD, no password, out of memory); the
 * caller reports that as a degraded link, never a crash.
 */

/* Text callbacks shared by the links that answer questions. */
typedef struct {
    /* Status text (lbr_status_format). Returns length. */
    size_t (*status)(char *buf, size_t cap);
    /* Current runtime config text. Returns length. */
    size_t (*config_get)(char *buf, size_t cap);
    /* Validate + store a new runtime config (applies next session). 0 = ok;
     * otherwise a reason is written to err. */
    int (*config_put)(const char *text, size_t len, char *err, size_t err_len);
    /* Called before an OTA reboot: close the session log cleanly. */
    void (*before_reboot)(void);
} lbr_link_hooks_t;

/* USB serial on UART0. Host commands (one per line):
 *   STAT                  status text
 *   LIST                  FILE_INFO records for every session file
 *   DUMP <name> [offset]  FILE_CHUNK records, then FILE_END
 * Everything the gateway sends is a COBS-framed packet; ESP log output is
 * carried as TEXT records (see lbr_usb_log_vprintf). */
lbr_transport_t *lbr_link_usb_start(const lbr_link_hooks_t *hooks);
/* Install as esp_log vprintf to route log lines through the USB link. */
int lbr_usb_log_vprintf(const char *fmt, va_list ap);

/* LoRa on an SX127x. Packets go out one at a time; while the radio is busy
 * or the duty budget is spent, lbr_link_lora_ready() is false. */
lbr_transport_t *lbr_link_lora_start(uint32_t duty_permille);
bool             lbr_link_lora_ready(void);

/* WiFi soft AP: TCP live stream plus HTTP (status, logs, OTA, config). */
lbr_transport_t *lbr_link_wifi_start(const lbr_link_hooks_t *hooks);

/* BLE GATT server exposing read-only status and version. */
lbr_transport_t *lbr_link_ble_start(const lbr_link_hooks_t *hooks);
