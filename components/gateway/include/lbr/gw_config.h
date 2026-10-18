#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Gateway runtime configuration: settings the pit crew may change without
 * reflashing (pushed over WiFi, stored in NVS). Text form, one "key=value"
 * per line; '#' starts a comment. A pushed config is validated in full
 * before it is stored, and only takes effect at the next session start, so
 * a running session never changes configuration midway. The exact text in
 * effect is persisted in the session record of every log.
 */

typedef struct {
    bool     usb_enable;
    bool     wifi_enable;
    bool     ble_enable;
    bool     lora_enable;
    uint16_t lora_period_ms;     /* one radio packet at most this often */
    uint16_t lora_duty_permille; /* long-run airtime ceiling */
    uint16_t stream_batch_ms;    /* USB/WiFi packet batching window */
    uint16_t node_timeout_ms;    /* silent this long => NODE_LOST */
} lbr_gw_config_t;

void lbr_gw_config_defaults(lbr_gw_config_t *c);

/* Parse and validate. On success returns 0 and fills `out` (defaults for
 * keys not given). On failure returns <0, leaves `out` untouched, and writes
 * a one-line reason to `err`. Unknown keys are rejected. */
int lbr_gw_config_parse(const char *text, size_t len, lbr_gw_config_t *out, char *err,
                        size_t err_len);

/* Canonical text form (every key, fixed order). */
size_t lbr_gw_config_format(const lbr_gw_config_t *c, char *buf, size_t cap);
