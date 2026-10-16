#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * One logical record model behind every live transport. Each implementation
 * owns its framing and bandwidth policy but MUST NOT change the meaning of a
 * record. See README "Live transports".
 */

typedef enum {
    LBR_LINK_LORA = 0, /* in-race, long range, tiny bandwidth, lossy */
    LBR_LINK_USB,      /* wired bench/pit, bulk */
    LBR_LINK_WIFI,     /* pit, bulk / OTA / config */
    LBR_LINK_BLE,      /* walk-up diagnostics, low power */
} lbr_link_kind_t;

typedef struct lbr_transport lbr_transport_t;

struct lbr_transport {
    const char     *name;
    lbr_link_kind_t kind;

    int (*init)(lbr_transport_t *self);

    /* Non-blocking: enqueue one framed record. Returns 0 on success, <0 if the
     * bounded queue is full. The caller MUST count the drop and continue - a
     * slow/absent link must never block acquisition or the authoritative log. */
    int (*send)(lbr_transport_t *self, const uint8_t *buf, size_t len);

    void *ctx; /* implementation state */
};

/* True if this link should carry only the prioritized in-race subset (LoRa). */
static inline bool lbr_link_is_bandwidth_limited(lbr_link_kind_t k)
{
    return k == LBR_LINK_LORA;
}
