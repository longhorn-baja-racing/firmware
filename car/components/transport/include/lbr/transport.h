#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * One logical record model behind every live transport. Each implementation
 * owns its framing and bandwidth policy but MUST NOT change the meaning of a
 * record. See README "Live transports" and packet.h.
 */

typedef enum {
    LBR_LINK_LORA = 0, /* in-race, long range, tiny bandwidth, lossy */
    LBR_LINK_USB,      /* wired bench/pit, bulk */
    LBR_LINK_WIFI,     /* pit, bulk / OTA / config */
    LBR_LINK_BLE,      /* walk-up diagnostics, low power */
    LBR_LINK_COUNT_
} lbr_link_kind_t;

typedef struct lbr_transport lbr_transport_t;

typedef struct {
    uint32_t packets_sent;
    uint32_t packets_dropped; /* queue full / link down: counted, never blocking */
    uint32_t bytes_sent;
    uint32_t errors;
    bool     connected;
} lbr_transport_stats_t;

struct lbr_transport {
    const char     *name;
    lbr_link_kind_t kind;

    int (*init)(lbr_transport_t *self);

    /* Non-blocking: enqueue one finished packet (packet.h). Returns 0 on
     * success, <0 if it was dropped. The transport counts the drop; a slow or
     * absent link never blocks acquisition or the authoritative log. */
    int (*send)(lbr_transport_t *self, const uint8_t *pkt, size_t len);

    void (*get_stats)(lbr_transport_t *self, lbr_transport_stats_t *out);

    void *ctx; /* implementation state */
};

/* True if this link should carry only the prioritized in-race subset (LoRa). */
static inline bool lbr_link_is_bandwidth_limited(lbr_link_kind_t k)
{
    return k == LBR_LINK_LORA;
}
