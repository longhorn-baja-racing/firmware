#include <string.h>

#include "gw_internal.h"
#include "lbr/bytes.h"
#include "lbr/nodes.h"
#include "lbr/status.h"
#include "lbr/version.h"

/* The Central's low-rate links: the LoRa in-race subset and the cockpit display
 * link (datagrams out, battery and status in). */

void lbr_gw_core_lora_tick(lbr_gw_core_t *c)
{
    uint8_t pkt[LBR_PKT_LORA_MAX];
    if (c->links.lora == NULL || c->hal.lora_ready == NULL || !c->hal.lora_ready(c->hal.ctx)) {
        return; /* radio busy or duty budget spent: the policy keeps the latest */
    }
    gw_lock(c, LBR_GW_LOCK_STATE);
    size_t n = lbr_lora_policy_build(&c->lora_policy, c->hal.now_us(c->hal.ctx), NODE_ID, 0, pkt,
                                     sizeof pkt);
    gw_unlock(c, LBR_GW_LOCK_STATE);
    if (n > 0) {
        c->links.lora->send(c->links.lora, pkt, n);
    }
}

void lbr_gw_core_cockpit_tick(lbr_gw_core_t *c)
{
    uint8_t pkt[LBR_PKT_MAX_SIZE];
    size_t  n = 0;
    gw_lock(c, LBR_GW_LOCK_STATE);
    uint64_t now    = c->hal.now_us(c->hal.ctx);
    int      change = lbr_cockpit_link_tick(&c->cockpit, now);
    if (c->links.cockpit != NULL) {
        n = lbr_cockpit_link_build(&c->cockpit, now, NODE_ID, &c->cockpit_values, &c->nodes, pkt,
                                   sizeof pkt);
    }
    gw_unlock(c, LBR_GW_LOCK_STATE);
    if (n > 0 && c->links.cockpit->send(c->links.cockpit, pkt, n) == 0) {
        atomic_fetch_add(&c->cockpit_sent, 1);
    }
    if (change > 0) {
        lbr_gw_core_event(c, LBR_EVT_NODE_FOUND, LBR_NODE_COCKPIT, false);
    } else if (change < 0) {
        lbr_gw_core_event(c, LBR_EVT_NODE_LOST, LBR_NODE_COCKPIT, true);
    }
}

void lbr_gw_core_on_cockpit_packet(lbr_gw_core_t *c, const uint8_t *buf, size_t len,
                                   uint64_t rx_local_us)
{
    gw_lock(c, LBR_GW_LOCK_STATE);
    lbr_cockpit_link_on_packet(&c->cockpit, buf, len,
                               c->hal.local_to_shared(c->hal.ctx, rx_local_us));
    gw_unlock(c, LBR_GW_LOCK_STATE);
}
