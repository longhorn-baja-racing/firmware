#include "lbr/node_table.h"

#include <string.h>

#include "lbr/nodes.h"

void lbr_node_table_init(lbr_node_table_t *t, uint64_t timeout_us)
{
    memset(t, 0, sizeof *t);
    t->timeout_us = timeout_us;
}

static lbr_node_entry_t *slot_for(lbr_node_table_t *t, uint8_t node_id)
{
    lbr_node_entry_t *free_slot = NULL;
    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE; i++) {
        if (t->nodes[i].node_id == node_id) {
            return &t->nodes[i];
        }
        if (t->nodes[i].node_id == 0 && free_slot == NULL) {
            free_slot = &t->nodes[i];
        }
    }
    if (free_slot != NULL) {
        memset(free_slot, 0, sizeof *free_slot);
        free_slot->node_id = node_id;
    }
    return free_slot;
}

lbr_node_table_event_t lbr_node_table_on_frame(lbr_node_table_t *t, const lbr_can_frame_t *f,
                                               uint64_t rx_us)
{
    uint8_t source = lbr_can_sender(f->id);
    if (source == 0) {
        return LBR_NT_NONE;
    }
    lbr_node_entry_t *e = slot_for(t, source);
    if (e == NULL) {
        t->table_full++;
        return LBR_NT_NONE;
    }
    lbr_node_table_event_t ev = LBR_NT_NONE;
    if (!e->present) {
        e->present = true;
        if (e->first_seen_us == 0) {
            e->first_seen_us = rx_us;
        }
        ev = LBR_NT_FOUND;
    }
    e->last_seen_us = rx_us;
    e->frames++;

    uint8_t state, flags;
    if (lbr_can_decode_heartbeat(f, &state, &flags) != 0) {
        e->hb_seen  = true;
        e->hb_state = state;
        e->hb_flags = flags;
    }
    int page = lbr_can_decode_health_page(f, &e->health);
    if (page >= 0) {
        e->pages_seen |= (uint16_t)(1u << page);
        e->last_health_us = rx_us;
    }
    return ev;
}

size_t lbr_node_table_expire(lbr_node_table_t *t, uint64_t now_us, uint8_t *lost, size_t cap)
{
    size_t n = 0;
    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE; i++) {
        lbr_node_entry_t *e = &t->nodes[i];
        if (e->node_id != 0 && e->present && now_us > e->last_seen_us &&
            now_us - e->last_seen_us > t->timeout_us) {
            e->present = false;
            if (n < cap) {
                lost[n++] = e->node_id;
            }
        }
    }
    return n;
}

const lbr_node_entry_t *lbr_node_table_find(const lbr_node_table_t *t, uint8_t node_id)
{
    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE; i++) {
        if (t->nodes[i].node_id == node_id && node_id != 0) {
            return &t->nodes[i];
        }
    }
    return NULL;
}

size_t lbr_node_table_missing(lbr_node_table_t *t, uint64_t since_us, uint64_t now_us,
                              uint8_t *missing, size_t cap)
{
    if (now_us < since_us || now_us - since_us <= t->timeout_us) {
        return 0;
    }
    size_t n = 0;
    for (size_t i = 0; i < lbr_node_count() && n < cap; i++) {
        uint8_t id = lbr_node_id_at(i);
        if (!lbr_node_is_can(id) || id == LBR_NODE_CENTRAL || id >= 32u) {
            continue;
        }
        uint32_t bit = 1u << id;
        if ((t->missing & bit) != 0 || lbr_node_table_find(t, id) != NULL) {
            continue;
        }
        t->missing |= bit;
        missing[n++] = id;
    }
    return n;
}

bool lbr_node_table_take_missing(lbr_node_table_t *t, uint8_t node_id)
{
    if (node_id >= 32u || (t->missing & (1u << node_id)) == 0) {
        return false;
    }
    t->missing &= ~(1u << node_id);
    return true;
}
