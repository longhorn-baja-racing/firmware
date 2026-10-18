#include "lbr/node_table.h"

#include <string.h>

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
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    if (id.source == 0) {
        return LBR_NT_NONE;
    }
    lbr_node_entry_t *e = slot_for(t, id.source);
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
