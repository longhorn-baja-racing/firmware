#include "lbr/replay.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/protocol.h"

/* Pure; see lbr/replay.h. The Central's core calls the same two functions
 * (observe_frame, check_nodes), so a replay does exactly what it did. */

size_t lbr_tick_encode(const lbr_tick_t *t, uint8_t *buf, size_t cap)
{
    if (t == NULL || buf == NULL || cap < LBR_TICK_REC_SIZE) {
        return 0;
    }
    lbr_put_u64(buf, t->t_us);
    buf[8] = (uint8_t)t->kind;
    buf[9] = t->can_up ? 1u : 0u;
    lbr_put_u16(buf + 10, t->node_timeout_ms);
    return LBR_TICK_REC_SIZE;
}

bool lbr_tick_decode(const uint8_t *buf, size_t len, lbr_tick_t *out)
{
    if (buf == NULL || out == NULL || len < LBR_TICK_REC_SIZE || buf[8] > LBR_TICK_HEARTBEAT) {
        return false;
    }
    *out = (lbr_tick_t){
        .t_us            = lbr_get_u64(buf),
        .kind            = (lbr_tick_kind_t)buf[8],
        .can_up          = buf[9] != 0,
        .node_timeout_ms = lbr_get_u16(buf + 10),
    };
    return true;
}

size_t lbr_replay_observe_frame(lbr_bus_decoder_t *dec, lbr_node_table_t *nodes, lbr_logic_t *logic,
                                const lbr_can_frame_t *f, uint64_t t_us, bool own_tx,
                                lbr_sample_t *out, size_t cap, bool *found, bool *back)
{
    size_t                 n  = lbr_bus_decode(dec, f, t_us, out, cap);
    lbr_node_table_event_t ev = own_tx ? LBR_NT_NONE : lbr_node_table_on_frame(nodes, f, t_us);
    *found                    = ev == LBR_NT_FOUND;
    *back                     = false;
    if (*found) {
        uint8_t                 source = lbr_can_sender(f->id);
        const lbr_node_entry_t *e      = lbr_node_table_find(nodes, source);
        *back = e != NULL && e->first_seen_us != t_us; /* after NODE_LOST */
        *back = lbr_node_table_take_missing(nodes, source) || *back;
    }
    for (size_t i = 0; logic != NULL && i < n; i++) {
        lbr_logic_offer(logic, &out[i], t_us);
    }
    return n;
}

void lbr_replay_check_nodes(lbr_node_table_t *nodes, uint64_t now_us, bool can_up,
                            uint64_t listen_since_us, uint8_t *lost, size_t *n_lost,
                            uint8_t *missing, size_t *n_missing, size_t cap)
{
    *n_lost = lbr_node_table_expire(nodes, now_us, lost, cap);
    /* Expected topology: a catalog node never heard is a fault too. */
    *n_missing = can_up ? lbr_node_table_missing(nodes, listen_since_us, now_us, missing, cap) : 0;
}

void lbr_replay_init(lbr_replay_t *r, const lbr_task *const *tasks, size_t n_tasks,
                     lbr_logic_sink_t sink)
{
    memset(r, 0, sizeof *r);
    lbr_bus_decoder_init(&r->decoder);
    lbr_node_table_init(&r->nodes, 3000000u); /* until START says otherwise */
    lbr_logic_init(&r->logic, tasks, n_tasks, sink);
}

void lbr_replay_record(lbr_replay_t *r, uint8_t type, const uint8_t *payload, size_t len)
{
    if (type == LBR_REC_CAN_FRAME) {
        uint64_t        t_us;
        uint8_t         flags;
        lbr_can_frame_t f;
        if (lbr_can_frame_rec_decode(payload, len, &t_us, &flags, &f) == 0) {
            r->bad_records++;
            return;
        }
        lbr_sample_t s[LBR_FRAME_MAX_FIELDS];
        bool         found, back;
        lbr_replay_observe_frame(&r->decoder, &r->nodes, &r->logic, &f, t_us,
                                 (flags & LBR_CAN_REC_FLAG_OWN_TX) != 0, s, LBR_FRAME_MAX_FIELDS,
                                 &found, &back);
        r->frames++;
        return;
    }
    if (type != LBR_REC_TICK) {
        return;
    }
    lbr_tick_t t;
    if (!lbr_tick_decode(payload, len, &t)) {
        r->bad_records++;
        return;
    }
    switch (t.kind) {
        case LBR_TICK_START:
            r->started          = true;
            r->can_up           = t.can_up;
            r->listen_since_us  = t.t_us;
            r->nodes.timeout_us = (uint64_t)t.node_timeout_ms * 1000u;
            break;
        case LBR_TICK_LOGIC:
            lbr_logic_step(&r->logic, t.t_us, &r->nodes);
            r->logic_ticks++;
            break;
        case LBR_TICK_HEARTBEAT: {
            uint8_t lost[LBR_NODE_TABLE_SIZE], missing[LBR_NODE_TABLE_SIZE];
            size_t  n_lost, n_missing;
            lbr_replay_check_nodes(&r->nodes, t.t_us, r->can_up, r->listen_since_us, lost, &n_lost,
                                   missing, &n_missing, LBR_NODE_TABLE_SIZE);
            r->heartbeats++;
            break;
        }
    }
}
