#include "lbr/status.h"

#include <stdarg.h>
#include <stdio.h>

#include "lbr/nodes.h"

const char *lbr_node_state_name(uint8_t state)
{
    switch (state) {
        case LBR_NODE_BOOT:
            return "BOOT";
        case LBR_NODE_RUN:
            return "RUN";
        case LBR_NODE_DEGRADED:
            return "DEGRADED";
        case LBR_NODE_CONFIG_INVALID:
            return "BADCONFIG";
        case LBR_NODE_FAULT:
            return "FAULT";
        default:
            return "?";
    }
}

typedef struct {
    char  *buf;
    size_t cap;
    size_t len;
} out_t;

static void put(out_t *o, const char *fmt, ...)
{
    if (o->len + 1 >= o->cap) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(o->buf + o->len, o->cap - o->len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        o->len += (size_t)n;
        if (o->len >= o->cap) {
            o->len = o->cap - 1;
        }
    }
}

static void put_common(out_t *o, const lbr_health_record_t *h)
{
    put(o, " %s", lbr_node_state_name(h->state));
    put(o, "%s", (h->flags & LBR_HF_TIME_SYNCED) ? " sync" : " nosync");
    if (h->flags & LBR_HF_SD_PRESENT) {
        put(o, " sd");
    }
    put(o, " up %lus", (unsigned long)(h->uptime_ms / 1000u));
}

size_t lbr_status_format(const lbr_health_record_t *gw, const char *log_name,
                         const lbr_node_table_t *nodes, uint64_t now_us, char *buf, size_t cap)
{
    out_t o = {buf, cap, 0};
    if (cap == 0) {
        return 0;
    }
    buf[0] = '\0';

    put(&o, "gw");
    put_common(&o, gw);
    put(&o, " log=%s", log_name && log_name[0] ? log_name : "none");
    put(&o, " can tec%u boff%lu\n", gw->can_tec, (unsigned long)gw->can_bus_off_count);

    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE; i++) {
        const lbr_node_entry_t *e = &nodes->nodes[i];
        if (e->node_id == 0) {
            continue;
        }
        const char *name = lbr_node_name(e->node_id);
        if (name[0] == '?') {
            put(&o, "node_%02x", e->node_id);
        } else {
            put(&o, "%s", name);
        }
        if (!e->present) {
            unsigned long ago = now_us > e->last_seen_us
                                    ? (unsigned long)((now_us - e->last_seen_us) / 1000000u)
                                    : 0;
            put(&o, " LOST %lus\n", ago);
            continue;
        }
        const lbr_health_record_t *h = &e->health;
        if (e->pages_seen == 0) {
            put(&o, " up (no heartbeat yet)\n");
            continue;
        }
        put_common(&o, h);
        put(&o, " sens %lx cal %lx fw %u.%u.%u", (unsigned long)h->sensor_status,
            (unsigned long)h->calibration_state, (unsigned)((h->fw_version >> 16) & 0xFF),
            (unsigned)((h->fw_version >> 8) & 0xFF), (unsigned)(h->fw_version & 0xFF));
        if (h->flags & LBR_HF_SIM_SENSORS) {
            put(&o, " SIM");
        }
        put(&o, "\n");
    }
    return o.len;
}
