#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/crc32.h"
#include "lbr/frames.h"
#include "lbr/node.h"
#include "lbr/nodes.h"
#include "lbr/version.h"

static int fail(char *err, size_t err_len, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static int fail(char *err, size_t err_len, const char *fmt, ...)
{
    if (err != NULL && err_len > 0) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }
    return -1;
}

static const char *sig_name(lbr_signal_id_t sig)
{
    const lbr_signal_info_t *info = lbr_signal_info(sig);
    return info != NULL ? info->name : "?";
}

static bool slot_has(const lbr_signal_id_t *signals, size_t n, lbr_signal_id_t sig, size_t *ch)
{
    for (size_t i = 0; i < n; i++) {
        if (signals[i] == sig) {
            if (ch != NULL) {
                *ch = i;
            }
            return true;
        }
    }
    return false;
}

size_t lbr_slot_frames(uint8_t node, const lbr_signal_id_t *signals, size_t n_signals,
                       const lbr_frame_def_t **frames, uint8_t (*chan)[LBR_FRAME_MAX_FIELDS],
                       size_t                  cap)
{
    size_t n = 0;
    for (size_t i = 0; i < lbr_frame_count() && n < cap; i++) {
        const lbr_frame_def_t *d = lbr_frame_at(i);
        if (d->sender != node || d->n_fields == 0 ||
            !slot_has(signals, n_signals, d->fields[0].signal, NULL)) {
            continue;
        }
        bool whole = true;
        for (size_t f = 0; f < d->n_fields; f++) {
            size_t ch  = 0;
            whole      = whole && slot_has(signals, n_signals, d->fields[f].signal, &ch);
            chan[n][f] = (uint8_t)ch;
        }
        if (whole) {
            frames[n++] = d;
        }
    }
    return n;
}

/* Checks shared by plans and runtime configurations. */
static int check_slot(uint8_t node, bool can, const char *name, const lbr_signal_id_t *signals,
                      size_t n_signals, uint16_t rate_hz, char *err, size_t err_len)
{
    if (name == NULL || signals == NULL) {
        return fail(err, err_len, "slot incomplete");
    }
    if (n_signals == 0 || n_signals > LBR_SENSOR_MAX_SIGNALS) {
        return fail(err, err_len, "slot %s: %u channels outside 1..%u", name, (unsigned)n_signals,
                    LBR_SENSOR_MAX_SIGNALS);
    }
    if (rate_hz == 0 || rate_hz > LBR_NODE_MAX_RATE_HZ || 1000000u % rate_hz != 0) {
        return fail(err, err_len, "slot %s: rate %u Hz not in 1..%u or not a divisor of 1 MHz",
                    name, (unsigned)rate_hz, LBR_NODE_MAX_RATE_HZ);
    }
    for (size_t c = 0; c < n_signals; c++) {
        lbr_signal_id_t sig = signals[c];
        if (lbr_signal_info(sig) == NULL) {
            return fail(err, err_len, "slot %s: signal 0x%04x not in catalog", name, (unsigned)sig);
        }
        if (sig == LBR_SIG_EVENT || sig == LBR_SIG_FAULT) {
            return fail(err, err_len, "slot %s: EVENT/FAULT are runtime signals", name);
        }
        const lbr_frame_def_t *d = lbr_frame_for_signal(sig, NULL);
        if (d == NULL) {
            return fail(err, err_len, "slot %s: %s travels in no CAN frame", name, sig_name(sig));
        }
        if (d->rate_hz != rate_hz) {
            return fail(err, err_len, "slot %s: %s runs at %u Hz but frame %s is %u Hz", name,
                        sig_name(sig), (unsigned)rate_hz, d->name, (unsigned)d->rate_hz);
        }
        if (can) {
            if (d->sender != node) {
                return fail(err, err_len, "slot %s: %s is in frame %s, which %s sends", name,
                            sig_name(sig), d->name, lbr_node_name(d->sender));
            }
            for (size_t f = 0; f < d->n_fields; f++) {
                if (!slot_has(signals, n_signals, d->fields[f].signal, NULL)) {
                    return fail(err, err_len, "slot %s: frame %s also needs %s", name, d->name,
                                sig_name(d->fields[f].signal));
                }
            }
        }
    }
    return 0;
}

/* No signal may be produced twice by one node (stream identity). */
static int check_unique(const lbr_signal_id_t *a, size_t na, const lbr_signal_id_t *b, size_t nb,
                        bool same, char *err, size_t err_len)
{
    for (size_t i = 0; i < na; i++) {
        for (size_t j = same ? i + 1 : 0; j < nb; j++) {
            if (a[i] == b[j]) {
                return fail(err, err_len, "signal %s produced twice", sig_name(a[i]));
            }
        }
    }
    return 0;
}

int lbr_plan_check(const lbr_node_plan_t *plan, bool can, char *err, size_t err_len)
{
    if (plan == NULL) {
        return fail(err, err_len, "no plan");
    }
    if (plan->node_id == LBR_NODE_INVALID || plan->node_id == LBR_NODE_RESERVED ||
        lbr_node_is_can(plan->node_id) != can) {
        return fail(err, err_len, "node id 0x%02x not allowed for a %s node", plan->node_id,
                    can ? "CAN" : "wireless");
    }
    if (plan->n_slots > LBR_NODE_MAX_SENSORS || (plan->n_slots > 0 && plan->slots == NULL)) {
        return fail(err, err_len, "too many slots (%u > %u)", (unsigned)plan->n_slots,
                    LBR_NODE_MAX_SENSORS);
    }
    for (size_t i = 0; i < plan->n_slots; i++) {
        const lbr_slot_t *s = &plan->slots[i];
        if (check_slot(plan->node_id, can, s->name, s->signals, s->n_signals, s->rate_hz, err,
                       err_len) != 0) {
            return -1;
        }
        for (size_t j = 0; j <= i; j++) {
            const lbr_slot_t *o = &plan->slots[j];
            if (check_unique(s->signals, s->n_signals, o->signals, o->n_signals, i == j, err,
                             err_len) != 0) {
                return -1;
            }
        }
    }
    return 0;
}

uint32_t lbr_node_can_load_bps(const lbr_node_config_t *cfg)
{
    uint64_t bits = 0;
    for (size_t i = 0; i < lbr_frame_count(); i++) {
        const lbr_frame_def_t *d = lbr_frame_at(i);
        if (d->sender == cfg->node_id) {
            bits += (uint64_t)d->rate_hz * lbr_can_frame_bits_worst(d->dlc);
        }
    }
    if (cfg->health_period_ms > 0) {
        uint64_t per_period = lbr_can_frame_bits_worst(2) +
                              (uint64_t)LBR_HEALTH_PAGE_COUNT * lbr_can_frame_bits_worst(8);
        bits += (per_period * 1000u + cfg->health_period_ms - 1u) / cfg->health_period_ms;
    }
    return (uint32_t)bits;
}

int lbr_node_config_validate(const lbr_node_config_t *cfg, char *err, size_t err_len)
{
    if (cfg == NULL) {
        return fail(err, err_len, "no configuration");
    }
    if (!lbr_node_is_can(cfg->node_id) || cfg->node_id == LBR_NODE_CENTRAL) {
        return fail(err, err_len, "node id 0x%02x not allowed for a sensor node", cfg->node_id);
    }
    if (cfg->n_acq > LBR_NODE_MAX_SENSORS || (cfg->n_acq > 0 && cfg->acq == NULL)) {
        return fail(err, err_len, "too many sensors (%u > %u)", (unsigned)cfg->n_acq,
                    LBR_NODE_MAX_SENSORS);
    }
    if (cfg->health_period_ms < 100 || cfg->health_period_ms > 10000) {
        return fail(err, err_len, "health period %u ms outside 100..10000",
                    (unsigned)cfg->health_period_ms);
    }
    if (cfg->listen_all && cfg->on_frame == NULL) {
        return fail(err, err_len, "listen_all set without an on_frame handler");
    }
    for (size_t i = 0; i < cfg->n_acq; i++) {
        const lbr_acq_t    *a = &cfg->acq[i];
        const lbr_sensor_t *s = a->sensor;
        if (s == NULL || s->init == NULL || s->read == NULL) {
            return fail(err, err_len, "sensor %u incomplete", (unsigned)i);
        }
        if (check_slot(cfg->node_id, true, s->name, s->signals, s->n_signals, a->rate_hz, err,
                       err_len) != 0) {
            return -1;
        }
        for (size_t j = 0; j <= i; j++) {
            const lbr_sensor_t *o = cfg->acq[j].sensor;
            if (check_unique(s->signals, s->n_signals, o->signals, o->n_signals, i == j, err,
                             err_len) != 0) {
                return -1;
            }
        }
    }
    return 0;
}

typedef struct {
    char  *buf;
    size_t cap;
    size_t len;
} text_t;

static void put(text_t *t, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void put(text_t *t, const char *fmt, ...)
{
    if (t->len >= t->cap) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(t->buf + t->len, t->cap - t->len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        t->len += (size_t)n;
        if (t->len >= t->cap) {
            t->len = t->cap - 1;
        }
    }
}

size_t lbr_node_config_text(const lbr_node_config_t *cfg, char *buf, size_t cap)
{
    if (cap == 0) {
        return 0;
    }
    buf[0]   = '\0';
    text_t t = {buf, cap, 0};
    put(&t, "node=%s\n", lbr_node_name(cfg->node_id));
    put(&t, "node_id=%u\n", (unsigned)cfg->node_id);
    put(&t, "fw_version=%u.%u.%u\n", LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
        LBR_FW_VERSION_PATCH);
    put(&t, "protocol_version=%u\n", LBR_PROTOCOL_VERSION);
    put(&t, "catalog_hash=%08lx\n", (unsigned long)lbr_catalog_hash());
    put(&t, "health_period_ms=%u\n", (unsigned)cfg->health_period_ms);
    put(&t, "local_log=%u\n", cfg->local_log ? 1u : 0u);
    for (size_t i = 0; i < cfg->n_acq; i++) {
        const lbr_acq_t *a = &cfg->acq[i];
        if (a->sensor == NULL) {
            continue;
        }
        put(&t, "sensor.%u=%s rate_hz=%u sim=%u signals=", (unsigned)i, a->sensor->name,
            (unsigned)a->rate_hz, a->sensor->simulated ? 1u : 0u);
        for (size_t c = 0; c < a->sensor->n_signals; c++) {
            put(&t, "%s%s", c ? "," : "", sig_name(a->sensor->signals[c]));
        }
        put(&t, "\n");
        if (a->sensor->params != NULL && a->sensor->params[0] != '\0') {
            put(&t, "sensor.%u.params=%s\n", (unsigned)i, a->sensor->params);
        }
    }
    return t.len;
}

uint32_t lbr_node_config_crc(const lbr_node_config_t *cfg)
{
    char   text[LBR_NODE_CONFIG_TEXT_MAX];
    size_t n = lbr_node_config_text(cfg, text, sizeof text);
    return lbr_crc32(text, n);
}

void lbr_reading_to_sample(uint8_t source, lbr_signal_id_t signal, const lbr_reading_t *r,
                           uint64_t k, uint32_t period_us, bool synced, lbr_sample_t *s)
{
    const lbr_signal_info_t *info = lbr_signal_info(signal);

    memset(s, 0, sizeof *s);
    s->signal    = signal;
    s->source    = source;
    s->type      = info ? info->type : LBR_VAL_RAW;
    s->synced    = synced;
    s->seq       = (uint32_t)k;
    s->t_mono_us = k * period_us;
    s->valid     = r->valid;
    if (!r->valid) {
        lbr_sample_set_unavailable(s, r->reason);
        return;
    }
    switch (s->type) {
        case LBR_VAL_F32:
            s->v.f32 = r->v.f32;
            break;
        case LBR_VAL_I32:
            s->v.i32 = r->v.i32;
            break;
        case LBR_VAL_BOOL:
            s->v.b = r->v.b;
            break;
        default:
            s->v.u32 = r->v.u32;
            break;
    }
}

void lbr_node_build(const lbr_node_plan_t *plan, lbr_sensor_fill_t unbound, lbr_sensor_t *sensors,
                    lbr_acq_t *acq, lbr_node_config_t *cfg)
{
    for (size_t i = 0; i < plan->n_slots; i++) {
        const lbr_slot_t *slot = &plan->slots[i];
        unbound(&sensors[i], slot->name, slot->signals, slot->n_signals);
        acq[i] = (lbr_acq_t){
            .sensor  = &sensors[i],
            .rate_hz = slot->rate_hz,
        };
    }
    *cfg = (lbr_node_config_t){
        .node_id          = plan->node_id,
        .acq              = acq,
        .n_acq            = plan->n_slots,
        .health_period_ms = plan->health_period_ms,
        .local_log        = plan->local_log,
        .listen_all       = plan->listen_all,
    };
}
