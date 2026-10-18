#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/crc32.h"
#include "lbr/node.h"
#include "lbr/nodes.h"
#include "lbr/version.h"

#define FRAME_BITS lbr_can_frame_bits_worst(8)

uint32_t lbr_node_can_load_bps(const lbr_node_config_t *cfg)
{
    uint64_t frames_per_s_x1000 = 0;
    for (size_t i = 0; i < cfg->n_acq; i++) {
        const lbr_acq_t *a = &cfg->acq[i];
        if (a->sensor == NULL || a->publish_div == 0) {
            continue;
        }
        /* Round the decimated rate up: worst case. */
        uint64_t pub_rate_x1000 =
            ((uint64_t)a->rate_hz * 1000u + a->publish_div - 1u) / a->publish_div;
        frames_per_s_x1000 += pub_rate_x1000 * a->sensor->n_signals;
    }
    if (cfg->health_period_ms > 0) {
        frames_per_s_x1000 += (uint64_t)LBR_HEALTH_PAGE_COUNT * 1000000u / cfg->health_period_ms;
    }
    return (uint32_t)((frames_per_s_x1000 * FRAME_BITS + 999u) / 1000u);
}

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

int lbr_node_config_validate(const lbr_node_config_t *cfg, uint32_t bitrate,
                             uint32_t budget_permille, char *err, size_t err_len)
{
    if (cfg == NULL) {
        return fail(err, err_len, "no configuration");
    }
    if (cfg->node_id == LBR_NODE_INVALID || cfg->node_id == LBR_NODE_RESERVED ||
        cfg->node_id == LBR_NODE_GATEWAY) {
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
        if (s == NULL || s->init == NULL || s->read == NULL || s->name == NULL ||
            s->signals == NULL) {
            return fail(err, err_len, "sensor %u incomplete", (unsigned)i);
        }
        if (s->n_signals == 0 || s->n_signals > LBR_SENSOR_MAX_SIGNALS) {
            return fail(err, err_len, "sensor %s: %u channels outside 1..%u", s->name,
                        (unsigned)s->n_signals, LBR_SENSOR_MAX_SIGNALS);
        }
        if (a->rate_hz == 0 || a->rate_hz > LBR_NODE_MAX_RATE_HZ) {
            return fail(err, err_len, "sensor %s: rate %u Hz outside 1..%u", s->name,
                        (unsigned)a->rate_hz, LBR_NODE_MAX_RATE_HZ);
        }
        if (a->publish_div == 0) {
            return fail(err, err_len, "sensor %s: publish_div is 0", s->name);
        }
        if (a->publish_div > 1 && !cfg->local_log) {
            /* Decimating without a local log would silently discard samples. */
            return fail(err, err_len, "sensor %s: publish_div %u needs local_log", s->name,
                        (unsigned)a->publish_div);
        }
        for (size_t c = 0; c < s->n_signals; c++) {
            lbr_signal_id_t sig = s->signals[c];
            if (lbr_signal_info(sig) == NULL) {
                return fail(err, err_len, "sensor %s: signal 0x%04x not in catalog", s->name,
                            (unsigned)sig);
            }
            if (sig == LBR_SIG_EVENT || sig == LBR_SIG_FAULT) {
                return fail(err, err_len, "sensor %s: EVENT/FAULT are runtime signals", s->name);
            }
            /* No signal may be produced twice by one node (stream identity). */
            for (size_t j = 0; j <= i; j++) {
                const lbr_sensor_t *o   = cfg->acq[j].sensor;
                size_t              lim = (j == i) ? c : o->n_signals;
                for (size_t k = 0; k < lim; k++) {
                    if (o->signals[k] == sig) {
                        return fail(err, err_len, "signal %s produced twice",
                                    lbr_signal_info(sig)->name);
                    }
                }
            }
        }
    }

    uint64_t load   = lbr_node_can_load_bps(cfg);
    uint64_t budget = (uint64_t)bitrate * budget_permille / 1000u;
    if (load > budget) {
        return fail(err, err_len, "CAN load %lu bit/s exceeds node budget %lu bit/s",
                    (unsigned long)load, (unsigned long)budget);
    }
    return 0;
}

typedef struct {
    char  *buf;
    size_t cap;
    size_t len;
} text_t;

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
        put(&t, "sensor.%u=%s rate_hz=%u publish_div=%u sim=%u signals=", (unsigned)i,
            a->sensor->name, (unsigned)a->rate_hz, (unsigned)a->publish_div,
            a->sensor->simulated ? 1u : 0u);
        for (size_t c = 0; c < a->sensor->n_signals; c++) {
            const lbr_signal_info_t *info = lbr_signal_info(a->sensor->signals[c]);
            put(&t, "%s%s", c ? "," : "", info ? info->name : "?");
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

bool lbr_stream_sample(lbr_stream_t *st, uint16_t publish_div, uint8_t source,
                       lbr_signal_id_t signal, const lbr_reading_t *r, uint64_t t_us, bool synced,
                       lbr_sample_t *s, uint32_t *pub_seq)
{
    const lbr_signal_info_t *info = lbr_signal_info(signal);

    memset(s, 0, sizeof *s);
    s->signal    = signal;
    s->source    = source;
    s->type      = info ? info->type : LBR_VAL_RAW;
    s->synced    = synced;
    s->seq       = st->seq++;
    s->t_mono_us = t_us;
    s->valid     = r->valid;
    if (r->valid) {
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
    } else {
        lbr_sample_set_unavailable(s, r->reason);
    }

    if (++st->div_count < (publish_div ? publish_div : 1u)) {
        return false;
    }
    st->div_count = 0;
    *pub_seq      = st->pub_seq++;
    return true;
}

void lbr_node_build(const lbr_node_plan_t *plan, bool sim, lbr_sensor_t *sensors, lbr_acq_t *acq,
                    lbr_node_config_t *cfg)
{
    for (size_t i = 0; i < plan->n_slots; i++) {
        const lbr_slot_t *slot = &plan->slots[i];
        if (sim) {
            lbr_sensor_sim(&sensors[i], slot->name, slot->signals, slot->n_signals);
        } else {
            lbr_sensor_unconfigured(&sensors[i], slot->name, slot->signals, slot->n_signals);
        }
        acq[i] = (lbr_acq_t){
            .sensor      = &sensors[i],
            .rate_hz     = slot->rate_hz,
            .publish_div = slot->publish_div,
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
