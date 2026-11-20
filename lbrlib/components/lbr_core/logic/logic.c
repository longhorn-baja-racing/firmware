#include "lbr/logic.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/bytes.h"
#include "lbr/frames.h"
#include "lbr/nodes.h"

/* Pure car-logic runner; see lbr/logic.h. */

struct lbr_cycle {
    lbr_logic_t            *l;
    const lbr_node_table_t *nodes;
    uint64_t                now_us;
    float                   dt_s;
};

/* A value older than this many of its frame periods reads STALE. */
#define STALE_PERIODS 3u
/* Signals in no data frame (events) go stale after this long. */
#define STALE_DEFAULT_US 3000000u

static int fail(char *err, size_t len, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

static int fail(char *err, size_t len, const char *fmt, ...)
{
    if (err != NULL && len > 0) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, len, fmt, ap);
        va_end(ap);
    }
    return -1;
}

int lbr_logic_check(const lbr_task *const *tasks, size_t n_tasks, char *err, size_t err_len)
{
    if (n_tasks > LBR_LOGIC_MAX_TASKS) {
        return fail(err, err_len, "%u tasks, at most %u", (unsigned)n_tasks, LBR_LOGIC_MAX_TASKS);
    }
    if (n_tasks > 0 && tasks == NULL) {
        return fail(err, err_len, "no task list");
    }
    for (size_t i = 0; i < n_tasks; i++) {
        const lbr_task *t = tasks[i];
        if (t == NULL || t->name == NULL || t->periodic == NULL) {
            return fail(err, err_len, "task %u has no name or no periodic function", (unsigned)i);
        }
        if (t->rate_hz == 0 || t->rate_hz > LBR_LOGIC_TICK_HZ ||
            LBR_LOGIC_TICK_HZ % t->rate_hz != 0) {
            return fail(err, err_len, "task %s: rate_hz %u must divide %u", t->name,
                        (unsigned)t->rate_hz, LBR_LOGIC_TICK_HZ);
        }
        for (size_t j = 0; j < i; j++) {
            if (strcmp(tasks[j]->name, t->name) == 0) {
                return fail(err, err_len, "two tasks named %s", t->name);
            }
        }
    }
    return 0;
}

void lbr_logic_init(lbr_logic_t *l, const lbr_task *const *tasks, size_t n_tasks,
                    lbr_logic_sink_t sink)
{
    memset(l, 0, sizeof *l);
    l->sink    = sink;
    l->valid   = lbr_logic_check(tasks, n_tasks, NULL, 0) == 0;
    l->tasks   = l->valid ? tasks : NULL;
    l->n_tasks = l->valid ? n_tasks : 0;
}

void lbr_logic_offer(lbr_logic_t *l, const lbr_sample_t *s, uint64_t rx_us)
{
    size_t i = lbr_signal_index(s->signal);
    if (i >= LBR_SIGNAL_COUNT) {
        return; /* not in this catalog (the decoder already counted it) */
    }
    l->values[i] = (lbr_logic_value_t){.last = *s, .rx_us = rx_us, .have = true};
}

/* ------------------------------------------------------------------------ */
/* Reading signals                                                           */

static lbr_status node_status(const lbr_node_table_t *nodes, uint8_t node)
{
    if (nodes == NULL || node == LBR_NODE_INVALID || node == LBR_NODE_CENTRAL) {
        return LBR_OK; /* the Central's own data never goes missing from itself */
    }
    const lbr_node_entry_t *e = lbr_node_table_find(nodes, node);
    if (e == NULL) {
        bool missing = node < 32u && (nodes->missing & (1u << node)) != 0;
        return missing ? LBR_NODE_MISSING : LBR_OK;
    }
    return e->present ? LBR_OK : LBR_NODE_LOST;
}

/* The latest sample of `sig`, if usable; otherwise the status that says why. */
static lbr_status lookup(const lbr_cycle *c, lbr_signal_id_t sig, const lbr_sample_t **out,
                         uint32_t *age_ms)
{
    *out     = NULL;
    *age_ms  = 0;
    size_t i = lbr_signal_index(sig);
    if (c == NULL || i >= LBR_SIGNAL_COUNT) {
        return LBR_UNSPECIFIED;
    }
    const lbr_frame_def_t *frame = lbr_frame_for_signal(sig, NULL);
    uint64_t               stale_us =
        frame != NULL ? (uint64_t)STALE_PERIODS * lbr_frame_period_us(frame) : STALE_DEFAULT_US;
    lbr_status               nst = node_status(c->nodes, frame != NULL ? frame->sender : 0);
    const lbr_logic_value_t *v   = &c->l->values[i];
    if (!v->have) {
        return nst != LBR_OK ? nst : LBR_NO_DATA;
    }
    uint64_t age = c->now_us > v->rx_us ? c->now_us - v->rx_us : 0;
    *age_ms      = age / 1000u > UINT32_MAX ? UINT32_MAX : (uint32_t)(age / 1000u);
    if (nst != LBR_OK) {
        return nst;
    }
    if (age > stale_us) {
        return LBR_STALE;
    }
    if (!v->last.valid) {
        return lbr_status_from_reason((lbr_unavail_reason_t)v->last.v.u32);
    }
    *out = &v->last;
    return LBR_OK;
}

static float as_float(const lbr_sample_t *s)
{
    switch (s->type) {
        case LBR_VAL_F32:
            return s->v.f32;
        case LBR_VAL_I32:
            return (float)s->v.i32;
        case LBR_VAL_BOOL:
            return s->v.b ? 1.0f : 0.0f;
        default:
            return (float)s->v.u32;
    }
}

lbr_f32 lbr_signal(lbr_cycle *c, lbr_signal_id_t sig)
{
    const lbr_sample_t *s;
    uint32_t            age;
    lbr_status          st = lookup(c, sig, &s, &age);
    if (st != LBR_OK || s == NULL) {
        lbr_f32 x = lbr_f32_missing(st);
        x.age_ms  = age;
        return x;
    }
    lbr_f32 x = lbr_f32_of(as_float(s));
    x.age_ms  = age;
    if (!isfinite(x.v)) {
        x = lbr_f32_missing(LBR_BAD_MATH); /* a NaN on the bus is not a number */
    }
    return x;
}

lbr_bool lbr_signal_bool(lbr_cycle *c, lbr_signal_id_t sig)
{
    lbr_f32 x = lbr_signal(c, sig);
    return (lbr_bool){.v = lbr_ok(x) && x.v != 0.0f, .st = x.st, .age_ms = x.age_ms};
}

lbr_i32 lbr_signal_i32(lbr_cycle *c, lbr_signal_id_t sig)
{
    const lbr_sample_t *s;
    uint32_t            age;
    lbr_status          st = lookup(c, sig, &s, &age);
    if (st != LBR_OK || s == NULL) {
        return (lbr_i32){.v = 0, .st = st, .age_ms = age};
    }
    if (s->type == LBR_VAL_F32) {
        float f = s->v.f32;
        if (!isfinite(f) || f < -2147483648.0f || f >= 2147483648.0f) {
            return (lbr_i32){.v = 0, .st = LBR_BAD_MATH, .age_ms = age};
        }
        return (lbr_i32){.v = (int32_t)f, .st = LBR_OK, .age_ms = age};
    }
    if (s->type == LBR_VAL_I32 || s->type == LBR_VAL_BOOL) {
        return (lbr_i32){
            .v = s->type == LBR_VAL_BOOL ? (int32_t)s->v.b : s->v.i32, .st = LBR_OK, .age_ms = age};
    }
    if (s->v.u32 > INT32_MAX) {
        return (lbr_i32){.v = 0, .st = LBR_BAD_MATH, .age_ms = age}; /* does not fit */
    }
    return (lbr_i32){.v = (int32_t)s->v.u32, .st = LBR_OK, .age_ms = age};
}

uint64_t lbr_now_us(const lbr_cycle *c)
{
    return c->now_us;
}

float lbr_dt_s(const lbr_cycle *c)
{
    return c->dt_s;
}

/* ------------------------------------------------------------------------ */
/* Outputs                                                                   */

static void emit_key(lbr_logic_t *l, const lbr_output *o)
{
    if (l->sink.record == NULL) {
        return;
    }
    uint8_t rec[4 + LBR_OUTPUT_KEY_MAX];
    size_t  n = strlen(o->key);
    lbr_put_u16(rec, o->id);
    rec[2] = (uint8_t)o->type;
    rec[3] = (uint8_t)o->unit;
    memcpy(rec + 4, o->key, n);
    l->sink.record(l->sink.ctx, LBR_REC_KEY, rec, 4 + n);
}

static bool key_ok(const char *key)
{
    size_t n = key != NULL ? strlen(key) : 0;
    if (n == 0 || n > LBR_OUTPUT_KEY_MAX) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        if (key[i] < 0x21 || key[i] > 0x7E || key[i] == ',' || key[i] == '"') {
            return false; /* printable, no spaces, safe in a CSV cell */
        }
    }
    return true;
}

/* The output's id in this runner, registering it (and announcing its key) on
 * first use; 0 if it cannot be logged. */
static uint16_t output_id(lbr_logic_t *l, lbr_output *o)
{
    if (o->id != 0 && o->id <= l->n_outputs && l->outputs[o->id - 1] == o) {
        return o->id;
    }
    for (size_t i = 0; i < l->n_outputs; i++) {
        if (l->outputs[i] == o) {
            o->id = (uint16_t)(i + 1);
            return o->id;
        }
    }
    if (l->n_outputs >= LBR_LOGIC_MAX_OUTPUTS || !key_ok(o->key) ||
        (o->type != LBR_VAL_F32 && o->type != LBR_VAL_BOOL) || lbr_logic_find(l, o->key) != NULL) {
        return 0; /* a second output with the same key would merge two series */
    }
    l->outputs[l->n_outputs++] = o;
    o->id                      = (uint16_t)l->n_outputs;
    emit_key(l, o);
    return o->id;
}

static void emit_value(lbr_cycle *c, lbr_output *o, lbr_status st, uint32_t bits)
{
    lbr_logic_t *l  = c->l;
    uint16_t     id = output_id(l, o);
    if (id == 0) {
        l->outputs_refused++;
        return;
    }
    o->written = true;
    o->last_us = c->now_us;
    o->last    = st != LBR_OK              ? lbr_f32_missing(st)
                 : o->type == LBR_VAL_BOOL ? lbr_f32_of(bits != 0 ? 1.0f : 0.0f)
                                           : lbr_f32_of(lbr_bits_f32(bits));
    if (l->sink.record == NULL) {
        return;
    }
    uint8_t rec[LBR_VALUE_REC_SIZE];
    lbr_put_u64(rec, c->now_us);
    lbr_put_u16(rec + 8, id);
    rec[10] = (uint8_t)st;
    lbr_put_u32(rec + 11, st == LBR_OK ? bits : 0u);
    l->sink.record(l->sink.ctx, LBR_REC_VALUE, rec, sizeof rec);
}

void lbr_out(lbr_cycle *c, lbr_output *o, lbr_f32 v)
{
    if (c == NULL || o == NULL) {
        return;
    }
    if (o->type == LBR_VAL_BOOL) {
        emit_value(c, o, v.st, lbr_ok(v) && v.v != 0.0f ? 1u : 0u);
    } else {
        emit_value(c, o, v.st, lbr_f32_bits(v.v));
    }
}

void lbr_out_bool(lbr_cycle *c, lbr_output *o, lbr_bool v)
{
    if (c == NULL || o == NULL) {
        return;
    }
    if (o->type == LBR_VAL_BOOL) {
        emit_value(c, o, v.st, v.v ? 1u : 0u);
    } else {
        emit_value(c, o, v.st, lbr_f32_bits(v.v ? 1.0f : 0.0f));
    }
}

const lbr_output *lbr_logic_find(const lbr_logic_t *l, const char *key)
{
    for (size_t i = 0; key != NULL && i < l->n_outputs; i++) {
        if (strcmp(l->outputs[i]->key, key) == 0) {
            return l->outputs[i];
        }
    }
    return NULL;
}

void lbr_logic_announce(lbr_logic_t *l)
{
    for (size_t i = 0; i < l->n_outputs; i++) {
        emit_key(l, l->outputs[i]);
    }
}

/* ------------------------------------------------------------------------ */
/* Running tasks                                                             */

void lbr_logic_step(lbr_logic_t *l, uint64_t now_us, const lbr_node_table_t *nodes)
{
    for (size_t i = 0; i < l->n_tasks; i++) {
        const lbr_task *t      = l->tasks[i];
        uint64_t        period = 1000000u / t->rate_hz;
        if (!l->armed[i]) {
            /* First run on the next grid point of the shared clock (ticks need
             * not land on it: the task runs on the first tick at or after it). */
            l->next_us[i] = (now_us + period - 1u) / period * period;
            l->armed[i]   = true;
        }
        if (now_us < l->next_us[i]) {
            continue;
        }
        lbr_cycle c = {.l = l, .nodes = nodes, .now_us = now_us, .dt_s = (float)period / 1e6f};
        if (!l->started[i]) {
            l->started[i] = true;
            if (t->init != NULL) {
                t->init(&c);
            }
        }
        t->periodic(&c);
        l->runs++;
        l->next_us[i] += period;
        if (now_us >= l->next_us[i]) {
            /* Fell behind: count it and rejoin the grid instead of bursting. */
            l->late++;
            l->next_us[i] = (now_us / period + 1u) * period;
        }
    }
}
