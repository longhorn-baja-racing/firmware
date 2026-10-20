#include "lbr/corner_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"

/* Pure corner-node logic; see lbr/corner_core.h. No platform headers here. */

static void say(lbr_corner_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void say(lbr_corner_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
{
    if (c->hal.log == NULL) {
        return;
    }
    char    line[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    c->hal.log(c->hal.ctx, lvl, line);
}

static void acq_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_corner_core_t *c    = ctx;
    const char        *name = lbr_event_name(code);
    say(c, fault ? LBR_LOG_LVL_ERROR : LBR_LOG_LVL_WARN, "%s %s arg %u", fault ? "FAULT" : "event",
        name != NULL ? name : "?", (unsigned)arg);
}

static bool has_signal(const lbr_sensor_t *s, lbr_signal_id_t sig, size_t *ch)
{
    for (size_t i = 0; i < s->n_signals; i++) {
        if (s->signals[i] == sig) {
            if (ch != NULL) {
                *ch = i;
            }
            return true;
        }
    }
    return false;
}

bool lbr_corner_core_init(lbr_corner_core_t *c, const lbr_node_config_t *cfg,
                          const lbr_corner_hal_t *hal, uint8_t batch)
{
    memset(c, 0, sizeof *c);
    c->cfg      = cfg;
    c->hal      = *hal;
    c->batch    = (batch == 0 || batch > LBR_WLINK_MAX_BATCH) ? LBR_CORNER_DEFAULT_BATCH : batch;
    c->imu_slot = -1;
    c->battery_slot = -1;

    int corner = lbr_node_corner_index(cfg->node_id);
    if (corner < 0 || cfg->n_acq > LBR_NODE_MAX_SENSORS) {
        say(c, LBR_LOG_LVL_ERROR, "node 0x%02x is not a corner node", cfg->node_id);
        return false;
    }
    c->n_frames = lbr_wlink_origin_frames(cfg->node_id, c->frames, 4, &c->sample_size);
    if (c->n_frames == 0 || c->sample_size > LBR_WLINK_MAX_SAMPLE) {
        say(c, LBR_LOG_LVL_ERROR, "no IMU frames for %s", lbr_node_name(cfg->node_id));
        return false;
    }
    lbr_signal_id_t        battery = (lbr_signal_id_t)(LBR_SIG_CORNER_FL_BATTERY + corner);
    size_t                 bf      = 0;
    const lbr_frame_def_t *status  = lbr_frame_for_signal(battery, &bf);
    if (status != NULL) {
        c->battery_field = status->fields[bf];
    }
    lbr_sample_t none = {.type = LBR_VAL_F32};
    lbr_sample_set_unavailable(&none, LBR_UNAVAIL_NOT_FITTED);
    uint8_t tmp[8]          = {0};
    c->battery_field.offset = 0;
    lbr_can_field_encode(&c->battery_field, &none, tmp);
    c->battery_raw = tmp[0];

    for (size_t i = 0; i < cfg->n_acq; i++) {
        lbr_sensor_t *s = cfg->acq[i].sensor;
        c->sim          = c->sim || s->simulated;
        lbr_acq_slot_init(&c->slot[i], s, cfg->acq[i].rate_hz, (uint16_t)i);
        if (has_signal(s, c->frames[0]->fields[0].signal, NULL)) {
            c->imu_slot = (int)i;
        }
        if (has_signal(s, battery, NULL)) {
            c->battery_slot = (int)i;
        }
    }
    if (c->imu_slot < 0) {
        say(c, LBR_LOG_LVL_ERROR, "no IMU slot");
        return false;
    }
    lbr_sensor_t *imu = cfg->acq[c->imu_slot].sensor;
    for (size_t f = 0; f < c->n_frames; f++) {
        for (size_t k = 0; k < c->frames[f]->n_fields; k++) {
            size_t ch = 0;
            if (!has_signal(imu, c->frames[f]->fields[k].signal, &ch)) {
                say(c, LBR_LOG_LVL_ERROR, "IMU slot lacks a field of %s", c->frames[f]->name);
                return false;
            }
            c->chan[f][k] = (uint8_t)ch;
        }
    }
    say(c, LBR_LOG_LVL_INFO, "%s: %u IMU frames, %u bytes per sample, batch %u%s",
        lbr_node_name(cfg->node_id), (unsigned)c->n_frames, (unsigned)c->sample_size,
        (unsigned)c->batch, c->sim ? " SIMULATED SENSORS" : "");
    return true;
}

void lbr_corner_core_slot_start(lbr_corner_core_t *c, size_t i)
{
    lbr_acq_slot_start(&c->slot[i], c->hal.mono_us(c->hal.ctx), acq_event, c);
}

uint64_t lbr_corner_core_slot_next_due(lbr_corner_core_t *c, size_t i)
{
    return lbr_acq_next_due(c->hal.now_us(c->hal.ctx), c->slot[i].period_us);
}

uint8_t lbr_corner_core_flags(lbr_corner_core_t *c)
{
    lbr_time_servo_t ts;
    c->hal.time_servo(c->hal.ctx, &ts);
    uint8_t f = 0;
    f |= (ts.synced && c->clock.server_synced) ? LBR_WLINK_F_SYNCED : 0u;
    if (c->imu_slot >= 0 && atomic_load(&c->slot[c->imu_slot].healthy) &&
        atomic_load(&c->slot[c->imu_slot].calibrated)) {
        f |= LBR_WLINK_F_SENSOR_OK;
    }
    f |= c->sim ? LBR_WLINK_F_SIM : 0u;
    f |= (uint8_t)((c->tl_seq & LBR_WLINK_F_TL_MASK) << LBR_WLINK_F_TL_SHIFT);
    return f;
}

static void add_batch(lbr_corner_core_t *c, lbr_pkt_builder_t *b, uint32_t k0, uint8_t n,
                      const uint8_t *samples, uint8_t flags)
{
    uint8_t         rec[LBR_WLINK_BATCH_HDR + sizeof c->cur];
    lbr_imu_batch_t batch = {
        .k0          = k0,
        .n           = n,
        .battery_raw = c->battery_raw,
        .flags       = flags,
        .sample_size = (uint8_t)c->sample_size,
        .samples     = samples,
    };
    size_t len = lbr_imu_batch_encode(&batch, rec, sizeof rec);
    if (len > 0) {
        lbr_pkt_add(b, LBR_REC_IMU_BATCH, rec, len);
    }
}

/* Send the current batch, with the previous one again ahead of it. */
static void flush(lbr_corner_core_t *c)
{
    if (c->cur_n == 0) {
        return;
    }
    uint8_t           pkt[LBR_PKT_MAX_SIZE];
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_CORNER, c->cfg->node_id,
                  atomic_fetch_add(&c->pkt_seq, 1), c->hal.now_us(c->hal.ctx));
    if (c->prev_n > 0) {
        add_batch(c, &b, c->prev_k0, c->prev_n, c->prev, c->prev_flags);
    }
    add_batch(c, &b, c->cur_k0, c->cur_n, c->cur, c->cur_flags);
    size_t n = lbr_pkt_finish(&b);
    if (n > 0 && c->hal.send(c->hal.ctx, pkt, n)) {
        atomic_fetch_add(&c->packets_sent, 1);
    } else {
        atomic_fetch_add(&c->packets_dropped, 1);
    }
    memcpy(c->prev, c->cur, (size_t)c->cur_n * c->sample_size);
    c->prev_k0    = c->cur_k0;
    c->prev_n     = c->cur_n;
    c->prev_flags = c->cur_flags;
    c->cur_n      = 0;
}

void lbr_corner_core_slot_tick(lbr_corner_core_t *c, size_t i)
{
    lbr_acq_slot_t  *a = &c->slot[i];
    lbr_reading_t    readings[LBR_SENSOR_MAX_SIGNALS];
    lbr_acq_tick_t   tick;
    lbr_time_servo_t ts;

    c->hal.time_servo(c->hal.ctx, &ts);
    uint32_t tl = lbr_acq_timeline(ts.synced, ts.step_count, ts.epoch_changes);
    if (!lbr_acq_slot_tick(a, c->hal.now_us(c->hal.ctx), c->hal.mono_us(c->hal.ctx), tl, readings,
                           &tick, acq_event, c)) {
        return;
    }
    if (tick.missed > 0) {
        atomic_fetch_add(&c->overruns, tick.missed);
    }
    lbr_sample_t samples[LBR_SENSOR_MAX_SIGNALS] = {0};
    for (size_t ch = 0; ch < a->sensor->n_signals; ch++) {
        lbr_reading_to_sample(c->cfg->node_id, a->sensor->signals[ch], &readings[ch], tick.k,
                              a->period_us, ts.synced, &samples[ch]);
    }
    atomic_fetch_add(&c->acquired, 1);

    if ((int)i == c->battery_slot) {
        for (size_t ch = 0; ch < a->sensor->n_signals; ch++) {
            if (samples[ch].signal == c->battery_field.signal) {
                uint8_t tmp[8] = {0};
                if (!lbr_can_field_encode(&c->battery_field, &samples[ch], tmp)) {
                    atomic_fetch_add(&c->out_of_range, 1);
                }
                c->battery_raw = tmp[0];
            }
        }
    }
    if ((int)i != c->imu_slot) {
        return;
    }

    /* Every jump of this clock is a new timeline the IMU Controller must
     * see, even when the synced flag does not change (step, new epoch). */
    if (c->have_timeline && tl != c->last_timeline) {
        c->tl_seq++;
    }
    c->have_timeline = true;
    c->last_timeline = tl;

    /* A batch holds contiguous grid points taken in one state only. */
    uint8_t flags = lbr_corner_core_flags(c);
    if (c->cur_n > 0 && ((uint32_t)tick.k != c->cur_k0 + c->cur_n || flags != c->cur_flags)) {
        flush(c);
    }
    if (c->cur_n == 0) {
        c->cur_k0    = (uint32_t)tick.k;
        c->cur_flags = flags;
    }
    uint8_t *dst = c->cur + (size_t)c->cur_n * c->sample_size;
    for (size_t f = 0; f < c->n_frames; f++) {
        const lbr_frame_def_t *d = c->frames[f];
        lbr_sample_t           values[LBR_FRAME_MAX_FIELDS];
        lbr_can_frame_t        frame;
        for (size_t k = 0; k < d->n_fields; k++) {
            values[k] = samples[c->chan[f][k]];
        }
        unsigned misfit = lbr_can_encode_data(d, values, tick.k, &frame);
        if (misfit > 0) {
            atomic_fetch_add(&c->out_of_range, misfit);
        }
        dst += lbr_wlink_strip_counter(d, &frame, dst);
    }
    if (++c->cur_n >= c->batch) {
        flush(c);
    }
}

uint32_t lbr_corner_core_clock_tick(lbr_corner_core_t *c)
{
    uint8_t           pkt[64];
    uint8_t           req[LBR_WLINK_TIME_REQ];
    lbr_pkt_builder_t b;
    lbr_time_servo_t  ts;

    lbr_wclock_request(&c->clock, c->hal.mono_us(c->hal.ctx), req);
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_CORNER, c->cfg->node_id,
                  atomic_fetch_add(&c->pkt_seq, 1), c->hal.now_us(c->hal.ctx));
    lbr_pkt_add(&b, LBR_REC_TIME_REQ, req, sizeof req);
    size_t n = lbr_pkt_finish(&b);
    if (n > 0 && c->hal.send(c->hal.ctx, pkt, n)) {
        atomic_fetch_add(&c->packets_sent, 1);
    } else {
        atomic_fetch_add(&c->packets_dropped, 1);
    }
    c->hal.time_servo(c->hal.ctx, &ts);
    return (ts.synced && c->clock.server_synced) ? LBR_CORNER_SYNC_PERIOD_MS
                                                 : LBR_CORNER_SYNC_FAST_MS;
}

typedef struct {
    lbr_corner_core_t *c;
    uint64_t           rx_local;
} rx_ctx_t;

static void on_record(uint8_t type, const uint8_t *payload, size_t len, void *ctx)
{
    rx_ctx_t          *r = ctx;
    lbr_corner_core_t *c = r->c;
    uint64_t           shared;
    uint16_t           epoch;
    bool               server_synced;
    if (type != LBR_REC_TIME_RESP || !lbr_wclock_on_response(&c->clock, payload, len, r->rx_local,
                                                             &shared, &epoch, &server_synced)) {
        return;
    }
    lbr_time_update_t u = c->hal.time_apply(c->hal.ctx, shared, r->rx_local, epoch);
    /* Only now: a sample flagged synced must already be on the new timeline. */
    c->clock.server_synced = server_synced;
    if (u == LBR_TIME_UPD_FIRST || u == LBR_TIME_UPD_EPOCH_CHANGE) {
        say(c, LBR_LOG_LVL_INFO, "clock synced to the IMU Controller (epoch %u, rtt %lu us)",
            (unsigned)epoch, (unsigned long)c->clock.last_rtt_us);
    } else if (u == LBR_TIME_UPD_STEP) {
        say(c, LBR_LOG_LVL_WARN, "clock stepped");
    }
}

void lbr_corner_core_on_packet(lbr_corner_core_t *c, const uint8_t *buf, size_t len,
                               uint64_t rx_local_us)
{
    lbr_pkt_header_t hdr;
    rx_ctx_t         r = {c, rx_local_us};
    if (!lbr_pkt_parse(buf, len, &hdr, NULL, NULL) || hdr.source != LBR_NODE_IMU) {
        atomic_fetch_add(&c->rejected, 1);
        return;
    }
    lbr_pkt_parse(buf, len, &hdr, on_record, &r);
}
