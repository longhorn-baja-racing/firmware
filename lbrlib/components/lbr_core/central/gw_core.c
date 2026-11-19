#include "gw_internal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/nodes.h"
#include "lbr/status.h"
#include "lbr/timebase.h"
#include "lbr/version.h"

/* Pure Central (gateway) logic; see lbr/gw_core.h. No platform headers here. */

void gw_say(lbr_gw_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
{
    if (c->hal.log == NULL) {
        return;
    }
    char    line[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    c->hal.log(c->hal.ctx, lvl, line);
}

void gw_lock(lbr_gw_core_t *c, lbr_gw_lock_t which)
{
    if (c->hal.lock != NULL) {
        c->hal.lock(c->hal.ctx, which);
    }
}

void gw_unlock(lbr_gw_core_t *c, lbr_gw_lock_t which)
{
    if (c->hal.unlock != NULL) {
        c->hal.unlock(c->hal.ctx, which);
    }
}

/* ------------------------------------------------------------------------ */
/* Frame fan-out                                                             */

static void stream_flush_locked(lbr_gw_core_t *c)
{
    if (!c->stream_open) {
        return;
    }
    size_t n = lbr_pkt_finish(&c->stream);
    if (n > 0) {
        if (c->links.usb != NULL) {
            c->links.usb->send(c->links.usb, c->stream_pkt, n);
        }
        if (c->links.wifi != NULL) {
            c->links.wifi->send(c->links.wifi, c->stream_pkt, n);
        }
    }
    c->stream_open = false;
}

static void stream_add(lbr_gw_core_t *c, uint8_t type, const uint8_t *rec, size_t len)
{
    if (c->links.usb == NULL && c->links.wifi == NULL) {
        return;
    }
    gw_lock(c, LBR_GW_LOCK_STREAM);
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!c->stream_open) {
            lbr_pkt_begin(&c->stream, c->stream_pkt, sizeof c->stream_pkt, LBR_LINK_USB, NODE_ID, 0,
                          c->hal.now_us(c->hal.ctx));
            c->stream_open = true;
        }
        if (lbr_pkt_add(&c->stream, type, rec, len)) {
            break;
        }
        stream_flush_locked(c);
    }
    gw_unlock(c, LBR_GW_LOCK_STREAM);
}

/*
 * Every frame, received or our own, goes through here: authoritative log
 * first, then the live views. `t_us` is the shared time base.
 */
static void handle_frame(lbr_gw_core_t *c, const lbr_can_frame_t *f, uint64_t t_us, bool own_tx)
{
    uint8_t flags = own_tx ? LBR_CAN_REC_FLAG_OWN_TX : 0u;
    uint8_t rec[LBR_CAN_FRAME_REC_SIZE];
    lbr_can_frame_rec_encode(t_us, flags, f, rec, sizeof rec);

    if (c->log != NULL && lbr_log_append_raw(c->log, LBR_REC_CAN_FRAME, rec, sizeof rec) == 0) {
        atomic_fetch_add(&c->frames_logged, 1);
    }
    stream_add(c, LBR_REC_CAN_FRAME, rec, sizeof rec);

    lbr_sample_t samples[LBR_FRAME_MAX_FIELDS];
    gw_lock(c, LBR_GW_LOCK_STATE);
    size_t                 n  = lbr_bus_decode(&c->decoder, f, t_us, samples, LBR_FRAME_MAX_FIELDS);
    lbr_node_table_event_t ev = own_tx ? LBR_NT_NONE : lbr_node_table_on_frame(&c->nodes, f, t_us);
    bool                   back = false;
    if (ev == LBR_NT_FOUND) {
        const lbr_node_entry_t *e = lbr_node_table_find(&c->nodes, lbr_can_sender(f->id));
        back                      = e != NULL && e->first_seen_us != t_us; /* after NODE_LOST */
        back = lbr_node_table_take_missing(&c->nodes, lbr_can_sender(f->id)) || back;
    }
    for (size_t i = 0; i < n; i++) {
        lbr_lora_policy_offer_sample(&c->lora_policy, &samples[i]);
        lbr_value_table_offer(&c->cockpit_values, &samples[i], t_us);
        lbr_logic_offer(&c->logic, &samples[i], t_us);
    }
    uint8_t source = lbr_can_sender(f->id);
    if (!own_tx && f->id == LBR_CAN_ID_HEALTH_BASE + 16u * source + LBR_HEALTH_PAGE_COUNT - 1u) {
        /* Offer health to LoRa once the last page is in, so every field of
         * this heartbeat (state in page 0, sensor status in page 7) is new. */
        const lbr_node_entry_t *e = lbr_node_table_find(&c->nodes, source);
        if (e != NULL) {
            lbr_lora_policy_offer_health(&c->lora_policy, &e->health);
        }
    }
    gw_unlock(c, LBR_GW_LOCK_STATE);

    if (ev == LBR_NT_FOUND) {
        gw_say(c, LBR_LOG_LVL_INFO, "node %s (0x%02x) on the bus", lbr_node_name(source), source);
        if (back) {
            lbr_gw_core_event(c, LBR_EVT_NODE_FOUND, source, false);
        }
    }
}

/* Transmit one of our own frames and record it like any other. */
void gw_tx(lbr_gw_core_t *c, const lbr_can_frame_t *f)
{
    if (c->can_up) {
        c->hal.can_send(c->hal.ctx, f);
    }
    handle_frame(c, f, c->hal.now_us(c->hal.ctx), true);
}

static void emit_event(lbr_gw_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault,
                       uint64_t t_us)
{
    lbr_sample_t s = {
        .signal    = fault ? LBR_SIG_FAULT : LBR_SIG_EVENT,
        .source    = NODE_ID,
        .type      = LBR_VAL_STATUS,
        .valid     = true,
        .synced    = true,
        .t_mono_us = t_us,
    };
    s.v.u32 = lbr_event_value(code, arg);
    gw_lock(c, LBR_GW_LOCK_EVENT);
    s.seq = c->evt_seq++;
    gw_unlock(c, LBR_GW_LOCK_EVENT);
    lbr_can_frame_t f;
    if (lbr_can_encode_event(&s, &f)) {
        gw_tx(c, &f);
    }
}

void lbr_gw_core_event(lbr_gw_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault)
{
    const char *name = lbr_event_name(code);
    gw_say(c, fault ? LBR_LOG_LVL_ERROR : LBR_LOG_LVL_WARN, "%s %s (0x%04x) arg %u",
           fault ? "FAULT" : "event", name != NULL ? name : "?", (unsigned)code, (unsigned)arg);
    uint64_t t = c->hal.now_us(c->hal.ctx);
    gw_lock(c, LBR_GW_LOCK_EVENT);
    if (!c->started) {
        /* No log or CAN yet: hold it for lbr_gw_core_start. */
        if (c->n_early < LBR_EARLY_EVENTS) {
            c->early[c->n_early++] = (lbr_early_event_t){(uint16_t)code, arg, fault, t};
        } else {
            c->early_dropped++;
        }
        gw_unlock(c, LBR_GW_LOCK_EVENT);
        return;
    }
    gw_unlock(c, LBR_GW_LOCK_EVENT);
    emit_event(c, code, arg, fault, t);
}

void lbr_gw_core_command(lbr_gw_core_t *c, uint8_t code, const uint8_t args[7])
{
    lbr_can_frame_t f;
    lbr_can_encode_command(code, args, &f);
    gw_tx(c, &f);
}

bool lbr_gw_core_send(lbr_gw_core_t *c, const lbr_can_frame_t *f)
{
    if (!c->can_up || !c->hal.can_send(c->hal.ctx, f)) {
        return false;
    }
    handle_frame(c, f, c->hal.now_us(c->hal.ctx), true);
    return true;
}

static void acq_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_gw_core_event(ctx, code, arg, fault);
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

/* Car logic outputs: authoritative log first, then the live stream, like frames. */
static void logic_record(void *ctx, uint8_t type, const uint8_t *payload, size_t len)
{
    lbr_gw_core_t *c = ctx;
    if (c->log != NULL) {
        (void)lbr_log_append_raw(c->log, type, payload, len); /* losses counted by the logger */
    }
    stream_add(c, type, payload, len);
}

void lbr_gw_core_init(lbr_gw_core_t *c, const lbr_gw_hal_t *hal, const lbr_gw_config_t *cfg,
                      const lbr_gw_boot_t *boot)
{
    memset(c, 0, sizeof *c);
    c->hal  = *hal;
    c->cfg  = *cfg;
    c->boot = *boot;
    c->config_crc =
        boot->config_text != NULL ? lbr_crc32(boot->config_text, strlen(boot->config_text)) : 0u;
    lbr_node_table_init(&c->nodes, (uint64_t)cfg->node_timeout_ms * 1000u);
    lbr_bus_decoder_init(&c->decoder);
    lbr_lora_policy_init(&c->lora_policy, lbr_lora_default_items, lbr_lora_default_item_count,
                         2000);
    lbr_value_table_init(&c->cockpit_values, LBR_GW_COCKPIT_STALE_US);
    for (size_t i = 0; i < lbr_cockpit_signal_count; i++) {
        lbr_value_table_watch(&c->cockpit_values, lbr_cockpit_signals[i], 0);
    }
    lbr_cockpit_link_init(&c->cockpit);
    lbr_logic_init(&c->logic, boot->tasks, boot->n_tasks,
                   (lbr_logic_sink_t){.ctx = c, .record = logic_record});
}

void lbr_gw_core_start(lbr_gw_core_t *c, lbr_logger_t *log, const lbr_gw_links_t *links,
                       bool can_up, bool can_pins_unset)
{
    c->log             = log;
    c->links           = *links;
    c->can_up          = can_up;
    c->listen_since_us = c->hal.now_us(c->hal.ctx);
    /* Drivers are bound by now (some read this core's own state). */
    for (size_t i = 0; i < c->boot.n_acq && i < LBR_NODE_MAX_SENSORS; i++) {
        const lbr_acq_t       *a  = &c->boot.acq[i];
        lbr_node_slot_state_t *st = &c->slot[i];
        c->sim                    = c->sim || a->sensor->simulated;
        lbr_acq_slot_init(&st->acq, a->sensor, a->rate_hz, (uint16_t)i);
        st->n_frames = lbr_slot_frames(NODE_ID, a->sensor->signals, a->sensor->n_signals,
                                       st->frames, st->chan, LBR_NODE_SLOT_MAX_FRAMES);
    }
    gw_say(c, LBR_LOG_LVL_INFO, "central fw %u.%u.%u protocol %u boot #%lu reset %d epoch %u%s",
           LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR, LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION,
           (unsigned long)c->boot.boot_count, (int)c->boot.reset_reason, (unsigned)c->boot.epoch,
           c->sim ? " SIMULATED SENSORS" : "");
    gw_lock(c, LBR_GW_LOCK_EVENT);
    c->started = true;
    gw_unlock(c, LBR_GW_LOCK_EVENT);
    lbr_gw_core_event(c, LBR_EVT_BOOT, (uint16_t)c->boot.reset_reason, false);
    for (size_t i = 0; i < c->n_early; i++) {
        const lbr_early_event_t *e = &c->early[i];
        emit_event(c, (lbr_event_code_t)e->code, e->arg, e->fault, e->t_us);
    }
    if (!c->boot.config_valid) {
        lbr_gw_core_event(c, LBR_EVT_CONFIG_INVALID, 0, true);
    }
    char err[96];
    if (lbr_logic_check(c->boot.tasks, c->boot.n_tasks, err, sizeof err) != 0) {
        /* The data path still runs; only the car logic is off, and says why. */
        gw_say(c, LBR_LOG_LVL_ERROR, "car logic not started: %s", err);
        lbr_gw_core_event(c, LBR_EVT_CONFIG_INVALID, LBR_GW_INVALID_LOGIC, true);
    }
    if (can_pins_unset) {
        lbr_gw_core_event(c, LBR_EVT_CAN_PINS_UNSET, 0, true);
    }
    if (log != NULL && lbr_log_is_open(log)) {
        uint8_t args[7] = {0};
        lbr_put_u32(args, c->boot.boot_count);
        lbr_gw_core_command(c, LBR_CMD_SESSION_START, args);
    }
}

/* ------------------------------------------------------------------------ */
/* Periodic work                                                             */

void lbr_gw_core_on_rx(lbr_gw_core_t *c, const lbr_can_frame_t *f, uint64_t rx_local_us)
{
    atomic_fetch_add(&c->frames_rx, 1);
    /* Master clock: local and shared time are the same. */
    handle_frame(c, f, c->hal.local_to_shared(c->hal.ctx, rx_local_us), false);
}

uint8_t lbr_gw_core_sync_send(lbr_gw_core_t *c)
{
    lbr_can_frame_t f;
    if (c->hal.sync_arm != NULL) {
        c->hal.sync_arm(c->hal.ctx);
    }
    lbr_can_encode_time_sync(c->sync_seq, c->boot.epoch, c->hal.now_us(c->hal.ctx), &f);
    gw_tx(c, &f);
    return c->sync_seq;
}

void lbr_gw_core_sync_done(lbr_gw_core_t *c, bool sent, uint64_t tx_local_us)
{
    if (sent) {
        lbr_can_frame_t f;
        lbr_can_encode_time_follow_up(c->sync_seq, c->hal.local_to_shared(c->hal.ctx, tx_local_us),
                                      &f);
        gw_tx(c, &f);
    } else {
        atomic_fetch_add(&c->sync_tx_fail, 1); /* no ACK / bus-off: nodes keep slewing */
    }
    c->sync_seq++;
}

void lbr_gw_core_stream_flush(lbr_gw_core_t *c)
{
    gw_lock(c, LBR_GW_LOCK_STREAM);
    stream_flush_locked(c);
    gw_unlock(c, LBR_GW_LOCK_STREAM);
}

void lbr_gw_core_slot_start(lbr_gw_core_t *c, size_t i)
{
    lbr_acq_slot_start(&c->slot[i].acq, c->hal.mono_us(c->hal.ctx), acq_event, c);
}

uint64_t lbr_gw_core_slot_next_due(lbr_gw_core_t *c, size_t i)
{
    return lbr_acq_next_due(c->hal.now_us(c->hal.ctx), c->slot[i].acq.period_us);
}

void lbr_gw_core_slot_tick(lbr_gw_core_t *c, size_t i)
{
    lbr_node_slot_state_t *st = &c->slot[i];
    lbr_reading_t          readings[LBR_SENSOR_MAX_SIGNALS];
    lbr_sample_t           samples[LBR_SENSOR_MAX_SIGNALS];
    lbr_acq_tick_t         tick;

    /* The master clock never steps: one timeline per boot. */
    uint64_t now = c->hal.now_us(c->hal.ctx);
    if (!lbr_acq_slot_tick(&st->acq, now, c->hal.mono_us(c->hal.ctx), 1u, readings, &tick,
                           acq_event, c)) {
        return;
    }
    if (tick.missed > 0) {
        atomic_fetch_add(&c->overruns, tick.missed);
    }
    lbr_sensor_t *sensor = st->acq.sensor;
    for (size_t ch = 0; ch < sensor->n_signals; ch++) {
        lbr_reading_to_sample(NODE_ID, sensor->signals[ch], &readings[ch], tick.k,
                              st->acq.period_us, true, &samples[ch]);
    }
    for (size_t f = 0; f < st->n_frames; f++) {
        const lbr_frame_def_t *d = st->frames[f];
        lbr_sample_t           values[LBR_FRAME_MAX_FIELDS];
        lbr_can_frame_t        frame;
        for (size_t k = 0; k < d->n_fields; k++) {
            values[k] = samples[st->chan[f][k]];
        }
        unsigned misfit = lbr_can_encode_data(d, values, tick.k, &frame);
        if (misfit > 0) {
            atomic_fetch_add(&c->out_of_range, misfit);
        }
        gw_tx(c, &frame);
    }
    atomic_store(&c->last_sample_us, now);
}

void lbr_gw_core_lock_state(void *core)
{
    gw_lock(core, LBR_GW_LOCK_STATE);
}

void lbr_gw_core_unlock_state(void *core)
{
    gw_unlock(core, LBR_GW_LOCK_STATE);
}

void lbr_gw_core_logic_tick(lbr_gw_core_t *c)
{
    gw_lock(c, LBR_GW_LOCK_STATE);
    lbr_logic_step(&c->logic, c->hal.now_us(c->hal.ctx), &c->nodes);
    gw_unlock(c, LBR_GW_LOCK_STATE);
}

size_t lbr_gw_core_status(lbr_gw_core_t *c, const char *log_name, char *buf, size_t cap)
{
    gw_lock(c, LBR_GW_LOCK_STATE);
    size_t n = lbr_status_format(&c->last_health, log_name, &c->nodes, c->hal.now_us(c->hal.ctx),
                                 buf, cap);
    gw_unlock(c, LBR_GW_LOCK_STATE);
    return n;
}
