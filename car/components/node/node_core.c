#include "lbr/node_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/crc32.h"
#include "lbr/nodes.h"
#include "lbr/version.h"

/* Pure sensor-node logic; see lbr/node_core.h. No platform headers here. */

static void say(lbr_node_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void say(lbr_node_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
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

static void lock(lbr_node_core_t *c)
{
    if (c->hal.lock != NULL) {
        c->hal.lock(c->hal.ctx);
    }
}

static void unlock(lbr_node_core_t *c)
{
    if (c->hal.unlock != NULL) {
        c->hal.unlock(c->hal.ctx);
    }
}

/* ------------------------------------------------------------------------ */
/* Publishing                                                                */

static void send_frame(lbr_node_core_t *c, const lbr_can_frame_t *f)
{
    if (c->can_up && c->hal.can_send(c->hal.ctx, f)) {
        atomic_fetch_add(&c->published, 1);
    }
    /* A failed send is counted by the CAN controller (tx_dropped). */
}

bool lbr_node_core_send(lbr_node_core_t *c, const lbr_can_frame_t *f)
{
    if (!c->can_up || !c->hal.can_send(c->hal.ctx, f)) {
        return false;
    }
    atomic_fetch_add(&c->published, 1);
    return true;
}

static void log_sample(lbr_node_core_t *c, const lbr_sample_t *s)
{
    if (c->log != NULL && lbr_log_append_sample(c->log, s) == 0) {
        atomic_fetch_add(&c->logged, 1);
    }
    /* Drops are counted by the logger (records_dropped / rejected_closed). */
}

void lbr_node_core_event(lbr_node_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault)
{
    const char *name = lbr_event_name(code);
    say(c, fault ? LBR_LOG_LVL_ERROR : LBR_LOG_LVL_WARN, "%s %s (0x%04x) arg %u",
        fault ? "FAULT" : "event", name != NULL ? name : "?", (unsigned)code, (unsigned)arg);

    lbr_sample_t s = {
        .signal    = fault ? LBR_SIG_FAULT : LBR_SIG_EVENT,
        .source    = c->cfg->node_id,
        .type      = LBR_VAL_STATUS,
        .valid     = true,
        .synced    = c->hal.time_synced(c->hal.ctx),
        .t_mono_us = c->hal.now_us(c->hal.ctx),
    };
    s.v.u32 = lbr_event_value(code, arg);
    lock(c);
    s.seq = c->evt_seq++;
    unlock(c);

    log_sample(c, &s);
    lbr_can_frame_t f;
    if (lbr_can_encode_event(&s, &f)) {
        send_frame(c, &f);
    }
}

void lbr_node_core_on_log_event(lbr_node_core_t *c, uint16_t code, bool fault)
{
    lbr_node_core_event(c, (lbr_event_code_t)code, 0, fault);
}

static void acq_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_node_core_event(ctx, code, arg, fault);
}

void lbr_node_core_set_app_flags(lbr_node_core_t *c, uint8_t flags)
{
    atomic_store(&c->app_flags, flags);
}

void lbr_node_core_count_loss(lbr_node_core_t *c, uint32_t n)
{
    atomic_fetch_add(&c->app_loss, n);
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

bool lbr_node_core_init(lbr_node_core_t *c, const lbr_node_config_t *cfg, const lbr_node_hal_t *hal,
                        const lbr_node_boot_t *boot)
{
    memset(c, 0, sizeof *c);
    c->cfg  = cfg;
    c->hal  = *hal;
    c->boot = *boot;
    for (size_t i = 0; i < cfg->n_acq && i < LBR_NODE_MAX_SENSORS; i++) {
        const lbr_acq_t       *a  = &cfg->acq[i];
        lbr_node_slot_state_t *st = &c->slot[i];
        c->sim                    = c->sim || (a->sensor != NULL && a->sensor->simulated);
        lbr_acq_slot_init(&st->acq, a->sensor, a->rate_hz, (uint16_t)i);
        if (a->sensor != NULL && a->sensor->signals != NULL) {
            st->n_frames = lbr_slot_frames(cfg->node_id, a->sensor->signals, a->sensor->n_signals,
                                           st->frames, st->chan, LBR_NODE_SLOT_MAX_FRAMES);
        }
    }
    lbr_node_config_text(cfg, c->config_text, sizeof c->config_text);
    c->config_crc = lbr_crc32(c->config_text, strlen(c->config_text));

    say(c, LBR_LOG_LVL_INFO, "%s (0x%02x) fw %u.%u.%u protocol %u boot #%lu reset %d%s",
        lbr_node_name(cfg->node_id), cfg->node_id, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
        LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION, (unsigned long)boot->boot_count,
        (int)boot->reset_reason, c->sim ? " SIMULATED SENSORS" : "");

    c->config_valid = lbr_node_config_validate(cfg, c->config_error, sizeof c->config_error) == 0;
    if (!c->config_valid) {
        say(c, LBR_LOG_LVL_ERROR, "configuration rejected: %s", c->config_error);
    }
    return c->config_valid;
}

void lbr_node_core_start(lbr_node_core_t *c, bool can_up, bool can_pins_unset, lbr_logger_t *log)
{
    c->can_up = can_up;
    c->log    = log;
    lbr_node_core_event(c, LBR_EVT_BOOT, (uint16_t)c->boot.reset_reason, false);
    if (can_pins_unset) {
        lbr_node_core_event(c, LBR_EVT_CAN_PINS_UNSET, 0, true);
    }
    if (!c->config_valid) {
        lbr_node_core_event(c, LBR_EVT_CONFIG_INVALID, 0, true);
    }
}

/* ------------------------------------------------------------------------ */
/* Acquisition                                                               */

void lbr_node_core_slot_start(lbr_node_core_t *c, size_t i)
{
    lbr_acq_slot_start(&c->slot[i].acq, c->hal.mono_us(c->hal.ctx), acq_event, c);
}

uint64_t lbr_node_core_slot_next_due(lbr_node_core_t *c, size_t i)
{
    return lbr_acq_next_due(c->hal.now_us(c->hal.ctx), c->slot[i].acq.period_us);
}

static uint32_t timeline(lbr_node_core_t *c, bool *synced)
{
    lbr_time_servo_t ts;
    c->hal.time_servo(c->hal.ctx, &ts);
    *synced = ts.synced;
    return lbr_acq_timeline(ts.synced, ts.step_count, ts.epoch_changes);
}

void lbr_node_core_slot_tick(lbr_node_core_t *c, size_t i)
{
    lbr_node_slot_state_t *st     = &c->slot[i];
    lbr_sensor_t          *sensor = st->acq.sensor;
    lbr_reading_t          readings[LBR_SENSOR_MAX_SIGNALS];
    lbr_sample_t           samples[LBR_SENSOR_MAX_SIGNALS];
    lbr_acq_tick_t         tick;
    bool                   synced;

    uint32_t tl  = timeline(c, &synced);
    uint64_t now = c->hal.now_us(c->hal.ctx);
    if (!lbr_acq_slot_tick(&st->acq, now, c->hal.mono_us(c->hal.ctx), tl, readings, &tick,
                           acq_event, c)) {
        return;
    }
    if (tick.missed > 0) {
        /* Acquisition fell behind its grid: the skipped samples are counted. */
        atomic_fetch_add(&c->overruns, tick.missed);
    }

    for (size_t ch = 0; ch < sensor->n_signals; ch++) {
        lbr_reading_to_sample(c->cfg->node_id, sensor->signals[ch], &readings[ch], tick.k,
                              st->acq.period_us, synced, &samples[ch]);
        log_sample(c, &samples[ch]);
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
        send_frame(c, &frame);
    }
    atomic_fetch_add(&c->acquired, 1);
    atomic_store(&c->last_sample_us, now);
}

/* ------------------------------------------------------------------------ */
/* CAN receive: time sync, commands, app frames                              */

void lbr_node_core_on_frame(lbr_node_core_t *c, const lbr_can_frame_t *f, uint64_t rx_local_us)
{
    uint8_t  seq;
    uint16_t epoch;
    uint64_t tx_us;
    uint8_t  code;
    uint8_t  args[7];

    if (lbr_can_decode_time_sync(f, &seq, &epoch)) {
        lbr_time_sync_on_sync(&c->sync, seq, epoch, rx_local_us);
        return;
    }
    if (lbr_can_decode_time_follow_up(f, &seq, &tx_us)) {
        uint64_t gw_us, local_us;
        if (lbr_time_sync_on_follow_up(&c->sync, seq, tx_us, &gw_us, &local_us, &epoch)) {
            lbr_time_update_t r = c->hal.time_apply(c->hal.ctx, gw_us, local_us, epoch);
            if (r == LBR_TIME_UPD_FIRST || r == LBR_TIME_UPD_EPOCH_CHANGE) {
                lbr_node_core_event(c, LBR_EVT_TIME_SYNCED, epoch, false);
            } else if (r == LBR_TIME_UPD_STEP) {
                lbr_node_core_event(c, LBR_EVT_TIME_STEP, 0, false);
            }
        } else {
            atomic_store(&c->sync_unmatched, c->sync.unmatched);
        }
        return;
    }
    if (lbr_can_decode_command(f, &code, args)) {
        /* No command changes what a sensor node does today; each is noted. */
        atomic_fetch_add(&c->commands, 1);
        say(c, LBR_LOG_LVL_INFO, "command 0x%02x from the Central", (unsigned)code);
        return;
    }
    if (c->cfg->on_frame != NULL) {
        c->cfg->on_frame(f, c->hal.local_to_shared(c->hal.ctx, rx_local_us), c->cfg->on_frame_ctx);
    }
}

/* ------------------------------------------------------------------------ */
/* Heartbeat                                                                 */

static uint16_t rate_hz(uint32_t now, uint32_t *prev, uint32_t period_ms)
{
    uint32_t d = now - *prev;
    *prev      = now;
    return (uint16_t)((uint64_t)d * 1000u / period_ms);
}

static void build_health(lbr_node_core_t *c, lbr_health_record_t *h, uint8_t *err_flags)
{
    const lbr_node_config_t *cfg    = c->cfg;
    const uint32_t           period = cfg->health_period_ms ? cfg->health_period_ms : 1000u;
    lbr_can_stats_t          cs;
    lbr_log_stats_t          ls = {0};
    lbr_time_servo_t         ts;

    c->hal.can_stats(c->hal.ctx, &cs);
    c->hal.time_servo(c->hal.ctx, &ts);
    if (c->log != NULL) {
        lbr_log_get_stats(c->log, &ls);
    }
    bool log_open   = c->log != NULL && lbr_log_is_open(c->log);
    bool sd_present = c->log != NULL && c->hal.sd_present != NULL && c->hal.sd_present(c->hal.ctx);

    uint32_t sensor_ok  = 0;
    uint32_t calibrated = 0;
    size_t   n_acq      = cfg->n_acq < LBR_NODE_MAX_SENSORS ? cfg->n_acq : LBR_NODE_MAX_SENSORS;
    for (size_t i = 0; i < n_acq; i++) {
        sensor_ok |= atomic_load(&c->slot[i].acq.healthy) ? (1u << i) : 0u;
        calibrated |= atomic_load(&c->slot[i].acq.calibrated) ? (1u << i) : 0u;
    }
    uint32_t all_sensors = (1u << n_acq) - 1u;
    uint8_t  app_flags   = (uint8_t)atomic_load(&c->app_flags);

    uint16_t flags = 0;
    flags |= ts.synced ? LBR_HF_TIME_SYNCED : 0;
    flags |= sd_present ? LBR_HF_SD_PRESENT : 0;
    flags |= log_open ? LBR_HF_SD_LOGGING : 0;
    flags |= c->sim ? LBR_HF_SIM_SENSORS : 0;
    flags |= c->config_valid ? LBR_HF_CONFIG_VALID : 0;
    flags |= cs.running ? LBR_HF_CAN_UP : 0;

    lbr_node_state_t state = LBR_NODE_RUN;
    if (!c->config_valid) {
        state = LBR_NODE_CONFIG_INVALID;
    } else if (!c->can_up) {
        state = LBR_NODE_FAULT;
    } else if (sensor_ok != all_sensors || !ts.synced || !cs.running ||
               (cfg->local_log && !log_open) ||
               (app_flags & (LBR_HB_ERR_LINK | LBR_HB_ERR_SENSOR))) {
        state = LBR_NODE_DEGRADED;
    }

    uint64_t now = c->hal.now_us(c->hal.ctx);
    *h           = (lbr_health_record_t){
        .node_id          = cfg->node_id,
        .state            = (uint8_t)state,
        .flags            = flags,
        .fw_version       = LBR_FW_VERSION,
        .protocol_version = LBR_PROTOCOL_VERSION,
        .boot_count       = (uint16_t)c->boot.boot_count,
        .catalog_hash     = lbr_catalog_hash(),
        .config_crc       = c->config_crc,
        .uptime_ms        = (uint32_t)(c->hal.mono_us(c->hal.ctx) / 1000u),
        .reset_reason     = (uint8_t)c->boot.reset_reason,
        .acquire_hz       = rate_hz(atomic_load(&c->acquired), &c->prev_acquired, period),
        .publish_hz       = rate_hz(atomic_load(&c->published), &c->prev_published, period),
        .log_hz           = rate_hz(atomic_load(&c->logged), &c->prev_logged, period),
        .queue_high_water = ls.queue_high_water,
        .overflow_count   = atomic_load(&c->overruns) + ls.records_dropped + ls.records_lost_write +
                            (cfg->local_log ? ls.records_rejected_closed : 0u),
        .rejected_count =
            cs.rx_foreign + atomic_load(&c->sync_unmatched) + atomic_load(&c->out_of_range),
        .write_error_count     = ls.write_errors + ls.sync_errors,
        .transport_error_count = cs.tx_failed + atomic_load(&c->app_loss),
        .can_tec               = cs.tec,
        .can_rec               = cs.rec,
        .can_bus_off_count     = cs.bus_off_count,
        .can_tx_dropped        = cs.tx_dropped,
        .can_rx_dropped        = cs.rx_dropped,
        .can_bus_errors        = cs.bus_errors,
        .sensor_status         = sensor_ok,
        .calibration_state     = calibrated,
        .last_sample_age_ms    = lbr_age_ms(now, atomic_load(&c->last_sample_us)),
        .last_sd_write_age_ms  = cfg->local_log ? lbr_age_ms(now, ls.last_write_us) : LBR_AGE_NONE,
        .time_step_count       = ts.step_count,
        .last_sync_error_us    = ts.last_error_us,
    };

    /* Anything lost since the last heartbeat raises LOSS for this period. */
    uint32_t loss = h->overflow_count + h->can_tx_dropped + h->can_rx_dropped +
                    h->transport_error_count + ls.records_rejected_closed;
    uint8_t  e    = app_flags;
    e |= ts.synced ? 0u : LBR_HB_ERR_NOT_SYNCED;
    e |= (sensor_ok != all_sensors || (calibrated & all_sensors) != all_sensors) ? LBR_HB_ERR_SENSOR
                                                                                 : 0u;
    e |= (cs.tec >= 128 || cs.rec >= 128 || !cs.running || cs.bus_off_count != c->last_bus_off)
             ? LBR_HB_ERR_CAN
             : 0u;
    e |= c->config_valid ? 0u : LBR_HB_ERR_CONFIG;
    e |= (cfg->local_log && !log_open) ? LBR_HB_ERR_STORAGE : 0u;
    e |= c->sim ? LBR_HB_ERR_SIM : 0u;
    e |= loss != c->prev_loss ? LBR_HB_ERR_LOSS : 0u;
    c->prev_loss = loss;
    *err_flags   = e;
}

void lbr_node_core_heartbeat(lbr_node_core_t *c, lbr_health_record_t *out)
{
    lbr_health_record_t h;
    uint8_t             err_flags;
    build_health(c, &h, &err_flags);

    if (h.can_bus_off_count != c->last_bus_off) {
        c->last_bus_off = h.can_bus_off_count;
        lbr_node_core_event(c, LBR_EVT_CAN_BUS_OFF, (uint16_t)c->last_bus_off, true);
    }
    if (c->can_up) {
        lbr_can_frame_t f;
        if (lbr_can_encode_heartbeat(c->cfg->node_id, h.state, err_flags, &f)) {
            c->hal.can_send(c->hal.ctx, &f);
        }
        for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
            if (lbr_can_encode_health_page(c->cfg->node_id, &h, p, &f)) {
                c->hal.can_send(c->hal.ctx, &f);
            }
        }
    }
    if (c->log != NULL) {
        lbr_log_append_health(c->log, &h);
    }
    if (out != NULL) {
        *out = h;
    }
}
