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

static void publish(lbr_node_core_t *c, const lbr_sample_t *s)
{
    lbr_can_frame_t f;
    if (c->can_up && lbr_can_encode_sample(s, &f) && c->hal.can_send(c->hal.ctx, &f)) {
        atomic_fetch_add(&c->published, 1);
    }
    /* A failed send is counted by the CAN controller (tx_dropped). */
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
    lbr_reading_t r = {.valid = true, .v.u32 = lbr_event_value(code, arg)};
    lbr_sample_t  s;
    uint32_t      pub_seq = 0;

    const char *name = lbr_event_name(code);
    say(c, fault ? LBR_LOG_LVL_ERROR : LBR_LOG_LVL_WARN, "%s %s (0x%04x) arg %u",
        fault ? "FAULT" : "event", name != NULL ? name : "?", (unsigned)code, (unsigned)arg);
    uint64_t t      = c->hal.now_us(c->hal.ctx);
    bool     synced = c->hal.time_synced(c->hal.ctx);
    lock(c);
    lbr_stream_sample(fault ? &c->fault_stream : &c->evt_stream, 1, c->cfg->node_id,
                      fault ? LBR_SIG_FAULT : LBR_SIG_EVENT, &r, t, synced, &s, &pub_seq);
    unlock(c);

    log_sample(c, &s);
    s.seq = pub_seq;
    publish(c, &s);
}

void lbr_node_core_on_log_event(lbr_node_core_t *c, uint16_t code, bool fault)
{
    lbr_node_core_event(c, (lbr_event_code_t)code, 0, fault);
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
        c->sim = c->sim || (cfg->acq[i].sensor != NULL && cfg->acq[i].sensor->simulated);
        atomic_store(&c->slot[i].calibrated, true);
    }
    lbr_node_config_text(cfg, c->config_text, sizeof c->config_text);
    c->config_crc = lbr_crc32(c->config_text, strlen(c->config_text));

    say(c, LBR_LOG_LVL_INFO, "%s (0x%02x) fw %u.%u.%u protocol %u boot #%lu reset %d%s",
        lbr_node_name(cfg->node_id), cfg->node_id, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
        LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION, (unsigned long)boot->boot_count,
        (int)boot->reset_reason, c->sim ? " SIMULATED SENSORS" : "");

    c->config_valid = lbr_node_config_validate(cfg, boot->can_bitrate, boot->can_budget_permille,
                                               c->config_error, sizeof c->config_error) == 0;
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

static void mark_all(lbr_reading_t *out, size_t n, lbr_unavail_reason_t why)
{
    for (size_t i = 0; i < n; i++) {
        out[i].valid  = false;
        out[i].reason = why;
        out[i].v.u32  = 0;
    }
}

void lbr_node_core_slot_start(lbr_node_core_t *c, size_t i)
{
    lbr_sensor_t          *sensor = c->cfg->acq[i].sensor;
    lbr_node_slot_state_t *st     = &c->slot[i];
    bool                   ok     = sensor->init(sensor) == 0;
    atomic_store(&st->healthy, ok);
    if (!ok) {
        lbr_node_core_event(c, LBR_EVT_SENSOR_INIT_FAIL, (uint16_t)i, true);
        st->next_retry_us = c->hal.mono_us(c->hal.ctx) + LBR_NODE_SENSOR_RETRY_US;
    }
}

void lbr_node_core_slot_tick(lbr_node_core_t *c, size_t i, uint32_t missed)
{
    const lbr_acq_t       *acq    = &c->cfg->acq[i];
    lbr_sensor_t          *sensor = acq->sensor;
    lbr_node_slot_state_t *st     = &c->slot[i];
    lbr_reading_t          readings[LBR_SENSOR_MAX_SIGNALS];

    if (missed > 0) {
        /* Acquisition fell behind its schedule: missed slots are counted. */
        atomic_fetch_add(&c->overruns, missed);
    }

    uint64_t t      = c->hal.now_us(c->hal.ctx);
    bool     synced = c->hal.time_synced(c->hal.ctx);

    if (atomic_load(&st->healthy)) {
        if (sensor->read(sensor, t, readings) != 0) {
            atomic_store(&st->healthy, false);
            lbr_node_core_event(c, LBR_EVT_SENSOR_INIT_FAIL, (uint16_t)i, true);
            st->next_retry_us = c->hal.mono_us(c->hal.ctx) + LBR_NODE_SENSOR_RETRY_US;
            mark_all(readings, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
        }
    } else {
        mark_all(readings, sensor->n_signals, LBR_UNAVAIL_SENSOR_FAULT);
        uint64_t now = c->hal.mono_us(c->hal.ctx);
        if (now >= st->next_retry_us) {
            if (sensor->init(sensor) == 0) {
                atomic_store(&st->healthy, true);
                lbr_node_core_event(c, LBR_EVT_SENSOR_RECOVERED, (uint16_t)i, false);
            } else {
                st->next_retry_us = now + LBR_NODE_SENSOR_RETRY_US;
            }
        }
    }
    atomic_store(&st->calibrated, sensor->is_calibrated == NULL || sensor->is_calibrated(sensor));

    for (size_t ch = 0; ch < sensor->n_signals; ch++) {
        lbr_sample_t s;
        uint32_t     pub_seq;
        bool pub = lbr_stream_sample(&st->streams[ch], acq->publish_div, c->cfg->node_id,
                                     sensor->signals[ch], &readings[ch], t, synced, &s, &pub_seq);
        log_sample(c, &s);
        if (pub) {
            s.seq = pub_seq;
            publish(c, &s);
        }
    }
    atomic_fetch_add(&c->acquired, 1);
    atomic_store(&c->last_sample_us, t);
}

/* ------------------------------------------------------------------------ */
/* CAN receive: time sync + app frames                                       */

void lbr_node_core_on_frame(lbr_node_core_t *c, const lbr_can_frame_t *f, uint64_t rx_local_us)
{
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    uint8_t      seq;
    uint16_t     epoch;
    uint64_t     tx_us;

    if (id.source == LBR_NODE_GATEWAY && lbr_can_decode_time_sync(f, &seq, &epoch)) {
        lbr_time_sync_on_sync(&c->sync, seq, epoch, rx_local_us);
        return;
    }
    if (id.source == LBR_NODE_GATEWAY && lbr_can_decode_time_follow_up(f, &seq, &tx_us)) {
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

static void build_health(lbr_node_core_t *c, lbr_health_record_t *h)
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
        sensor_ok |= atomic_load(&c->slot[i].healthy) ? (1u << i) : 0u;
        calibrated |= atomic_load(&c->slot[i].calibrated) ? (1u << i) : 0u;
    }
    uint32_t all_sensors = (1u << n_acq) - 1u;

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
               (cfg->local_log && !log_open)) {
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
        .rejected_count   = cs.rx_foreign + atomic_load(&c->sync_unmatched),
        .write_error_count     = ls.write_errors + ls.sync_errors,
        .transport_error_count = cs.tx_failed,
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
}

void lbr_node_core_heartbeat(lbr_node_core_t *c, lbr_health_record_t *out)
{
    lbr_health_record_t h;
    build_health(c, &h);

    if (h.can_bus_off_count != c->last_bus_off) {
        c->last_bus_off = h.can_bus_off_count;
        lbr_node_core_event(c, LBR_EVT_CAN_BUS_OFF, (uint16_t)c->last_bus_off, true);
    }
    if (c->can_up) {
        for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
            lbr_can_frame_t f;
            lbr_can_encode_health_page(c->cfg->node_id, &h, p, &f);
            c->hal.can_send(c->hal.ctx, &f);
        }
    }
    if (c->log != NULL) {
        lbr_log_append_health(c->log, &h);
    }
    if (out != NULL) {
        *out = h;
    }
}
