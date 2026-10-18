#include "lbr/gw_core.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "lbr/crc32.h"
#include "lbr/nodes.h"
#include "lbr/status.h"
#include "lbr/timebase.h"
#include "lbr/version.h"

/* Pure gateway logic; see lbr/gw_core.h. No platform headers here. */

#define NODE_ID LBR_NODE_GATEWAY

static void say(lbr_gw_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

static void say(lbr_gw_core_t *c, lbr_log_level_t lvl, const char *fmt, ...)
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

static void lock(lbr_gw_core_t *c, lbr_gw_lock_t which)
{
    if (c->hal.lock != NULL) {
        c->hal.lock(c->hal.ctx, which);
    }
}

static void unlock(lbr_gw_core_t *c, lbr_gw_lock_t which)
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

static void stream_add(lbr_gw_core_t *c, const uint8_t *rec, size_t len)
{
    if (c->links.usb == NULL && c->links.wifi == NULL) {
        return;
    }
    lock(c, LBR_GW_LOCK_STREAM);
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!c->stream_open) {
            lbr_pkt_begin(&c->stream, c->stream_pkt, sizeof c->stream_pkt, LBR_LINK_USB, NODE_ID, 0,
                          c->hal.now_us(c->hal.ctx));
            c->stream_open = true;
        }
        if (lbr_pkt_add(&c->stream, LBR_REC_CAN_FRAME, rec, len)) {
            break;
        }
        stream_flush_locked(c);
    }
    unlock(c, LBR_GW_LOCK_STREAM);
}

/*
 * Every frame, received or our own, goes through here: authoritative log
 * first, then the live views. `t_us` is the shared time base.
 */
static void handle_frame(lbr_gw_core_t *c, const lbr_can_frame_t *f, uint64_t t_us, bool own_tx)
{
    uint8_t flags = LBR_CAN_REC_FLAG_EXT | (own_tx ? LBR_CAN_REC_FLAG_OWN_TX : 0u);
    uint8_t rec[LBR_CAN_FRAME_REC_SIZE];
    lbr_can_frame_rec_encode(t_us, flags, f, rec, sizeof rec);

    if (c->log != NULL && lbr_log_append_raw(c->log, LBR_REC_CAN_FRAME, rec, sizeof rec) == 0) {
        atomic_fetch_add(&c->frames_logged, 1);
    }
    stream_add(c, rec, sizeof rec);

    lbr_sample_t s;
    bool         is_sample = lbr_can_decode_sample(f, t_us, &s);

    lock(c, LBR_GW_LOCK_STATE);
    lbr_node_table_event_t ev = own_tx ? LBR_NT_NONE : lbr_node_table_on_frame(&c->nodes, f, t_us);
    if (is_sample) {
        lbr_lora_policy_offer_sample(&c->lora_policy, &s);
    } else if (!own_tx) {
        lbr_can_id_t id = lbr_can_id_parse(f->id);
        /* Offer health to LoRa when the state page (0) arrives. */
        if (id.kind == LBR_CAN_KIND_SYSTEM && id.sub == LBR_SYS_HEALTH_BASE) {
            const lbr_node_entry_t *e = lbr_node_table_find(&c->nodes, id.source);
            if (e != NULL) {
                lbr_lora_policy_offer_health(&c->lora_policy, &e->health);
            }
        }
    }
    unlock(c, LBR_GW_LOCK_STATE);

    if (ev == LBR_NT_FOUND) {
        uint8_t source = lbr_can_id_parse(f->id).source;
        say(c, LBR_LOG_LVL_INFO, "node %s (0x%02x) on the bus", lbr_node_name(source), source);
    }
}

/* Transmit one of our own frames and record it like any other. */
static void gw_tx(lbr_gw_core_t *c, const lbr_can_frame_t *f)
{
    if (c->can_up) {
        c->hal.can_send(c->hal.ctx, f);
    }
    handle_frame(c, f, c->hal.now_us(c->hal.ctx), true);
}

void lbr_gw_core_event(lbr_gw_core_t *c, lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_reading_t   r = {.valid = true, .v.u32 = lbr_event_value(code, arg)};
    lbr_sample_t    s;
    uint32_t        pub_seq = 0;
    lbr_can_frame_t f;

    say(c, fault ? LBR_LOG_LVL_ERROR : LBR_LOG_LVL_WARN, "%s 0x%04x arg %u",
        fault ? "FAULT" : "event", (unsigned)code, (unsigned)arg);
    uint64_t t = c->hal.now_us(c->hal.ctx);
    lock(c, LBR_GW_LOCK_EVENT);
    lbr_stream_sample(fault ? &c->fault_stream : &c->evt_stream, 1, NODE_ID,
                      fault ? LBR_SIG_FAULT : LBR_SIG_EVENT, &r, t, true, &s, &pub_seq);
    unlock(c, LBR_GW_LOCK_EVENT);
    s.seq = pub_seq;
    if (lbr_can_encode_sample(&s, &f)) {
        gw_tx(c, &f);
    }
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

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
    lbr_lora_policy_init(&c->lora_policy, lbr_lora_default_items, lbr_lora_default_item_count,
                         2000);
}

void lbr_gw_core_start(lbr_gw_core_t *c, lbr_logger_t *log, const lbr_gw_links_t *links,
                       bool can_up, bool can_pins_unset)
{
    c->log    = log;
    c->links  = *links;
    c->can_up = can_up;
    say(c, LBR_LOG_LVL_INFO, "gateway fw %u.%u.%u protocol %u boot #%lu reset %d epoch %u",
        LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR, LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION,
        (unsigned long)c->boot.boot_count, (int)c->boot.reset_reason, (unsigned)c->boot.epoch);
    lbr_gw_core_event(c, LBR_EVT_BOOT, (uint16_t)c->boot.reset_reason, false);
    if (!c->boot.config_valid) {
        lbr_gw_core_event(c, LBR_EVT_CONFIG_INVALID, 0, true);
    }
    if (can_pins_unset) {
        lbr_gw_core_event(c, LBR_EVT_CAN_PINS_UNSET, 0, true);
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
    lbr_can_encode_time_sync(NODE_ID, c->sync_seq, c->boot.epoch, &f);
    gw_tx(c, &f);
    return c->sync_seq;
}

void lbr_gw_core_sync_done(lbr_gw_core_t *c, bool sent, uint64_t tx_local_us)
{
    if (sent) {
        lbr_can_frame_t f;
        lbr_can_encode_time_follow_up(NODE_ID, c->sync_seq,
                                      c->hal.local_to_shared(c->hal.ctx, tx_local_us), &f);
        gw_tx(c, &f);
    } else {
        atomic_fetch_add(&c->sync_tx_fail, 1); /* no ACK / bus-off: nodes keep slewing */
    }
    c->sync_seq++;
}

void lbr_gw_core_stream_flush(lbr_gw_core_t *c)
{
    lock(c, LBR_GW_LOCK_STREAM);
    stream_flush_locked(c);
    unlock(c, LBR_GW_LOCK_STREAM);
}

void lbr_gw_core_lora_tick(lbr_gw_core_t *c)
{
    uint8_t pkt[LBR_PKT_LORA_MAX];
    if (c->links.lora == NULL || c->hal.lora_ready == NULL || !c->hal.lora_ready(c->hal.ctx)) {
        return; /* radio busy or duty budget spent: the policy keeps the latest */
    }
    lock(c, LBR_GW_LOCK_STATE);
    size_t n = lbr_lora_policy_build(&c->lora_policy, c->hal.now_us(c->hal.ctx), NODE_ID, 0, pkt,
                                     sizeof pkt);
    unlock(c, LBR_GW_LOCK_STATE);
    if (n > 0) {
        c->links.lora->send(c->links.lora, pkt, n);
    }
}

static void link_totals(const lbr_gw_core_t *c, uint32_t *dropped, uint32_t *errors)
{
    lbr_transport_t *links[] = {c->links.usb, c->links.wifi, c->links.lora, c->links.ble};
    *dropped                 = 0;
    *errors                  = 0;
    for (size_t i = 0; i < sizeof links / sizeof links[0]; i++) {
        if (links[i] != NULL && links[i]->get_stats != NULL) {
            lbr_transport_stats_t st = {0};
            links[i]->get_stats(links[i], &st);
            *dropped += st.packets_dropped;
            *errors += st.errors;
        }
    }
}

static void build_health(lbr_gw_core_t *c, lbr_health_record_t *h)
{
    lbr_can_stats_t cs;
    lbr_log_stats_t ls = {0};
    uint32_t        link_dropped, link_errors;

    c->hal.can_stats(c->hal.ctx, &cs);
    if (c->log != NULL) {
        lbr_log_get_stats(c->log, &ls);
    }
    link_totals(c, &link_dropped, &link_errors);

    bool     log_open = c->log != NULL && lbr_log_is_open(c->log);
    uint16_t flags    = LBR_HF_TIME_SYNCED | LBR_HF_TIME_MASTER;
    flags |= (c->hal.sd_present != NULL && c->hal.sd_present(c->hal.ctx)) ? LBR_HF_SD_PRESENT : 0;
    flags |= log_open ? LBR_HF_SD_LOGGING : 0;
    flags |= c->boot.config_valid ? LBR_HF_CONFIG_VALID : 0;
    flags |= cs.running ? LBR_HF_CAN_UP : 0;

    lbr_node_state_t state = LBR_NODE_RUN;
    if (!c->can_up) {
        state = LBR_NODE_FAULT;
    } else if (!log_open || !cs.running || !c->boot.config_valid) {
        state = LBR_NODE_DEGRADED; /* no authoritative log is the worst case short of no bus */
    }

    uint32_t rx     = atomic_load(&c->frames_rx);
    uint32_t logged = atomic_load(&c->frames_logged);
    uint64_t now    = c->hal.now_us(c->hal.ctx);
    *h              = (lbr_health_record_t){
        .node_id          = NODE_ID,
        .state            = (uint8_t)state,
        .flags            = flags,
        .fw_version       = LBR_FW_VERSION,
        .protocol_version = LBR_PROTOCOL_VERSION,
        .boot_count       = (uint16_t)c->boot.boot_count,
        .catalog_hash     = lbr_catalog_hash(),
        .config_crc       = c->config_crc,
        .uptime_ms        = (uint32_t)(c->hal.mono_us(c->hal.ctx) / 1000u),
        .reset_reason     = (uint8_t)c->boot.reset_reason,
        .acquire_hz       = (uint16_t)((rx - c->prev_rx) * 1000u / LBR_GW_HEALTH_PERIOD_MS),
        .publish_hz       = 0,
        .log_hz           = (uint16_t)((logged - c->prev_logged) * 1000u / LBR_GW_HEALTH_PERIOD_MS),
        .queue_high_water = ls.queue_high_water,
        .overflow_count   = ls.records_dropped + ls.records_rejected_closed + ls.records_lost_write,
        .rejected_count   = cs.rx_foreign,
        .write_error_count     = ls.write_errors + ls.sync_errors,
        .transport_error_count = link_dropped + link_errors + atomic_load(&c->sync_tx_fail),
        .can_tec               = cs.tec,
        .can_rec               = cs.rec,
        .can_bus_off_count     = cs.bus_off_count,
        .can_tx_dropped        = cs.tx_dropped,
        .can_rx_dropped        = cs.rx_dropped,
        .can_bus_errors        = cs.bus_errors,
        .sensor_status         = 0,
        .calibration_state     = 0,
        .last_sample_age_ms    = LBR_AGE_NONE,
        .last_sd_write_age_ms  = lbr_age_ms(now, ls.last_write_us),
    };
    c->prev_rx     = rx;
    c->prev_logged = logged;
}

void lbr_gw_core_heartbeat(lbr_gw_core_t *c, lbr_health_record_t *out)
{
    lbr_health_record_t h;
    uint8_t             lost[LBR_NODE_TABLE_SIZE];

    build_health(c, &h);
    for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
        lbr_can_frame_t f;
        lbr_can_encode_health_page(NODE_ID, &h, p, &f);
        gw_tx(c, &f);
    }
    if (h.can_bus_off_count != c->last_bus_off) {
        c->last_bus_off = h.can_bus_off_count;
        lbr_gw_core_event(c, LBR_EVT_CAN_BUS_OFF, (uint16_t)c->last_bus_off, true);
    }

    lock(c, LBR_GW_LOCK_STATE);
    c->last_health = h;
    lbr_lora_policy_offer_health(&c->lora_policy, &h);
    size_t n_lost = lbr_node_table_expire(&c->nodes, c->hal.now_us(c->hal.ctx), lost, sizeof lost);
    unlock(c, LBR_GW_LOCK_STATE);
    for (size_t i = 0; i < n_lost; i++) {
        lbr_gw_core_event(c, LBR_EVT_NODE_LOST, lost[i], true);
    }
    if (out != NULL) {
        *out = h;
    }
}

size_t lbr_gw_core_status(lbr_gw_core_t *c, const char *log_name, char *buf, size_t cap)
{
    lock(c, LBR_GW_LOCK_STATE);
    size_t n = lbr_status_format(&c->last_health, log_name, &c->nodes, c->hal.now_us(c->hal.ctx),
                                 buf, cap);
    unlock(c, LBR_GW_LOCK_STATE);
    return n;
}
