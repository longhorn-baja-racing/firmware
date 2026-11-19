/*
 * The platform the firmware sees: per-node HAL tables, SD cards and log files,
 * the Central's links, and the virtual 5 GHz WiFi.
 */
#include "sim_internal.h"

/* ------------------------------------------------------------------------ */
/* Per-node platform: the HAL the firmware cores call                        */

hal_ctx_t s_ctx[SIM_MAX_NODES];

static uint64_t h_mono(void *ctx)
{
    hal_ctx_t *c = ctx;
    return sim_local_us(c->s, c->n);
}

uint64_t h_now(void *ctx)
{
    hal_ctx_t *c = ctx;
    return lbr_time_servo_now(&c->s->nodes[c->n].servo, sim_local_us(c->s, c->n));
}

uint64_t h_l2s(void *ctx, uint64_t local)
{
    hal_ctx_t *c = ctx;
    return lbr_time_servo_map(&c->s->nodes[c->n].servo, local);
}

static bool h_synced(void *ctx)
{
    hal_ctx_t *c = ctx;
    return c->s->nodes[c->n].servo.synced;
}

static lbr_time_update_t h_apply(void *ctx, uint64_t gw, uint64_t local, uint16_t epoch)
{
    hal_ctx_t         *c       = ctx;
    lbr_time_update_t  r       = lbr_time_servo_update(&c->s->nodes[c->n].servo, gw, local, epoch);
    static const char *names[] = {"FIRST", "EPOCH_CHANGE", "STEP", "SLEW", "IGNORED"};
    sim_trace(c->s, c->n, "time.apply", "shared %llu us at local %llu us -> %s (error %ld us)",
              (unsigned long long)gw, (unsigned long long)local, names[r],
              (long)c->s->nodes[c->n].servo.last_error_us);
    return r;
}

static void h_servo(void *ctx, lbr_time_servo_t *out)
{
    hal_ctx_t *c = ctx;
    *out         = c->s->nodes[c->n].servo;
}

static bool h_send(void *ctx, const lbr_can_frame_t *f)
{
    hal_ctx_t *c  = ctx;
    bool       ok = vbus_send(c->s, c->n, f);
    char       d[48];
    sim_trace(c->s, c->n, "twai.send", "id 0x%03lx dlc %u %s%s", (unsigned long)f->id,
              (unsigned)f->dlc, frame_desc(f, d, sizeof d), ok ? "" : " -> DROPPED");
    return ok;
}

static void h_stats(void *ctx, lbr_can_stats_t *out)
{
    hal_ctx_t *c = ctx;
    *out         = c->s->nodes[c->n].can.st;
}

static bool h_sd_present(void *ctx)
{
    hal_ctx_t *c = ctx;
    return c->s->nodes[c->n].keeper.sd_present;
}

static void h_log(void *ctx, lbr_log_level_t level, const char *text)
{
    hal_ctx_t         *c    = ctx;
    sim_t             *s    = c->s;
    static const char *lv[] = {"E", "W", "I"};
    if (s->quiet) {
        return;
    }
    fprintf(s->out, "{\"ev\":\"log\",\"t\":%.6f,\"node\":\"%s\",\"level\":\"%s\",\"text\":", t_s(s),
            s->nodes[c->n].name, lv[level]);
    json_str(s->out, text);
    fprintf(s->out, "}\n");
}

static void h_gw_sync_arm(void *ctx)
{
    hal_ctx_t *c = ctx;
    sim_trace(c->s, c->n, "twai.arm_sync", "capture TX-done time of the next TIME_SYNC");
}

static bool h_gw_lora_ready(void *ctx)
{
    hal_ctx_t *c = ctx;
    return sim_local_us(c->s, GW) >= c->s->gw.lora_next_tx_local;
}

static void h_gw_lock(void *ctx, lbr_gw_lock_t which)
{
    (void)ctx;
    (void)which; /* single-threaded simulation */
}

static bool h_wifi_send(void *ctx, const uint8_t *pkt, size_t len)
{
    hal_ctx_t *c = ctx;
    return wifi_send(c->s, c->n, GW, pkt, len);
}

const lbr_node_hal_t k_node_hal = {
    .mono_us         = h_mono,
    .now_us          = h_now,
    .local_to_shared = h_l2s,
    .time_synced     = h_synced,
    .time_apply      = h_apply,
    .time_servo      = h_servo,
    .can_send        = h_send,
    .can_stats       = h_stats,
    .sd_present      = h_sd_present,
    .log             = h_log,
};

const lbr_cockpit_hal_t k_cockpit_hal = {
    .mono_us = h_mono,
    .send    = h_wifi_send,
    .log     = h_log,
};

const lbr_gw_hal_t k_gw_hal = {
    .mono_us         = h_mono,
    .now_us          = h_now,
    .local_to_shared = h_l2s,
    .can_send        = h_send,
    .can_stats       = h_stats,
    .sync_arm        = h_gw_sync_arm,
    .sd_present      = h_sd_present,
    .lora_ready      = h_gw_lora_ready,
    .lock            = h_gw_lock,
    .unlock          = h_gw_lock,
    .log             = h_log,
};

/* ------------------------------------------------------------------------ */
/* SD cards and log files                                                    */

void path_in_out(const sim_t *s, const char *name, char *out, size_t cap)
{
    snprintf(out, cap, "%s/%s", s->out_dir, name);
}

static int lf_write(void *ctx, const uint8_t *buf, size_t len)
{
    hal_ctx_t  *c = ctx;
    sim_node_t *n = &c->s->nodes[c->n];
    if (!n->sd_inserted || n->log_file == NULL) {
        sim_trace(c->s, c->n, "sd.write", "%u bytes -> NO CARD", (unsigned)len);
        return -1;
    }
    if (n->sd_fail_writes > 0) {
        n->sd_fail_writes--;
        sim_trace(c->s, c->n, "sd.write", "%u bytes -> I/O ERROR", (unsigned)len);
        return -1;
    }
    n->log_bytes += len;
    sim_trace(c->s, c->n, "sd.write", "%u bytes", (unsigned)len);
    return fwrite(buf, 1, len, n->log_file) == len ? 0 : -1;
}

static int lf_sync(void *ctx)
{
    hal_ctx_t  *c = ctx;
    sim_node_t *n = &c->s->nodes[c->n];
    if (!n->sd_inserted || n->log_file == NULL) {
        return -1;
    }
    sim_trace(c->s, c->n, "sd.fsync", "%s", n->log_name);
    return fflush(n->log_file) == 0 ? 0 : -1;
}

static int lf_close(void *ctx)
{
    hal_ctx_t  *c = ctx;
    sim_node_t *n = &c->s->nodes[c->n];
    if (n->log_file != NULL) {
        fclose(n->log_file);
        n->log_file = NULL;
    }
    return 0;
}

/* Log keeper HAL: a card that can be pulled or made to fail writes. */
static bool k_mount(void *ctx)
{
    hal_ctx_t *c  = ctx;
    bool       ok = c->s->nodes[c->n].sd_inserted;
    sim_trace(c->s, c->n, "sd.mount", "%s", ok ? "ok" : "no card");
    return ok;
}

static void k_unmount(void *ctx)
{
    lf_close(ctx);
}

static bool k_open_file(void *ctx, lbr_log_storage_t *st, char name[LBR_LOG_KEEPER_NAME_MAX])
{
    hal_ctx_t  *c = ctx;
    sim_t      *s = c->s;
    sim_node_t *n = &s->nodes[c->n];
    char        path[512];
    n->session_no++;
    if (n->is_gateway) {
        snprintf(n->log_name, sizeof n->log_name, "%08u.LBR", (unsigned)n->session_no);
    } else {
        snprintf(n->log_name, sizeof n->log_name, "%s-%04u.LBR", n->name, (unsigned)n->session_no);
    }
    path_in_out(s, n->log_name, path, sizeof path);
    n->log_file = fopen(path, "wb");
    if (n->log_file == NULL) {
        sim_note(s, "cannot create %s: %s", path, strerror(errno));
        return false;
    }
    snprintf(name, LBR_LOG_KEEPER_NAME_MAX, "%.*s", LBR_LOG_KEEPER_NAME_MAX - 1, n->log_name);
    *st = (lbr_log_storage_t){lf_write, lf_sync, lf_close, ctx};
    sim_trace(s, c->n, "sd.open", "%s", n->log_name);
    if (!s->quiet || n->is_gateway) {
        sim_note(s, "%s: SD session %s opened", n->name, n->log_name);
    }
    return true;
}

static void k_event(void *ctx, uint16_t code, bool fault)
{
    hal_ctx_t *c = ctx;
    sim_t     *s = c->s;
    if (c->n == GW) {
        lbr_gw_core_event(s->gw.core, (lbr_event_code_t)code, 0, fault);
    } else {
        lbr_node_core_on_log_event(s->nodes[c->n].core, code, fault);
    }
}

static const lbr_log_keeper_hal_t k_keeper_hal = {
    .mount     = k_mount,
    .unmount   = k_unmount,
    .open_file = k_open_file,
    .on_event  = k_event,
};

void keeper_start(sim_t *s, int i, lbr_logger_t *log, const char *config_text)
{
    sim_node_t          *n    = &s->nodes[i];
    lbr_log_keeper_hal_t hal  = k_keeper_hal;
    hal.ctx                   = &s_ctx[i];
    lbr_log_session_t session = {
        .fw_version   = LBR_FW_VERSION,
        .node_id      = n->id,
        .reset_reason = n->reset_reason,
        .boot_count   = n->boot_count,
        .config_text  = config_text,
    };
    lbr_log_keeper_init(&n->keeper, log, &hal, &session);
    /* As lbr_log_service_start: the first session opens before anything is logged. */
    lbr_log_keeper_step(&n->keeper, sim_local_us(s, i), h_now(&s_ctx[i]));
}

/* ------------------------------------------------------------------------ */
/* Gateway links                                                             */

int usb_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t    *s = self->ctx;
    uint8_t   raw[LBR_PKT_MAX_SIZE];
    uint8_t   enc[LBR_PKT_MAX_SIZE + LBR_PKT_MAX_SIZE / 254 + 4];
    sim_gw_t *g = &s->gw;
    if (len > sizeof raw) {
        g->usb_st.packets_dropped++;
        return -1;
    }
    memcpy(raw, pkt, len);
    lbr_pkt_restamp(raw, len, LBR_LINK_USB, g->usb_seq++);
    size_t n = lbr_cobs_encode(raw, len, enc, sizeof enc - 1);
    if (n == 0) {
        g->usb_st.packets_dropped++;
        return -1;
    }
    enc[n++] = 0x00;
    if (g->usb_file != NULL) {
        fwrite(enc, 1, n, g->usb_file);
    }
    g->usb_st.packets_sent++;
    g->usb_st.bytes_sent += (uint32_t)n;
    sim_trace(s, GW, "usb.write", "%u bytes (%u records)", (unsigned)n, (unsigned)raw[3]);
    return 0;
}

int wifi_send_pit(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t *s = self->ctx;
    (void)pkt;
    s->gw.wifi_st.packets_sent++;
    s->gw.wifi_st.bytes_sent += (uint32_t)len;
    return 0;
}

int lora_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t    *s = self->ctx;
    sim_gw_t *g = &s->gw;
    uint8_t   raw[LBR_PKT_LORA_MAX];
    if (len > sizeof raw) {
        g->lora_st.packets_dropped++;
        return -1;
    }
    memcpy(raw, pkt, len);
    lbr_pkt_restamp(raw, len, LBR_LINK_LORA, g->lora_seq++);
    uint32_t air          = lbr_lora_airtime_us(&g->modem, len);
    uint64_t now          = sim_local_us(s, GW);
    g->lora_next_tx_local = lbr_lora_next_tx_us(now, air, g->cfg.lora_duty_permille);
    g->lora_st.packets_sent++;
    g->lora_st.bytes_sent += (uint32_t)len;
    g->last_lora_records = raw[3];
    sim_trace(s, GW, "lora.tx", "%u bytes, %u records, %.1f ms on air", (unsigned)len,
              (unsigned)raw[3], (double)air / 1000.0);
    return 0;
}

/* The Central's cockpit datagrams (as the app's UDP link would send them). */
int cockpit_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t    *s       = self->ctx;
    sim_gw_t *g       = &s->gw;
    int       cockpit = s->nodes[GW].peer;
    if (cockpit < 0 || !wifi_send(s, GW, cockpit, pkt, len)) {
        g->cockpit_st.packets_dropped++;
        return -1;
    }
    g->cockpit_st.packets_sent++;
    g->cockpit_st.bytes_sent += (uint32_t)len;
    return 0;
}

void link_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    sim_t *s = self->ctx;
    if (self == &s->gw.usb) {
        *out = s->gw.usb_st;
    } else if (self == &s->gw.wifi) {
        *out = s->gw.wifi_st;
    } else if (self == &s->gw.cockpit) {
        *out = s->gw.cockpit_st;
    } else {
        *out = s->gw.lora_st;
    }
}

/* ------------------------------------------------------------------------ */
/* Virtual 5 GHz WiFi                                                        */

wifi_dgram_t s_wifi[WIFI_POOL];

/* The wireless end of a pair (the cockpit) owns the link (range, loss). */
static sim_wifi_t *link_of(sim_t *s, int a, int b)
{
    sim_node_t *na = &s->nodes[a], *nb = &s->nodes[b];
    return na->kind == SIM_NODE_COCKPIT ? &na->wifi : &nb->wifi;
}

bool wifi_send(sim_t *s, int from, int to, const uint8_t *buf, size_t len)
{
    if (to < 0 || len > LBR_PKT_MAX_SIZE) {
        return false;
    }
    sim_wifi_t *w = link_of(s, from, to);
    w->sent++;
    sim_trace(s, from, "udp.send", "%u bytes to %s", (unsigned)len, s->nodes[to].name);
    if (!w->up || (w->loss_permille > 0 && sim_rand(s) * 1000.0 < (double)w->loss_permille)) {
        w->dropped++; /* sent, never arrives: the receiver sees a seq gap */
        return true;
    }
    for (int i = 0; i < WIFI_POOL; i++) {
        if (!s_wifi[i].used) {
            s_wifi[i] = (wifi_dgram_t){.used = true, .from = from, .to = to, .len = len};
            memcpy(s_wifi[i].data, buf, len);
            uint64_t lat = WIFI_BASE_US + (uint64_t)(sim_rand(s) * 2000.0);
            sim_schedule(s, s->now + lat, EV_WIFI, to, i);
            return true;
        }
    }
    s->wifi_pool_full++;
    w->dropped++;
    return true;
}

void wifi_deliver(sim_t *s, int slot)
{
    wifi_dgram_t *d = &s_wifi[slot];
    d->used         = false;
    sim_node_t *n   = &s->nodes[d->to];
    if (!n->booted) {
        return; /* powered off: nothing listens */
    }
    link_of(s, d->from, d->to)->delivered++;
    uint64_t local = sim_local_us(s, d->to);
    sim_trace(s, d->to, "udp.recv", "%u bytes from %s", (unsigned)d->len, s->nodes[d->from].name);
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            lbr_gw_core_on_cockpit_packet(s->gw.core, d->data, d->len, local);
            break;
        case SIM_NODE_COCKPIT:
            lbr_cockpit_core_on_packet(n->cockpit, d->data, d->len, local);
            break;
        case SIM_NODE_CAN:
            break; /* no WiFi */
    }
}
