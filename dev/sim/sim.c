/*
 * Simulator core: event scheduler, per-node platforms (the HAL tables the
 * firmware cores call), virtual WiFi, the Central's SD card and links, the
 * command language, and JSON output for the UI.
 */
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "central_plan.h"
#include "corner_plan.h"
#include "dashboard_plan.h"
#include "driver_input_plan.h"
#include "imu_plan.h"
#include "lbr/cobs.h"
#include "lbr/dash_link.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/status.h"
#include "lbr/version.h"
#include "powertrain_plan.h"
#include "sim.h"
#include "stress_plan.h"
#include "temperature_plan.h"

#define GW            0 /* index of the Central in s->nodes */
#define BOOT_TIME_US  300000u
#define LOG_QUEUE     (64u * 1024u) /* CONFIG_LBR_LOG_QUEUE_KB on the gateway */
#define LOG_SYNC_MS   1000u
#define SYNC_TIMEOUT  50000u
#define WORLD_STEP_US 10000u
#define TRACE_BUDGET  400u
#define WIFI_POOL     512
#define WIFI_BASE_US  1000u /* one-way 5 GHz UDP latency, plus up to 2 ms jitter */

/* ------------------------------------------------------------------------ */
/* Small helpers                                                             */

double sim_rand(sim_t *s)
{
    /* xorshift64* */
    s->rng ^= s->rng >> 12;
    s->rng ^= s->rng << 25;
    s->rng ^= s->rng >> 27;
    return (double)((s->rng * 0x2545F4914F6CDD1DULL) >> 11) / 9007199254740992.0;
}

int sim_node_index(const sim_t *s, const char *name)
{
    for (int i = 0; i < s->n_nodes; i++) {
        if (strcmp(s->nodes[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

int sim_node_by_id(const sim_t *s, uint8_t id)
{
    for (int i = 0; i < s->n_nodes; i++) {
        if (s->nodes[i].id == id) {
            return i;
        }
    }
    return -1;
}

uint64_t sim_local_us(const sim_t *s, int node)
{
    const sim_node_t *n = &s->nodes[node];
    if (s->now <= n->boot_at) {
        return 0;
    }
    double el = (double)(s->now - n->boot_at);
    return (uint64_t)(el * (1.0 + n->ppm * 1e-6));
}

uint64_t sim_from_local(const sim_t *s, int node, uint64_t local_us)
{
    const sim_node_t *n = &s->nodes[node];
    return n->boot_at + (uint64_t)ceil((double)local_us / (1.0 + n->ppm * 1e-6));
}

/* JSON string body (names and messages are ASCII; escape the specials). */
static void json_str(FILE *o, const char *t)
{
    fputc('"', o);
    for (const char *p = t; *p != '\0'; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') {
            fputc('\\', o);
            fputc(c, o);
        } else if (c < 0x20 || c > 0x7E) {
            fprintf(o, "\\u%04x", c);
        } else {
            fputc(c, o);
        }
    }
    fputc('"', o);
}

static double t_s(const sim_t *s)
{
    return (double)s->now / 1e6;
}

void sim_trace(sim_t *s, int node, const char *cmd, const char *fmt, ...)
{
    if (s->quiet || node < 0 || !s->nodes[node].traced) {
        return;
    }
    if (s->trace_budget == 0) {
        s->trace_dropped++;
        return;
    }
    s->trace_budget--;
    char    args[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(args, sizeof args, fmt, ap);
    va_end(ap);
    fprintf(s->out, "{\"ev\":\"cmd\",\"t\":%.6f,\"node\":\"%s\",\"cmd\":", t_s(s),
            s->nodes[node].name);
    json_str(s->out, cmd);
    fprintf(s->out, ",\"args\":");
    json_str(s->out, args);
    fprintf(s->out, "}\n");
}

void sim_note(sim_t *s, const char *fmt, ...)
{
    char    text[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    if (s->quiet) {
        fprintf(s->out, "  %8.3f  %s\n", t_s(s), text);
        return;
    }
    fprintf(s->out, "{\"ev\":\"note\",\"t\":%.6f,\"text\":", t_s(s));
    json_str(s->out, text);
    fprintf(s->out, "}\n");
}

static const char *frame_desc(const lbr_can_frame_t *f, char *buf, size_t cap)
{
    const lbr_frame_def_t *d = lbr_frame_def((uint16_t)f->id);
    if (d != NULL) {
        snprintf(buf, cap, "%s", d->name);
    } else if (f->id == LBR_CAN_ID_TIME_SYNC) {
        snprintf(buf, cap, "TIME_SYNC");
    } else if (f->id == LBR_CAN_ID_TIME_FOLLOW_UP) {
        snprintf(buf, cap, "FOLLOW_UP");
    } else if (f->id == LBR_CAN_ID_COMMAND) {
        snprintf(buf, cap, "COMMAND");
    } else if (f->id > LBR_CAN_ID_EVENT_BASE && f->id <= LBR_CAN_ID_EVENT_BASE + LBR_NODE_CAN_MAX) {
        snprintf(buf, cap, "%s", (f->data[4] & 1u) ? "FAULT" : "EVENT");
    } else if (f->id > LBR_CAN_ID_HEARTBEAT_BASE && f->id < LBR_CAN_ID_HEALTH_BASE) {
        snprintf(buf, cap, "HEARTBEAT");
    } else if (f->id >= LBR_CAN_ID_HEALTH_BASE) {
        snprintf(buf, cap, "HEALTH p%u", (unsigned)(f->id & 0xFu));
    } else {
        snprintf(buf, cap, "id 0x%03lx", (unsigned long)f->id);
    }
    return buf;
}

/* ------------------------------------------------------------------------ */
/* Event queue (binary heap)                                                 */

typedef struct {
    uint64_t t;
    uint64_t order; /* FIFO among equal times */
    uint8_t  type;
    int8_t   node;
    int32_t  slot;
    uint32_t gen;
} event_t;

static event_t *s_heap;
static size_t   s_heap_len, s_heap_cap;
static uint64_t s_order;

static bool ev_less(const event_t *a, const event_t *b)
{
    return a->t < b->t || (a->t == b->t && a->order < b->order);
}

void sim_schedule(sim_t *s, uint64_t t, sim_ev_type_t type, int node, int slot)
{
    if (s_heap_len == s_heap_cap) {
        s_heap_cap = s_heap_cap ? s_heap_cap * 2 : 1024;
        s_heap     = realloc(s_heap, s_heap_cap * sizeof *s_heap);
        if (s_heap == NULL) {
            fprintf(stderr, "sim: out of memory\n");
            exit(2);
        }
    }
    event_t e = {
        .t     = t < s->now ? s->now : t,
        .order = s_order++,
        .type  = (uint8_t)type,
        .node  = (int8_t)node,
        .slot  = slot,
        .gen   = node >= 0 ? s->nodes[node].gen : 0,
    };
    size_t i = s_heap_len++;
    while (i > 0) {
        size_t p = (i - 1) / 2;
        if (!ev_less(&e, &s_heap[p])) {
            break;
        }
        s_heap[i] = s_heap[p];
        i         = p;
    }
    s_heap[i] = e;
}

static event_t pop_event(void)
{
    event_t top  = s_heap[0];
    event_t last = s_heap[--s_heap_len];
    size_t  i    = 0;
    for (;;) {
        size_t         l = 2 * i + 1, r = l + 1, m = i;
        const event_t *mv = &last;
        if (l < s_heap_len && ev_less(&s_heap[l], mv)) {
            m  = l;
            mv = &s_heap[l];
        }
        if (r < s_heap_len && ev_less(&s_heap[r], mv)) {
            m = r;
        }
        if (m == i) {
            break;
        }
        s_heap[i] = s_heap[m];
        i         = m;
    }
    if (s_heap_len > 0) {
        s_heap[i] = last;
    }
    return top;
}

/* ------------------------------------------------------------------------ */
/* Per-node platform: the HAL the firmware cores call                        */

typedef struct {
    sim_t *s;
    int    n;
} hal_ctx_t;

static hal_ctx_t s_ctx[SIM_MAX_NODES];

static uint64_t h_mono(void *ctx)
{
    hal_ctx_t *c = ctx;
    return sim_local_us(c->s, c->n);
}

static uint64_t h_now(void *ctx)
{
    hal_ctx_t *c = ctx;
    return lbr_time_servo_now(&c->s->nodes[c->n].servo, sim_local_us(c->s, c->n));
}

static uint64_t h_l2s(void *ctx, uint64_t local)
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
    return wifi_send(c->s, c->n, c->s->nodes[c->n].peer, pkt, len);
}

/* IMU Controller: its corner receiver's platform. */
static uint16_t h_epoch(void *ctx)
{
    hal_ctx_t *c = ctx;
    return c->s->nodes[c->n].servo.epoch;
}

static bool h_rx_send_frame(void *ctx, const lbr_can_frame_t *f)
{
    hal_ctx_t *c = ctx;
    return lbr_node_core_send(c->s->nodes[c->n].core, f);
}

static bool h_rx_reply(void *ctx, uint8_t node, const uint8_t *pkt, size_t len)
{
    hal_ctx_t *c  = ctx;
    int        to = sim_node_by_id(c->s, node);
    return to >= 0 && wifi_send(c->s, c->n, to, pkt, len);
}

static void h_rx_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    hal_ctx_t *c = ctx;
    lbr_node_core_event(c->s->nodes[c->n].core, code, arg, fault);
}

static const lbr_node_hal_t k_node_hal = {
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

static const lbr_corner_hal_t k_corner_hal = {
    .mono_us    = h_mono,
    .now_us     = h_now,
    .time_servo = h_servo,
    .time_apply = h_apply,
    .send       = h_wifi_send,
    .log        = h_log,
};

static const lbr_dash_hal_t k_dash_hal = {
    .mono_us = h_mono,
    .send    = h_wifi_send,
    .log     = h_log,
};

static const lbr_corner_rx_hal_t k_rx_hal = {
    .now_us     = h_now,
    .synced     = h_synced,
    .epoch      = h_epoch,
    .send_frame = h_rx_send_frame,
    .reply      = h_rx_reply,
    .event      = h_rx_event,
};

static const lbr_gw_hal_t k_gw_hal = {
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

static void path_in_out(const sim_t *s, const char *name, char *out, size_t cap)
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

static void keeper_start(sim_t *s, int i, lbr_logger_t *log, const char *config_text)
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

static int usb_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
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

static int wifi_send_pit(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t *s = self->ctx;
    (void)pkt;
    s->gw.wifi_st.packets_sent++;
    s->gw.wifi_st.bytes_sent += (uint32_t)len;
    return 0;
}

static int lora_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
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

/* The Central's dashboard datagrams (as the app's UDP link would send them). */
static int dash_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    sim_t    *s    = self->ctx;
    sim_gw_t *g    = &s->gw;
    int       dash = s->nodes[GW].peer;
    if (dash < 0 || !wifi_send(s, GW, dash, pkt, len)) {
        g->dash_st.packets_dropped++;
        return -1;
    }
    g->dash_st.packets_sent++;
    g->dash_st.bytes_sent += (uint32_t)len;
    return 0;
}

static void link_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    sim_t *s = self->ctx;
    if (self == &s->gw.usb) {
        *out = s->gw.usb_st;
    } else if (self == &s->gw.wifi) {
        *out = s->gw.wifi_st;
    } else if (self == &s->gw.dash) {
        *out = s->gw.dash_st;
    } else {
        *out = s->gw.lora_st;
    }
}

/* ------------------------------------------------------------------------ */
/* Virtual 5 GHz WiFi                                                        */

typedef struct {
    bool    used;
    int     from, to;
    size_t  len;
    uint8_t data[LBR_PKT_MAX_SIZE];
} wifi_dgram_t;

static wifi_dgram_t s_wifi[WIFI_POOL];

/* The wireless end of a pair owns the link (range, loss). */
static sim_wifi_t *link_of(sim_t *s, int a, int b)
{
    sim_node_t *na = &s->nodes[a], *nb = &s->nodes[b];
    if (na->kind == SIM_NODE_CORNER || na->kind == SIM_NODE_DASH) {
        return &na->wifi;
    }
    return &nb->wifi;
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

static void wifi_deliver(sim_t *s, int slot)
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
            lbr_gw_core_on_dash_packet(s->gw.core, d->data, d->len, local);
            break;
        case SIM_NODE_CAN:
            if (n->corner_rx != NULL) {
                lbr_corner_rx_on_packet(n->corner_rx, d->data, d->len, h_now(&s_ctx[d->to]));
            }
            break;
        case SIM_NODE_CORNER:
            lbr_corner_core_on_packet(n->corner, d->data, d->len, local);
            break;
        case SIM_NODE_DASH:
            lbr_dash_core_on_packet(n->dash, d->data, d->len, local);
            break;
    }
}

/* ------------------------------------------------------------------------ */
/* Boot                                                                      */

static uint64_t period_us(uint16_t rate_hz)
{
    return 1000000u / (rate_hz ? rate_hz : 1u);
}

static void schedule_local(sim_t *s, int node, int job, uint64_t local, sim_ev_type_t type)
{
    s->nodes[node].next_local[job] = local;
    sim_schedule(s, sim_from_local(s, node, local), type, node, job);
}

/* Wake slot k at its next grid point (shared time), as node_esp.c does. */
static void schedule_acq(sim_t *s, int ni, size_t k)
{
    sim_node_t *n     = &s->nodes[ni];
    uint64_t    local = sim_local_us(s, ni);
    uint64_t    now, due;
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            now = h_now(&s_ctx[ni]);
            due = lbr_gw_core_slot_next_due(s->gw.core, k);
            break;
        case SIM_NODE_CORNER:
            now = h_now(&s_ctx[ni]);
            due = lbr_corner_core_slot_next_due(n->corner, k);
            break;
        case SIM_NODE_DASH:
            now = local; /* the dashboard keeps no shared clock */
            due = lbr_dash_core_slot_next_due(n->dash, k);
            break;
        default:
            now = h_now(&s_ctx[ni]);
            due = lbr_node_core_slot_next_due(n->core, k);
            break;
    }
    schedule_local(s, ni, (int)k, local + (due > now ? due - now : 0u), EV_ACQ);
}

static void reset_can(sim_node_t *n, bool accept_all)
{
    bool   connected = n->can.connected;
    size_t cap       = n->can.tx_cap;
    memset(&n->can, 0, sizeof n->can);
    n->can.connected  = connected;
    n->can.started    = true;
    n->can.accept_all = accept_all;
    n->can.tx_cap     = cap;
    n->can.st.running = true;
}

static void boot_gateway(sim_t *s)
{
    sim_node_t *n = &s->nodes[GW];
    sim_gw_t   *g = &s->gw;
    lbr_time_servo_init(&n->servo);
    lbr_time_servo_set_master(&n->servo);
    reset_can(n, true);
    world_bind_sensors(s, GW);

    g->epoch = (uint16_t)((uint16_t)(sim_rand(s) * 65535.0) | 1u);
    lbr_gw_config_defaults(&g->cfg);
    g->cfg.ble_enable = false; /* no BLE model */
    int len = snprintf(g->config_text, sizeof g->config_text,
                       "node=central\nnode_id=%u\nfw_version=%u.%u.%u\nsimulator=1\nseed=%llu\n"
                       "protocol_version=%u\ncatalog_hash=%08lx\ncan_bitrate=%lu\n"
                       "time_sync_hz=%u\n",
                       LBR_NODE_CENTRAL, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
                       LBR_FW_VERSION_PATCH, (unsigned long long)s->seed, LBR_PROTOCOL_VERSION,
                       (unsigned long)lbr_catalog_hash(), (unsigned long)s->bus.bitrate,
                       LBR_TIME_SYNC_HZ);
    if (len > 0 && (size_t)len < sizeof g->config_text) {
        lbr_gw_config_format(&g->cfg, g->config_text + len, sizeof g->config_text - (size_t)len);
    }

    lbr_log_config_t lc = {
        .queue_mem        = g->log_queue,
        .queue_cap        = LOG_QUEUE,
        .chunk            = g->log_chunk,
        .chunk_cap        = LBR_LOG_MAX_RECORD + 512u,
        .sync_interval_ms = LOG_SYNC_MS,
    };
    lbr_log_init(&g->log, &lc);

    g->usb   = (lbr_transport_t){"usb", LBR_LINK_USB, NULL, usb_send, link_stats, s};
    g->wifi  = (lbr_transport_t){"wifi", LBR_LINK_WIFI, NULL, wifi_send_pit, link_stats, s};
    g->lora  = (lbr_transport_t){"lora", LBR_LINK_LORA, NULL, lora_send, link_stats, s};
    g->dash  = (lbr_transport_t){"dash", LBR_LINK_DASH, NULL, dash_send, link_stats, s};
    g->modem = (lbr_lora_modem_t){.sf = 9, .bw_hz = 125000, .cr = 1, .preamble = 8, .crc = true};
    g->lora_next_tx_local = 0;
    g->sync_pending       = false;

    lbr_gw_boot_t boot = {
        .reset_reason = (lbr_reset_reason_t)n->reset_reason,
        .boot_count   = n->boot_count,
        .epoch        = g->epoch,
        .config_valid = true,
        .config_text  = g->config_text,
        .acq          = n->acq,
        .n_acq        = n->cfg.n_acq,
    };
    if (g->core == NULL) {
        g->core = calloc(1, sizeof *g->core);
    }
    lbr_gw_core_init(g->core, &k_gw_hal, &g->cfg, &boot);
    g->core->hal.ctx = &s_ctx[GW];
    /* Same order as the Central app: core, its own link-state driver, the
     * log service, then start. */
    lbr_sensor_dash_link(&n->sensors[CENTRAL_SLOT_DASH_STATUS], "dash_status",
                         n->plan->slots[CENTRAL_SLOT_DASH_STATUS].signals,
                         n->plan->slots[CENTRAL_SLOT_DASH_STATUS].n_signals, &g->core->dash,
                         lbr_gw_core_lock_state, lbr_gw_core_unlock_state, g->core);
    keeper_start(s, GW, &g->log, g->config_text);
    lbr_gw_links_t links = {
        .usb  = g->cfg.usb_enable ? &g->usb : NULL,
        .wifi = g->cfg.wifi_enable ? &g->wifi : NULL,
        .lora = g->cfg.lora_enable ? &g->lora : NULL,
        .dash = g->cfg.dash_enable ? &g->dash : NULL,
    };
    lbr_gw_core_start(g->core, &g->log, &links, true, false);

    uint64_t l = sim_local_us(s, GW);
    schedule_local(s, GW, SIM_JOB_SYNC, l + 1000000u / LBR_TIME_SYNC_HZ, EV_GW_SYNC);
    schedule_local(s, GW, SIM_JOB_STREAM, l + g->cfg.stream_batch_ms * 1000u, EV_GW_STREAM);
    schedule_local(s, GW, SIM_JOB_LORA, l + g->cfg.lora_period_ms * 1000u, EV_GW_LORA);
    schedule_local(s, GW, SIM_JOB_PUMP, l + 20000u, EV_GW_PUMP);
    schedule_local(s, GW, SIM_JOB_DASH, l + g->cfg.dash_period_ms * 1000u, EV_GW_DASH);
    schedule_local(s, GW, SIM_JOB_HEALTH, l + LBR_GW_HEALTH_PERIOD_MS * 1000u, EV_HEALTH);
    for (size_t k = 0; k < n->cfg.n_acq; k++) {
        lbr_gw_core_slot_start(g->core, k);
        schedule_acq(s, GW, k);
    }
}

static void start_local_log(sim_t *s, int node)
{
    sim_node_t    *n = &s->nodes[node];
    static uint8_t q[96u * 1024u];
    static uint8_t chunk[LBR_LOG_MAX_RECORD + 512u];
    if (n->local_log == NULL) {
        n->local_log = calloc(1, sizeof *n->local_log);
    }
    lbr_log_config_t lc = {
        .queue_mem        = q,
        .queue_cap        = sizeof q,
        .chunk            = chunk,
        .chunk_cap        = sizeof chunk,
        .sync_interval_ms = LOG_SYNC_MS,
    };
    lbr_log_init(n->local_log, &lc);
    keeper_start(s, node, n->local_log, n->core->config_text);
}

static void boot_can_node(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    reset_can(n, n->plan->listen_all);
    world_bind_sensors(s, i);
    if (n->id == LBR_NODE_IMU) {
        /* As the IMU Controller app: the corner receiver feeds a slot. */
        if (n->corner_rx == NULL) {
            n->corner_rx = calloc(1, sizeof *n->corner_rx);
        }
        lbr_corner_rx_hal_t rh = k_rx_hal;
        rh.ctx                 = &s_ctx[i];
        lbr_corner_rx_init(n->corner_rx, &rh);
        const lbr_slot_t *st = &n->plan->slots[IMU_SLOT_CORNER_STATUS];
        lbr_sensor_corner_status(&n->sensors[IMU_SLOT_CORNER_STATUS], st->name, st->signals,
                                 st->n_signals, n->corner_rx);
    }
    if (n->core == NULL) {
        n->core = calloc(1, sizeof *n->core);
    }
    lbr_node_hal_t  hal  = k_node_hal;
    lbr_node_boot_t boot = {
        .reset_reason = (lbr_reset_reason_t)n->reset_reason,
        .boot_count   = n->boot_count,
        .can_bitrate  = s->bus.bitrate,
    };
    hal.ctx    = &s_ctx[i];
    bool valid = lbr_node_core_init(n->core, &n->cfg, &hal, &boot);
    if (n->plan->local_log) {
        start_local_log(s, i);
    }
    lbr_node_core_start(n->core, true, false, n->plan->local_log ? n->local_log : NULL);

    uint64_t l = sim_local_us(s, i);
    if (valid) {
        for (size_t k = 0; k < n->cfg.n_acq; k++) {
            lbr_node_core_slot_start(n->core, k);
            schedule_acq(s, i, k);
        }
    }
    schedule_local(s, i, SIM_JOB_HEALTH, l + n->cfg.health_period_ms * 1000u, EV_HEALTH);
    if (n->local_log != NULL) {
        schedule_local(s, i, SIM_JOB_PUMP, l + 20000u, EV_GW_PUMP); /* LOG_PUMP_MS on target */
    }
    if (n->corner_rx != NULL) {
        schedule_local(s, i, SIM_JOB_AUX, l + LBR_CORNER_SERVICE_MS * 1000u, EV_CORNER_RX);
    }
}

static void boot_corner(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    world_bind_sensors(s, i);
    if (n->corner == NULL) {
        n->corner = calloc(1, sizeof *n->corner);
    }
    lbr_corner_hal_t hal = k_corner_hal;
    hal.ctx              = &s_ctx[i];
    if (!lbr_corner_core_init(n->corner, &n->cfg, &hal, LBR_CORNER_DEFAULT_BATCH)) {
        return;
    }
    uint64_t l = sim_local_us(s, i);
    for (size_t k = 0; k < n->cfg.n_acq; k++) {
        lbr_corner_core_slot_start(n->corner, k);
        schedule_acq(s, i, k);
    }
    schedule_local(s, i, SIM_JOB_AUX, l + 100000u, EV_CORNER_CLOCK);
}

static void boot_dash(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    world_bind_sensors(s, i);
    if (n->dash == NULL) {
        n->dash = calloc(1, sizeof *n->dash);
    }
    lbr_dash_hal_t hal = k_dash_hal;
    hal.ctx            = &s_ctx[i];
    lbr_dash_core_init(n->dash, &n->cfg, &hal);
    for (size_t k = 0; k < n->cfg.n_acq; k++) {
        lbr_dash_core_slot_start(n->dash, k);
        schedule_acq(s, i, k);
    }
}

static void boot_node(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    n->booted     = true;
    n->boot_at    = s->now;
    n->boot_count++;
    lbr_time_servo_init(&n->servo);
    for (int k = 0; k < s->n_nodes; k++) {
        s_ctx[k] = (hal_ctx_t){s, k};
    }
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            boot_gateway(s);
            break;
        case SIM_NODE_CORNER:
            boot_corner(s, i);
            break;
        case SIM_NODE_DASH:
            boot_dash(s, i);
            break;
        default:
            boot_can_node(s, i);
            break;
    }
}

static void power_off(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    n->booted     = false;
    n->gen++; /* drop its pending timers */
    n->can.started    = false;
    n->can.st.running = false;
    n->can.tx_len     = 0;
    if (n->log_file != NULL) {
        /* Power loss: no END record; whatever was still queued is gone. */
        fclose(n->log_file);
        n->log_file = NULL;
    }
}

/* ------------------------------------------------------------------------ */
/* Bus callbacks                                                             */

/* What a log reader decodes from the bus (the Central's log holds exactly
 * these frames): latest value per signal, events per node. */
static void record_bus_frame(sim_t *s, const lbr_can_frame_t *f, uint64_t gw_shared)
{
    lbr_sample_t smp[LBR_FRAME_MAX_FIELDS];
    size_t       n = lbr_bus_decode(&s->decoder, f, gw_shared, smp, LBR_FRAME_MAX_FIELDS);
    for (size_t k = 0; k < n; k++) {
        const lbr_sample_t *s2 = &smp[k];
        if (s2->signal >= SIM_MAX_SIGNALS) {
            continue;
        }
        sim_value_t *v = &s->values[s2->signal];
        v->seen        = true;
        v->s           = *s2;
        v->rx_t        = s->now;
        v->source      = s2->source;
        if (s2->signal != LBR_SIG_EVENT && s2->signal != LBR_SIG_FAULT) {
            continue;
        }
        uint16_t code = (uint16_t)(s2->v.u32 & 0xFFFFu);
        uint16_t arg  = (uint16_t)(s2->v.u32 >> 16);
        int      src  = sim_node_by_id(s, s2->source);
        if (src >= 0) {
            s->event_counts[src][code & 0x7Fu]++;
        }
        const char *name = lbr_event_name((lbr_event_code_t)code);
        if (s->quiet) {
            continue;
        }
        fprintf(s->out,
                "{\"ev\":\"event\",\"t\":%.6f,\"node\":\"%s\",\"name\":\"%s\",\"arg\":%u,"
                "\"fault\":%s}\n",
                t_s(s), lbr_node_name(s2->source), name != NULL ? name : "?", (unsigned)arg,
                s2->signal == LBR_SIG_FAULT ? "true" : "false");
    }
}

void sim_deliver(sim_t *s, int node, const lbr_can_frame_t *f, uint64_t t)
{
    (void)t;
    sim_node_t *n = &s->nodes[node];
    if (!n->booted) {
        return;
    }
    uint64_t local = sim_local_us(s, node);
    if (n->is_gateway) {
        record_bus_frame(s, f, h_l2s(&s_ctx[GW], local));
        lbr_gw_core_on_rx(s->gw.core, f, local);
    } else if (n->core != NULL) {
        lbr_node_core_on_frame(n->core, f, local);
    }
}

void sim_tx_done(sim_t *s, int node, const lbr_can_frame_t *f, bool ok, uint64_t t)
{
    (void)t;
    if (node != GW) {
        return;
    }
    record_bus_frame(s, f, h_now(&s_ctx[GW]));
    uint8_t  seq;
    uint16_t epoch;
    if (ok && s->gw.sync_pending && lbr_can_decode_time_sync(f, &seq, &epoch) &&
        seq == s->gw.sync_seq) {
        s->gw.sync_pending = false;
        uint64_t local     = sim_local_us(s, GW);
        sim_trace(s, GW, "twai.tx_done", "TIME_SYNC %u finished at local %llu us", (unsigned)seq,
                  (unsigned long long)local);
        lbr_gw_core_sync_done(s->gw.core, true, local);
    }
}

/* ------------------------------------------------------------------------ */
/* Event dispatch                                                            */

static void acq_tick(sim_t *s, int ni, size_t k)
{
    sim_node_t *n = &s->nodes[ni];
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            lbr_gw_core_slot_tick(s->gw.core, k);
            break;
        case SIM_NODE_CORNER:
            lbr_corner_core_slot_tick(n->corner, k);
            break;
        case SIM_NODE_DASH:
            lbr_dash_core_slot_tick(n->dash, k);
            break;
        default:
            lbr_node_core_slot_tick(n->core, k);
            break;
    }
}

static void handle(sim_t *s, const event_t *e)
{
    if (e->type == EV_WIFI) {
        wifi_deliver(s, e->slot); /* frees the pool slot whatever the receiver's state */
        return;
    }
    if (e->node >= 0) {
        sim_node_t *n     = &s->nodes[e->node];
        bool        stale = e->gen != n->gen && e->type != EV_BOOT && e->type != EV_BUS_OFF_END;
        if (stale || (!n->booted && e->type != EV_BOOT)) {
            return;
        }
    }
    sim_node_t *n  = e->node >= 0 ? &s->nodes[e->node] : NULL;
    sim_gw_t   *g  = &s->gw;
    int         ni = e->node;

    switch ((sim_ev_type_t)e->type) {
        case EV_BOOT:
            if (n->powered && !n->booted && e->gen == n->gen) {
                boot_node(s, ni);
            }
            break;
        case EV_ACQ:
            acq_tick(s, ni, (size_t)e->slot);
            schedule_acq(s, ni, (size_t)e->slot);
            break;
        case EV_HEALTH:
            if (n->is_gateway) {
                lbr_gw_core_heartbeat(g->core, &n->last_health);
                schedule_local(s, ni, SIM_JOB_HEALTH,
                               n->next_local[SIM_JOB_HEALTH] + LBR_GW_HEALTH_PERIOD_MS * 1000u,
                               EV_HEALTH);
            } else {
                lbr_node_core_heartbeat(n->core, &n->last_health);
                schedule_local(s, ni, SIM_JOB_HEALTH,
                               n->next_local[SIM_JOB_HEALTH] + n->cfg.health_period_ms * 1000u,
                               EV_HEALTH);
            }
            break;
        case EV_GW_SYNC:
            if (g->sync_pending) {
                /* Previous one still waiting: hardware would still block. */
            } else {
                g->sync_seq     = lbr_gw_core_sync_send(g->core);
                g->sync_pending = true;
                sim_schedule(s, s->now + SYNC_TIMEOUT, EV_GW_SYNC_TIMEOUT, GW, g->sync_seq);
            }
            schedule_local(s, GW, SIM_JOB_SYNC,
                           n->next_local[SIM_JOB_SYNC] + 1000000u / LBR_TIME_SYNC_HZ, EV_GW_SYNC);
            break;
        case EV_GW_SYNC_TIMEOUT:
            if (g->sync_pending && (uint8_t)e->slot == g->sync_seq) {
                g->sync_pending = false;
                g->sync_timeouts++;
                sim_trace(s, GW, "twai.tx_done", "TIME_SYNC %u not sent within 50 ms",
                          (unsigned)g->sync_seq);
                lbr_gw_core_sync_done(g->core, false, 0);
            }
            break;
        case EV_GW_STREAM:
            lbr_gw_core_stream_flush(g->core);
            schedule_local(s, GW, SIM_JOB_STREAM,
                           n->next_local[SIM_JOB_STREAM] + g->cfg.stream_batch_ms * 1000u,
                           EV_GW_STREAM);
            break;
        case EV_GW_LORA:
            lbr_gw_core_lora_tick(g->core);
            schedule_local(s, GW, SIM_JOB_LORA,
                           n->next_local[SIM_JOB_LORA] + g->cfg.lora_period_ms * 1000u, EV_GW_LORA);
            break;
        case EV_GW_DASH:
            lbr_gw_core_dash_tick(g->core);
            schedule_local(s, GW, SIM_JOB_DASH,
                           n->next_local[SIM_JOB_DASH] + g->cfg.dash_period_ms * 1000u, EV_GW_DASH);
            break;
        case EV_GW_PUMP:
            lbr_log_keeper_step(&n->keeper, sim_local_us(s, ni), h_now(&s_ctx[ni]));
            schedule_local(s, ni, SIM_JOB_PUMP, n->next_local[SIM_JOB_PUMP] + 20000u, EV_GW_PUMP);
            break;
        case EV_CORNER_CLOCK: {
            uint32_t ms = lbr_corner_core_clock_tick(n->corner);
            schedule_local(s, ni, SIM_JOB_AUX, n->next_local[SIM_JOB_AUX] + ms * 1000u,
                           EV_CORNER_CLOCK);
            break;
        }
        case EV_CORNER_RX:
            lbr_corner_rx_service(n->corner_rx, n->core);
            schedule_local(s, ni, SIM_JOB_AUX,
                           n->next_local[SIM_JOB_AUX] + LBR_CORNER_SERVICE_MS * 1000u,
                           EV_CORNER_RX);
            break;
        case EV_BUS:
            vbus_event(s, e->slot);
            break;
        case EV_BUS_OFF_END:
            if ((uint32_t)e->slot == n->gen) {
                vbus_recover(s, ni);
            }
            break;
        case EV_WORLD:
            world_step(s, WORLD_STEP_US / 1e6);
            sim_schedule(s, s->now + WORLD_STEP_US, EV_WORLD, -1, 0);
            break;
        case EV_WIFI:
            break;
    }
}

void sim_run_until(sim_t *s, uint64_t t_us)
{
    while (s_heap_len > 0 && s_heap[0].t <= t_us) {
        event_t e = pop_event();
        s->now    = e.t;
        handle(s, &e);
    }
    s->now = t_us;
}

/* ------------------------------------------------------------------------ */
/* Setup                                                                     */

typedef struct {
    const char            *name;
    uint8_t                id;
    sim_node_kind_t        kind;
    const lbr_node_plan_t *plan;
    bool                   c5;     /* ESP32-C5 (else classic ESP32) */
    size_t                 tx_cap; /* CONFIG_LBR_CAN_TX_QUEUE_LEN */
} node_def_t;

static void make_dir(const char *path)
{
#ifdef _WIN32
    mkdir(path);
#else
    mkdir(path, 0755);
#endif
}

/* Create the output directory and its parents (errors surface when the
 * first file cannot be created). */
static void mkdir_p(const char *path)
{
    char buf[512];
    snprintf(buf, sizeof buf, "%s", path);
    for (char *p = buf + 1; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') {
            char keep = *p;
            *p        = '\0';
            make_dir(buf);
            *p = keep;
        }
    }
    make_dir(buf);
}

int sim_init(sim_t *s, uint64_t seed, const char *out_dir, FILE *out, bool quiet)
{
    /* DAQ-WIRING.md: the Central first (index GW), the CAN controllers in
     * their daisy-chain order, then the wireless nodes. */
    const node_def_t defs[] = {
        {"central", LBR_NODE_CENTRAL, SIM_NODE_CENTRAL, &lbr_plan_central, true, 64},
        {"imu", LBR_NODE_IMU, SIM_NODE_CAN, &lbr_plan_imu, true, 127},
        {"powertrain", LBR_NODE_POWERTRAIN, SIM_NODE_CAN, &lbr_plan_powertrain, false, 64},
        {"driver_input", LBR_NODE_DRIVER_INPUT, SIM_NODE_CAN, &lbr_plan_driver_input, false, 64},
        {"temperature", LBR_NODE_TEMPERATURE, SIM_NODE_CAN, &lbr_plan_temperature, false, 64},
        {"stress", LBR_NODE_STRESS, SIM_NODE_CAN, &lbr_plan_stress, false, 64},
        {"dashboard", LBR_NODE_DASHBOARD, SIM_NODE_DASH, &lbr_plan_dashboard, true, 0},
        {"corner_fl", LBR_NODE_CORNER_FL, SIM_NODE_CORNER, &lbr_plan_corner[0], true, 0},
        {"corner_fr", LBR_NODE_CORNER_FR, SIM_NODE_CORNER, &lbr_plan_corner[1], true, 0},
        {"corner_bl", LBR_NODE_CORNER_BL, SIM_NODE_CORNER, &lbr_plan_corner[2], true, 0},
        {"corner_br", LBR_NODE_CORNER_BR, SIM_NODE_CORNER, &lbr_plan_corner[3], true, 0},
    };
    memset(s, 0, sizeof *s);
    memset(s_wifi, 0, sizeof s_wifi);
    s->seed    = seed;
    s->rng     = seed * 0x9E3779B97F4A7C15ULL + 1u;
    s->out_dir = out_dir;
    s->out     = out;
    s->quiet   = quiet;
    s->n_nodes = (int)(sizeof defs / sizeof defs[0]);
    vbus_init(s, 500000u); /* CONFIG_LBR_CAN_BITRATE */
    world_init(&s->world);
    lbr_bus_decoder_init(&s->decoder);

    mkdir_p(out_dir);

    s->gw.log_queue = malloc(LOG_QUEUE);
    s->gw.log_chunk = malloc(LBR_LOG_MAX_RECORD + 512u);
    char path[512];
    path_in_out(s, "usb_stream.bin", path, sizeof path);
    s->gw.usb_file = fopen(path, "wb");
    if (s->gw.log_queue == NULL || s->gw.log_chunk == NULL) {
        return -1;
    }

    int imu = -1, dash = -1;
    for (int i = 0; i < s->n_nodes; i++) {
        sim_node_t *n    = &s->nodes[i];
        n->name          = defs[i].name;
        n->id            = defs[i].id;
        n->kind          = defs[i].kind;
        n->plan          = defs[i].plan;
        n->chip          = defs[i].c5 ? &lbr_io_chip_esp32c5 : &lbr_io_chip_esp32;
        n->is_gateway    = i == GW;
        n->powered       = true;
        n->can.connected = defs[i].tx_cap > 0;
        n->can.tx_cap    = defs[i].tx_cap;
        n->sd_inserted   = true;
        n->reset_reason  = LBR_RESET_POWER_ON;
        n->peer          = -1;
        n->wifi.up       = true;
        imu              = n->id == LBR_NODE_IMU ? i : imu;
        dash             = n->id == LBR_NODE_DASHBOARD ? i : dash;
        /* Real crystals: within +/- 40 ppm. */
        n->ppm = (sim_rand(s) - 0.5) * 80.0;
        /* Power comes up together; boot times differ a little. */
        uint64_t at = n->is_gateway ? 0 : 50000u + (uint64_t)(sim_rand(s) * 400000.0);
        sim_schedule(s, at, EV_BOOT, i, 0);
    }
    /* WiFi pairs: corners talk to the IMU Controller, the dashboard to the Central. */
    for (int i = 0; i < s->n_nodes; i++) {
        if (s->nodes[i].kind == SIM_NODE_CORNER) {
            s->nodes[i].peer = imu;
        }
    }
    s->nodes[dash].peer = GW;
    s->nodes[GW].peer   = dash;
    sim_schedule(s, 0, EV_WORLD, -1, 0);
    return 0;
}

void sim_free(sim_t *s)
{
    for (int i = 0; i < s->n_nodes; i++) {
        sim_node_t *n = &s->nodes[i];
        if (n->booted && n->keeper.log != NULL) {
            lbr_log_keeper_close(&n->keeper, h_now(&s_ctx[i])); /* clean shutdown: END record */
        }
        if (n->log_file != NULL) {
            fclose(n->log_file);
            n->log_file = NULL;
        }
    }
    if (s->gw.usb_file != NULL) {
        fclose(s->gw.usb_file);
    }
    for (int i = 0; i < s->n_nodes; i++) {
        free(s->nodes[i].core);
        free(s->nodes[i].local_log);
        free(s->nodes[i].corner_rx);
        free(s->nodes[i].corner);
        free(s->nodes[i].dash);
    }
    free(s->gw.core);
    free(s->gw.log_queue);
    free(s->gw.log_chunk);
    free(s_heap);
    s_heap     = NULL;
    s_heap_len = s_heap_cap = 0;
}

/* ------------------------------------------------------------------------ */
/* Metrics (shared by expect and the snapshot)                               */

static int64_t sync_error_us(sim_t *s, int i)
{
    if (!s->nodes[i].booted || !s->nodes[GW].booted || !s->nodes[i].servo.synced) {
        return 0;
    }
    uint64_t est   = lbr_time_servo_map(&s->nodes[i].servo, sim_local_us(s, i));
    uint64_t truth = sim_local_us(s, GW); /* shared time is the Central's clock */
    return (int64_t)(est - truth);
}

static bool wireless(const sim_node_t *n)
{
    return n->kind == SIM_NODE_CORNER || n->kind == SIM_NODE_DASH;
}

/* A wireless node as its receiver sees it: up while its datagrams arrive. */
static bool link_up(const sim_t *s, int i)
{
    const sim_node_t *n = &s->nodes[i];
    if (n->kind == SIM_NODE_DASH) {
        return s->gw.core != NULL && s->nodes[GW].booted && s->gw.core->dash.up;
    }
    int imu = n->peer;
    int c   = lbr_node_corner_index(n->id);
    return imu >= 0 && s->nodes[imu].booted && s->nodes[imu].corner_rx != NULL && c >= 0 &&
           s->nodes[imu].corner_rx->peer[c].up;
}

static const char *state_of(const sim_t *s, int i)
{
    const sim_node_t *n = &s->nodes[i];
    if (!n->powered) {
        return "OFF";
    }
    if (!n->booted) {
        return "BOOT";
    }
    if (n->kind == SIM_NODE_CORNER) {
        /* No heartbeat of its own: what the IMU Controller reports for it. */
        if (!link_up(s, i)) {
            return "LOST";
        }
        int     c     = lbr_node_corner_index(n->id);
        uint8_t flags = s->nodes[n->peer].corner_rx->peer[c].flags;
        return ((flags & LBR_WLINK_F_SENSOR_OK) && (flags & LBR_WLINK_F_SYNCED)) ? "RUN"
                                                                                 : "DEGRADED";
    }
    if (n->kind == SIM_NODE_DASH) {
        return link_up(s, i) ? "RUN" : "LOST";
    }
    if (n->last_health.node_id == 0) {
        return "BOOT";
    }
    return lbr_node_state_name(n->last_health.state);
}

static double bus_load_pct(const sim_t *s)
{
    return (double)s->bus.busy_us_last * 100.0 / 1e6;
}

static bool gw_sees(const sim_t *s, int i)
{
    if (wireless(&s->nodes[i])) {
        return link_up(s, i);
    }
    if (s->gw.core == NULL) {
        return false;
    }
    const lbr_node_entry_t *e = lbr_node_table_find(&s->gw.core->nodes, s->nodes[i].id);
    return e != NULL && e->present;
}

static const lbr_corner_peer_t *corner_peer(const sim_t *s, int i)
{
    const sim_node_t *n = &s->nodes[i];
    int               c = lbr_node_corner_index(n->id);
    if (c < 0 || n->peer < 0 || s->nodes[n->peer].corner_rx == NULL) {
        return NULL;
    }
    return &s->nodes[n->peer].corner_rx->peer[c];
}

/* ------------------------------------------------------------------------ */
/* Snapshot                                                                  */

static void snap_slots(FILE *o, sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    fprintf(o, ",\"slots\":[");
    for (size_t k = 0; n->plan != NULL && k < n->plan->n_slots; k++) {
        const lbr_slot_t     *sl   = &n->plan->slots[k];
        const char           *kind = n->slots[k].kind == SIM_SLOT_IO         ? "io"
                                     : n->slots[k].kind == SIM_SLOT_NMEA     ? "nmea"
                                     : n->slots[k].kind == SIM_SLOT_FIRMWARE ? "firmware"
                                                                             : "part";
        const lbr_acq_slot_t *a    = NULL;
        if (n->kind == SIM_NODE_CENTRAL && s->gw.core != NULL) {
            a = &s->gw.core->slot[k].acq;
        } else if (n->kind == SIM_NODE_CAN && n->core != NULL) {
            a = &n->core->slot[k].acq;
        } else if (n->kind == SIM_NODE_CORNER && n->corner != NULL) {
            a = &n->corner->slot[k];
        } else if (n->kind == SIM_NODE_DASH && n->dash != NULL) {
            a = &n->dash->slot[k];
        }
        bool healthy = n->booted && a != NULL && atomic_load(&a->healthy);
        fprintf(o,
                "%s{\"name\":\"%s\",\"rate_hz\":%u,\"kind\":\"%s\",\"unplugged\":%s,"
                "\"healthy\":%s,\"signals\":[",
                k ? "," : "", sl->name, (unsigned)sl->rate_hz, kind,
                n->slots[k].unplugged ? "true" : "false", healthy ? "true" : "false");
        for (size_t c2 = 0; c2 < sl->n_signals; c2++) {
            fprintf(o, "%s\"%s\"", c2 ? "," : "", lbr_signal_info(sl->signals[c2])->name);
        }
        fprintf(o, "]}");
    }
    fprintf(o, "]");
}

void sim_snapshot(sim_t *s)
{
    static const char *kinds[] = {"central", "can", "corner", "dash"};
    FILE              *o       = s->out;
    fprintf(o, "{\"ev\":\"snap\",\"t\":%.6f,\"seed\":%llu,", t_s(s), (unsigned long long)s->seed);
    fprintf(o,
            "\"bus\":{\"bitrate\":%lu,\"load\":%.2f,\"fps\":%u,\"frames\":%llu,\"errors\":%llu,"
            "\"noise_permille\":%u},",
            (unsigned long)s->bus.bitrate, bus_load_pct(s), (unsigned)s->bus.frames_last,
            (unsigned long long)s->bus.frames_total, (unsigned long long)s->bus.errors_total,
            (unsigned)s->bus.noise_permille);

    fprintf(o, "\"nodes\":[");
    for (int i = 0; i < s->n_nodes; i++) {
        sim_node_t                *n = &s->nodes[i];
        const lbr_health_record_t *h = &n->last_health;
        const lbr_can_stats_t     *c = &n->can.st;
        fprintf(o,
                "%s{\"name\":\"%s\",\"id\":%u,\"kind\":\"%s\",\"state\":\"%s\",\"powered\":%s,"
                "\"booted\":%s,\"connected\":%s,\"traced\":%s,\"seen\":%s,\"synced\":%s,"
                "\"sync_err_us\":%lld,\"ppm\":%.1f,\"boot_count\":%u,\"uptime_s\":%.1f,"
                "\"flags\":%u,",
                i ? "," : "", n->name, (unsigned)n->id, kinds[n->kind], state_of(s, i),
                n->powered ? "true" : "false", n->booted ? "true" : "false",
                n->can.connected ? "true" : "false", n->traced ? "true" : "false",
                (i == GW || gw_sees(s, i)) ? "true" : "false", n->servo.synced ? "true" : "false",
                (long long)sync_error_us(s, i), n->ppm, (unsigned)n->boot_count,
                (double)sim_local_us(s, i) / 1e6, (unsigned)h->flags);
        if (wireless(n)) {
            const lbr_corner_peer_t *p = corner_peer(s, i);
            fprintf(o,
                    "\"wifi\":{\"up\":%s,\"link\":%s,\"loss_permille\":%u,\"sent\":%u,"
                    "\"delivered\":%u,\"dropped\":%u,\"forwarded\":%u,\"lost\":%u,\"gaps\":%u}",
                    n->wifi.up ? "true" : "false", link_up(s, i) ? "true" : "false",
                    (unsigned)n->wifi.loss_permille, (unsigned)n->wifi.sent,
                    (unsigned)n->wifi.delivered, (unsigned)n->wifi.dropped,
                    p != NULL ? (unsigned)p->forwarded : 0u, p != NULL ? (unsigned)p->lost : 0u,
                    p != NULL ? (unsigned)p->packet_gaps
                    : n->kind == SIM_NODE_DASH && s->gw.core != NULL
                        ? (unsigned)s->gw.core->dash.packet_gaps
                        : 0u);
            if (n->corner != NULL) {
                fprintf(o,
                        ",\"corner\":{\"acquired\":%u,\"overruns\":%u,\"steps\":%u,"
                        "\"clock_ok\":%u,\"clock_rejected\":%u,\"rtt_us\":%u}",
                        (unsigned)atomic_load(&n->corner->acquired),
                        (unsigned)atomic_load(&n->corner->overruns), (unsigned)n->servo.step_count,
                        (unsigned)n->corner->clock.accepted, (unsigned)n->corner->clock.rejected,
                        (unsigned)n->corner->clock.last_rtt_us);
            }
        } else {
            fprintf(o,
                    "\"can\":{\"running\":%s,\"tec\":%u,\"rec\":%u,\"tx\":%u,\"rx\":%u,"
                    "\"tx_dropped\":%u,\"bus_off\":%u,\"errors\":%u,\"queue\":%u},"
                    "\"health\":{\"acquire_hz\":%u,\"publish_hz\":%u,\"log_hz\":%u,"
                    "\"overflow\":%u,\"transport_errors\":%u,\"sensor_status\":%lu,"
                    "\"calibration\":%lu,\"steps\":%u}",
                    c->running ? "true" : "false", (unsigned)c->tec, (unsigned)c->rec,
                    (unsigned)c->tx_frames, (unsigned)c->rx_frames, (unsigned)c->tx_dropped,
                    (unsigned)c->bus_off_count, (unsigned)c->bus_errors, (unsigned)n->can.tx_len,
                    (unsigned)h->acquire_hz, (unsigned)h->publish_hz, (unsigned)h->log_hz,
                    (unsigned)h->overflow_count, (unsigned)h->transport_error_count,
                    (unsigned long)h->sensor_status, (unsigned long)h->calibration_state,
                    (unsigned)h->time_step_count);
        }
        snap_slots(o, s, i);
        fprintf(o, "}");
    }
    fprintf(o, "],");

    lbr_log_stats_t ls = {0};
    lbr_log_get_stats(&s->gw.log, &ls);
    fprintf(o,
            "\"gw\":{\"epoch\":%u,\"sd\":%s,\"log_open\":%s,\"log_file\":\"%s\",\"log_bytes\":%llu,"
            "\"records\":%u,\"dropped\":%u,\"write_errors\":%u,\"queue_hw\":%u,"
            "\"usb_pkts\":%u,\"usb_bytes\":%u,\"wifi_pkts\":%u,\"lora_pkts\":%u,\"lora_bytes\":%u,"
            "\"lora_records\":%u,\"dash_pkts\":%u,\"sync_fail\":%u},",
            (unsigned)s->gw.epoch, s->nodes[GW].sd_inserted ? "true" : "false",
            s->gw.log.open ? "true" : "false", s->nodes[GW].keeper.name,
            (unsigned long long)s->nodes[GW].log_bytes, (unsigned)ls.records_written,
            (unsigned)(ls.records_dropped + ls.records_lost_write + ls.records_rejected_closed),
            (unsigned)ls.write_errors, (unsigned)ls.queue_high_water,
            (unsigned)s->gw.usb_st.packets_sent, (unsigned)s->gw.usb_st.bytes_sent,
            (unsigned)s->gw.wifi_st.packets_sent, (unsigned)s->gw.lora_st.packets_sent,
            (unsigned)s->gw.lora_st.bytes_sent, (unsigned)s->gw.last_lora_records,
            (unsigned)s->gw.dash_st.packets_sent, (unsigned)s->gw.sync_timeouts);

    /* Latest value of every signal seen on the bus (what the Central logs). */
    fprintf(o, "\"signals\":{");
    bool first = true;
    for (size_t i = 0; i < lbr_signal_count(); i++) {
        const lbr_signal_info_t *info = lbr_signal_at(i);
        const sim_value_t       *v    = &s->values[info->id];
        if (!v->seen || info->id == LBR_SIG_EVENT || info->id == LBR_SIG_FAULT) {
            continue;
        }
        fprintf(o, "%s\"%s\":{\"src\":\"%s\",\"unit\":\"%s\",\"age_ms\":%llu,\"synced\":%s,",
                first ? "" : ",", info->name, lbr_node_name(v->source), lbr_unit_name(info->unit),
                (unsigned long long)((s->now - v->rx_t) / 1000u), v->s.synced ? "true" : "false");
        first = false;
        if (!v->s.valid) {
            fprintf(o, "\"valid\":false,\"reason\":\"%s\"}",
                    lbr_unavail_reason_name((lbr_unavail_reason_t)v->s.v.u32));
        } else if (info->type == LBR_VAL_F32) {
            double d = (double)v->s.v.f32;
            fprintf(o, "\"valid\":true,\"v\":%.4g}", isfinite(d) ? d : 0.0);
        } else if (info->type == LBR_VAL_I32) {
            fprintf(o, "\"valid\":true,\"v\":%ld}", (long)v->s.v.i32);
        } else if (info->type == LBR_VAL_BOOL) {
            fprintf(o, "\"valid\":true,\"v\":%u}", v->s.v.b ? 1u : 0u);
        } else {
            fprintf(o, "\"valid\":true,\"v\":%lu}", (unsigned long)v->s.v.u32);
        }
    }
    fprintf(o, "},");

    fprintf(o, "\"wire\":[");
    size_t shown = 0;
    for (size_t k = 0; k < SIM_RECENT; k++) {
        const sim_wire_t *w = &s->recent[(s->recent_head + k) % SIM_RECENT];
        if (w->t == 0 || s->now - w->t > 200000u) {
            continue;
        }
        char d[48];
        fprintf(o,
                "%s{\"t\":%.6f,\"src\":\"%s\",\"id\":%lu,\"prio\":%u,\"what\":\"%s\",\"error\":%s}",
                shown++ ? "," : "", (double)w->t / 1e6, lbr_node_name(lbr_can_sender(w->f.id)),
                (unsigned long)w->f.id, (unsigned)(w->f.id >> 8), frame_desc(&w->f, d, sizeof d),
                w->error ? "true" : "false");
    }
    fprintf(o, "],");

    /* What the cockpit display shows: the dashboard core's own tables. */
    fprintf(o, "\"dash\":[");
    int dash = sim_node_by_id(s, LBR_NODE_DASHBOARD);
    if (dash >= 0 && s->nodes[dash].booted && s->nodes[dash].dash != NULL) {
        static lbr_value_table_t values;
        static lbr_dash_node_t   dnodes[LBR_DASH_MAX_NODES];
        uint64_t                 dnow = sim_local_us(s, dash);
        lbr_dash_core_snapshot(s->nodes[dash].dash, &values, dnodes);
        for (size_t k = 0; k < lbr_dash_signal_count; k++) {
            lbr_sample_t             smp;
            const lbr_signal_info_t *info = lbr_signal_info(lbr_dash_signals[k]);
            lbr_value_state_t  st = lbr_value_table_get(&values, lbr_dash_signals[k], dnow, &smp);
            static const char *names[] = {"none", "stale", "invalid", "ok"};
            fprintf(o, "%s{\"sig\":\"%s\",\"unit\":\"%s\",\"state\":\"%s\"", k ? "," : "",
                    info->name, lbr_unit_name(info->unit), names[st]);
            if (st == LBR_VALUE_OK && info->type == LBR_VAL_F32) {
                fprintf(o, ",\"v\":%.4g", (double)smp.v.f32);
            } else if (st == LBR_VALUE_OK || st == LBR_VALUE_INVALID) {
                fprintf(o, ",\"v\":%lu", (unsigned long)smp.v.u32);
            }
            fprintf(o, "}");
        }
    }
    fprintf(o, "],");

    const sim_world_t *w = &s->world;
    fprintf(o,
            "\"world\":{\"mode\":\"%s\",\"speed\":%.2f,\"heading\":%.1f,\"x\":%.1f,\"y\":%.1f,"
            "\"throttle\":%.1f,\"brake\":%.0f,\"steer\":%.1f},",
            w->mode == SIM_DRIVE_RACE ? "race" : "idle", w->speed, w->heading * 57.2957795, w->x_m,
            w->y_m, w->throttle, w->brake_kpa, w->steer_deg);
    fprintf(o, "\"trace_dropped\":%u}\n", (unsigned)s->trace_dropped);
    s->trace_dropped = 0;
}

/* ------------------------------------------------------------------------ */
/* Commands                                                                  */

static int err(sim_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static int err(sim_t *s, const char *fmt, ...)
{
    char    text[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof text, fmt, ap);
    va_end(ap);
    if (s->quiet) {
        fprintf(s->out, "  %8.3f  error: %s\n", t_s(s), text);
    } else {
        fprintf(s->out, "{\"ev\":\"error\",\"msg\":");
        json_str(s->out, text);
        fprintf(s->out, "}\n");
    }
    return -1;
}

static const lbr_signal_info_t *signal_by_name(const char *name)
{
    for (size_t i = 0; i < lbr_signal_count(); i++) {
        if (strcmp(lbr_signal_at(i)->name, name) == 0) {
            return lbr_signal_at(i);
        }
    }
    return NULL;
}

static int need_node(sim_t *s, const char *name)
{
    int i = name != NULL ? sim_node_index(s, name) : -1;
    if (i < 0) {
        err(s, "unknown node '%s'", name != NULL ? name : "");
    }
    return i;
}

static int slot_index(const sim_node_t *n, const char *name)
{
    if (n->plan == NULL || name == NULL) {
        return -1;
    }
    for (size_t k = 0; k < n->plan->n_slots; k++) {
        if (strcmp(n->plan->slots[k].name, name) == 0) {
            return (int)k;
        }
    }
    return -1;
}

static bool compare(double got, const char *op, double want)
{
    if (strcmp(op, "==") == 0) {
        return fabs(got - want) < 1e-9;
    }
    if (strcmp(op, "!=") == 0) {
        return fabs(got - want) >= 1e-9;
    }
    if (strcmp(op, "<") == 0) {
        return got < want;
    }
    if (strcmp(op, "<=") == 0) {
        return got <= want;
    }
    if (strcmp(op, ">") == 0) {
        return got > want;
    }
    if (strcmp(op, ">=") == 0) {
        return got >= want;
    }
    return false;
}

static void check_result(sim_t *s, bool ok, const char *what, const char *got)
{
    if (ok) {
        s->checks_passed++;
    } else {
        s->checks_failed++;
    }
    if (s->quiet) {
        fprintf(s->out, "  %8.3f  %s  %s (got %s)\n", t_s(s), ok ? "PASS" : "FAIL", what, got);
    } else {
        fprintf(s->out, "{\"ev\":\"check\",\"t\":%.6f,\"ok\":%s,\"what\":", t_s(s),
                ok ? "true" : "false");
        json_str(s->out, what);
        fprintf(s->out, ",\"got\":");
        json_str(s->out, got);
        fprintf(s->out, "}\n");
    }
}

/* Numeric metric of one node; false if unknown. */
static bool node_metric(sim_t *s, int i, const char *m, double *v)
{
    sim_node_t              *n = &s->nodes[i];
    const lbr_corner_peer_t *p = corner_peer(s, i);
    if (strcmp(m, "seen") == 0) {
        *v = (i == GW || gw_sees(s, i)) ? 1 : 0;
    } else if (strcmp(m, "synced") == 0) {
        /* The dashboard keeps no shared clock: nothing to sync, reported 1. */
        *v = (n->servo.synced || n->kind == SIM_NODE_DASH) ? 1 : 0;
    } else if (strcmp(m, "link") == 0) {
        *v = !wireless(n) || link_up(s, i) ? 1 : 0;
    } else if (strcmp(m, "lost") == 0) {
        *v = p != NULL ? p->lost : 0; /* corner samples that never reached CAN */
    } else if (strcmp(m, "forwarded") == 0) {
        *v = p != NULL ? p->forwarded : 0;
    } else if (strcmp(m, "gaps") == 0) {
        *v = p != NULL                                        ? p->packet_gaps
             : n->kind == SIM_NODE_DASH && s->gw.core != NULL ? s->gw.core->dash.packet_gaps
                                                              : 0;
    } else if (strcmp(m, "transport_errors") == 0) {
        *v = n->last_health.transport_error_count;
    } else if (strcmp(m, "sync_err_us") == 0) {
        *v = (double)llabs((long long)sync_error_us(s, i));
    } else if (strcmp(m, "tx_dropped") == 0) {
        *v = n->can.st.tx_dropped;
    } else if (strcmp(m, "bus_off") == 0) {
        *v = n->can.st.bus_off_count;
    } else if (strcmp(m, "tec") == 0) {
        *v = n->can.st.tec;
    } else if (strcmp(m, "overflow") == 0) {
        *v = n->last_health.overflow_count;
    } else if (strcmp(m, "steps") == 0) {
        *v = n->last_health.time_step_count;
    } else if (strcmp(m, "boot_count") == 0) {
        *v = n->boot_count;
    } else if (strncmp(m, "event:", 6) == 0) {
        for (uint16_t code = 1; code < 0x80; code++) {
            const char *name = lbr_event_name((lbr_event_code_t)code);
            if (name != NULL && strcmp(name, m + 6) == 0) {
                *v = s->event_counts[i][code];
                return true;
            }
        }
        return false;
    } else {
        return false;
    }
    return true;
}

/* The gateway's log file read back with the firmware's own reader. */
static bool log_metric(sim_t *s, const char *m, double *v)
{
    char path[512];
    path_in_out(s, s->nodes[GW].log_name, path, sizeof path);
    if (s->gw.log.open) {
        lbr_log_pump(&s->gw.log, h_now(&s_ctx[GW]));
    }
    if (s->nodes[GW].log_file != NULL) {
        fflush(s->nodes[GW].log_file);
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        *v = 0;
        return strcmp(m, "intact") == 0 || strcmp(m, "frames") == 0;
    }
    fseek(f, 0, SEEK_END);
    long     len = ftell(f);
    uint8_t *buf = malloc(len > 0 ? (size_t)len : 1u);
    fseek(f, 0, SEEK_SET);
    size_t got = buf != NULL ? fread(buf, 1, (size_t)len, f) : 0;
    fclose(f);
    lbr_log_reader_t rd;
    lbr_log_record_t rec;
    uint32_t         frames = 0;
    lbr_log_reader_init(&rd, buf, got);
    while (lbr_log_reader_next(&rd, &rec)) {
        frames += rec.type == LBR_REC_CAN_FRAME;
    }
    free(buf);
    if (strcmp(m, "intact") == 0) {
        *v = rd.corrupt_regions == 0 && rd.records > 0 ? 1 : 0;
    } else if (strcmp(m, "frames") == 0) {
        *v = frames;
    } else {
        return false;
    }
    return true;
}

/* Group targets of expect: all (every node but the Central), can (the wired
 * sensor controllers), corners, wireless (corners and the dashboard). */
static int group_of(const char *target)
{
    static const char *const groups[] = {"all", "can", "corners", "wireless"};
    for (int g = 0; g < 4; g++) {
        if (strcmp(target, groups[g]) == 0) {
            return g;
        }
    }
    return -1;
}

static bool in_group(const sim_t *s, int g, int i)
{
    const sim_node_t *n = &s->nodes[i];
    switch (g) {
        case 0:
            return i != GW;
        case 1:
            return n->kind == SIM_NODE_CAN;
        case 2:
            return n->kind == SIM_NODE_CORNER;
        default:
            return wireless(n);
    }
}

static int cmd_expect(sim_t *s, char **tok, int nt, const char *line)
{
    /* expect <target> <metric> <op> <value> */
    if (nt < 5) {
        return err(s, "usage: expect <node|all|bus|central|log> <metric> <op> <value>");
    }
    const char *target = tok[1], *metric = tok[2], *op = tok[3], *want = tok[4];
    char        got[64];
    const char *what = line + strlen("expect ");

    int group = group_of(target);
    if (strcmp(metric, "state") == 0) {
        bool all = group >= 0;
        bool ok  = true;
        got[0]   = '\0';
        for (int i = 0; i < s->n_nodes; i++) {
            if ((all && !in_group(s, group, i)) ||
                (!all && strcmp(s->nodes[i].name, target) != 0)) {
                continue;
            }
            bool eq = strcmp(state_of(s, i), want) == 0;
            ok      = ok && (strcmp(op, "!=") == 0 ? !eq : eq);
            if (!eq || !all) {
                snprintf(got + strlen(got), sizeof got - strlen(got), "%s%s=%s", got[0] ? " " : "",
                         s->nodes[i].name, state_of(s, i));
            }
        }
        if (!all && got[0] == '\0') {
            return err(s, "unknown node '%s'", target);
        }
        check_result(s, ok, what, got[0] ? got : "all match");
        return 0;
    }

    /* A signal as the gateway last received it: valid, value, age_ms, reason. */
    const lbr_signal_info_t *sig = signal_by_name(target);
    if (sig != NULL) {
        const sim_value_t *sv = &s->values[sig->id];
        if (strcmp(metric, "reason") == 0) {
            const char *r  = !sv->seen ? "NEVER"
                             : sv->s.valid
                                 ? "NONE"
                                 : lbr_unavail_reason_name((lbr_unavail_reason_t)sv->s.v.u32);
            bool        eq = strcmp(r, want) == 0;
            check_result(s, strcmp(op, "!=") == 0 ? !eq : eq, what, r);
            return 0;
        }
        double sv_v = 0;
        if (strcmp(metric, "valid") == 0) {
            sv_v = sv->seen && sv->s.valid ? 1 : 0;
        } else if (strcmp(metric, "value") == 0) {
            sv_v = sig->type == LBR_VAL_F32   ? (double)sv->s.v.f32
                   : sig->type == LBR_VAL_I32 ? (double)sv->s.v.i32
                                              : (double)sv->s.v.u32;
        } else if (strcmp(metric, "age_ms") == 0) {
            sv_v = sv->seen ? (double)(s->now - sv->rx_t) / 1000.0 : 1e12;
        } else {
            return err(s, "unknown signal metric '%s'", metric);
        }
        snprintf(got, sizeof got, "%g", sv_v);
        check_result(s, compare(sv_v, op, atof(want)), what, got);
        return 0;
    }

    double wv = atof(want), v = 0;
    if (strcmp(target, "bus") == 0) {
        if (strcmp(metric, "load") == 0) {
            v = bus_load_pct(s);
        } else if (strcmp(metric, "errors") == 0) {
            v = (double)s->bus.errors_total;
        } else {
            return err(s, "unknown bus metric '%s'", metric);
        }
    } else if (strcmp(target, "central") == 0 && strcmp(metric, "frames_logged") != 0 &&
               strcmp(metric, "log_dropped") != 0 && strcmp(metric, "lora_pkts") != 0 &&
               strcmp(metric, "usb_pkts") != 0 && strcmp(metric, "sync_fail") != 0 &&
               strcmp(metric, "dash_pkts") != 0) {
        if (!node_metric(s, GW, metric, &v)) {
            return err(s, "unknown central metric '%s'", metric);
        }
    } else if (strcmp(target, "central") == 0) {
        lbr_log_stats_t ls = {0};
        lbr_log_get_stats(&s->gw.log, &ls);
        if (strcmp(metric, "frames_logged") == 0) {
            v = s->gw.core != NULL ? atomic_load(&s->gw.core->frames_logged) : 0;
        } else if (strcmp(metric, "log_dropped") == 0) {
            v = ls.records_dropped + ls.records_lost_write;
        } else if (strcmp(metric, "lora_pkts") == 0) {
            v = s->gw.lora_st.packets_sent;
        } else if (strcmp(metric, "usb_pkts") == 0) {
            v = s->gw.usb_st.packets_sent;
        } else if (strcmp(metric, "dash_pkts") == 0) {
            v = s->gw.dash_st.packets_sent;
        } else {
            v = s->gw.sync_timeouts;
        }
    } else if (strcmp(target, "log") == 0) {
        if (!log_metric(s, metric, &v)) {
            return err(s, "unknown log metric '%s'", metric);
        }
    } else if (group >= 0) {
        /* Worst case across the group. */
        bool   any   = false;
        double worst = 0;
        bool   ok    = true;
        for (int i = 0; i < s->n_nodes; i++) {
            double x;
            if (!in_group(s, group, i)) {
                continue;
            }
            if (!node_metric(s, i, metric, &x)) {
                return err(s, "unknown node metric '%s'", metric);
            }
            if (!compare(x, op, wv)) {
                ok = false;
                if (!any || fabs(x) > fabs(worst)) {
                    worst = x;
                    snprintf(got, sizeof got, "%s=%g", s->nodes[i].name, x);
                    any = true;
                }
            }
        }
        check_result(s, ok, what, ok ? "all match" : got);
        return 0;
    } else {
        int i = need_node(s, target);
        if (i < 0) {
            return -1;
        }
        if (!node_metric(s, i, metric, &v)) {
            return err(s, "unknown node metric '%s'", metric);
        }
    }
    snprintf(got, sizeof got, "%g", v);
    check_result(s, compare(v, op, wv), what, got);
    return 0;
}

static void ok_reply(sim_t *s, const char *cmd)
{
    if (!s->quiet) {
        fprintf(s->out, "{\"ev\":\"ok\",\"cmd\":");
        json_str(s->out, cmd);
        fprintf(s->out, "}\n");
    }
}

int sim_command(sim_t *s, const char *line)
{
    char copy[512];
    snprintf(copy, sizeof copy, "%s", line);
    char *tok[12];
    int   nt   = 0;
    char *save = NULL;
    for (char *p = strtok_r(copy, " \t\r\n", &save); p != NULL && nt < 12;
         p       = strtok_r(NULL, " \t\r\n", &save)) {
        tok[nt++] = p;
    }
    if (nt == 0 || tok[0][0] == '#') {
        return 0;
    }
    const char *c   = tok[0];
    s->trace_budget = TRACE_BUDGET;

    if (strcmp(c, "run") == 0) {
        double ms = nt > 1 ? atof(tok[1]) : 100.0;
        if (ms < 0 || ms > 600000) {
            return err(s, "run takes 0..600000 ms");
        }
        sim_run_until(s, s->now + (uint64_t)(ms * 1000.0));
        if (!s->quiet) {
            sim_snapshot(s);
        }
        return 0;
    }
    if (strcmp(c, "status") == 0) {
        sim_snapshot(s);
        return 0;
    }
    if (strcmp(c, "expect") == 0) {
        return cmd_expect(s, tok, nt, line);
    }
    if (strcmp(c, "trace") == 0) {
        if (nt < 2) {
            return err(s, "usage: trace <node|all|none>");
        }
        bool all = strcmp(tok[1], "all") == 0, none = strcmp(tok[1], "none") == 0;
        int  i = all || none ? -1 : need_node(s, tok[1]);
        if (!all && !none && i < 0) {
            return -1;
        }
        for (int k = 0; k < s->n_nodes; k++) {
            s->nodes[k].traced = all || k == i;
        }
    } else if (strcmp(c, "unplug") == 0 || strcmp(c, "plug") == 0) {
        int i = nt > 2 ? need_node(s, tok[1]) : err(s, "usage: %s <node> <slot>", c);
        if (i < 0) {
            return -1;
        }
        int k = slot_index(&s->nodes[i], tok[2]);
        if (k < 0) {
            return err(s, "%s has no slot '%s'", tok[1], tok[2]);
        }
        s->nodes[i].slots[k].unplugged = c[0] == 'u';
        sim_note(s, "%s.%s %s", tok[1], tok[2], c[0] == 'u' ? "unplugged" : "plugged in");
    } else if (strcmp(c, "open") == 0 || strcmp(c, "close") == 0 || strcmp(c, "auto") == 0 ||
               strcmp(c, "set") == 0) {
        const lbr_signal_info_t *info = nt > 1 ? signal_by_name(tok[1]) : NULL;
        if (info == NULL) {
            return err(s, "unknown signal '%s'", nt > 1 ? tok[1] : "");
        }
        sim_override_t *o = &s->overrides[info->id];
        if (strcmp(c, "open") == 0) {
            o->open = true;
        } else if (strcmp(c, "close") == 0) {
            o->open = false;
        } else if (strcmp(c, "auto") == 0) {
            o->set = false;
        } else {
            if (nt < 3) {
                return err(s, "usage: set <SIGNAL> <value>");
            }
            o->set   = true;
            o->value = (float)atof(tok[2]);
        }
    } else if (strcmp(c, "wifi") == 0) {
        /* wifi <node> up|down|loss <permille>: a corner's or the dashboard's link. */
        int i =
            nt > 2 ? need_node(s, tok[1]) : err(s, "usage: wifi <node> up|down|loss <permille>");
        if (i < 0) {
            return -1;
        }
        sim_node_t *n = &s->nodes[i];
        if (!wireless(n)) {
            return err(s, "%s has no WiFi link (corner nodes and the dashboard do)", n->name);
        }
        if (strcmp(tok[2], "up") == 0 || strcmp(tok[2], "down") == 0) {
            n->wifi.up = tok[2][0] == 'u';
            sim_note(s, "%s WiFi %s", n->name, n->wifi.up ? "back in range" : "out of range");
        } else if (strcmp(tok[2], "loss") == 0 && nt > 3 && atoi(tok[3]) >= 0 &&
                   atoi(tok[3]) <= 1000) {
            n->wifi.loss_permille = (uint32_t)atoi(tok[3]);
        } else {
            return err(s, "usage: wifi <node> up|down|loss <0..1000 permille>");
        }
    } else if (strcmp(c, "disconnect") == 0 || strcmp(c, "connect") == 0) {
        int i = need_node(s, nt > 1 ? tok[1] : NULL);
        if (i < 0) {
            return -1;
        }
        if (wireless(&s->nodes[i])) {
            return err(s, "%s is not on CAN (use: wifi %s down)", tok[1], tok[1]);
        }
        s->nodes[i].can.connected = c[0] == 'c';
        if (c[0] == 'c') {
            s->nodes[i].can.st.tec = 0;
            vbus_kick(s);
        }
        sim_note(s, "%s CAN cable %s", tok[1], c[0] == 'c' ? "connected" : "disconnected");
    } else if (strcmp(c, "busoff") == 0) {
        int i = need_node(s, nt > 1 ? tok[1] : NULL);
        if (i < 0) {
            return -1;
        }
        if (wireless(&s->nodes[i])) {
            return err(s, "%s is not on CAN", tok[1]);
        }
        s->nodes[i].can.st.tec = 255;
        vbus_bus_off(s, i);
    } else if (strcmp(c, "drift") == 0) {
        int i = nt > 2 ? need_node(s, tok[1]) : err(s, "usage: drift <node> <ppm>");
        if (i < 0) {
            return -1;
        }
        double ppm = atof(tok[2]);
        if (fabs(ppm) > 500.0) {
            return err(s, "drift is limited to +/-500 ppm");
        }
        /* Keep the local clock continuous: re-anchor its origin. */
        sim_node_t *n     = &s->nodes[i];
        uint64_t    local = sim_local_us(s, i);
        n->ppm            = ppm;
        n->boot_at        = s->now - (uint64_t)((double)local / (1.0 + ppm * 1e-6));
    } else if (strcmp(c, "reboot") == 0 || strcmp(c, "power") == 0) {
        int i = need_node(s, nt > 1 ? tok[1] : NULL);
        if (i < 0) {
            return -1;
        }
        sim_node_t *n  = &s->nodes[i];
        bool        on = strcmp(c, "reboot") == 0 || (nt > 2 && strcmp(tok[2], "on") == 0);
        if (strcmp(c, "power") == 0 && nt < 3) {
            return err(s, "usage: power <node> on|off");
        }
        power_off(s, i);
        n->powered      = on;
        n->reset_reason = strcmp(c, "reboot") == 0 ? LBR_RESET_SOFTWARE : LBR_RESET_POWER_ON;
        n->last_health  = (lbr_health_record_t){0};
        if (on) {
            sim_schedule(s, s->now + BOOT_TIME_US, EV_BOOT, i, 0);
        }
        sim_note(s, "%s %s", tok[1],
                 strcmp(c, "reboot") == 0 ? "rebooting"
                 : on                     ? "powered on"
                                          : "powered off");
    } else if (strcmp(c, "sd") == 0) {
        /* sd [node] remove|insert|fail <writes>; the node defaults to the Central. */
        int k = 1, i = GW;
        if (nt > 2 && sim_node_index(s, tok[1]) >= 0) {
            i = sim_node_index(s, tok[1]);
            k = 2;
        }
        sim_node_t *n = &s->nodes[i];
        if (nt <= k || (strcmp(tok[k], "fail") == 0 && nt <= k + 1)) {
            return err(s, "usage: sd [node] remove|insert|fail <writes>");
        }
        if (strcmp(tok[k], "remove") == 0) {
            n->sd_inserted = false;
            if (n->log_file != NULL) {
                fclose(n->log_file);
                n->log_file = NULL;
            }
            sim_note(s, "%s SD card removed", n->name);
        } else if (strcmp(tok[k], "insert") == 0) {
            n->sd_inserted = true;
            sim_note(s, "%s SD card inserted", n->name);
        } else if (strcmp(tok[k], "fail") == 0) {
            n->sd_fail_writes = (uint32_t)atoi(tok[k + 1]);
        } else {
            return err(s, "usage: sd [node] remove|insert|fail <writes>");
        }
    } else if (strcmp(c, "noise") == 0) {
        int pm = nt > 1 ? atoi(tok[1]) : -1;
        if (pm < 0 || pm > 500) {
            return err(s, "usage: noise <0..500 permille of frames>");
        }
        s->bus.noise_permille = (uint32_t)pm;
    } else if (strcmp(c, "drive") == 0) {
        if (nt < 2 || (strcmp(tok[1], "race") != 0 && strcmp(tok[1], "idle") != 0)) {
            return err(s, "usage: drive race|idle");
        }
        s->world.mode = tok[1][0] == 'r' ? SIM_DRIVE_RACE : SIM_DRIVE_IDLE;
    } else {
        return err(s, "unknown command '%s'", c);
    }
    ok_reply(s, line);
    return 0;
}
