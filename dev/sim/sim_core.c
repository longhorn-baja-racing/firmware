/*
 * Simulator core: small helpers, the event queue, event dispatch, and setup.
 */
#include "sim_internal.h"

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
    if ((int64_t)s->now <= n->boot_at) {
        return 0;
    }
    double el = (double)((int64_t)s->now - n->boot_at);
    return (uint64_t)(el * (1.0 + n->ppm * 1e-6));
}

uint64_t sim_from_local(const sim_t *s, int node, uint64_t local_us)
{
    const sim_node_t *n = &s->nodes[node];
    int64_t           t = n->boot_at + (int64_t)ceil((double)local_us / (1.0 + n->ppm * 1e-6));
    return t < 0 ? 0u : (uint64_t)t;
}

/* JSON string body (names and messages are ASCII; escape the specials). */
void json_str(FILE *o, const char *t)
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

double t_s(const sim_t *s)
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

const char *frame_desc(const lbr_can_frame_t *f, char *buf, size_t cap)
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
/* Event dispatch                                                            */

static void acq_tick(sim_t *s, int ni, size_t k)
{
    sim_node_t *n = &s->nodes[ni];
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            lbr_gw_core_slot_tick(s->gw.core, k);
            break;
        case SIM_NODE_COCKPIT:
            lbr_cockpit_core_slot_tick(n->cockpit, k);
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
        case EV_GW_COCKPIT:
            lbr_gw_core_cockpit_tick(g->core);
            schedule_local(s, GW, SIM_JOB_COCKPIT,
                           n->next_local[SIM_JOB_COCKPIT] + g->cfg.cockpit_period_ms * 1000u,
                           EV_GW_COCKPIT);
            break;
        case EV_GW_LOGIC:
            lbr_gw_core_logic_tick(g->core);
            schedule_local(s, GW, SIM_JOB_LOGIC,
                           n->next_local[SIM_JOB_LOGIC] + 1000000u / LBR_LOGIC_TICK_HZ,
                           EV_GW_LOGIC);
            break;
        case EV_GW_PUMP:
            lbr_log_keeper_step(&n->keeper, sim_local_us(s, ni), h_now(&s_ctx[ni]));
            schedule_local(s, ni, SIM_JOB_PUMP, n->next_local[SIM_JOB_PUMP] + 20000u, EV_GW_PUMP);
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
    if (t_us > s->now) {
        s->now = t_us;
    }
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
    /* DAQ-WIRING.md: the Central first (index GW), the other CAN nodes, then
     * the cockpit (WiFi only). */
    const node_def_t defs[] = {
        {"central", LBR_NODE_CENTRAL, SIM_NODE_CENTRAL, &lbr_plan_central, true, 64},
        {"imu", LBR_NODE_IMU, SIM_NODE_CAN, &lbr_plan_imu, true, 64},
        {"powertrain", LBR_NODE_POWERTRAIN, SIM_NODE_CAN, &lbr_plan_powertrain, false, 64},
        {"driver_input", LBR_NODE_DRIVER_INPUT, SIM_NODE_CAN, &lbr_plan_driver_input, false, 64},
        {"temperature", LBR_NODE_TEMPERATURE, SIM_NODE_CAN, &lbr_plan_temperature, false, 64},
        {"stress", LBR_NODE_STRESS, SIM_NODE_CAN, &lbr_plan_stress, false, 64},
        {"corner_fl", LBR_NODE_CORNER_FL, SIM_NODE_CAN, &lbr_plan_corner[0], true, 64},
        {"corner_fr", LBR_NODE_CORNER_FR, SIM_NODE_CAN, &lbr_plan_corner[1], true, 64},
        {"corner_bl", LBR_NODE_CORNER_BL, SIM_NODE_CAN, &lbr_plan_corner[2], true, 64},
        {"corner_br", LBR_NODE_CORNER_BR, SIM_NODE_CAN, &lbr_plan_corner[3], true, 64},
        {"cockpit", LBR_NODE_COCKPIT, SIM_NODE_COCKPIT, &lbr_plan_cockpit, true, 0},
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

    int cockpit = -1;
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
        cockpit          = n->id == LBR_NODE_COCKPIT ? i : cockpit;
        /* Real crystals: within +/- 40 ppm. */
        n->ppm = (sim_rand(s) - 0.5) * 80.0;
        /* Power comes up together; boot times differ a little. */
        uint64_t at = n->is_gateway ? 0 : 50000u + (uint64_t)(sim_rand(s) * 400000.0);
        sim_schedule(s, at, EV_BOOT, i, 0);
    }
    /* The one WiFi pair: the cockpit and the Central. */
    s->nodes[cockpit].peer = GW;
    s->nodes[GW].peer      = cockpit;
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
        free(s->nodes[i].cockpit);
    }
    free(s->gw.core);
    free(s->gw.log_queue);
    free(s->gw.log_chunk);
    free(s_heap);
    s_heap     = NULL;
    s_heap_len = s_heap_cap = 0;
}
