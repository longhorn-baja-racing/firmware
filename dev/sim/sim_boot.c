/*
 * Booting and powering nodes, scheduling their jobs, and the CAN bus callbacks.
 */
#include "sim_internal.h"

/* ------------------------------------------------------------------------ */
/* Boot                                                                      */

static uint64_t period_us(uint16_t rate_hz)
{
    return 1000000u / (rate_hz ? rate_hz : 1u);
}

void schedule_local(sim_t *s, int node, int job, uint64_t local, sim_ev_type_t type)
{
    s->nodes[node].next_local[job] = local;
    sim_schedule(s, sim_from_local(s, node, local), type, node, job);
}

/* Wake slot k at its next grid point (shared time), as node_esp.c does. */
void schedule_acq(sim_t *s, int ni, size_t k)
{
    sim_node_t *n     = &s->nodes[ni];
    uint64_t    local = sim_local_us(s, ni);
    uint64_t    now, due;
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            now = h_now(&s_ctx[ni]);
            due = lbr_gw_core_slot_next_due(s->gw.core, k);
            break;
        case SIM_NODE_COCKPIT:
            now = local; /* the cockpit keeps no shared clock */
            due = lbr_cockpit_core_slot_next_due(n->cockpit, k);
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

    g->usb     = (lbr_transport_t){"usb", LBR_LINK_USB, NULL, usb_send, link_stats, s};
    g->wifi    = (lbr_transport_t){"wifi", LBR_LINK_WIFI, NULL, wifi_send_pit, link_stats, s};
    g->lora    = (lbr_transport_t){"lora", LBR_LINK_LORA, NULL, lora_send, link_stats, s};
    g->cockpit = (lbr_transport_t){"cockpit", LBR_LINK_COCKPIT, NULL, cockpit_send, link_stats, s};
    g->modem   = (lbr_lora_modem_t){.sf = 9, .bw_hz = 125000, .cr = 1, .preamble = 8, .crc = true};
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
        .tasks        = car_logic.tasks,
        .n_tasks      = car_logic.n,
    };
    if (g->core == NULL) {
        g->core = calloc(1, sizeof *g->core);
    }
    lbr_gw_core_init(g->core, &k_gw_hal, &g->cfg, &boot);
    g->core->hal.ctx = &s_ctx[GW];
    /* Same order as the Central app: core, its own link-state driver, the
     * log service, then start. */
    lbr_sensor_cockpit_link(&n->sensors[CENTRAL_SLOT_COCKPIT_STATUS], "cockpit_status",
                            n->plan->slots[CENTRAL_SLOT_COCKPIT_STATUS].signals,
                            n->plan->slots[CENTRAL_SLOT_COCKPIT_STATUS].n_signals,
                            &g->core->cockpit, lbr_gw_core_lock_state, lbr_gw_core_unlock_state,
                            g->core);
    keeper_start(s, GW, &g->log, g->config_text);
    lbr_gw_links_t links = {
        .usb     = g->cfg.usb_enable ? &g->usb : NULL,
        .wifi    = g->cfg.wifi_enable ? &g->wifi : NULL,
        .lora    = g->cfg.lora_enable ? &g->lora : NULL,
        .cockpit = g->cfg.cockpit_enable ? &g->cockpit : NULL,
    };
    lbr_gw_core_start(g->core, &g->log, &links, true, false);

    uint64_t l = sim_local_us(s, GW);
    schedule_local(s, GW, SIM_JOB_SYNC, l + 1000000u / LBR_TIME_SYNC_HZ, EV_GW_SYNC);
    schedule_local(s, GW, SIM_JOB_STREAM, l + g->cfg.stream_batch_ms * 1000u, EV_GW_STREAM);
    schedule_local(s, GW, SIM_JOB_LORA, l + g->cfg.lora_period_ms * 1000u, EV_GW_LORA);
    schedule_local(s, GW, SIM_JOB_PUMP, l + 20000u, EV_GW_PUMP);
    schedule_local(s, GW, SIM_JOB_COCKPIT, l + g->cfg.cockpit_period_ms * 1000u, EV_GW_COCKPIT);
    schedule_local(s, GW, SIM_JOB_HEALTH, l + LBR_GW_HEALTH_PERIOD_MS * 1000u, EV_HEALTH);
    schedule_local(s, GW, SIM_JOB_LOGIC, l + 1000000u / LBR_LOGIC_TICK_HZ, EV_GW_LOGIC);
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
}

static void boot_cockpit(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    world_bind_sensors(s, i);
    if (n->cockpit == NULL) {
        n->cockpit = calloc(1, sizeof *n->cockpit);
    }
    lbr_cockpit_hal_t hal = k_cockpit_hal;
    hal.ctx               = &s_ctx[i];
    lbr_cockpit_core_init(n->cockpit, &n->cfg, &hal);
    for (size_t k = 0; k < n->cfg.n_acq; k++) {
        lbr_cockpit_core_slot_start(n->cockpit, k);
        schedule_acq(s, i, k);
    }
}

void boot_node(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    n->booted     = true;
    n->boot_at    = (int64_t)s->now;
    n->boot_count++;
    lbr_time_servo_init(&n->servo);
    for (int k = 0; k < s->n_nodes; k++) {
        s_ctx[k] = (hal_ctx_t){s, k};
    }
    switch (n->kind) {
        case SIM_NODE_CENTRAL:
            boot_gateway(s);
            break;
        case SIM_NODE_COCKPIT:
            boot_cockpit(s, i);
            break;
        default:
            boot_can_node(s, i);
            break;
    }
}

void power_off(sim_t *s, int i)
{
    sim_node_t *n = &s->nodes[i];
    n->booted     = false;
    n->gen++;                       /* drop its pending timers */
    lbr_time_servo_init(&n->servo); /* its clock state is gone with the power */
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
