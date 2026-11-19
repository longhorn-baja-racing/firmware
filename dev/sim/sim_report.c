/*
 * Metrics (shared by expect and the snapshot) and the JSON snapshot for the UI.
 */
#include "sim_internal.h"

/* ------------------------------------------------------------------------ */
/* Metrics (shared by expect and the snapshot)                               */

int64_t sync_error_us(sim_t *s, int i)
{
    if (i == GW || s->nodes[i].kind == SIM_NODE_COCKPIT) {
        return 0; /* the reference itself, or the cockpit: it keeps no shared clock */
    }
    if (!s->nodes[i].booted || !s->nodes[GW].booted || !s->nodes[i].servo.synced) {
        return INT32_MAX; /* nothing to compare with: never reads as in sync */
    }
    uint64_t est   = lbr_time_servo_map(&s->nodes[i].servo, sim_local_us(s, i));
    uint64_t truth = sim_local_us(s, GW); /* shared time is the Central's clock */
    return (int64_t)(est - truth);
}

bool wireless(const sim_node_t *n)
{
    return n->kind == SIM_NODE_COCKPIT;
}

/* The cockpit as the Central sees it: up while its datagrams arrive. */
bool link_up(const sim_t *s, int i)
{
    return s->nodes[i].kind == SIM_NODE_COCKPIT && s->gw.core != NULL && s->nodes[GW].booted &&
           s->gw.core->cockpit.up;
}

const char *state_of(const sim_t *s, int i)
{
    const sim_node_t *n = &s->nodes[i];
    if (!n->powered) {
        return "OFF";
    }
    if (!n->booted) {
        return "BOOT";
    }
    if (n->kind == SIM_NODE_COCKPIT) {
        return link_up(s, i) ? "RUN" : "LOST";
    }
    if (n->last_health.node_id == 0) {
        return "BOOT";
    }
    return lbr_node_state_name(n->last_health.state);
}

double bus_load_pct(const sim_t *s)
{
    return (double)s->bus.busy_us_last * 100.0 / 1e6;
}

bool gw_sees(const sim_t *s, int i)
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
        } else if (n->kind == SIM_NODE_COCKPIT && n->cockpit != NULL) {
            a = &n->cockpit->slot[k];
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
    static const char *kinds[] = {"central", "can", "cockpit"};
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
                n->booted ? (double)sim_local_us(s, i) / 1e6 : 0.0, (unsigned)h->flags);
        if (wireless(n)) {
            fprintf(o,
                    "\"wifi\":{\"up\":%s,\"link\":%s,\"loss_permille\":%u,\"sent\":%u,"
                    "\"delivered\":%u,\"dropped\":%u,\"gaps\":%u}",
                    n->wifi.up ? "true" : "false", link_up(s, i) ? "true" : "false",
                    (unsigned)n->wifi.loss_permille, (unsigned)n->wifi.sent,
                    (unsigned)n->wifi.delivered, (unsigned)n->wifi.dropped,
                    n->cockpit != NULL ? (unsigned)atomic_load(&n->cockpit->packet_gaps) : 0u);
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
            "\"lora_records\":%u,\"cockpit_pkts\":%u,\"sync_fail\":%u},",
            (unsigned)s->gw.epoch, s->nodes[GW].sd_inserted ? "true" : "false",
            (s->nodes[GW].booted && s->gw.log.open) ? "true" : "false", s->nodes[GW].keeper.name,
            (unsigned long long)s->nodes[GW].log_bytes, (unsigned)ls.records_written,
            (unsigned)(ls.records_dropped + ls.records_lost_write + ls.records_rejected_closed),
            (unsigned)ls.write_errors, (unsigned)ls.queue_high_water,
            (unsigned)s->gw.usb_st.packets_sent, (unsigned)s->gw.usb_st.bytes_sent,
            (unsigned)s->gw.wifi_st.packets_sent, (unsigned)s->gw.lora_st.packets_sent,
            (unsigned)s->gw.lora_st.bytes_sent, (unsigned)s->gw.last_lora_records,
            (unsigned)s->gw.cockpit_st.packets_sent, (unsigned)s->gw.sync_timeouts);

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

    /* What the cockpit display shows: the cockpit core's own tables. */
    fprintf(o, "\"cockpit\":[");
    int cockpit = sim_node_by_id(s, LBR_NODE_COCKPIT);
    if (cockpit >= 0 && s->nodes[cockpit].booted && s->nodes[cockpit].cockpit != NULL) {
        static lbr_value_table_t  values;
        static lbr_cockpit_node_t dnodes[LBR_COCKPIT_MAX_NODES];
        uint64_t                  dnow = sim_local_us(s, cockpit);
        lbr_cockpit_core_snapshot(s->nodes[cockpit].cockpit, &values, dnodes);
        for (size_t k = 0; k < lbr_cockpit_signal_count; k++) {
            lbr_sample_t             smp;
            const lbr_signal_info_t *info = lbr_signal_info(lbr_cockpit_signals[k]);
            lbr_value_state_t st = lbr_value_table_get(&values, lbr_cockpit_signals[k], dnow, &smp);
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
