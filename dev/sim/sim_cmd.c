/*
 * The command language: scenario commands and expect checks.
 */
#include "sim_internal.h"

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

/* True if some part or wired input produces `sig` (so it can be opened). */
static bool has_circuit(const sim_t *s, lbr_signal_id_t sig)
{
    for (int i = 0; i < s->n_nodes; i++) {
        const sim_node_t *n = &s->nodes[i];
        for (size_t k = 0; n->plan != NULL && k < n->plan->n_slots; k++) {
            const lbr_slot_t *sl   = &n->plan->slots[k];
            sim_slot_kind_t   kind = n->slots[k].kind;
            for (size_t c = 0; c < sl->n_signals; c++) {
                if (sl->signals[c] == sig) {
                    return kind == SIM_SLOT_IO || kind == SIM_SLOT_DIRECT;
                }
            }
        }
    }
    return false;
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

bool compare(double got, const char *op, double want)
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
    sim_node_t *n = &s->nodes[i];
    if (strcmp(m, "seen") == 0) {
        *v = (i == GW || gw_sees(s, i)) ? 1 : 0;
    } else if (strcmp(m, "synced") == 0) {
        /* The cockpit keeps no shared clock: nothing to sync, reported 1. */
        *v = (n->servo.synced || n->kind == SIM_NODE_COCKPIT) ? 1 : 0;
    } else if (strcmp(m, "link") == 0) {
        *v = !wireless(n) || link_up(s, i) ? 1 : 0;
    } else if (strcmp(m, "gaps") == 0) {
        /* Datagrams from the Central that never reached the cockpit. */
        *v = n->kind == SIM_NODE_COCKPIT && n->cockpit != NULL
                 ? atomic_load(&n->cockpit->packet_gaps)
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
        /* What the Central's own pump task would have written by now. */
        if (s->nodes[GW].booted) {
            lbr_log_pump(&s->gw.log, h_now(&s_ctx[GW]));
        }
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

/* Group targets of expect: all (every node but the Central), can (the CAN
 * sensor nodes, corners included), corners (the four wheel hub nodes),
 * wireless (the cockpit). */
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
            return n->id >= LBR_NODE_CORNER_FL && n->id <= LBR_NODE_CORNER_BR;
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
            if (strcmp(op, "==") != 0 && strcmp(op, "!=") != 0) {
                return err(s, "state takes == or !=");
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

    /* A car logic output (out:<key>) as last written: valid, value, status. */
    if (strncmp(target, "out:", 4) == 0) {
        const lbr_output *o =
            s->gw.core != NULL ? lbr_logic_find(&s->gw.core->logic, target + 4) : NULL;
        if (strcmp(metric, "status") == 0) {
            const char *st = o == NULL || !o->written ? "NEVER" : lbr_status_name(o->last.st);
            bool        eq = strcmp(st, want) == 0;
            check_result(s, strcmp(op, "!=") == 0 ? !eq : eq, what, st);
            return 0;
        }
        double ov = 0;
        if (strcmp(metric, "valid") == 0) {
            ov = o != NULL && o->written && lbr_ok(o->last) ? 1 : 0;
        } else if (strcmp(metric, "value") == 0) {
            if (o == NULL || !o->written || !lbr_ok(o->last)) {
                check_result(s, false, what, o == NULL ? "never written" : "unavailable");
                return 0;
            }
            ov = o->last.v;
        } else {
            return err(s, "unknown output metric '%s' (valid, value, status)", metric);
        }
        snprintf(got, sizeof got, "%g", ov);
        check_result(s, compare(ov, op, atof(want)), what, got);
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
            if (!sv->seen || !sv->s.valid) {
                check_result(s, false, what, sv->seen ? "unavailable" : "never seen");
                return 0;
            }
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
               strcmp(metric, "cockpit_pkts") != 0) {
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
        } else if (strcmp(metric, "cockpit_pkts") == 0) {
            v = s->gw.cockpit_st.packets_sent;
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
        if (s->nodes[i].slots[k].kind == SIM_SLOT_FIRMWARE) {
            return err(s, "%s.%s is computed by the firmware, not a part: nothing to unplug",
                       tok[1], tok[2]);
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
        if ((strcmp(c, "open") == 0 || strcmp(c, "close") == 0) && !has_circuit(s, info->id)) {
            return err(s,
                       "%s has no wired input or part to open (it is computed or comes "
                       "from a receiver)",
                       info->name);
        }
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
        /* wifi <node> up|down|loss <permille>: the cockpit's link. */
        int i =
            nt > 2 ? need_node(s, tok[1]) : err(s, "usage: wifi <node> up|down|loss <permille>");
        if (i < 0) {
            return -1;
        }
        sim_node_t *n = &s->nodes[i];
        if (!wireless(n)) {
            return err(s, "%s has no WiFi link (only the cockpit does)", n->name);
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
        n->boot_at        = (int64_t)s->now - (int64_t)((double)local / (1.0 + ppm * 1e-6));
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
        if (i != GW && (n->plan == NULL || !n->plan->local_log)) {
            return err(s, "%s has no SD card", n->name);
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
