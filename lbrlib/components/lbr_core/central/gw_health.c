#include <string.h>

#include "gw_internal.h"
#include "lbr/bytes.h"
#include "lbr/nodes.h"
#include "lbr/status.h"
#include "lbr/version.h"

/* The Central's heartbeat and health record: its own state, its links, its
 * log, and the nodes it has lost or never heard. */

static void link_totals(const lbr_gw_core_t *c, uint32_t *dropped, uint32_t *errors)
{
    lbr_transport_t *links[] = {c->links.usb, c->links.wifi, c->links.lora, c->links.ble,
                                c->links.cockpit};
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

static void build_health(lbr_gw_core_t *c, lbr_health_record_t *h, uint8_t *err_flags)
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
    flags |= c->sim ? LBR_HF_SIM_SENSORS : 0;

    uint32_t sensor_ok  = 0;
    uint32_t calibrated = 0;
    size_t   n_acq = c->boot.n_acq < LBR_NODE_MAX_SENSORS ? c->boot.n_acq : LBR_NODE_MAX_SENSORS;
    for (size_t i = 0; i < n_acq; i++) {
        /* Healthy: the driver is up and its latest reading had every channel. */
        bool ok = atomic_load(&c->slot[i].acq.healthy) && atomic_load(&c->slot[i].acq.complete);
        sensor_ok |= ok ? (1u << i) : 0u;
        calibrated |= atomic_load(&c->slot[i].acq.calibrated) ? (1u << i) : 0u;
    }
    uint32_t all_sensors = (1u << n_acq) - 1u;
    gw_lock(c, LBR_GW_LOCK_STATE);
    bool cockpit_up = c->cockpit.up;
    /* Drops inside the Central, never silent. Its own state: car logic outputs
     * refused, frames from nodes beyond the node table. The live links: LoRa
     * events evicted and nodes it cannot track, cockpit datagrams lost or
     * rejected. */
    uint32_t state_dropped = c->logic.outputs_refused + c->nodes.table_full;
    uint32_t link_lost     = c->lora_policy.events_dropped + c->lora_policy.nodes_untracked +
                             c->cockpit.packet_gaps + c->cockpit.rejected;
    gw_unlock(c, LBR_GW_LOCK_STATE);
    bool cockpit_down = c->cfg.cockpit_enable && c->links.cockpit != NULL && !cockpit_up;

    lbr_node_state_t state = LBR_NODE_RUN;
    if (!c->can_up) {
        state = LBR_NODE_FAULT;
    } else if (!log_open || !cs.running || !c->boot.config_valid || sensor_ok != all_sensors) {
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
        .overflow_count = ls.records_dropped + ls.records_rejected_closed + ls.records_lost_write +
                          atomic_load(&c->overruns) + c->early_dropped + state_dropped,
        .rejected_count = cs.rx_foreign + atomic_load(&c->out_of_range),
        .write_error_count = ls.write_errors + ls.sync_errors,
        .transport_error_count =
            link_dropped + link_errors + atomic_load(&c->sync_tx_fail) + link_lost,
        .can_tec              = cs.tec,
        .can_rec              = cs.rec,
        .can_bus_off_count    = cs.bus_off_count,
        .can_tx_dropped       = cs.tx_dropped,
        .can_rx_dropped       = cs.rx_dropped,
        .can_bus_errors       = cs.bus_errors,
        .sensor_status        = sensor_ok,
        .calibration_state    = calibrated,
        .last_sample_age_ms   = lbr_age_ms(now, atomic_load(&c->last_sample_us)),
        .last_sd_write_age_ms = lbr_age_ms(now, ls.last_write_us),
    };
    c->prev_rx     = rx;
    c->prev_logged = logged;

    uint32_t loss =
        h->overflow_count + h->can_tx_dropped + h->can_rx_dropped + h->transport_error_count;
    uint8_t e = 0;
    e |= (sensor_ok != all_sensors || (calibrated & all_sensors) != all_sensors) ? LBR_HB_ERR_SENSOR
                                                                                 : 0u;
    e |= (cs.tec >= 128 || cs.rec >= 128 || !cs.running || cs.bus_off_count != c->last_bus_off)
             ? LBR_HB_ERR_CAN
             : 0u;
    e |= c->boot.config_valid ? 0u : LBR_HB_ERR_CONFIG;
    e |= log_open ? 0u : LBR_HB_ERR_STORAGE;
    e |= c->sim ? LBR_HB_ERR_SIM : 0u;
    e |= loss != c->prev_loss ? LBR_HB_ERR_LOSS : 0u;
    e |= cockpit_down ? LBR_HB_ERR_LINK : 0u;
    c->prev_loss = loss;
    *err_flags   = e;
}

void lbr_gw_core_heartbeat(lbr_gw_core_t *c, lbr_health_record_t *out)
{
    lbr_health_record_t h;
    uint8_t             lost[LBR_NODE_TABLE_SIZE];
    uint8_t             missing[LBR_NODE_TABLE_SIZE];
    uint8_t             err_flags;
    lbr_can_frame_t     f;

    build_health(c, &h, &err_flags);
    lbr_can_encode_heartbeat(NODE_ID, h.state, err_flags, &f);
    gw_tx(c, &f);
    for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
        lbr_can_encode_health_page(NODE_ID, &h, p, &f);
        gw_tx(c, &f);
    }
    if (h.can_bus_off_count != c->last_bus_off) {
        c->last_bus_off = h.can_bus_off_count;
        lbr_gw_core_event(c, LBR_EVT_CAN_BUS_OFF, (uint16_t)c->last_bus_off, true);
    }

    gw_lock(c, LBR_GW_LOCK_STATE);
    c->last_health = h;
    lbr_lora_policy_offer_health(&c->lora_policy, &h);
    lbr_logic_announce(&c->logic);
    uint64_t now = c->hal.now_us(c->hal.ctx);
    gw_log_tick(c, LBR_TICK_HEARTBEAT, now);
    size_t n_lost, n_missing;
    lbr_replay_check_nodes(&c->nodes, now, c->can_up, c->listen_since_us, lost, &n_lost, missing,
                           &n_missing, LBR_NODE_TABLE_SIZE);
    gw_unlock(c, LBR_GW_LOCK_STATE);
    for (size_t i = 0; i < n_lost; i++) {
        lbr_gw_core_event(c, LBR_EVT_NODE_LOST, lost[i], true);
    }
    for (size_t i = 0; i < n_missing; i++) {
        gw_say(c, LBR_LOG_LVL_WARN, "node %s (0x%02x) expected but never heard",
               lbr_node_name(missing[i]), missing[i]);
        lbr_gw_core_event(c, LBR_EVT_NODE_MISSING, missing[i], true);
    }
    if (out != NULL) {
        *out = h;
    }
}
