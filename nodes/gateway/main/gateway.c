#include "gateway.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gw_store.h"
#include "lbr/can.h"
#include "lbr/can_bus.h"
#include "lbr/crc32.h"
#include "lbr/gw_config.h"
#include "lbr/health.h"
#include "lbr/log_service.h"
#include "lbr/lora_policy.h"
#include "lbr/node.h"
#include "lbr/node_table.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/status.h"
#include "lbr/timebase.h"
#include "lbr/transport_links.h"
#include "lbr/version.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "gateway";

#define NODE_ID          LBR_NODE_GATEWAY
#define RX_TASK_PRIO     12
#define SYNC_TASK_PRIO   11
#define STREAM_TASK_PRIO 7
#define HEALTH_TASK_PRIO 6
#define LORA_TASK_PRIO   5
#define HEALTH_PERIOD_MS 1000
#define CONFIG_TEXT_MAX  2048

static lbr_gw_config_t    s_gw;
static char               s_config_text[CONFIG_TEXT_MAX];
static lbr_reset_reason_t s_reset_reason;
static uint32_t           s_boot_count;
static bool               s_config_valid = true;
static bool               s_can_up;
static lbr_logger_t      *s_log;

/* Live links (NULL when disabled or unavailable). */
static lbr_transport_t *s_usb;
static lbr_transport_t *s_wifi;
static lbr_transport_t *s_lora;
static lbr_transport_t *s_ble;

/* Node table and LoRa policy are shared by the RX, health, and LoRa tasks. */
static SemaphoreHandle_t s_state_lock;
static lbr_node_table_t  s_nodes;
static lbr_lora_policy_t s_lora_policy;

/* Stream batcher (USB + WiFi): raw CAN frame records. */
static SemaphoreHandle_t s_stream_lock;
static uint8_t           s_stream_pkt[LBR_PKT_MAX_SIZE];
static lbr_pkt_builder_t s_stream;
static bool              s_stream_open;

/* Event streams of the gateway's own EVENT / FAULT samples. */
static SemaphoreHandle_t s_evt_lock;
static lbr_stream_t      s_evt_stream;
static lbr_stream_t      s_fault_stream;

static atomic_uint s_frames_rx;
static atomic_uint s_frames_logged;
static atomic_uint s_sync_tx_fail;

/* ------------------------------------------------------------------------ */
/* Frame fan-out                                                             */

static void stream_flush_locked(void)
{
    if (!s_stream_open) {
        return;
    }
    size_t n = lbr_pkt_finish(&s_stream);
    if (n > 0) {
        if (s_usb != NULL) {
            s_usb->send(s_usb, s_stream_pkt, n);
        }
        if (s_wifi != NULL) {
            s_wifi->send(s_wifi, s_stream_pkt, n);
        }
    }
    s_stream_open = false;
}

static void stream_add(const uint8_t *rec, size_t len)
{
    if (s_usb == NULL && s_wifi == NULL) {
        return;
    }
    xSemaphoreTake(s_stream_lock, portMAX_DELAY);
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!s_stream_open) {
            lbr_pkt_begin(&s_stream, s_stream_pkt, sizeof s_stream_pkt, LBR_LINK_USB, NODE_ID, 0,
                          lbr_time_now_us());
            s_stream_open = true;
        }
        if (lbr_pkt_add(&s_stream, LBR_REC_CAN_FRAME, rec, len)) {
            break;
        }
        stream_flush_locked();
    }
    xSemaphoreGive(s_stream_lock);
}

/*
 * Every frame, received or our own, goes through here: authoritative log
 * first, then the live views. `t_us` is the shared time base.
 */
static void handle_frame(const lbr_can_frame_t *f, uint64_t t_us, bool own_tx)
{
    uint8_t flags = LBR_CAN_REC_FLAG_EXT | (own_tx ? LBR_CAN_REC_FLAG_OWN_TX : 0u);
    uint8_t rec[LBR_CAN_FRAME_REC_SIZE];
    lbr_can_frame_rec_encode(t_us, flags, f, rec, sizeof rec);

    if (lbr_log_append_raw(s_log, LBR_REC_CAN_FRAME, rec, sizeof rec) == 0) {
        atomic_fetch_add(&s_frames_logged, 1);
    }
    stream_add(rec, sizeof rec);

    lbr_sample_t s;
    bool         is_sample = lbr_can_decode_sample(f, t_us, &s);

    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    lbr_node_table_event_t ev = own_tx ? LBR_NT_NONE : lbr_node_table_on_frame(&s_nodes, f, t_us);
    if (is_sample) {
        lbr_lora_policy_offer_sample(&s_lora_policy, &s);
    } else if (!own_tx) {
        lbr_can_id_t id = lbr_can_id_parse(f->id);
        /* Offer health to LoRa when the state page (0) arrives. */
        if (id.kind == LBR_CAN_KIND_SYSTEM && id.sub == LBR_SYS_HEALTH_BASE) {
            const lbr_node_entry_t *e = lbr_node_table_find(&s_nodes, id.source);
            if (e != NULL) {
                lbr_lora_policy_offer_health(&s_lora_policy, &e->health);
            }
        }
    }
    xSemaphoreGive(s_state_lock);

    if (ev == LBR_NT_FOUND) {
        uint8_t source = lbr_can_id_parse(f->id).source;
        ESP_LOGI(TAG, "node %s (0x%02x) on the bus", lbr_node_name(source), source);
    }
}

/* Transmit one of our own frames and record it like any other. */
static void gw_tx(const lbr_can_frame_t *f)
{
    if (s_can_up) {
        lbr_can_bus_send(f);
    }
    handle_frame(f, lbr_time_now_us(), true);
}

static void gw_event(lbr_event_code_t code, uint16_t arg, bool fault)
{
    lbr_reading_t   r = {.valid = true, .v.u32 = lbr_event_value(code, arg)};
    lbr_sample_t    s;
    uint32_t        pub_seq = 0;
    lbr_can_frame_t f;

    ESP_LOGW(TAG, "%s 0x%04x arg %u", fault ? "FAULT" : "event", (unsigned)code, (unsigned)arg);
    if (s_evt_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_evt_lock, portMAX_DELAY);
    lbr_stream_sample(fault ? &s_fault_stream : &s_evt_stream, 1, NODE_ID,
                      fault ? LBR_SIG_FAULT : LBR_SIG_EVENT, &r, lbr_time_now_us(), true, &s,
                      &pub_seq);
    xSemaphoreGive(s_evt_lock);
    s.seq = pub_seq;
    if (lbr_can_encode_sample(&s, &f)) {
        gw_tx(&f);
    }
}

static void on_log_event(uint16_t code, bool fault)
{
    gw_event((lbr_event_code_t)code, 0, fault);
}

/* ------------------------------------------------------------------------ */
/* Tasks                                                                     */

static void rx_task(void *arg)
{
    (void)arg;
    lbr_can_rx_t rx;
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        if (!lbr_can_bus_receive(&rx, 500)) {
            continue;
        }
        atomic_fetch_add(&s_frames_rx, 1);
        /* Master clock: local and shared time are the same. */
        handle_frame(&rx.frame, lbr_time_local_to_shared(rx.rx_local_us), false);
    }
}

static void sync_task(void *arg)
{
    (void)arg;
    uint16_t   epoch = (uint16_t)(esp_random() | 1u); /* new timeline every boot */
    uint8_t    seq   = 0;
    TickType_t wake  = xTaskGetTickCount();

    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / LBR_TIME_SYNC_HZ));
        esp_task_wdt_reset();
        if (!s_can_up) {
            continue;
        }
        lbr_can_frame_t f;
        uint64_t        tx_local;
        lbr_can_bus_arm_sync_tx();
        lbr_can_encode_time_sync(NODE_ID, seq, epoch, &f);
        gw_tx(&f);
        if (lbr_can_bus_wait_sync_tx_done(&tx_local, 50)) {
            lbr_can_encode_time_follow_up(NODE_ID, seq, lbr_time_local_to_shared(tx_local), &f);
            gw_tx(&f);
        } else {
            atomic_fetch_add(&s_sync_tx_fail, 1); /* no ACK / bus-off: nodes keep slewing */
        }
        seq++;
    }
}

static void stream_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_gw.stream_batch_ms));
        xSemaphoreTake(s_stream_lock, portMAX_DELAY);
        stream_flush_locked();
        xSemaphoreGive(s_stream_lock);
    }
}

static void lora_task(void *arg)
{
    (void)arg;
    uint8_t pkt[LBR_PKT_LORA_MAX];
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_gw.lora_period_ms));
        if (!lbr_link_lora_ready()) {
            continue; /* radio busy or duty budget spent: the policy keeps the latest */
        }
        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        size_t n =
            lbr_lora_policy_build(&s_lora_policy, lbr_time_now_us(), NODE_ID, 0, pkt, sizeof pkt);
        xSemaphoreGive(s_state_lock);
        if (n > 0) {
            s_lora->send(s_lora, pkt, n);
        }
    }
}

static void link_totals(uint32_t *dropped, uint32_t *errors)
{
    lbr_transport_t *links[] = {s_usb, s_wifi, s_lora, s_ble};
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

static void build_health(lbr_health_record_t *h)
{
    static uint32_t prev_rx, prev_logged;
    lbr_can_stats_t cs;
    lbr_log_stats_t ls;
    uint32_t        link_dropped, link_errors;

    lbr_can_bus_get_stats(&cs);
    lbr_log_get_stats(s_log, &ls);
    link_totals(&link_dropped, &link_errors);

    bool     log_open = lbr_log_is_open(s_log);
    uint16_t flags    = LBR_HF_TIME_SYNCED | LBR_HF_TIME_MASTER;
    flags |= lbr_log_service_sd_present() ? LBR_HF_SD_PRESENT : 0;
    flags |= log_open ? LBR_HF_SD_LOGGING : 0;
    flags |= s_config_valid ? LBR_HF_CONFIG_VALID : 0;
    flags |= cs.running ? LBR_HF_CAN_UP : 0;

    lbr_node_state_t state = LBR_NODE_RUN;
    if (!s_can_up) {
        state = LBR_NODE_FAULT;
    } else if (!log_open || !cs.running || !s_config_valid) {
        state = LBR_NODE_DEGRADED; /* no authoritative log is the worst case short of no bus */
    }

    uint32_t rx     = atomic_load(&s_frames_rx);
    uint32_t logged = atomic_load(&s_frames_logged);
    uint64_t now    = lbr_time_now_us();
    *h              = (lbr_health_record_t){
        .node_id               = NODE_ID,
        .state                 = (uint8_t)state,
        .flags                 = flags,
        .fw_version            = LBR_FW_VERSION,
        .protocol_version      = LBR_PROTOCOL_VERSION,
        .boot_count            = (uint16_t)s_boot_count,
        .catalog_hash          = lbr_catalog_hash(),
        .config_crc            = lbr_crc32(s_config_text, strlen(s_config_text)),
        .uptime_ms             = (uint32_t)(lbr_time_mono_us() / 1000u),
        .reset_reason          = (uint8_t)s_reset_reason,
        .acquire_hz            = (uint16_t)((rx - prev_rx) * 1000u / HEALTH_PERIOD_MS),
        .publish_hz            = 0,
        .log_hz                = (uint16_t)((logged - prev_logged) * 1000u / HEALTH_PERIOD_MS),
        .queue_high_water      = ls.queue_high_water,
        .overflow_count        = ls.records_dropped + ls.records_rejected_closed,
        .rejected_count        = cs.rx_foreign,
        .write_error_count     = ls.write_errors + ls.sync_errors,
        .transport_error_count = link_dropped + link_errors + atomic_load(&s_sync_tx_fail),
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
    prev_rx     = rx;
    prev_logged = logged;
}

static void health_task(void *arg)
{
    (void)arg;
    TickType_t          wake         = xTaskGetTickCount();
    uint32_t            last_bus_off = 0;
    lbr_health_record_t h;
    uint8_t             lost[LBR_NODE_TABLE_SIZE];

    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(HEALTH_PERIOD_MS));
        esp_task_wdt_reset();
        build_health(&h);

        for (unsigned p = 0; p < LBR_HEALTH_PAGE_COUNT; p++) {
            lbr_can_frame_t f;
            lbr_can_encode_health_page(NODE_ID, &h, p, &f);
            gw_tx(&f);
        }
        if (h.can_bus_off_count != last_bus_off) {
            last_bus_off = h.can_bus_off_count;
            gw_event(LBR_EVT_CAN_BUS_OFF, (uint16_t)last_bus_off, true);
        }

        xSemaphoreTake(s_state_lock, portMAX_DELAY);
        lbr_lora_policy_offer_health(&s_lora_policy, &h);
        size_t n_lost = lbr_node_table_expire(&s_nodes, lbr_time_now_us(), lost, sizeof lost);
        xSemaphoreGive(s_state_lock);
        for (size_t i = 0; i < n_lost; i++) {
            gw_event(LBR_EVT_NODE_LOST, lost[i], true);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Link hooks                                                                */

static size_t hook_status(char *buf, size_t cap)
{
    lbr_health_record_t h;
    char                name[LBR_SD_NAME_MAX];
    build_health(&h);
    lbr_log_service_file_name(name);
    xSemaphoreTake(s_state_lock, portMAX_DELAY);
    size_t n = lbr_status_format(&h, name, &s_nodes, lbr_time_now_us(), buf, cap);
    xSemaphoreGive(s_state_lock);
    return n;
}

static void hook_before_reboot(void)
{
    lbr_log_close(s_log, lbr_time_now_us());
}

static int log_vprintf(const char *fmt, va_list ap)
{
    /* Log lines go to the USB link (when up) and into the authoritative log. */
    char    line[192];
    va_list copy;
    va_copy(copy, ap);
    int n = vsnprintf(line, sizeof line, fmt, copy);
    va_end(copy);
    if (n > 0 && s_log != NULL) {
        size_t len = (size_t)n < sizeof line ? (size_t)n : sizeof line - 1;
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        lbr_log_append_text(s_log, line);
    }
    return s_usb != NULL ? lbr_usb_log_vprintf(fmt, ap) : vprintf(fmt, ap);
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

static void build_config_text(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    lbr_can_bus_config_t  can = lbr_can_bus_default_config();
    int n = snprintf(s_config_text, sizeof s_config_text,
                     "node=gateway\nnode_id=%u\nfw_version=%u.%u.%u\nbuild=%s\nidf=%s\n"
                     "protocol_version=%u\ncatalog_hash=%08lx\ncan_bitrate=%lu\n"
                     "time_sync_hz=%u\nlora_freq_hz=%d\nlora_sf=%d\nlora_bw_khz=%d\n"
                     "lora_cr=4/%d\nlora_tx_dbm=%d\nexpected_nodes=",
                     NODE_ID, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR, LBR_FW_VERSION_PATCH,
                     app->version, app->idf_ver, LBR_PROTOCOL_VERSION,
                     (unsigned long)lbr_catalog_hash(), (unsigned long)can.bitrate,
                     LBR_TIME_SYNC_HZ, CONFIG_LBR_LORA_FREQ_HZ, CONFIG_LBR_LORA_SF,
                     CONFIG_LBR_LORA_BW_KHZ, CONFIG_LBR_LORA_CR + 4, CONFIG_LBR_LORA_TX_DBM);
    for (uint8_t id = LBR_NODE_IMU_GPS; id <= LBR_NODE_DASHBOARD && n > 0; id++) {
        n += snprintf(s_config_text + n, sizeof s_config_text - (size_t)n, "%s%s",
                      id == LBR_NODE_IMU_GPS ? "" : ",", lbr_node_name(id));
    }
    if (n > 0 && (size_t)n < sizeof s_config_text) {
        n += snprintf(s_config_text + n, sizeof s_config_text - (size_t)n, "\n");
        gw_store_get(s_config_text + n, sizeof s_config_text - (size_t)n);
    }
}

static void start_links(void)
{
    static const lbr_link_hooks_t hooks = {
        .status        = hook_status,
        .config_get    = gw_store_get,
        .config_put    = gw_store_put,
        .before_reboot = hook_before_reboot,
    };
    if (s_gw.usb_enable && (s_usb = lbr_link_usb_start(&hooks)) == NULL) {
        ESP_LOGE(TAG, "USB link unavailable");
    }
    if (s_gw.wifi_enable && (s_wifi = lbr_link_wifi_start(&hooks)) == NULL) {
        ESP_LOGW(TAG, "WiFi link unavailable");
    }
    if (s_gw.ble_enable && (s_ble = lbr_link_ble_start(&hooks)) == NULL) {
        ESP_LOGW(TAG, "BLE link unavailable");
    }
    if (s_gw.lora_enable) {
        s_lora = lbr_link_lora_start(s_gw.lora_duty_permille);
        if (s_lora == NULL) {
            ESP_LOGW(TAG, "LoRa link unavailable (pins TBD?)");
        } else {
            xTaskCreate(lora_task, "gw_lora", 4096, NULL, LORA_TASK_PRIO, NULL);
        }
    }
}

void gateway_run(void)
{
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    s_reset_reason = lbr_health_reset_reason();
    s_boot_count   = lbr_health_boot_count();
    s_state_lock   = xSemaphoreCreateMutex();
    s_stream_lock  = xSemaphoreCreateMutex();
    s_evt_lock     = xSemaphoreCreateMutex();
    lbr_time_set_master();

    /* Validate configuration before the session starts. */
    s_config_valid = gw_store_load(&s_gw) >= 0;
    build_config_text();
    lbr_node_table_init(&s_nodes, (uint64_t)s_gw.node_timeout_ms * 1000u);
    lbr_lora_policy_init(&s_lora_policy, lbr_lora_default_items, lbr_lora_default_item_count, 2000);

    /* Authoritative log first: everything after this point is recorded. */
    lbr_log_service_config_t lc = {
        .node_id          = NODE_ID,
        .reset_reason     = (uint8_t)s_reset_reason,
        .boot_count       = s_boot_count,
        .config_text      = s_config_text,
        .queue_size       = (size_t)CONFIG_LBR_LOG_QUEUE_KB * 1024u,
        .sync_interval_ms = CONFIG_LBR_LOG_SYNC_INTERVAL_MS,
        .now_us           = lbr_time_now_us,
        .on_event         = on_log_event,
    };
    if (!lbr_log_service_start(&lc)) {
        ESP_LOGE(TAG, "log service failed to start");
    }
    s_log = lbr_log_service_logger();

    start_links();
    esp_log_set_vprintf(log_vprintf);

    ESP_LOGI(TAG, "gateway fw %u.%u.%u protocol %u boot #%lu reset %d", LBR_FW_VERSION_MAJOR,
             LBR_FW_VERSION_MINOR, LBR_FW_VERSION_PATCH, LBR_PROTOCOL_VERSION,
             (unsigned long)s_boot_count, (int)s_reset_reason);

    lbr_can_bus_config_t can = lbr_can_bus_default_config();
    can.accept_all           = true;
    lbr_can_err_t can_rc     = lbr_can_bus_start(&can);
    s_can_up                 = can_rc == LBR_CAN_OK;

    gw_event(LBR_EVT_BOOT, (uint16_t)s_reset_reason, false);
    if (!s_config_valid) {
        gw_event(LBR_EVT_CONFIG_INVALID, 0, true);
    }
    if (can_rc == LBR_CAN_ERR_PINS_UNSET) {
        gw_event(LBR_EVT_CAN_PINS_UNSET, 0, true);
    }

    if (s_can_up) {
        xTaskCreate(rx_task, "gw_rx", 4096, NULL, RX_TASK_PRIO, NULL);
        xTaskCreate(sync_task, "gw_sync", 3072, NULL, SYNC_TASK_PRIO, NULL);
    }
    xTaskCreate(stream_task, "gw_stream", 3072, NULL, STREAM_TASK_PRIO, NULL);
    xTaskCreate(health_task, "gw_health", 4096, NULL, HEALTH_TASK_PRIO, NULL);

    /* Reaching here means the image boots and its services start: confirm it
     * so the bootloader does not roll an OTA update back. */
    esp_ota_mark_app_valid_cancel_rollback();

    for (;;) {
        vTaskDelay(portMAX_DELAY);
    }
}
