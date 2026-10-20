#include "central.h"

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
#include "central_plan.h"
#include "esp_timer.h"
#include "gw_store.h"
#include "lbr/can_bus.h"
#include "lbr/dash_link.h"
#include "lbr/gw_core.h"
#include "lbr/log_service.h"
#include "lbr/nmea.h"
#include "lbr/node_app.h"
#include "lbr/nodes.h"
#include "lbr/timebase.h"
#include "lbr/transport_links.h"
#include "lbr/version.h"
#include "lbr/wlink.h"
#include "lbr/wlink_esp.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/*
 * ESP32-C5 platform for the Central Controller: FreeRTOS tasks, TWAI, SD log
 * service, the live links, its GPS and dashboard-status slots, the dashboard
 * datagrams, NVS config, and OTA confirmation. All decisions are in
 * gw_core.c, which the simulator runs unchanged.
 */

static const char *TAG = "central";

#define RX_TASK_PRIO     12
#define SYNC_TASK_PRIO   11
#define STREAM_TASK_PRIO 7
#define HEALTH_TASK_PRIO 6
#define LORA_TASK_PRIO   5
#define ACQ_TASK_PRIO    10
#define DASH_TASK_PRIO   5
#define CONFIG_TEXT_MAX  2048

static lbr_gw_core_t     s_core;
static lbr_gw_config_t   s_gw;
static char              s_config_text[CONFIG_TEXT_MAX];
static lbr_logger_t     *s_log;
static lbr_transport_t  *s_usb;
static SemaphoreHandle_t s_locks[LBR_GW_LOCK_COUNT_];

/* The Central's own slots (GPS, dashboard status). */
static lbr_sensor_t      s_sensors[CENTRAL_SLOT_COUNT];
static lbr_acq_t         s_acq[CENTRAL_SLOT_COUNT];
static lbr_node_config_t s_plan_cfg;

typedef struct {
    size_t             index;
    TaskHandle_t       task;
    esp_timer_handle_t timer;
} acq_task_t;

static acq_task_t s_acq_tasks[CENTRAL_SLOT_COUNT];

/* Dashboard datagrams over the pit access point. */
static lbr_udp_t            *s_dash_udp;
static lbr_transport_t       s_dash;
static lbr_transport_stats_t s_dash_st;

/* ------------------------------------------------------------------------ */
/* HAL                                                                       */

static uint64_t hal_mono_us(void *ctx)
{
    (void)ctx;
    return lbr_time_mono_us();
}

static uint64_t hal_now_us(void *ctx)
{
    (void)ctx;
    return lbr_time_now_us();
}

static uint64_t hal_local_to_shared(void *ctx, uint64_t local_us)
{
    (void)ctx;
    return lbr_time_local_to_shared(local_us);
}

static bool hal_can_send(void *ctx, const lbr_can_frame_t *f)
{
    (void)ctx;
    return lbr_can_bus_send(f);
}

static void hal_can_stats(void *ctx, lbr_can_stats_t *out)
{
    (void)ctx;
    lbr_can_bus_get_stats(out);
}

static void hal_sync_arm(void *ctx)
{
    (void)ctx;
    lbr_can_bus_arm_sync_tx();
}

static bool hal_sd_present(void *ctx)
{
    (void)ctx;
    return lbr_log_service_sd_present();
}

static bool hal_lora_ready(void *ctx)
{
    (void)ctx;
    return lbr_link_lora_ready();
}

static void hal_lock(void *ctx, lbr_gw_lock_t which)
{
    (void)ctx;
    xSemaphoreTake(s_locks[which], portMAX_DELAY);
}

static void hal_unlock(void *ctx, lbr_gw_lock_t which)
{
    (void)ctx;
    xSemaphoreGive(s_locks[which]);
}

static void hal_log(void *ctx, lbr_log_level_t level, const char *text)
{
    (void)ctx;
    if (level == LBR_LOG_LVL_ERROR) {
        ESP_LOGE(TAG, "%s", text);
    } else if (level == LBR_LOG_LVL_WARN) {
        ESP_LOGW(TAG, "%s", text);
    } else {
        ESP_LOGI(TAG, "%s", text);
    }
}

static const lbr_gw_hal_t k_hal = {
    .mono_us         = hal_mono_us,
    .now_us          = hal_now_us,
    .local_to_shared = hal_local_to_shared,
    .can_send        = hal_can_send,
    .can_stats       = hal_can_stats,
    .sync_arm        = hal_sync_arm,
    .sd_present      = hal_sd_present,
    .lora_ready      = hal_lora_ready,
    .lock            = hal_lock,
    .unlock          = hal_unlock,
    .log             = hal_log,
};

/* ------------------------------------------------------------------------ */
/* Tasks                                                                     */

static void rx_task(void *arg)
{
    (void)arg;
    lbr_can_rx_t rx;
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        if (lbr_can_bus_receive(&rx, 500)) {
            lbr_gw_core_on_rx(&s_core, &rx.frame, rx.rx_local_us);
        }
    }
}

static void sync_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / LBR_TIME_SYNC_HZ));
        esp_task_wdt_reset();
        uint64_t tx_local = 0;
        uint8_t  seq      = lbr_gw_core_sync_send(&s_core);
        bool     sent     = lbr_can_bus_wait_sync_tx_done(seq, &tx_local, 50);
        lbr_gw_core_sync_done(&s_core, sent, tx_local);
    }
}

static void stream_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_gw.stream_batch_ms));
        lbr_gw_core_stream_flush(&s_core);
    }
}

static void lora_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_gw.lora_period_ms));
        lbr_gw_core_lora_tick(&s_core);
    }
}

static void acq_timer_cb(void *arg)
{
    acq_task_t *t = arg;
    xTaskNotifyGive(t->task);
}

/* As node_esp.c: wake at each grid point of the shared clock. */
static void acq_task(void *arg)
{
    acq_task_t *t = arg;
    esp_task_wdt_add(NULL);
    lbr_gw_core_slot_start(&s_core, t->index);
    for (;;) {
        uint64_t due = lbr_gw_core_slot_next_due(&s_core, t->index);
        uint64_t now = lbr_time_now_us();
        esp_timer_start_once(t->timer, due > now ? due - now : 1u);
        uint32_t woke = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        esp_task_wdt_reset();
        if (woke > 0) {
            lbr_gw_core_slot_tick(&s_core, t->index);
        } else {
            esp_timer_stop(t->timer);
        }
    }
}

static void dash_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(s_gw.dash_period_ms));
        lbr_gw_core_dash_tick(&s_core);
    }
}

static void health_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    esp_task_wdt_add(NULL);
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LBR_GW_HEALTH_PERIOD_MS));
        esp_task_wdt_reset();
        lbr_gw_core_heartbeat(&s_core, NULL);
    }
}

/* ------------------------------------------------------------------------ */
/* Link hooks                                                                */

static size_t hook_status(char *buf, size_t cap)
{
    char name[LBR_SD_NAME_MAX];
    lbr_log_service_file_name(name);
    return lbr_gw_core_status(&s_core, name, buf, cap);
}

static void hook_before_reboot(void)
{
    if (!lbr_log_service_close(3000)) {
        ESP_LOGE(TAG, "log did not close cleanly before reboot");
    }
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

static void on_log_event(uint16_t code, bool fault)
{
    lbr_gw_core_event(&s_core, (lbr_event_code_t)code, 0, fault);
}

/* ------------------------------------------------------------------------ */
/* Dashboard link                                                            */

static int dash_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    (void)self;
    /* The dashboard is the only listener on its port: broadcast on the AP. */
    if (!lbr_udp_send(s_dash_udp, LBR_UDP_BROADCAST, LBR_WLINK_DASH_PORT, pkt, len)) {
        s_dash_st.packets_dropped++;
        return -1;
    }
    s_dash_st.packets_sent++;
    s_dash_st.bytes_sent += (uint32_t)len;
    return 0;
}

static void dash_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    (void)self;
    *out = s_dash_st;
}

static void on_dash_datagram(lbr_udp_t *self, const uint8_t *buf, size_t len, uint32_t ip,
                             uint16_t port, uint64_t rx_local_us, void *ctx)
{
    (void)self;
    (void)ip;
    (void)port;
    (void)ctx;
    lbr_gw_core_on_dash_packet(&s_core, buf, len, rx_local_us);
}

static lbr_transport_t *start_dash_link(bool wifi_up)
{
#if CONFIG_LBR_DASH_ENABLE
    if (!s_gw.dash_enable || !wifi_up) {
        ESP_LOGW(TAG, "dashboard link off (needs the WiFi pit link)");
        return NULL;
    }
    s_dash_udp = lbr_udp_open(LBR_WLINK_STATUS_PORT, on_dash_datagram, NULL);
    if (s_dash_udp == NULL) {
        return NULL;
    }
    s_dash = (lbr_transport_t){
        .name = "dash", .kind = LBR_LINK_DASH, .send = dash_send, .get_stats = dash_stats};
    return &s_dash;
#else
    (void)wifi_up;
    return NULL;
#endif
}

/* ------------------------------------------------------------------------ */
/* Own slots                                                                 */

static void bind_slots(void)
{
    lbr_node_build(&lbr_plan_central, LBR_NODE_UNBOUND_DRIVER, s_sensors, s_acq, &s_plan_cfg);
#if CONFIG_LBR_GPS_RX_GPIO >= 0
    /* GPS: any NMEA 0183 module on a UART (CONFIG_LBR_GPS_*). */
    static const lbr_gps_uart_cfg_t gps = {
        .uart_num = CONFIG_LBR_GPS_UART_NUM,
        .rx_gpio  = CONFIG_LBR_GPS_RX_GPIO,
        .tx_gpio  = CONFIG_LBR_GPS_TX_GPIO,
        .baud     = CONFIG_LBR_GPS_BAUD,
    };
    const lbr_slot_t *slot = &lbr_plan_central.slots[CENTRAL_SLOT_GPS];
    lbr_sensor_gps_nmea(&s_sensors[CENTRAL_SLOT_GPS], "gps_nmea", slot->signals, slot->n_signals,
                        &gps);
#endif
}

static void start_slots(void)
{
    for (size_t i = 0; i < CENTRAL_SLOT_COUNT; i++) {
        acq_task_t *t                 = &s_acq_tasks[i];
        t->index                      = i;
        esp_timer_create_args_t targs = {
            .callback = acq_timer_cb,
            .arg      = t,
            .name     = "central_acq",
        };
        char name[16];
        snprintf(name, sizeof name, "acq_%u", (unsigned)i);
        if (esp_timer_create(&targs, &t->timer) != ESP_OK ||
            xTaskCreate(acq_task, name, 4096, t, ACQ_TASK_PRIO, &t->task) != pdPASS) {
            ESP_LOGE(TAG, "cannot start slot %s", s_acq[i].sensor->name);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

static void build_config_text(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    lbr_can_bus_config_t  can = lbr_can_bus_default_config();
    int n = snprintf(s_config_text, sizeof s_config_text,
                     "node=central\nnode_id=%u\nfw_version=%u.%u.%u\nbuild=%s\nidf=%s\n"
                     "protocol_version=%u\ncatalog_hash=%08lx\ncan_bitrate=%lu\n"
                     "time_sync_hz=%u\nlora_freq_hz=%d\nlora_sf=%d\nlora_bw_khz=%d\n"
                     "lora_cr=4/%d\nlora_tx_dbm=%d\nexpected_nodes=",
                     LBR_NODE_CENTRAL, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
                     LBR_FW_VERSION_PATCH, app->version, app->idf_ver, LBR_PROTOCOL_VERSION,
                     (unsigned long)lbr_catalog_hash(), (unsigned long)can.bitrate,
                     LBR_TIME_SYNC_HZ, CONFIG_LBR_LORA_FREQ_HZ, CONFIG_LBR_LORA_SF,
                     CONFIG_LBR_LORA_BW_KHZ, CONFIG_LBR_LORA_CR + 4, CONFIG_LBR_LORA_TX_DBM);
    for (uint8_t id = LBR_NODE_IMU; id <= LBR_NODE_CORNER_BR && n > 0; id++) {
        n += snprintf(s_config_text + n, sizeof s_config_text - (size_t)n, "%s%s",
                      id == LBR_NODE_IMU ? "" : ",", lbr_node_name(id));
    }
    for (size_t i = 0; i < CENTRAL_SLOT_COUNT && n > 0; i++) {
        const lbr_sensor_t *s = s_acq[i].sensor;
        n += snprintf(s_config_text + n, sizeof s_config_text - (size_t)n,
                      "\nsensor.%u=%s rate_hz=%u sim=%u", (unsigned)i, s->name,
                      (unsigned)s_acq[i].rate_hz, s->simulated ? 1u : 0u);
    }
    if (n > 0 && (size_t)n < sizeof s_config_text) {
        n += snprintf(s_config_text + n, sizeof s_config_text - (size_t)n, "\n");
        gw_store_get(s_config_text + n, sizeof s_config_text - (size_t)n);
    }
}

static void start_links(lbr_gw_links_t *links)
{
    static const lbr_link_hooks_t hooks = {
        .status        = hook_status,
        .config_get    = gw_store_get,
        .config_put    = gw_store_put,
        .before_reboot = hook_before_reboot,
    };
    if (s_gw.usb_enable && (links->usb = lbr_link_usb_start(&hooks)) == NULL) {
        ESP_LOGE(TAG, "USB link unavailable");
    }
    if (s_gw.wifi_enable && (links->wifi = lbr_link_wifi_start(&hooks)) == NULL) {
        ESP_LOGW(TAG, "WiFi link unavailable");
    }
    if (s_gw.ble_enable && (links->ble = lbr_link_ble_start(&hooks)) == NULL) {
        ESP_LOGW(TAG, "BLE link unavailable");
    }
    if (s_gw.lora_enable && (links->lora = lbr_link_lora_start(s_gw.lora_duty_permille)) == NULL) {
        ESP_LOGW(TAG, "LoRa link unavailable (pins TBD?)");
    }
    links->dash = start_dash_link(links->wifi != NULL);
    s_usb       = links->usb;
}

void central_run(void)
{
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    for (int i = 0; i < LBR_GW_LOCK_COUNT_; i++) {
        s_locks[i] = xSemaphoreCreateMutex();
    }
    lbr_time_set_master();

    /* Validate configuration before the session starts. */
    bool config_valid = gw_store_load(&s_gw) >= 0;
    bind_slots();
    build_config_text();
    lbr_gw_boot_t boot = {
        .reset_reason = lbr_health_reset_reason(),
        .boot_count   = lbr_health_boot_count(),
        .epoch        = (uint16_t)(esp_random() | 1u), /* new timeline every boot */
        .config_valid = config_valid,
        .config_text  = s_config_text,
        .acq          = s_acq,
        .n_acq        = CENTRAL_SLOT_COUNT,
    };
    lbr_gw_core_init(&s_core, &k_hal, &s_gw, &boot);
    /* The dashboard-status slot reads this core's own link state. */
    const lbr_slot_t *ds = &lbr_plan_central.slots[CENTRAL_SLOT_DASH_STATUS];
    lbr_sensor_dash_link(&s_sensors[CENTRAL_SLOT_DASH_STATUS], ds->name, ds->signals, ds->n_signals,
                         &s_core.dash, lbr_gw_core_lock_state, lbr_gw_core_unlock_state, &s_core);

    /* Authoritative log first: everything after this point is recorded. */
    lbr_log_service_config_t lc = {
        .node_id          = LBR_NODE_CENTRAL,
        .reset_reason     = (uint8_t)boot.reset_reason,
        .boot_count       = boot.boot_count,
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

    lbr_gw_links_t links = {0};
    start_links(&links);
    esp_log_set_vprintf(log_vprintf);

    lbr_can_bus_config_t can = lbr_can_bus_default_config();
    can.accept_all           = true;
    lbr_can_err_t can_rc     = lbr_can_bus_start(&can);

    lbr_gw_core_start(&s_core, s_log, &links, can_rc == LBR_CAN_OK,
                      can_rc == LBR_CAN_ERR_PINS_UNSET);

    if (s_core.can_up) {
        xTaskCreate(rx_task, "gw_rx", 4096, NULL, RX_TASK_PRIO, NULL);
        xTaskCreate(sync_task, "gw_sync", 3072, NULL, SYNC_TASK_PRIO, NULL);
    }
    start_slots();
    if (links.dash != NULL) {
        xTaskCreate(dash_task, "gw_dash", 4096, NULL, DASH_TASK_PRIO, NULL);
    }
    if (links.lora != NULL) {
        xTaskCreate(lora_task, "gw_lora", 4096, NULL, LORA_TASK_PRIO, NULL);
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
