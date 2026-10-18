#include "gateway.h"

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
#include "lbr/can_bus.h"
#include "lbr/gw_core.h"
#include "lbr/log_service.h"
#include "lbr/nodes.h"
#include "lbr/timebase.h"
#include "lbr/transport_links.h"
#include "lbr/version.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/*
 * ESP32 platform for the gateway: FreeRTOS tasks, TWAI, SD log service, the
 * live links, NVS config, and OTA confirmation. All gateway decisions are in
 * gw_core.c, which the simulator runs unchanged.
 */

static const char *TAG = "gateway";

#define RX_TASK_PRIO     12
#define SYNC_TASK_PRIO   11
#define STREAM_TASK_PRIO 7
#define HEALTH_TASK_PRIO 6
#define LORA_TASK_PRIO   5
#define CONFIG_TEXT_MAX  2048

static lbr_gw_core_t     s_core;
static lbr_gw_config_t   s_gw;
static char              s_config_text[CONFIG_TEXT_MAX];
static lbr_logger_t     *s_log;
static lbr_transport_t  *s_usb;
static SemaphoreHandle_t s_locks[LBR_GW_LOCK_COUNT_];

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
                     LBR_NODE_GATEWAY, LBR_FW_VERSION_MAJOR, LBR_FW_VERSION_MINOR,
                     LBR_FW_VERSION_PATCH, app->version, app->idf_ver, LBR_PROTOCOL_VERSION,
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
    s_usb = links->usb;
}

void gateway_run(void)
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
    build_config_text();
    lbr_gw_boot_t boot = {
        .reset_reason = lbr_health_reset_reason(),
        .boot_count   = lbr_health_boot_count(),
        .epoch        = (uint16_t)(esp_random() | 1u), /* new timeline every boot */
        .config_valid = config_valid,
        .config_text  = s_config_text,
    };
    lbr_gw_core_init(&s_core, &k_hal, &s_gw, &boot);

    /* Authoritative log first: everything after this point is recorded. */
    lbr_log_service_config_t lc = {
        .node_id          = LBR_NODE_GATEWAY,
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
