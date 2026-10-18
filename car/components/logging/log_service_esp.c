#include "lbr/log_service.h"

#include <stdatomic.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/log_keeper.h"
#include "lbr/protocol.h"
#include "lbr/version.h"

static const char *TAG = "lbr_logsvc";

#define PUMP_PERIOD_MS 20
#define CHUNK_SIZE     8192
#define TASK_PRIO      5

static lbr_log_service_config_t s_cfg;
static lbr_logger_t             s_log;
static lbr_log_keeper_t         s_keeper;
static SemaphoreHandle_t        s_lock;
static atomic_bool              s_sd_present;
static char                     s_name[LBR_SD_NAME_MAX];
static TaskHandle_t             s_writer;
static TaskHandle_t             s_close_waiter; /* set while a close is requested */
static atomic_bool              s_closed_for_good;
static portMUX_TYPE             s_name_mux = portMUX_INITIALIZER_UNLOCKED;

static void lock(void *ctx)
{
    (void)ctx;
    xSemaphoreTake(s_lock, portMAX_DELAY);
}

static void unlock(void *ctx)
{
    (void)ctx;
    xSemaphoreGive(s_lock);
}

static void set_name(const char *name)
{
    portENTER_CRITICAL(&s_name_mux);
    strncpy(s_name, name, sizeof s_name - 1);
    s_name[sizeof s_name - 1] = '\0';
    portEXIT_CRITICAL(&s_name_mux);
}

_Static_assert(LBR_LOG_KEEPER_NAME_MAX == LBR_SD_NAME_MAX, "keeper and SD name sizes differ");

/* Keeper HAL: the SD card through log_sd_esp.c. */
static bool k_mount(void *ctx)
{
    (void)ctx;
    return lbr_sd_mount() == LBR_SD_OK;
}

static void k_unmount(void *ctx)
{
    (void)ctx;
    lbr_sd_unmount();
}

static bool k_open_file(void *ctx, lbr_log_storage_t *st, char name[LBR_LOG_KEEPER_NAME_MAX])
{
    (void)ctx;
    return lbr_sd_open_session(st, name) == LBR_SD_OK;
}

static void k_event(void *ctx, uint16_t code, bool fault)
{
    (void)ctx;
    if (code == LBR_EVT_SD_WRITE_ERROR) {
        ESP_LOGE(TAG, "SD write error; closing session and remounting");
    }
    if (s_cfg.on_event != NULL) {
        s_cfg.on_event(code, fault);
    }
}

static void writer_task(void *arg)
{
    (void)arg;
    for (;;) {
        /* Sleep one pump period, or less if a close is requested. */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PUMP_PERIOD_MS));

        if (s_close_waiter != NULL) {
            /* Only this task touches the file, so closing here cannot race a pump. */
            lbr_log_keeper_close(&s_keeper, s_cfg.now_us());
            set_name("");
            atomic_store(&s_closed_for_good, true);
            TaskHandle_t waiter = s_close_waiter;
            s_close_waiter      = NULL;
            xTaskNotifyGive(waiter);
        }
        if (atomic_load(&s_closed_for_good)) {
            continue;
        }
        lbr_log_keeper_step(&s_keeper, (uint64_t)esp_timer_get_time(), s_cfg.now_us());
        atomic_store(&s_sd_present, s_keeper.sd_present);
        set_name(s_keeper.name);
    }
}

bool lbr_log_service_start(const lbr_log_service_config_t *cfg)
{
    s_cfg          = *cfg;
    s_lock         = xSemaphoreCreateMutex();
    uint8_t *queue = heap_caps_malloc(cfg->queue_size, MALLOC_CAP_8BIT);
    uint8_t *chunk = heap_caps_malloc(CHUNK_SIZE, MALLOC_CAP_DMA);
    if (s_lock == NULL || queue == NULL || chunk == NULL) {
        ESP_LOGE(TAG, "out of memory for a %u byte log queue", (unsigned)cfg->queue_size);
        return false;
    }
    lbr_log_config_t lc = {
        .queue_mem        = queue,
        .queue_cap        = cfg->queue_size,
        .chunk            = chunk,
        .chunk_cap        = CHUNK_SIZE,
        .sync_interval_ms = cfg->sync_interval_ms,
        .lock             = lock,
        .unlock           = unlock,
    };
    if (lbr_log_init(&s_log, &lc) != 0) {
        return false;
    }
    static const lbr_log_keeper_hal_t hal = {
        .mount     = k_mount,
        .unmount   = k_unmount,
        .open_file = k_open_file,
        .on_event  = k_event,
    };
    lbr_log_session_t session = {
        .fw_version   = LBR_FW_VERSION,
        .node_id      = cfg->node_id,
        .reset_reason = cfg->reset_reason,
        .boot_count   = cfg->boot_count,
        .config_text  = cfg->config_text,
    };
    lbr_log_keeper_init(&s_keeper, &s_log, &hal, &session);
    /* Mount and open the first session now, before the caller logs its boot
     * events and first samples (they would be rejected as "log closed"). */
    lbr_log_keeper_step(&s_keeper, (uint64_t)esp_timer_get_time(), cfg->now_us());
    atomic_store(&s_sd_present, s_keeper.sd_present);
    set_name(s_keeper.name);
    return xTaskCreate(writer_task, "lbr_logw", 4096, NULL, TASK_PRIO, &s_writer) == pdPASS;
}

bool lbr_log_service_close(uint32_t timeout_ms)
{
    if (s_writer == NULL) {
        return false;
    }
    ulTaskNotifyTake(pdTRUE, 0);
    s_close_waiter = xTaskGetCurrentTaskHandle();
    xTaskNotifyGive(s_writer);
    return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
}

lbr_logger_t *lbr_log_service_logger(void)
{
    return &s_log;
}

bool lbr_log_service_sd_present(void)
{
    return atomic_load(&s_sd_present);
}

void lbr_log_service_file_name(char out[LBR_SD_NAME_MAX])
{
    portENTER_CRITICAL(&s_name_mux);
    memcpy(out, s_name, LBR_SD_NAME_MAX);
    portEXIT_CRITICAL(&s_name_mux);
}
