#include "lbr/update_esp.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

/*
 * ESP32 flash access for CAN firmware updates and the boot-time guard; see
 * lbr/update_esp.h. Every decision about the transfer is in update.c.
 */

static const char *TAG = "lbr_update";

#define NVS_NS  "lbr_update"
#define NVS_KEY "node_id"

static const esp_partition_t *s_part;   /* spare slot being written or staged */
static esp_ota_handle_t       s_handle; /* open while receiving */
static bool                   s_open;
static bool                   s_staged;
static bool                   s_pending; /* running image awaits confirmation */
static int64_t                s_boot_us;

/* ------------------------------------------------------------------------ */
/* Flash access                                                              */

static lbr_upd_result_t ota_begin(void *ctx, uint32_t size)
{
    (void)ctx;
    if (s_open) {
        esp_ota_abort(s_handle);
        s_open = false;
    }
    s_staged = false;
    s_part   = esp_ota_get_next_update_partition(NULL);
    if (s_part == NULL) {
        return LBR_UPD_ERR_NO_SLOT;
    }
    if (size > s_part->size) {
        return LBR_UPD_ERR_SIZE;
    }
    /* Sequential writes: each sector is erased as the image reaches it, so no
     * call blocks for the whole slot. */
    esp_err_t err = esp_ota_begin(s_part, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(err));
        return LBR_UPD_ERR_FLASH;
    }
    s_open = true;
    ESP_LOGI(TAG, "receiving %lu bytes into %s", (unsigned long)size, s_part->label);
    return LBR_UPD_OK;
}

static lbr_upd_result_t ota_write(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    if (!s_open) {
        return LBR_UPD_ERR_STATE;
    }
    esp_err_t err = esp_ota_write(s_handle, data, len);
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
        return LBR_UPD_ERR_IMAGE; /* the first byte is not an image header */
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write: %s", esp_err_to_name(err));
        return LBR_UPD_ERR_FLASH;
    }
    return LBR_UPD_OK;
}

static lbr_upd_result_t ota_finish(void *ctx)
{
    (void)ctx;
    if (!s_open) {
        return LBR_UPD_ERR_STATE;
    }
    s_open        = false;
    esp_err_t err = esp_ota_end(s_handle); /* checks the header, chip, and image hash */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "image rejected: %s", esp_err_to_name(err));
        return err == ESP_ERR_OTA_VALIDATE_FAILED ? LBR_UPD_ERR_IMAGE : LBR_UPD_ERR_FLASH;
    }
    /* Same project = firmware for this kind of node. Which corner it is
     * cannot be read from the image; lbr_update_esp_boot checks that. */
    esp_app_desc_t        incoming;
    const esp_app_desc_t *running = esp_app_get_description();
    if (esp_ota_get_partition_description(s_part, &incoming) != ESP_OK) {
        return LBR_UPD_ERR_IMAGE;
    }
    if (strncmp(incoming.project_name, running->project_name, sizeof incoming.project_name) != 0) {
        ESP_LOGE(TAG, "image is %s, this board runs %s: refused", incoming.project_name,
                 running->project_name);
        return LBR_UPD_ERR_WRONG_FW;
    }
    s_staged = true;
    ESP_LOGI(TAG, "staged %s %s; waiting for ACTIVATE", incoming.project_name, incoming.version);
    return LBR_UPD_OK;
}

static void ota_abort(void *ctx)
{
    (void)ctx;
    if (s_open) {
        esp_ota_abort(s_handle);
    }
    s_open   = false;
    s_staged = false;
}

static void ota_activate(void *ctx)
{
    (void)ctx;
    if (!s_staged) {
        return;
    }
    esp_err_t err = esp_ota_set_boot_partition(s_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition: %s", esp_err_to_name(err));
        s_staged = false;
        return;
    }
    ESP_LOGW(TAG, "rebooting into the new image");
    vTaskDelay(pdMS_TO_TICKS(100)); /* let the OK reply leave the TX queue */
    esp_restart();
}

static void ota_info(void *ctx, uint32_t *hash, bool *pending)
{
    (void)ctx;
    uint8_t sha[32] = {0};
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha);
    *hash    = (uint32_t)sha[0] << 24 | (uint32_t)sha[1] << 16 | (uint32_t)sha[2] << 8 | sha[3];
    *pending = s_pending;
}

static const lbr_upd_ota_t k_ota = {
    .begin    = ota_begin,
    .write    = ota_write,
    .finish   = ota_finish,
    .abort    = ota_abort,
    .activate = ota_activate,
    .info     = ota_info,
};

const lbr_upd_ota_t *lbr_update_esp_ota(void)
{
    return &k_ota;
}

/* ------------------------------------------------------------------------ */
/* Boot guard                                                                */

static void roll_back(const char *why)
{
    ESP_LOGE(TAG, "new image %s: rolling back to the previous one", why);
    esp_ota_mark_app_invalid_rollback_and_reboot();
    /* Only returns if there is no previous image to go back to. */
    ESP_LOGE(TAG, "no previous image: staying on this one");
    s_pending = false;
}

void lbr_update_esp_boot(uint8_t node_id)
{
    s_boot_us     = esp_timer_get_time();
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }

    esp_ota_img_states_t state;
    s_pending = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
                state == ESP_OTA_IMG_PENDING_VERIFY;

    nvs_handle_t h;
    uint8_t      stored = 0;
    bool         known  = false;
    if (err == ESP_OK && nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        known = nvs_get_u8(h, NVS_KEY, &stored) == ESP_OK;
        if (!s_pending && (!known || stored != node_id)) {
            /* Flashed over USB (or first boot): this board is now node_id. */
            nvs_set_u8(h, NVS_KEY, node_id);
            nvs_commit(h);
        }
        nvs_close(h);
    }
    if (s_pending) {
        ESP_LOGW(TAG, "first boot of an image received over CAN: confirming within %u s",
                 LBR_UPDATE_CONFIRM_TIMEOUT_S);
        if (known && stored != node_id) {
            ESP_LOGE(TAG, "this board is node %u but the image is node %u", stored, node_id);
            roll_back("is firmware for another node");
        }
    }
}

void lbr_update_esp_confirm_tick(bool healthy)
{
    if (!s_pending) {
        return;
    }
    if (healthy) {
        esp_ota_mark_app_valid_cancel_rollback();
        s_pending = false;
        ESP_LOGI(TAG, "new image confirmed (synced to the Central over CAN)");
    } else if (esp_timer_get_time() - s_boot_us > (int64_t)LBR_UPDATE_CONFIRM_TIMEOUT_S * 1000000) {
        roll_back("never synced to the Central");
    }
}
