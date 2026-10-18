#include "gw_store.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "gw_store";

#define NVS_NS   "lbr"
#define NVS_KEY  "gwcfg"
#define MAX_TEXT 1024

static lbr_gw_config_t s_active;

int gw_store_load(lbr_gw_config_t *out)
{
    char         text[MAX_TEXT];
    size_t       len = sizeof text;
    nvs_handle_t h;
    int          rc = 1;

    lbr_gw_config_defaults(out);
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_str(h, NVS_KEY, text, &len) == ESP_OK) {
            char err[96] = "";
            if (lbr_gw_config_parse(text, strlen(text), out, err, sizeof err) == 0) {
                rc = 0;
            } else {
                ESP_LOGE(TAG, "stored config invalid (%s); using defaults", err);
                lbr_gw_config_defaults(out);
                rc = -1;
            }
        }
        nvs_close(h);
    }
    s_active = *out;
    return rc;
}

int gw_store_put(const char *text, size_t len, char *err, size_t err_len)
{
    lbr_gw_config_t c;
    char            canon[MAX_TEXT];
    nvs_handle_t    h;

    if (lbr_gw_config_parse(text, len, &c, err, err_len) != 0) {
        return -1;
    }
    /* Store the canonical form, so what is stored is exactly what applies. */
    lbr_gw_config_format(&c, canon, sizeof canon);
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        snprintf(err, err_len, "NVS unavailable");
        return -1;
    }
    esp_err_t e = nvs_set_str(h, NVS_KEY, canon);
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    if (e != ESP_OK) {
        snprintf(err, err_len, "NVS write failed");
        return -1;
    }
    ESP_LOGI(TAG, "new config stored; applies at next session start");
    return 0;
}

size_t gw_store_get(char *buf, size_t cap)
{
    return lbr_gw_config_format(&s_active, buf, cap);
}
