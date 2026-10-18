#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "lbr/cobs.h"
#include "lbr/log_sd.h"
#include "lbr/packet.h"
#include "lbr/transport_links.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

/*
 * WiFi pit link: soft access point (WPA2) with
 *   - TCP port CONFIG_LBR_WIFI_STREAM_PORT: live packet stream (COBS frames,
 *     same as USB), one client at a time;
 *   - HTTP on port 80:
 *       GET  /status        status text
 *       GET  /logs          session files ("name size" lines)
 *       GET  /logs/<name>   download a session file
 *       GET  /config        runtime config text
 *       POST /config        validate + store (applies at next session start)
 *       POST /ota           firmware image; requires "X-LBR-Confirm: reboot"
 *                           because a successful update restarts the gateway
 *                           and ends the current session.
 * Sending is non-blocking: packets that do not fit the stream buffer (slow
 * or no client) are dropped and counted.
 */

static const char *TAG = "lbr_wifi";

#define STREAM_BUF_SIZE (24 * 1024)
#define BODY_MAX        2048
#define FILE_CHUNK      2048

static lbr_link_hooks_t     s_hooks;
static lbr_transport_t      s_link;
static StreamBufferHandle_t s_stream;
static SemaphoreHandle_t    s_tx_lock;
static uint8_t              s_pkt_copy[LBR_PKT_MAX_SIZE];
static uint8_t              s_enc[LBR_COBS_MAX_ENCODED(LBR_PKT_MAX_SIZE) + 1];
static uint32_t             s_seq;
static atomic_bool          s_client;
static atomic_uint          s_sent, s_dropped, s_bytes, s_errors;

/* ------------------------------------------------------------------------ */
/* Live stream                                                               */

static int wifi_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    (void)self;
    if (!atomic_load(&s_client) || len > LBR_PKT_MAX_SIZE) {
        atomic_fetch_add(&s_dropped, 1);
        return -1;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    memcpy(s_pkt_copy, pkt, len);
    lbr_pkt_restamp(s_pkt_copy, len, LBR_LINK_WIFI, s_seq);
    size_t n   = lbr_cobs_encode(s_pkt_copy, len, s_enc, sizeof s_enc - 1);
    s_enc[n++] = 0x00;
    int rc     = -1;
    if (xStreamBufferSpacesAvailable(s_stream) >= n &&
        xStreamBufferSend(s_stream, s_enc, n, 0) == n) {
        s_seq++;
        atomic_fetch_add(&s_sent, 1);
        atomic_fetch_add(&s_bytes, (unsigned)n);
        rc = 0;
    } else {
        atomic_fetch_add(&s_dropped, 1);
    }
    xSemaphoreGive(s_tx_lock);
    return rc;
}

static void wifi_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    (void)self;
    out->packets_sent    = atomic_load(&s_sent);
    out->packets_dropped = atomic_load(&s_dropped);
    out->bytes_sent      = atomic_load(&s_bytes);
    out->errors          = atomic_load(&s_errors);
    out->connected       = atomic_load(&s_client);
}

static void stream_task(void *arg)
{
    (void)arg;
    static uint8_t buf[1460];
    int            listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listener < 0) {
        ESP_LOGE(TAG, "stream socket failed");
        vTaskDelete(NULL);
        return;
    }
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CONFIG_LBR_WIFI_STREAM_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    if (bind(listener, (struct sockaddr *)&addr, sizeof addr) != 0 || listen(listener, 1) != 0) {
        ESP_LOGE(TAG, "stream bind/listen failed");
        close(listener);
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        int client = accept(listener, NULL, NULL);
        if (client < 0) {
            continue;
        }
        xStreamBufferReset(s_stream);
        atomic_store(&s_client, true);
        ESP_LOGI(TAG, "stream client connected");
        for (;;) {
            size_t n = xStreamBufferReceive(s_stream, buf, sizeof buf, pdMS_TO_TICKS(200));
            if (n > 0 && send(client, buf, n, 0) != (ssize_t)n) {
                atomic_fetch_add(&s_errors, 1);
                break;
            }
        }
        atomic_store(&s_client, false);
        close(client);
        ESP_LOGI(TAG, "stream client gone");
    }
}

/* ------------------------------------------------------------------------ */
/* HTTP                                                                      */

static esp_err_t send_text(httpd_req_t *req, size_t (*fn)(char *, size_t))
{
    static char text[2048];
    size_t      n = fn != NULL ? fn(text, sizeof text) : 0;
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, text, (ssize_t)n);
}

static esp_err_t h_status(httpd_req_t *req)
{
    return send_text(req, s_hooks.status);
}

static esp_err_t h_config_get(httpd_req_t *req)
{
    return send_text(req, s_hooks.config_get);
}

static int read_body(httpd_req_t *req, char *buf, size_t cap)
{
    if (req->content_len >= cap) {
        return -1;
    }
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n <= 0) {
            return -1;
        }
        got += (size_t)n;
    }
    buf[got] = '\0';
    return (int)got;
}

static esp_err_t h_config_put(httpd_req_t *req)
{
    static char body[BODY_MAX];
    char        err[128] = "";
    int         n        = read_body(req, body, sizeof body);
    if (n < 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large or truncated");
    }
    if (s_hooks.config_put == NULL || s_hooks.config_put(body, (size_t)n, err, sizeof err) != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, err[0] ? err : "rejected");
    }
    return httpd_resp_sendstr(req, "ok: stored, applies at the next session start\n");
}

static void list_cb(const char *name, uint32_t size, void *ctx)
{
    httpd_req_t *req = ctx;
    char         line[48];
    int          n = snprintf(line, sizeof line, "%s %lu\n", name, (unsigned long)size);
    httpd_resp_send_chunk(req, line, n);
}

static esp_err_t h_logs(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    if (lbr_sd_list(list_cb, req) < 0) {
        httpd_resp_send_chunk(req, "no card\n", HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t h_log_file(httpd_req_t *req)
{
    static uint8_t buf[FILE_CHUNK];
    const char    *name = req->uri + strlen("/logs/");
    if (!lbr_sd_valid_name(name) || lbr_sd_file_size(name) < 0) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such session file");
    }
    httpd_resp_set_type(req, "application/octet-stream");
    uint32_t off = 0;
    for (;;) {
        int n = lbr_sd_read(name, off, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        if (httpd_resp_send_chunk(req, (const char *)buf, n) != ESP_OK) {
            return ESP_FAIL;
        }
        off += (uint32_t)n;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (s_hooks.before_reboot != NULL) {
        s_hooks.before_reboot();
    }
    esp_restart();
}

static esp_err_t h_ota(httpd_req_t *req)
{
    static char buf[FILE_CHUNK];
    char        confirm[16] = "";
    if (httpd_req_get_hdr_value_str(req, "X-LBR-Confirm", confirm, sizeof confirm) != ESP_OK ||
        strcmp(confirm, "reboot") != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "OTA restarts the gateway: send X-LBR-Confirm: reboot");
    }
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_ota_handle_t       ota;
    if (part == NULL || esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA partition");
    }
    size_t left = req->content_len;
    while (left > 0) {
        int n = httpd_req_recv(req, buf, left < sizeof buf ? left : sizeof buf);
        if (n <= 0 || esp_ota_write(ota, buf, (size_t)n) != ESP_OK) {
            esp_ota_abort(ota);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "upload failed");
        }
        left -= (size_t)n;
    }
    /* esp_ota_end validates the image (header, checksum, signature if on). */
    if (esp_ota_end(ota) != ESP_OK || esp_ota_set_boot_partition(part) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image rejected");
    }
    ESP_LOGW(TAG, "OTA image accepted into %s; restarting", part->label);
    httpd_resp_sendstr(req, "ok: rebooting into the new image\n");
    /* Room for the log hook and ESP_LOG formatting on the way down. */
    xTaskCreate(reboot_task, "lbr_reboot", 4096, NULL, 1, NULL);
    return ESP_OK;
}

static bool start_http(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.uri_match_fn      = httpd_uri_match_wildcard;
    cfg.stack_size        = 6144;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        return false;
    }
    const httpd_uri_t uris[] = {
        {.uri = "/status", .method = HTTP_GET, .handler = h_status},
        {.uri = "/config", .method = HTTP_GET, .handler = h_config_get},
        {.uri = "/config", .method = HTTP_POST, .handler = h_config_put},
        {.uri = "/logs", .method = HTTP_GET, .handler = h_logs},
        {.uri = "/logs/*", .method = HTTP_GET, .handler = h_log_file},
        {.uri = "/ota", .method = HTTP_POST, .handler = h_ota},
    };
    for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Bring-up                                                                  */

lbr_transport_t *lbr_link_wifi_start(const lbr_link_hooks_t *hooks)
{
    if (strlen(CONFIG_LBR_WIFI_PASSWORD) < 8) {
        ESP_LOGW(TAG, "WiFi disabled: set CONFIG_LBR_WIFI_PASSWORD (8+ chars)");
        return NULL;
    }
    s_hooks   = *hooks;
    s_tx_lock = xSemaphoreCreateMutex();
    s_stream  = xStreamBufferCreate(STREAM_BUF_SIZE, 1);
    if (s_tx_lock == NULL || s_stream == NULL) {
        return NULL;
    }

    esp_err_t err = esp_netif_init();
    if (err == ESP_OK) {
        err = esp_event_loop_create_default();
        if (err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK; /* already created by the app */
        }
    }
    if (err != ESP_OK || esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "netif init failed");
        return NULL;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t      ap   = {
        .ap =
            {
                .ssid           = CONFIG_LBR_WIFI_SSID,
                .ssid_len       = (uint8_t)strlen(CONFIG_LBR_WIFI_SSID),
                .password       = CONFIG_LBR_WIFI_PASSWORD,
                .channel        = CONFIG_LBR_WIFI_CHANNEL,
                .authmode       = WIFI_AUTH_WPA2_PSK,
                .max_connection = 4,
                .pmf_cfg        = {.required = false},
            },
    };
    if (esp_wifi_init(&init) != ESP_OK || esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_AP, &ap) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "WiFi start failed");
        return NULL;
    }
    if (!start_http() || xTaskCreate(stream_task, "lbr_wifi_tx", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "WiFi services failed to start");
        return NULL;
    }
    ESP_LOGI(TAG, "AP \"%s\" up; stream on port %d", CONFIG_LBR_WIFI_SSID,
             CONFIG_LBR_WIFI_STREAM_PORT);
    s_link = (lbr_transport_t){
        .name      = "wifi",
        .kind      = LBR_LINK_WIFI,
        .send      = wifi_send,
        .get_stats = wifi_stats,
    };
    return &s_link;
}
