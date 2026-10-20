#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lbr/packet.h"
#include "lbr/wlink_esp.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* ESP32-C5 WiFi + UDP for the wireless links; see lbr/wlink_esp.h. */

static const char *TAG = "lbr_wlink";

struct lbr_udp {
    int           sock;
    lbr_udp_rx_cb cb;
    void         *ctx;
};

static bool              s_wifi_up;
static volatile uint32_t s_sta_gw;
static esp_netif_t      *s_sta_netif;

static bool base_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err == ESP_OK) {
        err = esp_netif_init();
    }
    if (err == ESP_OK) {
        err = esp_event_loop_create_default();
        if (err == ESP_ERR_INVALID_STATE) {
            err = ESP_OK; /* created already */
        }
    }
    return err == ESP_OK;
}

void lbr_wifi_apply_band(void)
{
#if CONFIG_SOC_WIFI_SUPPORT_5G
#if CONFIG_LBR_WIFI_5GHZ
    esp_err_t err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY);
#else
    esp_err_t err = esp_wifi_set_band_mode(WIFI_BAND_MODE_2G_ONLY);
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "band mode not applied: %s", esp_err_to_name(err));
    }
#endif
}

bool lbr_wifi_start_ap(const char *ssid, const char *password, int channel)
{
    if (s_wifi_up) {
        return true;
    }
    if (strlen(password) < 8) {
        ESP_LOGW(TAG, "access point \"%s\" off: its password must be 8+ characters", ssid);
        return false;
    }
    if (!base_init() || esp_netif_create_default_wifi_ap() == NULL) {
        ESP_LOGE(TAG, "netif init failed");
        return false;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t      ap   = {
        .ap = {.channel        = (uint8_t)channel,
               .authmode       = WIFI_AUTH_WPA2_PSK,
               .max_connection = 8,
               .pmf_cfg        = {.required = false}},
    };
    strncpy((char *)ap.ap.ssid, ssid, sizeof ap.ap.ssid);
    strncpy((char *)ap.ap.password, password, sizeof ap.ap.password);
    ap.ap.ssid_len = (uint8_t)strnlen(ssid, sizeof ap.ap.ssid);
    if (esp_wifi_init(&init) != ESP_OK || esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed");
        return false;
    }
    lbr_wifi_apply_band();
    if (esp_wifi_set_config(WIFI_IF_AP, &ap) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "access point start failed");
        return false;
    }
    s_wifi_up = true;
    ESP_LOGI(TAG, "access point \"%s\" up on channel %d", ssid, channel);
    return true;
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
        s_sta_gw = 0;
        esp_wifi_connect(); /* keep trying: the car is moving in and out of range */
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *ev = data;
        s_sta_gw                    = ev->ip_info.gw.addr;
        ESP_LOGI(TAG, "joined; access point at " IPSTR, IP2STR(&ev->ip_info.gw));
    }
}

bool lbr_wifi_start_sta(const char *ssid, const char *password)
{
    if (s_wifi_up) {
        return true;
    }
    if (strlen(password) < 8) {
        ESP_LOGW(TAG, "WiFi off: the password for \"%s\" must be 8+ characters", ssid);
        return false;
    }
    if (!base_init() || (s_sta_netif = esp_netif_create_default_wifi_sta()) == NULL) {
        ESP_LOGE(TAG, "netif init failed");
        return false;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t      sta  = {.sta = {.threshold.authmode = WIFI_AUTH_WPA2_PSK}};
    strncpy((char *)sta.sta.ssid, ssid, sizeof sta.sta.ssid);
    strncpy((char *)sta.sta.password, password, sizeof sta.sta.password);
    if (esp_wifi_init(&init) != ESP_OK ||
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL) != ESP_OK ||
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL) != ESP_OK ||
        esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed");
        return false;
    }
    lbr_wifi_apply_band();
    if (esp_wifi_set_config(WIFI_IF_STA, &sta) != ESP_OK || esp_wifi_start() != ESP_OK) {
        ESP_LOGE(TAG, "station start failed");
        return false;
    }
    /* Low latency over power: the IMU stream is 20 datagrams a second. */
    esp_wifi_set_ps(WIFI_PS_NONE);
    s_wifi_up = true;
    return true;
}

uint32_t lbr_wifi_sta_gateway(void)
{
    return s_sta_gw;
}

static void rx_task(void *arg)
{
    lbr_udp_t     *u = arg;
    static uint8_t buf[LBR_PKT_MAX_SIZE];
    for (;;) {
        struct sockaddr_in from;
        socklen_t          flen = sizeof from;
        int      n = recvfrom(u->sock, buf, sizeof buf, 0, (struct sockaddr *)&from, &flen);
        uint64_t t = (uint64_t)esp_timer_get_time();
        if (n > 0) {
            u->cb(buf, (size_t)n, from.sin_addr.s_addr, ntohs(from.sin_port), t, u->ctx);
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

lbr_udp_t *lbr_udp_open(uint16_t port, lbr_udp_rx_cb cb, void *ctx)
{
    static lbr_udp_t pool[3];
    static size_t    used;
    if (used >= sizeof pool / sizeof pool[0]) {
        return NULL;
    }
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP socket failed");
        return NULL;
    }
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof yes);
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof addr) != 0) {
        ESP_LOGE(TAG, "UDP bind %u failed", (unsigned)port);
        close(sock);
        return NULL;
    }
    lbr_udp_t *u = &pool[used++];
    *u           = (lbr_udp_t){.sock = sock, .cb = cb, .ctx = ctx};
    if (cb != NULL && xTaskCreate(rx_task, "lbr_udp_rx", 4096, u, 11, NULL) != pdPASS) {
        ESP_LOGE(TAG, "UDP receive task failed");
        return NULL;
    }
    return u;
}

bool lbr_udp_send(lbr_udp_t *u, uint32_t ip, uint16_t port, const uint8_t *buf, size_t len)
{
    if (u == NULL || ip == 0) {
        return false;
    }
    struct sockaddr_in to = {
        .sin_family      = AF_INET,
        .sin_port        = htons(port),
        .sin_addr.s_addr = ip,
    };
    return sendto(u->sock, buf, len, MSG_DONTWAIT, (struct sockaddr *)&to, sizeof to) == (int)len;
}
