#include <stdatomic.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lbr/lora_policy.h"
#include "lbr/packet.h"
#include "lbr/transport_links.h"
#include "sdkconfig.h"

/*
 * LoRa link on an SX1276/77/78/79 (RFM95-class) module, TX only. One packet
 * is in flight at a time. After each packet the link waits out the duty
 * budget (lbr_lora_next_tx_us) before reporting ready again, so the in-race
 * link stays within its regulatory and self-imposed airtime limits.
 */

static const char *TAG = "lbr_lora";

/* The classic ESP32 has a free SPI3 for the radio. The ESP32-C5 has one
 * general-purpose host (SPI2), which the radio shares with the SD card:
 * same SCLK/MOSI/MISO pins, separate chip selects (docs/hardware.md). */
#if SOC_SPI_PERIPH_NUM > 2
#define LORA_SPI_HOST SPI3_HOST
#else
#define LORA_SPI_HOST SPI2_HOST
#endif

/* SX127x registers (LoRa mode). */
#define REG_FIFO           0x00
#define REG_OP_MODE        0x01
#define REG_FRF_MSB        0x06
#define REG_FRF_MID        0x07
#define REG_FRF_LSB        0x08
#define REG_PA_CONFIG      0x09
#define REG_OCP            0x0B
#define REG_FIFO_ADDR_PTR  0x0D
#define REG_FIFO_TX_BASE   0x0E
#define REG_IRQ_FLAGS      0x12
#define REG_MODEM_CONFIG_1 0x1D
#define REG_MODEM_CONFIG_2 0x1E
#define REG_PREAMBLE_MSB   0x20
#define REG_PREAMBLE_LSB   0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_SYNC_WORD      0x39
#define REG_DIO_MAPPING_1  0x40
#define REG_VERSION        0x42
#define REG_PA_DAC         0x4D

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP      0x00
#define MODE_STDBY      0x01
#define MODE_TX         0x03
#define IRQ_TX_DONE     0x08
#define PA_BOOST        0x80
#define SX127X_VERSION  0x12

static spi_device_handle_t s_spi;
static QueueHandle_t       s_txq;
static lbr_transport_t     s_link;
static lbr_lora_modem_t    s_modem;
static uint32_t            s_duty_permille;
static uint32_t            s_seq;
static atomic_bool         s_ready;
static atomic_uint         s_sent, s_dropped, s_bytes, s_errors;

typedef struct {
    uint8_t len;
    uint8_t data[LBR_PKT_LORA_MAX];
} tx_item_t;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .length  = 16,
        .tx_data = {(uint8_t)(reg | 0x80), val},
    };
    return spi_device_polling_transmit(s_spi, &t);
}

static uint8_t reg_read(uint8_t reg)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length  = 16,
        .tx_data = {(uint8_t)(reg & 0x7F), 0},
    };
    if (spi_device_polling_transmit(s_spi, &t) != ESP_OK) {
        return 0;
    }
    return t.rx_data[1];
}

static esp_err_t fifo_write(const uint8_t *data, size_t len)
{
    uint8_t buf[1 + LBR_PKT_LORA_MAX];
    buf[0] = REG_FIFO | 0x80;
    memcpy(buf + 1, data, len);
    spi_transaction_t t = {.length = 8 * (len + 1), .tx_buffer = buf};
    return spi_device_polling_transmit(s_spi, &t);
}

static bool radio_init(void)
{
    if (CONFIG_LBR_LORA_RST_GPIO >= 0) {
        gpio_set_direction(CONFIG_LBR_LORA_RST_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(CONFIG_LBR_LORA_RST_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
        gpio_set_level(CONFIG_LBR_LORA_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uint8_t version = reg_read(REG_VERSION);
    if (version != SX127X_VERSION) {
        ESP_LOGE(TAG, "SX127x not found (version 0x%02x)", version);
        return false;
    }
    reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);

    uint64_t frf = ((uint64_t)CONFIG_LBR_LORA_FREQ_HZ << 19) / 32000000u;
    reg_write(REG_FRF_MSB, (uint8_t)(frf >> 16));
    reg_write(REG_FRF_MID, (uint8_t)(frf >> 8));
    reg_write(REG_FRF_LSB, (uint8_t)frf);

    uint8_t bw_code = CONFIG_LBR_LORA_BW_KHZ >= 500 ? 9 : CONFIG_LBR_LORA_BW_KHZ >= 250 ? 8 : 7;
    reg_write(REG_MODEM_CONFIG_1, (uint8_t)((bw_code << 4) | (CONFIG_LBR_LORA_CR << 1)));
    reg_write(REG_MODEM_CONFIG_2, (uint8_t)((CONFIG_LBR_LORA_SF << 4) | 0x04 /* CRC on */));
    uint32_t tsym_us = (1u << CONFIG_LBR_LORA_SF) * 1000u / CONFIG_LBR_LORA_BW_KHZ;
    reg_write(REG_MODEM_CONFIG_3, (uint8_t)((tsym_us > 16000u ? 0x08 : 0x00) | 0x04 /* AGC */));
    reg_write(REG_PREAMBLE_MSB, 0);
    reg_write(REG_PREAMBLE_LSB, 8);
    reg_write(REG_SYNC_WORD, CONFIG_LBR_LORA_SYNC_WORD);

    /* PA_BOOST output; +20 dBm needs the high-power DAC setting. */
    int dbm = CONFIG_LBR_LORA_TX_DBM;
    if (dbm > 17) {
        reg_write(REG_PA_DAC, 0x87);
        reg_write(REG_PA_CONFIG, (uint8_t)(PA_BOOST | (dbm - 5)));
        reg_write(REG_OCP, 0x20 | 0x12); /* 150 mA over-current trip */
    } else {
        reg_write(REG_PA_DAC, 0x84);
        reg_write(REG_PA_CONFIG, (uint8_t)(PA_BOOST | (dbm - 2)));
        reg_write(REG_OCP, 0x20 | 0x0B); /* 100 mA over-current trip */
    }
    reg_write(REG_FIFO_TX_BASE, 0);
    reg_write(REG_DIO_MAPPING_1, 0x40); /* DIO0 = TxDone */
    reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);

    s_modem = (lbr_lora_modem_t){
        .sf       = CONFIG_LBR_LORA_SF,
        .bw_hz    = CONFIG_LBR_LORA_BW_KHZ * 1000u,
        .cr       = CONFIG_LBR_LORA_CR,
        .preamble = 8,
        .crc      = true,
    };
    return true;
}

static bool transmit(const uint8_t *data, size_t len, uint32_t airtime_us)
{
    reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
    reg_write(REG_FIFO_ADDR_PTR, 0);
    if (fifo_write(data, len) != ESP_OK) {
        return false;
    }
    reg_write(REG_PAYLOAD_LENGTH, (uint8_t)len);
    reg_write(REG_IRQ_FLAGS, 0xFF);
    reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    /* Poll TxDone (DIO0 is optional wiring). Generous timeout: 2x airtime. */
    int64_t deadline = esp_timer_get_time() + 2 * (int64_t)airtime_us + 50000;
    while (esp_timer_get_time() < deadline) {
        bool done = CONFIG_LBR_LORA_DIO0_GPIO >= 0 ? gpio_get_level(CONFIG_LBR_LORA_DIO0_GPIO)
                                                   : (reg_read(REG_IRQ_FLAGS) & IRQ_TX_DONE);
        if (done) {
            reg_write(REG_IRQ_FLAGS, 0xFF);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    reg_write(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
    return false;
}

static void tx_task(void *arg)
{
    (void)arg;
    tx_item_t item;
    for (;;) {
        atomic_store(&s_ready, true);
        xQueueReceive(s_txq, &item, portMAX_DELAY);
        atomic_store(&s_ready, false);

        lbr_pkt_restamp(item.data, item.len, LBR_LINK_LORA, s_seq);
        uint32_t airtime = lbr_lora_airtime_us(&s_modem, item.len);
        uint64_t start   = (uint64_t)esp_timer_get_time();
        if (transmit(item.data, item.len, airtime)) {
            s_seq++;
            atomic_fetch_add(&s_sent, 1);
            atomic_fetch_add(&s_bytes, item.len);
        } else {
            atomic_fetch_add(&s_errors, 1);
            ESP_LOGW(TAG, "TX timeout");
        }
        uint64_t next = lbr_lora_next_tx_us(start, airtime, s_duty_permille);
        uint64_t now  = (uint64_t)esp_timer_get_time();
        if (next > now) {
            vTaskDelay(pdMS_TO_TICKS((next - now) / 1000u + 1u));
        }
    }
}

static int lora_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    (void)self;
    tx_item_t item;
    if (len > LBR_PKT_LORA_MAX || !atomic_load(&s_ready)) {
        atomic_fetch_add(&s_dropped, 1);
        return -1;
    }
    item.len = (uint8_t)len;
    memcpy(item.data, pkt, len);
    if (xQueueSend(s_txq, &item, 0) != pdTRUE) {
        atomic_fetch_add(&s_dropped, 1);
        return -1;
    }
    atomic_store(&s_ready, false);
    return 0;
}

static void lora_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    (void)self;
    out->packets_sent    = atomic_load(&s_sent);
    out->packets_dropped = atomic_load(&s_dropped);
    out->bytes_sent      = atomic_load(&s_bytes);
    out->errors          = atomic_load(&s_errors);
    out->connected       = s_spi != NULL;
}

bool lbr_link_lora_ready(void)
{
    return atomic_load(&s_ready);
}

lbr_transport_t *lbr_link_lora_start(uint32_t duty_permille)
{
    if (CONFIG_LBR_LORA_SCLK_GPIO < 0 || CONFIG_LBR_LORA_MOSI_GPIO < 0 ||
        CONFIG_LBR_LORA_MISO_GPIO < 0 || CONFIG_LBR_LORA_CS_GPIO < 0) {
        ESP_LOGE(TAG, "LoRa pins not configured (CONFIG_LBR_LORA_* are TBD)");
        return NULL;
    }
    s_duty_permille      = duty_permille;
    spi_bus_config_t bus = {
        .mosi_io_num     = CONFIG_LBR_LORA_MOSI_GPIO,
        .miso_io_num     = CONFIG_LBR_LORA_MISO_GPIO,
        .sclk_io_num     = CONFIG_LBR_LORA_SCLK_GPIO,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LBR_PKT_LORA_MAX + 1,
    };
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode           = 0,
        .spics_io_num   = CONFIG_LBR_LORA_CS_GPIO,
        .queue_size     = 1,
    };
    esp_err_t bus_rc = spi_bus_initialize(LORA_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
    if ((bus_rc != ESP_OK && bus_rc != ESP_ERR_INVALID_STATE) || /* shared: already up */
        spi_bus_add_device(LORA_SPI_HOST, &dev, &s_spi) != ESP_OK) {
        ESP_LOGE(TAG, "SPI init failed");
        return NULL;
    }
    if (CONFIG_LBR_LORA_DIO0_GPIO >= 0) {
        gpio_set_direction(CONFIG_LBR_LORA_DIO0_GPIO, GPIO_MODE_INPUT);
    }
    if (!radio_init()) {
        return NULL;
    }
    s_txq = xQueueCreate(1, sizeof(tx_item_t));
    if (s_txq == NULL || xTaskCreate(tx_task, "lbr_lora", 4096, NULL, 4, NULL) != pdPASS) {
        return NULL;
    }
    ESP_LOGI(TAG, "SX127x up: %d Hz SF%d BW%dk CR4/%d %d dBm", CONFIG_LBR_LORA_FREQ_HZ,
             CONFIG_LBR_LORA_SF, CONFIG_LBR_LORA_BW_KHZ, CONFIG_LBR_LORA_CR + 4,
             CONFIG_LBR_LORA_TX_DBM);
    s_link = (lbr_transport_t){
        .name      = "lora",
        .kind      = LBR_LINK_LORA,
        .send      = lora_send,
        .get_stats = lora_stats,
    };
    return &s_link;
}
