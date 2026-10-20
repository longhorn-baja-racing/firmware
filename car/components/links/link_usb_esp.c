#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lbr/bytes.h"
#include "lbr/cobs.h"
#include "lbr/crc32.h"
#include "lbr/log_sd.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/protocol.h"
#include "lbr/timebase.h"
#include "lbr/transport_links.h"
#include "sdkconfig.h"

/*
 * USB serial link on UART0 (through the board's USB-UART bridge). Every byte
 * the Central writes is a COBS frame + 0x00, including log text, so a host
 * reader never has to separate binary from text. Telemetry never blocks: a
 * frame that does not fit in the UART TX buffer is dropped and counted. Log
 * dumps are bulk transfers and wait for buffer space instead.
 */

#define UART_PORT     UART_NUM_0
#define RX_BUF        1024
#define CMD_LINE_MAX  80
#define DUMP_READ     4096
#define DUMP_REC_DATA 200
#define LOG_LINE_MAX  200

static lbr_link_hooks_t  s_hooks;
static lbr_transport_t   s_link;
static SemaphoreHandle_t s_tx_lock;
static uint8_t           s_enc[LBR_COBS_MAX_ENCODED(LBR_PKT_MAX_SIZE) + 1];
static uint8_t           s_pkt_copy[LBR_PKT_MAX_SIZE];
static uint32_t          s_seq;
static atomic_bool       s_ready;
static atomic_uint       s_sent, s_dropped, s_bytes, s_errors;

/* Encode and queue one packet. `wait` lets bulk dumps wait for space. */
static int usb_write(const uint8_t *pkt, size_t len, bool wait)
{
    if (!atomic_load(&s_ready) || len > LBR_PKT_MAX_SIZE) {
        atomic_fetch_add(&s_dropped, 1);
        return -1;
    }
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    memcpy(s_pkt_copy, pkt, len);
    lbr_pkt_restamp(s_pkt_copy, len, LBR_LINK_USB, s_seq);
    size_t n   = lbr_cobs_encode(s_pkt_copy, len, s_enc, sizeof s_enc - 1);
    s_enc[n++] = 0x00;

    size_t free_bytes = 0;
    uart_get_tx_buffer_free_size(UART_PORT, &free_bytes);
    while (wait && free_bytes < n) {
        xSemaphoreGive(s_tx_lock);
        vTaskDelay(1);
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        uart_get_tx_buffer_free_size(UART_PORT, &free_bytes);
    }
    int rc = -1;
    if (free_bytes >= n && uart_write_bytes(UART_PORT, s_enc, n) == (int)n) {
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

static int usb_send(lbr_transport_t *self, const uint8_t *pkt, size_t len)
{
    (void)self;
    return usb_write(pkt, len, false);
}

static void usb_stats(lbr_transport_t *self, lbr_transport_stats_t *out)
{
    (void)self;
    out->packets_sent    = atomic_load(&s_sent);
    out->packets_dropped = atomic_load(&s_dropped);
    out->bytes_sent      = atomic_load(&s_bytes);
    out->errors          = atomic_load(&s_errors);
    out->connected       = atomic_load(&s_ready);
}

/* Send one record in its own packet (command replies, log text). */
static int send_record(uint8_t type, const uint8_t *payload, size_t len, bool wait)
{
    uint8_t           pkt[LBR_PKT_HEADER_SIZE + LBR_PKT_REC_HDR + 255 + LBR_PKT_CRC_SIZE];
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, LBR_NODE_CENTRAL, 0, lbr_time_now_us());
    if (!lbr_pkt_add(&b, type, payload, len)) {
        return -1;
    }
    return usb_write(pkt, lbr_pkt_finish(&b), wait);
}

int lbr_usb_log_vprintf(const char *fmt, va_list ap)
{
    char line[LOG_LINE_MAX];
    int  n = vsnprintf(line, sizeof line, fmt, ap);
    if (n <= 0) {
        return n;
    }
    if (s_tx_lock != NULL && xSemaphoreGetMutexHolder(s_tx_lock) == xTaskGetCurrentTaskHandle()) {
        /* Logged from inside our own write path: re-entering would deadlock. */
        atomic_fetch_add(&s_dropped, 1);
        return n;
    }
    size_t len = (size_t)n < sizeof line ? (size_t)n : sizeof line - 1;
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
        len--;
    }
    send_record(LBR_REC_TEXT, (const uint8_t *)line, len, false);
    return n;
}

/* ------------------------------------------------------------------------ */
/* Commands                                                                  */

static void reply_text(const char *text)
{
    size_t len = strlen(text);
    for (size_t off = 0; off < len; off += 255) {
        size_t n = len - off > 255 ? 255 : len - off;
        send_record(LBR_REC_TEXT, (const uint8_t *)text + off, n, true);
    }
}

static void on_list(const char *name, uint32_t size, void *ctx)
{
    (void)ctx;
    uint8_t rec[4 + LBR_SD_NAME_MAX];
    size_t  nlen = strlen(name);
    lbr_put_u32(rec, size);
    memcpy(rec + 4, name, nlen);
    send_record(LBR_REC_FILE_INFO, rec, 4 + nlen, true);
}

static void do_dump(const char *name, uint32_t offset)
{
    static uint8_t buf[DUMP_READ];
    uint8_t        pkt[LBR_PKT_MAX_SIZE];
    uint32_t       crc = 0;

    if (!lbr_sd_valid_name(name) || lbr_sd_file_size(name) < 0) {
        reply_text("ERR no such file");
        return;
    }
    for (;;) {
        int n = lbr_sd_read(name, offset, buf, sizeof buf);
        if (n <= 0) {
            break;
        }
        lbr_pkt_builder_t b;
        lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, LBR_NODE_CENTRAL, 0, lbr_time_now_us());
        for (int pos = 0; pos < n; pos += DUMP_REC_DATA) {
            uint8_t rec[4 + DUMP_REC_DATA];
            size_t  len = (size_t)(n - pos) > DUMP_REC_DATA ? DUMP_REC_DATA : (size_t)(n - pos);
            lbr_put_u32(rec, offset + (uint32_t)pos);
            memcpy(rec + 4, buf + pos, len);
            if (!lbr_pkt_add(&b, LBR_REC_FILE_CHUNK, rec, 4 + len)) {
                usb_write(pkt, lbr_pkt_finish(&b), true);
                lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, LBR_NODE_CENTRAL, 0,
                              lbr_time_now_us());
                lbr_pkt_add(&b, LBR_REC_FILE_CHUNK, rec, 4 + len);
            }
        }
        usb_write(pkt, lbr_pkt_finish(&b), true);
        crc = lbr_crc32_update(crc, buf, (size_t)n);
        offset += (uint32_t)n;
    }
    /* FILE_END: total bytes sent from the requested offset, CRC-32 of them. */
    uint8_t end[8 + LBR_SD_NAME_MAX];
    size_t  nlen = strlen(name);
    lbr_put_u32(end, offset);
    lbr_put_u32(end + 4, crc);
    memcpy(end + 8, name, nlen);
    send_record(LBR_REC_FILE_END, end, 8 + nlen, true);
}

static void handle_line(char *line)
{
    char *arg = strchr(line, ' ');
    if (arg != NULL) {
        *arg++ = '\0';
    }
    if (strcmp(line, "STAT") == 0) {
        static char text[1024];
        if (s_hooks.status != NULL) {
            s_hooks.status(text, sizeof text);
            reply_text(text);
        }
    } else if (strcmp(line, "LIST") == 0) {
        if (lbr_sd_list(on_list, NULL) < 0) {
            reply_text("ERR no card");
        }
    } else if (strcmp(line, "DUMP") == 0 && arg != NULL) {
        char    *off  = strchr(arg, ' ');
        uint32_t from = 0;
        if (off != NULL) {
            *off++ = '\0';
            from   = (uint32_t)strtoul(off, NULL, 10);
        }
        do_dump(arg, from);
    } else {
        reply_text("ERR unknown command (STAT, LIST, DUMP <name> [offset])");
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    char   line[CMD_LINE_MAX];
    size_t len = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(UART_PORT, &c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == '\n' || c == '\r') {
            if (len > 0) {
                line[len] = '\0';
                handle_line(line);
                len = 0;
            }
        } else if (len + 1 < sizeof line && c >= 0x20 && c < 0x7F) {
            line[len++] = (char)c;
        } else {
            len = 0; /* overlong or binary: discard the line */
            atomic_fetch_add(&s_errors, 1);
        }
    }
}

lbr_transport_t *lbr_link_usb_start(const lbr_link_hooks_t *hooks)
{
    s_hooks           = *hooks;
    s_tx_lock         = xSemaphoreCreateMutex();
    uart_config_t cfg = {
        .baud_rate  = CONFIG_LBR_USB_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (s_tx_lock == NULL ||
        uart_driver_install(UART_PORT, RX_BUF, CONFIG_LBR_USB_TX_BUFFER_KB * 1024, 0, NULL, 0) !=
            ESP_OK ||
        uart_param_config(UART_PORT, &cfg) != ESP_OK ||
        xTaskCreate(rx_task, "lbr_usb_rx", 4096, NULL, 5, NULL) != pdPASS) {
        return NULL;
    }
    /* Route console writes (printf, stdio) through the driver too, so they
     * queue whole and cannot land in the middle of a COBS frame. */
    uart_vfs_dev_use_driver(UART_PORT);
    atomic_store(&s_ready, true);
    s_link = (lbr_transport_t){
        .name      = "usb",
        .kind      = LBR_LINK_USB,
        .send      = usb_send,
        .get_stats = usb_stats,
    };
    return &s_link;
}
