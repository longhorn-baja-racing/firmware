#include "lbr/log_sd.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"
#if CONFIG_LBR_SD_BUS_SPI
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#else
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "lbr_sd";

static sdmmc_card_t *s_card;
static bool          s_spi_bus_ready;

typedef struct {
    int fd;
} sd_file_t;

static sd_file_t s_file = {.fd = -1};

bool lbr_sd_valid_name(const char *name)
{
    /* NNNNNNNN.LBR */
    if (name == NULL || strlen(name) != 12 || strcmp(name + 8, ".LBR") != 0) {
        return false;
    }
    for (int i = 0; i < 8; i++) {
        if (name[i] < '0' || name[i] > '9') {
            return false;
        }
    }
    return true;
}

lbr_sd_err_t lbr_sd_mount(void)
{
    if (s_card != NULL) {
        return LBR_SD_OK;
    }
    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false, /* never erase a card that might hold data */
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };
    esp_err_t err;

#if CONFIG_LBR_SD_BUS_SPI
    if (CONFIG_LBR_SD_SPI_MOSI_GPIO < 0 || CONFIG_LBR_SD_SPI_MISO_GPIO < 0 ||
        CONFIG_LBR_SD_SPI_SCLK_GPIO < 0 || CONFIG_LBR_SD_SPI_CS_GPIO < 0) {
        ESP_LOGE(TAG, "SD SPI pins not configured (CONFIG_LBR_SD_SPI_* are TBD)");
        return LBR_SD_ERR_PINS_UNSET;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    if (!s_spi_bus_ready) {
        spi_bus_config_t bus = {
            .mosi_io_num     = CONFIG_LBR_SD_SPI_MOSI_GPIO,
            .miso_io_num     = CONFIG_LBR_SD_SPI_MISO_GPIO,
            .sclk_io_num     = CONFIG_LBR_SD_SPI_SCLK_GPIO,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = 16 * 1024,
        };
        err = spi_bus_initialize(host.slot, &bus, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
            return LBR_SD_ERR_MOUNT;
        }
        s_spi_bus_ready = true;
    }
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs               = CONFIG_LBR_SD_SPI_CS_GPIO;
    slot.gpio_cd               = CONFIG_LBR_SD_CD_GPIO;
    slot.host_id               = host.slot;
    err = esp_vfs_fat_sdspi_mount(LBR_SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
#else
    sdmmc_host_t        host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width               = CONFIG_LBR_SD_SDMMC_4BIT ? 4 : 1;
    slot.cd                  = CONFIG_LBR_SD_CD_GPIO;
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    err = esp_vfs_fat_sdmmc_mount(LBR_SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
#endif

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mount failed: %s (no card?)", esp_err_to_name(err));
        s_card = NULL;
        return LBR_SD_ERR_MOUNT;
    }
    if (mkdir(LBR_SD_LOG_DIR, 0775) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "cannot create %s", LBR_SD_LOG_DIR);
        lbr_sd_unmount();
        return LBR_SD_ERR_FILE;
    }
    ESP_LOGI(TAG, "mounted %s, %llu MiB", s_card->cid.name,
             (unsigned long long)s_card->csd.capacity * s_card->csd.sector_size / (1024 * 1024));
    return LBR_SD_OK;
}

void lbr_sd_unmount(void)
{
    if (s_file.fd >= 0) {
        close(s_file.fd);
        s_file.fd = -1;
    }
    if (s_card != NULL) {
        esp_vfs_fat_sdcard_unmount(LBR_SD_MOUNT_POINT, s_card);
        s_card = NULL;
    }
}

bool lbr_sd_is_mounted(void)
{
    return s_card != NULL;
}

static int file_write(void *ctx, const uint8_t *buf, size_t len)
{
    sd_file_t *f = ctx;
    while (len > 0) {
        ssize_t n = write(f->fd, buf, len);
        if (n <= 0) {
            return -1;
        }
        buf += n;
        len -= (size_t)n;
    }
    return 0;
}

static int file_sync(void *ctx)
{
    sd_file_t *f = ctx;
    return fsync(f->fd) == 0 ? 0 : -1;
}

static int file_close(void *ctx)
{
    sd_file_t *f  = ctx;
    int        rc = close(f->fd);
    f->fd         = -1;
    return rc == 0 ? 0 : -1;
}

static uint32_t highest_session_number(void)
{
    uint32_t max = 0;
    DIR     *d   = opendir(LBR_SD_LOG_DIR);
    if (d == NULL) {
        return 0;
    }
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (lbr_sd_valid_name(e->d_name)) {
            uint32_t n = (uint32_t)strtoul(e->d_name, NULL, 10);
            if (n > max) {
                max = n;
            }
        }
    }
    closedir(d);
    return max;
}

lbr_sd_err_t lbr_sd_open_session(lbr_log_storage_t *out, char name[LBR_SD_NAME_MAX])
{
    if (s_card == NULL) {
        return LBR_SD_ERR_NOT_MOUNTED;
    }
    if (s_file.fd >= 0) {
        close(s_file.fd);
        s_file.fd = -1;
    }
    uint32_t next = highest_session_number() + 1;
    char     path[64];
    snprintf(name, LBR_SD_NAME_MAX, "%08lu.LBR", (unsigned long)(next % 100000000u));
    snprintf(path, sizeof path, LBR_SD_LOG_DIR "/%s", name);

    s_file.fd = open(path, O_WRONLY | O_CREAT | O_EXCL, 0664);
    if (s_file.fd < 0) {
        ESP_LOGE(TAG, "cannot create %s (errno %d)", path, errno);
        return LBR_SD_ERR_FILE;
    }
    out->write = file_write;
    out->sync  = file_sync;
    out->close = file_close;
    out->ctx   = &s_file;
    ESP_LOGI(TAG, "session file %s", path);
    return LBR_SD_OK;
}

int lbr_sd_list(lbr_sd_list_cb cb, void *ctx)
{
    if (s_card == NULL) {
        return LBR_SD_ERR_NOT_MOUNTED;
    }
    DIR *d = opendir(LBR_SD_LOG_DIR);
    if (d == NULL) {
        return LBR_SD_ERR_FILE;
    }
    int            count = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (lbr_sd_valid_name(e->d_name)) {
            int32_t size = lbr_sd_file_size(e->d_name);
            cb(e->d_name, size < 0 ? 0u : (uint32_t)size, ctx);
            count++;
        }
    }
    closedir(d);
    return count;
}

int32_t lbr_sd_file_size(const char *name)
{
    char        path[64];
    struct stat st;
    if (s_card == NULL || !lbr_sd_valid_name(name)) {
        return -1;
    }
    snprintf(path, sizeof path, LBR_SD_LOG_DIR "/%s", name);
    return stat(path, &st) == 0 ? (int32_t)st.st_size : -1;
}

int lbr_sd_read(const char *name, uint32_t offset, uint8_t *buf, size_t cap)
{
    char path[64];
    if (s_card == NULL) {
        return LBR_SD_ERR_NOT_MOUNTED;
    }
    if (!lbr_sd_valid_name(name)) {
        return LBR_SD_ERR_FILE;
    }
    snprintf(path, sizeof path, LBR_SD_LOG_DIR "/%s", name);
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        return LBR_SD_ERR_FILE;
    }
    int n = -1;
    if (lseek(fd, (off_t)offset, SEEK_SET) == (off_t)offset) {
        n = (int)read(fd, buf, cap);
    }
    close(fd);
    return n < 0 ? LBR_SD_ERR_FILE : n;
}
