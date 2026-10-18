#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/logging.h"

/*
 * microSD backend for the logger (ESP-IDF FATFS). Hardware-facing; not built
 * on host. A missing or failed card is a normal, reported state: mount
 * returns an error and the caller keeps running and retries.
 *
 * Session files are /sdcard/lbr/NNNNNNNN.LBR, numbered one past the highest
 * existing file, so a new boot never overwrites an older session.
 */

#define LBR_SD_MOUNT_POINT "/sdcard"
#define LBR_SD_LOG_DIR     LBR_SD_MOUNT_POINT "/lbr"
#define LBR_SD_NAME_MAX    16

typedef enum {
    LBR_SD_OK              = 0,
    LBR_SD_ERR_PINS_UNSET  = -1, /* Kconfig SD pins still -1 (TBD) */
    LBR_SD_ERR_MOUNT       = -2, /* no card, unformatted, or bus error */
    LBR_SD_ERR_FILE        = -3,
    LBR_SD_ERR_NOT_MOUNTED = -4,
} lbr_sd_err_t;

lbr_sd_err_t lbr_sd_mount(void);
void         lbr_sd_unmount(void);
bool         lbr_sd_is_mounted(void);

/* Create the next session file and return a storage backend writing to it. */
lbr_sd_err_t lbr_sd_open_session(lbr_log_storage_t *out, char name[LBR_SD_NAME_MAX]);

/* Enumerate session files (name without directory, size in bytes). */
typedef void (*lbr_sd_list_cb)(const char *name, uint32_t size, void *ctx);
int lbr_sd_list(lbr_sd_list_cb cb, void *ctx);

/* Read part of a session file (for USB/WiFi dumps). Returns bytes read,
 * 0 at end of file, <0 on error. `name` must be a bare session file name. */
int lbr_sd_read(const char *name, uint32_t offset, uint8_t *buf, size_t cap);

/* Size of a session file, or <0 if it does not exist. */
int32_t lbr_sd_file_size(const char *name);

/* True if `name` is a well-formed session file name (no paths). */
bool lbr_sd_valid_name(const char *name);
