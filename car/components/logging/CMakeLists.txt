# logging.c is pure (host-tested); log_sd_esp.c mounts the card and provides
# the FATFS-backed lbr_log_storage_t; log_service_esp.c runs the writer task.
idf_component_register(
    SRCS "logging.c" "log_sd_esp.c" "log_service_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol health can util
    PRIV_REQUIRES fatfs sdmmc esp_driver_sdspi esp_driver_sdmmc esp_driver_spi vfs log
                  freertos esp_timer heap
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
