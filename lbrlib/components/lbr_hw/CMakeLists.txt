# LBRLib layer 3, ESP32 glue every board uses: the TWAI CAN bus, the clock port,
# reset reasons, ADC/pulse/switch inputs, the GPS UART, OTA slots, and the SD log.
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES lbr_base lbr_core
    PRIV_REQUIRES esp_driver_twai esp_driver_gpio esp_driver_uart esp_driver_spi esp_driver_sdspi
                  esp_driver_sdmmc esp_adc esp_timer esp_system esp_partition esp_app_format
                  app_update nvs_flash fatfs sdmmc vfs heap freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
