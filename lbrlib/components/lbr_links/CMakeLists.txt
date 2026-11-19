# LBRLib layer 3, ESP32 glue for the Central and the cockpit: the USB, LoRa,
# WiFi, and BLE links, and UDP for the cockpit link.
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES lbr_base lbr_core lbr_hw
    PRIV_REQUIRES esp_driver_uart esp_driver_spi esp_driver_gpio esp_wifi esp_netif esp_event
                  esp_http_server esp_timer esp_system app_update nvs_flash lwip bt freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
