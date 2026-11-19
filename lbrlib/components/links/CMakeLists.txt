# The Central's live links (ESP only): USB serial, LoRa, the WiFi pit access
# point, BLE. Sensor nodes never pull in this component.
idf_component_register(
    SRCS "link_usb_esp.c" "link_lora_esp.c" "link_wifi_esp.c" "link_ble_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES transport protocol health util
    PRIV_REQUIRES esp_driver_uart esp_driver_spi esp_driver_gpio esp_timer freertos log
                  timebase logging esp_wifi esp_netif esp_event esp_http_server app_update
                  lwip esp_system bt wlink
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
