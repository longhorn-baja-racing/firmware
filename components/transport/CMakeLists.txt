# packet.c and lora_policy.c are pure (host-tested). The link_*_esp.c files
# are the concrete gateway links. Only the gateway app depends on this
# component's links; sensor nodes never pull in WiFi or BLE.
idf_component_register(
    SRCS "packet.c" "lora_policy.c"
         "link_usb_esp.c" "link_lora_esp.c" "link_wifi_esp.c" "link_ble_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol health util
    PRIV_REQUIRES esp_driver_uart esp_driver_spi esp_driver_gpio esp_timer freertos log
                  timebase logging esp_wifi esp_netif esp_event esp_http_server app_update
                  lwip esp_system bt
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
