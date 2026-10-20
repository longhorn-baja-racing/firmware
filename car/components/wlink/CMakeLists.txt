# wlink.c, corner_core.c, corner_rx.c, dash_link.c and dash_core.c are pure
# (host-tested); udp_esp.c is the ESP32-C5 WiFi + UDP glue. Only the nodes
# with a wireless link (imu, corner, central, dashboard) depend on it.
idf_component_register(
    SRCS "wlink.c" "corner_core.c" "corner_rx.c" "dash_link.c" "dash_core.c" "udp_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol can node sensors timebase transport util health
    PRIV_REQUIRES esp_wifi esp_netif esp_event esp_timer lwip freertos log nvs_flash
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
