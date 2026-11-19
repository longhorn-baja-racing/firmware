# Firmware update over CAN. update.c and upd_relay.c are pure (host-tested);
# update_esp.c is the sensor nodes' flash access and boot-time guard.
idf_component_register(
    SRCS "update.c" "upd_relay.c" "update_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES can protocol util
    PRIV_REQUIRES app_update esp_app_format esp_partition nvs_flash esp_timer esp_system freertos
                  log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
