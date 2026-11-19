# LBRLib layer 4, the Central role: LBR_CENTRAL_MAIN and its FreeRTOS runtime (the
# time master, the authoritative SD log, the live links, the car logic tick, and the
# Central's NVS-stored runtime configuration).
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES lbr_base lbr_core lbr_hw lbr_links
    PRIV_REQUIRES esp_app_format app_update esp_timer esp_system nvs_flash freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
