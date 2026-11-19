# LBRLib layer 4, the cockpit role: LBR_COCKPIT_MAIN and its FreeRTOS runtime (WiFi to
# the Central, the cockpit's own slots, and the display driver's task).
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES lbr_base lbr_core lbr_hw lbr_links
    PRIV_REQUIRES esp_timer freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
