# LBRLib layer 4, the sensor-node role: LBR_NODE_MAIN and its FreeRTOS runtime.
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES lbr_base lbr_core lbr_hw
    PRIV_REQUIRES esp_timer esp_system heap freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
