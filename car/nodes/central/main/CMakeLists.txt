idf_component_register(
    SRCS "main.c" "central.c" "central_plan.c" "gw_store.c"
    INCLUDE_DIRS "."
    REQUIRES can gateway health links logging node protocol sensors timebase transport util wlink
             nvs_flash app_update esp_timer esp_system freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
