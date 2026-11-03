idf_component_register(
    SRCS "main.c" "cockpit_plan.c" "display.c" "display_graphics.c"
    INCLUDE_DIRS "."
    REQUIRES node sensors protocol timebase wlink transport freertos esp_timer log lvgl
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
