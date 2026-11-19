idf_component_register(
    SRCS "main.c" "cockpit_plan.c" "display.c" "display_graphics.c"
    INCLUDE_DIRS "."
    REQUIRES lbr_core lbr_cockpit log lvgl
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../../lbrlib/cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
