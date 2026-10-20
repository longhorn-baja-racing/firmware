idf_component_register(
    SRCS "main.c" "corner_plan.c"
    INCLUDE_DIRS "."
    REQUIRES node sensors protocol timebase wlink transport freertos esp_timer log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
