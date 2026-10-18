idf_component_register(
    SRCS "main.c" "dashboard_plan.c" "display.c"
    INCLUDE_DIRS "."
    REQUIRES node sensors protocol can timebase
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
