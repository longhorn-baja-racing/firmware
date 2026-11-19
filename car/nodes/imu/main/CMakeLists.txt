idf_component_register(
    SRCS "main.c" "imu_plan.c"
    INCLUDE_DIRS "."
    REQUIRES lbr node sensors protocol
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../../lbrlib/cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
