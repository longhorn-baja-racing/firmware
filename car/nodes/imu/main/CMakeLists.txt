idf_component_register(
    SRCS "main.c" "imu_plan.c"
    INCLUDE_DIRS "."
    REQUIRES node sensors protocol timebase wlink transport freertos
)
include("${CMAKE_CURRENT_LIST_DIR}/../../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
