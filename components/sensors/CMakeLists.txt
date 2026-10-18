# Generic sensor interface plus part-independent drivers. Real part drivers go
# here as sensor_<part>.c, each exposing a function that fills an lbr_sensor_t.
idf_component_register(
    SRCS "sensor_unconfigured.c" "sensor_sim.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
