# health.c is pure (host-tested); health_esp.c holds the platform hooks.
idf_component_register(
    SRCS "health.c" "health_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES util
    PRIV_REQUIRES esp_system
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
