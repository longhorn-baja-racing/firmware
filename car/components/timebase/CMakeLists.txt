# timebase.c is pure (host-tested); timebase_port_esp.c backs it with esp_timer.
idf_component_register(
    SRCS "timebase.c" "timebase_port_esp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_timer freertos
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
