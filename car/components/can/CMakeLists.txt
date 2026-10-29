# can.c is pure (host-tested); can_bus_esp.c is the TWAI adapter.
idf_component_register(
    SRCS "can.c" "can_bus_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol health util
    PRIV_REQUIRES esp_driver_twai esp_timer freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
