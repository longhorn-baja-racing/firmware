# Generic sensor interface plus part-independent drivers. sensor_io.c and nmea.c are pure
# (host-tested); *_esp.c own the pins. Part drivers go here as sensor_<part>.c, each
# exposing a function that fills an lbr_sensor_t.
set(srcs "sensor_unconfigured.c" "sensor_io.c" "nmea.c" "sensor_io_esp.c" "sensor_gps_nmea_esp.c")
# The bench simulator is compiled only into bench builds, never a race build.
if(CONFIG_LBR_SENSOR_SIM)
    list(APPEND srcs "sensor_sim.c")
endif()
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES protocol
    PRIV_REQUIRES esp_adc esp_driver_gpio esp_driver_uart esp_timer freertos log
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
