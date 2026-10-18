# node_config.c, node_core.c, node_table.c and value_table.c are pure (host-tested); node_esp.c is the
# FreeRTOS runtime shared by every sensor node.
idf_component_register(
    SRCS "node_config.c" "node_core.c" "node_table.c" "value_table.c" "node_esp.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol can sensors health util logging timebase
    PRIV_REQUIRES esp_timer freertos log esp_system heap
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
