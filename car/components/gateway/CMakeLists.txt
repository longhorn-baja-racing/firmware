# Gateway-only logic, all pure (host-tested and run by the simulator): the
# gateway core, runtime config, and the shared status text.
idf_component_register(
    SRCS "gw_core.c" "gw_config.c" "status.c"
    INCLUDE_DIRS "include"
    REQUIRES can health logging node protocol timebase transport util
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
