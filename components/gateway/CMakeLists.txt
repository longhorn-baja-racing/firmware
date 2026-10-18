# Gateway-only logic that is pure (host-tested): runtime config and the
# shared status text.
idf_component_register(
    SRCS "gw_config.c" "status.c"
    INCLUDE_DIRS "include"
    REQUIRES health node protocol
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
