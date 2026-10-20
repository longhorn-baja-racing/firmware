# packet.c and lora_policy.c are pure (host-tested): the one packet framing
# every live link and wireless link uses. The radio links themselves are in
# the links component (Central only).
idf_component_register(
    SRCS "packet.c" "lora_policy.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol health util
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
