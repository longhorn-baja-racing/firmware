idf_component_register(
    SRCS "protocol.c" "frames.c"
    INCLUDE_DIRS "include"
    REQUIRES util catalog
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
