# LBRLib's app-facing API: values with status (value.c, pure) and the umbrella
# header <lbr/lbr.h>.
idf_component_register(
    SRCS "value.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
