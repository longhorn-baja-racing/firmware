# LBRLib's app-facing API (all pure): values with status (value.c), car logic tasks
# and outputs (logic.c), and the umbrella header lbr.h.
idf_component_register(
    SRCS "value.c" "logic.c"
    INCLUDE_DIRS "include"
    REQUIRES protocol node util
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
