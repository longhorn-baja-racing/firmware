# This car's logic: LBR_TASKs the Central runs (lbr/logic.h). Add a task in its own
# file here, list it in tasks.c, and add the file below.
idf_component_register(
    SRCS "tasks.c" "cvt.c" "brakes.c" "vehicle.c"
    INCLUDE_DIRS "include"
    REQUIRES lbr_core
)
include("${CMAKE_CURRENT_LIST_DIR}/../../lbrlib/cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
