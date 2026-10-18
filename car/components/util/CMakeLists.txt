idf_component_register(
    SRCS "crc32.c" "cobs.c" "ringbuf.c"
    INCLUDE_DIRS "include"
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
