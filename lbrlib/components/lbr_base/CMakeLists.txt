# LBRLib layer 1, base (pure C, no ESP-IDF headers): byte order, CRC, COBS, ring
# buffer; the protocol, catalog codecs, and CAN frame table; heartbeat/health
# codec; CAN codec; the time-sync servo; the log format and session keeper;
# packet framing. Depends only on the car's catalog.
idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES catalog
)
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/warnings.cmake")
lbr_apply_warnings(${COMPONENT_LIB})
