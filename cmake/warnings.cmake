# Shared compile-warning policy for LBR firmware.
#
# Firmware must build with warnings treated as errors (see README "Test workflow").
# Apply to a target with:  lbr_apply_warnings(<target>)
#
# For ESP-IDF components/apps, call this in the component CMakeLists after
# idf_component_register(), passing ${COMPONENT_LIB}. For host test targets,
# pass the plain target name.
#
# -Wpedantic applies to host builds only: ESP-IDF's own headers use GNU
# extensions (#include_next), so on target it would flag IDF, not our code.
# The pure sources are compiled on host in CI, so they still get the check.

set(LBR_WARN_FLAGS -Wall -Wextra -Werror -Wshadow -Wstrict-prototypes -Wmissing-prototypes)
if(NOT ESP_PLATFORM)
    list(APPEND LBR_WARN_FLAGS -Wpedantic)
endif()

function(lbr_apply_warnings target)
    target_compile_options(${target} PRIVATE ${LBR_WARN_FLAGS})
endfunction()
