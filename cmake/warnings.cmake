# Shared compile-warning policy for LBR firmware.
#
# Firmware must build with warnings treated as errors (see README "Test workflow").
# Apply to a target with:  lbr_apply_warnings(<target>)
#
# For ESP-IDF components/apps, call this in the component CMakeLists after
# idf_component_register(), passing ${COMPONENT_LIB}. For host test targets,
# pass the plain target name.

set(LBR_WARN_FLAGS -Wall -Wextra -Wpedantic -Werror)

function(lbr_apply_warnings target)
    target_compile_options(${target} PRIVATE ${LBR_WARN_FLAGS})
endfunction()
