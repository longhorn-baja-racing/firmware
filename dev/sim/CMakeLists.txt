# Car simulator: the real firmware logic (lbr_core, the node plans) on a
# simulated car. Scenarios in scenarios/ run as CTest tests.
add_executable(lbr_sim main.c sim.c vbus.c world.c)
target_link_libraries(lbr_sim PRIVATE lbr_plans lbr_core m)
lbr_apply_warnings(lbr_sim)

file(GLOB LBR_SIM_SCENARIOS CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/scenarios/*.sim")
foreach(scenario IN LISTS LBR_SIM_SCENARIOS)
    get_filename_component(name "${scenario}" NAME_WE)
    add_test(NAME sim_${name}
             COMMAND lbr_sim --out "${CMAKE_CURRENT_BINARY_DIR}/runs/${name}" "${scenario}")
endforeach()
