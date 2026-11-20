# Car simulator: the real firmware logic (lbr_core, the node plans) on a
# simulated car. Scenarios in scenarios/ run as CTest tests.
add_executable(lbr_sim main.c sim_core.c sim_platform.c sim_boot.c sim_report.c sim_cmd.c vbus.c world.c)
target_link_libraries(lbr_sim PRIVATE lbr_plans lbr_car_logic lbr_core m)
lbr_apply_warnings(lbr_sim)

file(GLOB LBR_SIM_SCENARIOS CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/scenarios/*.sim")
foreach(scenario IN LISTS LBR_SIM_SCENARIOS)
    get_filename_component(name "${scenario}" NAME_WE)
    add_test(NAME sim_${name}
             COMMAND lbr_sim --out "${CMAKE_CURRENT_BINARY_DIR}/runs/${name}" "${scenario}")
    set_tests_properties(sim_${name} PROPERTIES FIXTURES_SETUP simlog_${name})
endforeach()

# Replay (lbr_replay): the car logic re-run on each scenario's Central log reproduces
# every recorded output bit for bit. (In sd_card only the first file starts at boot;
# the ones opened after the card comes back start mid-session.)
foreach(name IN ITEMS nominal car_logic can_faults node_missing batteries sensor_faults time_sync
                      wifi sd_card)
    add_test(NAME replay_${name}
             COMMAND lbr_replay "${CMAKE_CURRENT_BINARY_DIR}/runs/${name}/00000001.lbrlog" --check)
    set_tests_properties(replay_${name} PROPERTIES FIXTURES_REQUIRED simlog_${name})
endforeach()
# After the Central reboots (time_sync), the new session replays too: tasks start
# cold on every start (their init resets their state).
add_test(NAME replay_after_reboot
         COMMAND lbr_replay "${CMAKE_CURRENT_BINARY_DIR}/runs/time_sync/00000002.lbrlog" --check)
set_tests_properties(replay_after_reboot PROPERTIES FIXTURES_REQUIRED simlog_time_sync)
# ... and a change to the logic shows: without the brakes task the replay must differ.
add_test(NAME replay_detects_change
         COMMAND lbr_replay "${CMAKE_CURRENT_BINARY_DIR}/runs/car_logic/00000001.lbrlog" --check
                 --drop-task brakes)
set_tests_properties(replay_detects_change PROPERTIES FIXTURES_REQUIRED simlog_car_logic WILL_FAIL TRUE)
# ... a log cut off mid-tick (power loss) still checks: the outputs the cut took
# are not differences ...
add_test(NAME replay_cut_mid_tick
         COMMAND sh -c "$<TARGET_FILE:cut_mid_tick> \"$1\" \"$2\" && $<TARGET_FILE:lbr_replay> \"$2\" --check"
                 cut_mid_tick "${CMAKE_CURRENT_BINARY_DIR}/runs/car_logic/00000001.lbrlog"
                 "${CMAKE_CURRENT_BINARY_DIR}/runs/car_logic/cut.lbrlog")
set_tests_properties(replay_cut_mid_tick PROPERTIES FIXTURES_REQUIRED simlog_car_logic)
# ... and a replay's own output replays again (its Replay/ records are left out).
add_test(NAME replay_of_replay
         COMMAND sh -c "$<TARGET_FILE:lbr_replay> \"$1\" \"$2\" && $<TARGET_FILE:lbr_replay> \"$2\" --check"
                 replay_of_replay "${CMAKE_CURRENT_BINARY_DIR}/runs/car_logic/00000001.lbrlog"
                 "${CMAKE_CURRENT_BINARY_DIR}/runs/car_logic/replayed.lbrlog")
set_tests_properties(replay_of_replay PROPERTIES FIXTURES_REQUIRED simlog_car_logic)
