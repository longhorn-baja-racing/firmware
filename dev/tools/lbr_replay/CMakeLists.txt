# lbr_replay: re-run the car logic on a recorded log (lbrlib/docs/logging.md, "Replay").
add_executable(lbr_replay lbr_replay.c)
target_link_libraries(lbr_replay PRIVATE lbr_car_logic lbr_core)
lbr_apply_warnings(lbr_replay)

# Test helper: a log cut off between a logic tick and its outputs.
add_executable(cut_mid_tick cut_mid_tick.c)
target_link_libraries(cut_mid_tick PRIVATE lbr_core)
lbr_apply_warnings(cut_mid_tick)
