# Development-only host build: unit tests, golden fixtures, and the car
# simulator. Nothing here is ever built into firmware (see car/).
#
#   cmake -S dev -B build-dev -G Ninja && cmake --build build-dev
#   ctest --test-dir build-dev --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(lbr_dev C)
enable_testing()

add_subdirectory(tests/host)
add_subdirectory(sim)
