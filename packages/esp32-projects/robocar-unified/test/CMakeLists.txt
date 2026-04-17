cmake_minimum_required(VERSION 3.16)
project(robocar_unified_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Use provided CMAKE_C_FLAGS or set defaults
if(NOT CMAKE_C_FLAGS)
    set(CMAKE_C_FLAGS "-Wall -Wextra -O0 -g -std=c11")
endif()

# Host test feature flags
add_compile_definitions(
    GOAL_STATE_HOST_TEST=1
    REACTIVE_CONTROLLER_HOST_TEST=1
    ULTRASONIC_HOST_TEST=1
)

# Main source directory
set(MAIN_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../main")

# Include directories
include_directories(
    "${MAIN_SRC_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Shared on-host link requirements
find_package(Threads REQUIRED)

enable_testing()

# goal_state host tests (Phase 4, Gate G3) — the only test suite landed in the
# first Phase 4 pass. test_reactive_controller.c / test_ultrasonic_stub.c are
# tracked as Phase 5 follow-ups.
add_executable(test_goal_state
    "${MAIN_SRC_DIR}/goal_state.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_goal_state.c"
)
target_link_libraries(test_goal_state PRIVATE Threads::Threads m)
add_test(NAME goal_state COMMAND test_goal_state)
