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
# test/include comes first so its esp_err.h / esp_log.h shims shadow any
# ESP-IDF header that leaks in via motor_controller.h (which unconditionally
# includes "esp_err.h") or gemini_parse.c (which uses ESP_LOG* macros). The
# companion host-test guards in goal_state.h / ultrasonic.h /
# reactive_controller.h handle their own typedef under #else.
include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MAIN_SRC_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Shared on-host link requirements
find_package(Threads REQUIRED)

# cJSON — required by gemini_parse.c. Prefer the installed library via
# pkg-config; fall back to Homebrew's prefix on macOS.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(CJSON libcjson)
endif()

if(NOT CJSON_FOUND)
    # Homebrew on Apple Silicon
    set(_CJSON_HINTS "/opt/homebrew/opt/cjson" "/opt/homebrew" "/usr/local/opt/cjson" "/usr/local")
    find_path(CJSON_INCLUDE_DIR cjson/cJSON.h HINTS ${_CJSON_HINTS} PATH_SUFFIXES include)
    find_library(CJSON_LIB cjson HINTS ${_CJSON_HINTS} PATH_SUFFIXES lib)
    if(CJSON_INCLUDE_DIR AND CJSON_LIB)
        set(CJSON_INCLUDE_DIRS "${CJSON_INCLUDE_DIR}/cjson")
        set(CJSON_LIBRARIES "${CJSON_LIB}")
        set(CJSON_FOUND TRUE)
    endif()
endif()

enable_testing()

# -----------------------------------------------------------------------------
# goal_state host tests (Phase 4, gate G3)
# -----------------------------------------------------------------------------
add_executable(test_goal_state
    "${MAIN_SRC_DIR}/goal_state.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_goal_state.c"
)
target_link_libraries(test_goal_state PRIVATE Threads::Threads m)
add_test(NAME goal_state COMMAND test_goal_state)

# -----------------------------------------------------------------------------
# ultrasonic stub regression (Phase 5, issue #226)
# -----------------------------------------------------------------------------
add_executable(test_ultrasonic_stub
    "${MAIN_SRC_DIR}/ultrasonic.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_ultrasonic_stub.c"
)
target_link_libraries(test_ultrasonic_stub PRIVATE Threads::Threads m)
add_test(NAME ultrasonic_stub COMMAND test_ultrasonic_stub)

# -----------------------------------------------------------------------------
# reactive_controller decision logic (Phase 5, gate G2, issue #226)
# -----------------------------------------------------------------------------
add_executable(test_reactive_controller
    "${MAIN_SRC_DIR}/goal_state.c"
    "${MAIN_SRC_DIR}/ultrasonic.c"
    "${MAIN_SRC_DIR}/reactive_controller.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/motor_controller_stub.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_reactive_controller.c"
)
target_link_libraries(test_reactive_controller PRIVATE Threads::Threads m)
add_test(NAME reactive_controller COMMAND test_reactive_controller)

# -----------------------------------------------------------------------------
# gemini parse (Phase 5, gate G1, issue #228)
#
# Only built when cJSON is available. Skipped (with a message) otherwise so
# the other host-test suites still build in environments without cJSON.
# -----------------------------------------------------------------------------
if(CJSON_FOUND)
    add_executable(test_gemini_parse
        "${MAIN_SRC_DIR}/gemini_parse.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/test_gemini_parse.c"
    )
    target_include_directories(test_gemini_parse PRIVATE ${CJSON_INCLUDE_DIRS})
    # pkg-config returns a plain -lcjson without a library path. Pass the
    # library search dirs explicitly so the linker finds libcjson.*.
    target_link_directories(test_gemini_parse PRIVATE ${CJSON_LIBRARY_DIRS})
    target_link_libraries(test_gemini_parse PRIVATE ${CJSON_LIBRARIES} Threads::Threads m)
    target_compile_definitions(test_gemini_parse PRIVATE
        "FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/fixtures\""
    )
    add_test(NAME gemini_parse COMMAND test_gemini_parse)
else()
    message(STATUS "cJSON not found — skipping test_gemini_parse. Install via `brew install cjson` or `apt install libcjson-dev`.")
endif()
