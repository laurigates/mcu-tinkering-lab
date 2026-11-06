# robocar-twin: the robocar planner loop on a virtual clock (ADR-030, PRD-017).
#
# Built from the firmware's own sources in ../unified/main against the host
# shims in ../unified/test/include, the same way the unified host tests are.
#
#   cmake -S packages/robocar/twin -B packages/robocar/twin/build
#   cmake --build packages/robocar/twin/build
#   ctest --test-dir packages/robocar/twin/build --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(robocar_twin C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_C_FLAGS)
    set(CMAKE_C_FLAGS "-Wall -Wextra -O2 -g")
endif()

set(UNIFIED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../unified")
set(MAIN_SRC_DIR "${UNIFIED_DIR}/main")
set(SHIM_DIR "${UNIFIED_DIR}/test/include")

# Same host flags as unified/test/CMakeLists.txt: goal_state.c takes its POSIX
# branch (and goal_state_set_clock_override, which the twin points at its
# virtual clock).
add_compile_definitions(
    GOAL_STATE_HOST_TEST=1
    REACTIVE_CONTROLLER_HOST_TEST=1
    ULTRASONIC_HOST_TEST=1
    _POSIX_C_SOURCE=200809L
)

find_package(Threads REQUIRED)

# cJSON — gemini_parse.c and the twin's protocol reader. Discovery copied from
# unified/test/CMakeLists.txt: pkg-config first, then Homebrew's prefixes.
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(CJSON libcjson)
endif()
if(NOT CJSON_FOUND)
    set(_CJSON_HINTS "/opt/homebrew/opt/cjson" "/opt/homebrew" "/usr/local/opt/cjson" "/usr/local")
    find_path(CJSON_INCLUDE_DIR cjson/cJSON.h HINTS ${_CJSON_HINTS} PATH_SUFFIXES include)
    find_library(CJSON_LIB cjson HINTS ${_CJSON_HINTS} PATH_SUFFIXES lib)
    if(CJSON_INCLUDE_DIR AND CJSON_LIB)
        set(CJSON_INCLUDE_DIRS "${CJSON_INCLUDE_DIR}/cjson")
        set(CJSON_LIBRARIES "${CJSON_LIB}")
        set(CJSON_FOUND TRUE)
    endif()
endif()
if(NOT CJSON_FOUND)
    message(FATAL_ERROR "robocar-twin needs cJSON (apt install libcjson-dev / brew install cjson)")
endif()

add_executable(robocar-twin
    src/twin_main.c
    src/twin_sha256.c
    src/twin_signals.c
    # The decision path, compiled unmodified from the firmware.
    "${MAIN_SRC_DIR}/planner_cycle_core.c"
    "${MAIN_SRC_DIR}/plan_activity.c"
    "${MAIN_SRC_DIR}/plan_budget.c"
    "${MAIN_SRC_DIR}/scene_change.c"
    "${MAIN_SRC_DIR}/ambient_audio.c"
    "${MAIN_SRC_DIR}/speech_budget.c"
    "${MAIN_SRC_DIR}/dialogue_style.c"
    "${MAIN_SRC_DIR}/speech_tags.c"
    "${MAIN_SRC_DIR}/goal_state.c"
    "${MAIN_SRC_DIR}/gemini_parse.c"
)
# Shims first so they shadow ESP-IDF headers, as in the unified host tests.
target_include_directories(robocar-twin PRIVATE
    "${SHIM_DIR}"
    "${MAIN_SRC_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    ${CJSON_INCLUDE_DIRS}
)
target_link_directories(robocar-twin PRIVATE ${CJSON_LIBRARY_DIRS})
target_link_libraries(robocar-twin PRIVATE ${CJSON_LIBRARIES} Threads::Threads m)

enable_testing()
add_test(NAME twin_smoke
    COMMAND sh "${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke.sh" "$<TARGET_FILE:robocar-twin>")
