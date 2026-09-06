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
# base64 streaming decoder (TTS audio path)
#
# base64.c has no ESP-IDF dependencies, so it compiles on the host as-is.
# -----------------------------------------------------------------------------
add_executable(test_base64_stream
    "${MAIN_SRC_DIR}/base64.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_base64_stream.c"
)
target_link_libraries(test_base64_stream PRIVATE Threads::Threads m)
add_test(NAME base64_stream COMMAND test_base64_stream)

# -----------------------------------------------------------------------------
# speech tag filter (inline TTS delivery tags)
#
# speech_tags.c is pure C with no ESP-IDF dependency, like base64.c.
# -----------------------------------------------------------------------------
add_executable(test_speech_tags
    "${MAIN_SRC_DIR}/speech_tags.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_speech_tags.c"
)
target_link_libraries(test_speech_tags PRIVATE Threads::Threads m)
add_test(NAME speech_tags COMMAND test_speech_tags)

# -----------------------------------------------------------------------------
# dialogue_style variation module (spoken-line repetition)
#
# dialogue_style.c is deliberately free of ESP-IDF and FreeRTOS (the PRNG is
# seeded from outside), so it compiles on the host with no shims at all.
# -----------------------------------------------------------------------------
add_executable(test_dialogue_style
    "${MAIN_SRC_DIR}/dialogue_style.c"
    "${MAIN_SRC_DIR}/speech_tags.c"  # note_spoken() strips delivery tags
    "${CMAKE_CURRENT_SOURCE_DIR}/test_dialogue_style.c"
)
target_link_libraries(test_dialogue_style PRIVATE Threads::Threads m)
add_test(NAME dialogue_style COMMAND test_dialogue_style)

# -----------------------------------------------------------------------------
# speech_budget rationing (how often the robot is offered the `speak` tool)
#
# Pure C with an injected clock, so the uint32 millisecond wrap is reachable in
# a test instead of only after 49 days of uptime.
# -----------------------------------------------------------------------------
add_executable(test_speech_budget
    "${MAIN_SRC_DIR}/speech_budget.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_speech_budget.c"
)
target_link_libraries(test_speech_budget PRIVATE Threads::Threads m)
add_test(NAME speech_budget COMMAND test_speech_budget)

# -----------------------------------------------------------------------------
# scene_change gate (does the view differ from the one last spoken about?)
#
# Pure C over a caller-supplied luma plane, so the synthetic frames a bench
# session cannot produce reliably — an exact brightness shift with no motion —
# are one memset here.
# -----------------------------------------------------------------------------
add_executable(test_scene_change
    "${MAIN_SRC_DIR}/scene_change.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_scene_change.c"
)
target_link_libraries(test_scene_change PRIVATE Threads::Threads m)
add_test(NAME scene_change COMMAND test_scene_change)

# -----------------------------------------------------------------------------
# ambient_audio gate (has the ROOM changed since the robot last spoke?)
#
# Pure C over a caller-supplied PCM frame with an injected clock, so the cases a
# bench cannot stage — an exact +12 dB gain change over a byte-identical
# soundscape, a controlled DC offset, and the uint32 millisecond wrap at day 49
# — are one memset each.
# -----------------------------------------------------------------------------
add_executable(test_ambient_audio
    "${MAIN_SRC_DIR}/ambient_audio.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_ambient_audio.c"
)
target_link_libraries(test_ambient_audio PRIVATE Threads::Threads m)
add_test(NAME ambient_audio COMMAND test_ambient_audio)

# -----------------------------------------------------------------------------
# plan_budget (the hard ceiling on planner spend)
#
# Pure C with no clock at all. The cases worth pinning are the ones that only
# appear after hours of running or after somebody else's JSON changes: a
# response whose token count could not be read must be CHARGED, not free, or the
# ceiling silently stops being a ceiling.
# -----------------------------------------------------------------------------
add_executable(test_plan_budget
    "${MAIN_SRC_DIR}/plan_budget.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_plan_budget.c"
)
target_link_libraries(test_plan_budget PRIVATE Threads::Threads m)
add_test(NAME plan_budget COMMAND test_plan_budget)

# -----------------------------------------------------------------------------
# plan_activity (should this cycle make a request at all?)
#
# Pure C over caller-supplied luma planes with an injected clock, so the whole
# nine-minute backoff ladder runs in microseconds and the uint32 millisecond
# wrap at day 49 is reachable without waiting 49 days. Links scene_change.c for
# the fingerprint representation it shares.
# -----------------------------------------------------------------------------
add_executable(test_plan_activity
    "${MAIN_SRC_DIR}/plan_activity.c"
    "${MAIN_SRC_DIR}/scene_change.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_plan_activity.c"
)
target_link_libraries(test_plan_activity PRIVATE Threads::Threads m)
add_test(NAME plan_activity COMMAND test_plan_activity)

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

# -----------------------------------------------------------------------------
# audio_clip (clip sizing, WAV framing, normalisation)
#
# Pure C, and the two load-bearing cases are ones the device cannot show you: a
# 32-bit overflow in the size arithmetic (a wrapped product is an ordinary small
# number, so the bug is a short allocation that is then overrun), and the exact
# 44 WAV header bytes — Gemini rejects raw PCM with a 400 that names no field, so
# a byte-order slip fails remotely and opaquely instead of here.
# -----------------------------------------------------------------------------
add_executable(test_audio_clip
    "${MAIN_SRC_DIR}/audio_clip.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_audio_clip.c"
)
target_link_libraries(test_audio_clip PRIVATE Threads::Threads m)
add_test(NAME audio_clip COMMAND test_audio_clip)

# -----------------------------------------------------------------------------
# audio_player producer accounting (TTS playback ring)
#
# audio_player.c is compiled UNMODIFIED against the shims in test/include
# (freertos/*, driver/*, esp_heap_caps.h) rather than being split behind an
# #ifdef: the code the test links has to be the code the device runs, or the
# test pins a transcription instead of the shipped path. Only the ring has real
# behaviour — see test/host_shims.c.
#
# The case it pins is one a bench cannot stage: an utterance whose PCM length
# leaves an odd remainder after the last 2048-byte batch. That made the write
# tally drift one byte per utterance, and the consequence surfaced minutes later
# and in another subsystem — a microphone that had gone permanently deaf.
# -----------------------------------------------------------------------------
add_executable(test_audio_player
    "${MAIN_SRC_DIR}/audio_player.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/host_shims.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_audio_player.c"
)
target_compile_definitions(test_audio_player PRIVATE AUDIO_PLAYER_HOST_TEST=1)
target_link_libraries(test_audio_player PRIVATE Threads::Threads m)
add_test(NAME audio_player COMMAND test_audio_player)

# -----------------------------------------------------------------------------
# pca9685_set_pwm_values (vendored esp-idf-lib driver, multi-channel write)
#
# The driver is compiled UNMODIFIED from the vendored component against the
# shims in test/include/pca9685_host, which is added AHEAD of the component so
# pca9685.h's <i2cdev.h> resolves to the shim. AddressSanitizer is on for this
# target because the defect it pins is a stack-buffer overflow: the driver
# indexed a buffer sized for `channels` entries by the absolute channel number,
# so motor_stop()'s (first_ch = 8, channels = 6) wrote 32 bytes past the end of
# a 24-byte VLA and the board double-faulted at the first boot with the PCA9685
# fitted. Content assertions alone could pass on lucky garbage; ASan cannot.
# -----------------------------------------------------------------------------
set(ESP_IDF_LIB_DIR "${MAIN_SRC_DIR}/../../main/components/esp-idf-lib/components")
add_executable(test_pca9685_multi
    "${ESP_IDF_LIB_DIR}/pca9685/pca9685.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_pca9685_multi.c"
)
target_include_directories(test_pca9685_multi BEFORE PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/pca9685_host"
    "${ESP_IDF_LIB_DIR}/pca9685"
)
# -Wno-unused-variable: the ESP_LOG* shims are no-ops, which leaves the driver's TAG unused.
target_compile_options(test_pca9685_multi PRIVATE -fsanitize=address -fno-omit-frame-pointer -Wno-unused-variable)
target_link_options(test_pca9685_multi PRIVATE -fsanitize=address)
target_link_libraries(test_pca9685_multi PRIVATE Threads::Threads m)
add_test(NAME pca9685_multi COMMAND test_pca9685_multi)

# -----------------------------------------------------------------------------
# servo_controller init ordering
#
# servo_controller.c is compiled UNMODIFIED against the shims in test/include
# (freertos/*, driver/gpio.h, esp_log.h) plus the pca9685_host i2cdev shim that
# i2c_bus.h drags in; the i2c_bus itself is a recording stub in the test. The
# case it pins is init centring the servos through a setter that refuses until
# init has finished — a failure with no bus traffic to show for it, which on the
# device read as "Failed to center servos" and a reboot.
# -----------------------------------------------------------------------------
add_executable(test_servo_controller
    "${MAIN_SRC_DIR}/servo_controller.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_servo_controller.c"
)
target_include_directories(test_servo_controller BEFORE PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/pca9685_host"
    "${ESP_IDF_LIB_DIR}/pca9685"
)
# -Wno-unused-variable: the ESP_LOG* shims are no-ops, which leaves TAG unused.
target_compile_options(test_servo_controller PRIVATE -Wno-unused-variable)
target_link_libraries(test_servo_controller PRIVATE Threads::Threads m)
add_test(NAME servo_controller COMMAND test_servo_controller)

# -----------------------------------------------------------------------------
# motor_controller idle re-write suppression
#
# motor_controller.c is compiled UNMODIFIED against the shims in test/include;
# the i2c_bus, the STBY GPIO and the clock are all stubs in the test. The cases
# it pins are the ones a bench cannot stage: a failed write must not be
# remembered as applied, the suppression must expire so a chip whose contents
# drifted is re-asserted, and that expiry must survive the uint32 millisecond
# wrap at day 49 rather than latching the robot into permanent suppression.
# -----------------------------------------------------------------------------
add_executable(test_motor_controller
    "${MAIN_SRC_DIR}/motor_controller.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_motor_controller.c"
)
target_include_directories(test_motor_controller BEFORE PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/pca9685_host"  # i2c_bus.h -> <i2cdev.h>
    "${ESP_IDF_LIB_DIR}/pca9685"
)
# -Wno-unused-variable: the ESP_LOG* shims are no-ops, which leaves TAG unused.
target_compile_options(test_motor_controller PRIVATE -Wno-unused-variable)
target_link_libraries(test_motor_controller PRIVATE Threads::Threads m)
add_test(NAME motor_controller COMMAND test_motor_controller)

# -----------------------------------------------------------------------------
# self_report per-peripheral health (issue #500)
#
# self_report.c is compiled UNMODIFIED against the shims in test/include plus
# the recording stubs in self_report_stubs.c, which back every live health
# accessor with a settable field. That is the whole point: the hardware phase
# no longer aborts the boot when one PCA9685-backed peripheral fails, so the
# facts line is the only thing that says which one is missing — and the case
# worth pinning (a live I2C bus with exactly one dead peripheral) is precisely
# the one a bench cannot stage.
# -----------------------------------------------------------------------------
add_executable(test_self_report
    "${MAIN_SRC_DIR}/self_report.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/self_report_stubs.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_self_report.c"
)
target_include_directories(test_self_report BEFORE PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include/pca9685_host"  # i2c_bus.h -> <i2cdev.h>
    "${ESP_IDF_LIB_DIR}/pca9685"
)
# -Wno-unused-variable: the ESP_LOG* shims are no-ops, which leaves TAG unused.
target_compile_options(test_self_report PRIVATE -Wno-unused-variable)
target_link_libraries(test_self_report PRIVATE Threads::Threads m)
add_test(NAME self_report COMMAND test_self_report)
