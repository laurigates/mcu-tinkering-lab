# Host-side unit tests for the pure control-logic modules (no Pico SDK).
# Build with the system compiler:
#   cmake -S tests/host -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.13)
project(balancebot_host_tests C)
set(CMAKE_C_STANDARD 11)

enable_testing()

set(SRC ${CMAKE_CURRENT_LIST_DIR}/../../src)

foreach(name pid imu_filter crc32 lqr param_record arm_gate cli_gate imu_cal status_color drive)
    add_executable(test_${name} test_${name}.c ${SRC}/${name}.c)
    target_include_directories(test_${name} PRIVATE ${SRC})
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(test_${name} m)
    add_test(NAME ${name} COMMAND test_${name})
endforeach()

# The param record carries a CRC32.
target_sources(test_param_record PRIVATE ${SRC}/crc32.c)
# The arm gate classifies samples with imu_filter.c's gravity check.
target_sources(test_arm_gate PRIVATE ${SRC}/imu_filter.c)
# So do the CLI's arm and load checks.
target_sources(test_cli_gate PRIVATE ${SRC}/imu_filter.c)
# The trim capture checks its result with param_record.c's pitch_trim_valid().
target_sources(test_imu_cal PRIVATE ${SRC}/param_record.c ${SRC}/crc32.c)
set_source_files_properties(${SRC}/imu_cal.c PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)
# drive.c runs on core 1, which is float-only.
set_source_files_properties(${SRC}/drive.c PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)
# The LQR sign-invariant test compares against the angle PID.
target_sources(test_lqr PRIVATE ${SRC}/pid.c)
# lqr.c mixes a double solver with a float control law: keep every
# float->double promotion explicit.
set_source_files_properties(${SRC}/lqr.c PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)

# tools/fit_leff.py (just balancebot::fit-leff) checks itself against
# simulated captures with a known L_eff and pitch offset.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME fit_leff
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_LIST_DIR}/../../tools/fit_leff.py --self-test)
# tools/analyze_capture.py (just balancebot::analyze) finds a known swing and
# offset in a synthetic capture.
add_test(NAME analyze_capture
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_CURRENT_LIST_DIR}/../../tools/analyze_capture.py --self-test)
