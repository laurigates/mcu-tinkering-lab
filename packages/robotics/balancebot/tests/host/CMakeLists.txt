# Host-side unit tests for the pure control-logic modules (no Pico SDK).
# Build with the system compiler:
#   cmake -S tests/host -B build-host && cmake --build build-host && ctest --test-dir build-host
cmake_minimum_required(VERSION 3.13)
project(balancebot_host_tests C)
set(CMAKE_C_STANDARD 11)

enable_testing()

set(SRC ${CMAKE_CURRENT_LIST_DIR}/../../src)

foreach(name pid imu_filter crc32)
    add_executable(test_${name} test_${name}.c ${SRC}/${name}.c)
    target_include_directories(test_${name} PRIVATE ${SRC})
    target_compile_options(test_${name} PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(test_${name} m)
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
