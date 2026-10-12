cmake_minimum_required(VERSION 3.13)

# Default board; override with -DPICO_BOARD=... (the justfile passes it).
set(PICO_BOARD seeed_xiao_rp2350 CACHE STRING "Board type")

# Must be include()d before project() — copied verbatim from
# $PICO_SDK_PATH/external/pico_sdk_import.cmake (SDK 2.2.0).
include(pico_sdk_import.cmake)

project(balancebot VERSION 0.1.0 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(balancebot
    src/main.c
    src/balance_control.c
    src/cli.c
    src/crc32.c
    src/imu_filter.c
    src/lqr.c
    src/mpu6050.c
    src/param_record.c
    src/param_store.c
    src/pid.c
    src/status_led.c
    src/stepper.c
    src/telemetry.c
)

pico_generate_pio_header(balancebot ${CMAKE_CURRENT_LIST_DIR}/src/stepper.pio)

target_include_directories(balancebot PRIVATE src)
target_compile_definitions(balancebot PRIVATE
    BALANCEBOT_VERSION="${PROJECT_VERSION}"
    # Core 0 runs the LQR solve (lqr_solve alone is ~2.1 KB of frame on
    # arm-none-eabi-gcc -O3, measured with -fstack-usage), over the SDK's
    # 2 KB default. SCRATCH_Y is 4 KB and holds only core 0's stack, so give
    # it all of it. Core 1 keeps its 2 KB in SCRATCH_X; pinned explicitly
    # because it otherwise inherits PICO_STACK_SIZE.
    PICO_STACK_SIZE=0x1000
    PICO_CORE1_STACK_SIZE=0x800
)
target_compile_options(balancebot PRIVATE -Wall -Wextra)
# Core 1 is float-only (the M33 FPU is single precision; double goes through
# SDK routines). Make any float->double promotion in the loop's sources a
# warning.
# lqr.c is covered too: its double math is the core-0 solve, written out
# explicitly, and its core-1 helpers are float.
set_source_files_properties(src/balance_control.c src/pid.c src/lqr.c src/imu_filter.c
    PROPERTIES COMPILE_OPTIONS -Wdouble-promotion)

target_link_libraries(balancebot
    pico_stdlib
    pico_multicore
    pico_flash
    pico_util
    hardware_adc
    hardware_flash
    hardware_i2c
    hardware_pio
    hardware_watchdog
)

# Console on USB CDC only (the robot has no UART header wired).
pico_enable_stdio_usb(balancebot 1)
pico_enable_stdio_uart(balancebot 0)

# Produce .uf2 (and .map/.dis) alongside the .elf.
pico_add_extra_outputs(balancebot)
