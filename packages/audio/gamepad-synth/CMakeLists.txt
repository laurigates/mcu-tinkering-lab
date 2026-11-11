cmake_minimum_required(VERSION 3.16)

# Bluepad32 + btstack components
# CI clones into external/; local dev symlinks or clones independently
set(EXTRA_COMPONENT_DIRS
    "external/bluepad32/src/components"
    "external/bluepad32/external/btstack/port/esp32/components"
)

# Version management
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gamepad-synth)

# bluepad32 release_v3.10.3's cmd_system.c calls gpio_wakeup_enable() without
# including driver/gpio.h, which ESP-IDF v5.5's GCC 14 rejects as an implicit
# declaration (#734). Upstream fixed it on develop (c8f2132d) after the last
# release, so force the header into that component while it still needs it.
# xbox-switch-bridge/CMakeLists.txt carries the same block.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/external/bluepad32/src/components/cmd_system/cmd_system.c" _bp32_cmd_system)
if(_bp32_cmd_system MATCHES "gpio_wakeup_enable" AND NOT _bp32_cmd_system MATCHES "driver/gpio\\.h")
    idf_component_get_property(_bp32_cmd_system_lib cmd_system COMPONENT_LIB)
    target_compile_options(${_bp32_cmd_system_lib} PRIVATE -include driver/gpio.h)
endif()
