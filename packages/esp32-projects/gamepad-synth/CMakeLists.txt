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
