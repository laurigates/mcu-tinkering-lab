cmake_minimum_required(VERSION 3.16)

# Reuse bluepad32 + btstack from xbox-switch-bridge's external directory
set(EXTRA_COMPONENT_DIRS
    "../xbox-switch-bridge/external/bluepad32/src/components"
    "../xbox-switch-bridge/external/bluepad32/external/btstack/port/esp32/components"
)

# Version management
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-ps4-speaker)
