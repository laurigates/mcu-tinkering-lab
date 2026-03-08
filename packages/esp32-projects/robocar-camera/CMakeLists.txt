cmake_minimum_required(VERSION 3.16)

# Set partition table for OTA support
set(CONFIG_PARTITION_TABLE_CUSTOM ON)
set(CONFIG_PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")

# Read version from release-please managed manifest for OTA version tracking
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-cam-robocar)
