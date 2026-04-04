# ESP32-CAM LLM Telegram Test Project
cmake_minimum_required(VERSION 3.16)

# Read version from release-please managed version.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

# Include project configuration before project() command
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32cam-llm-telegram)
