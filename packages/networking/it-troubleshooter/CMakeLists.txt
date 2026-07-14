cmake_minimum_required(VERSION 3.16)

# Generate a stub credentials.h for CI / web-flasher builds where the real
# credentials.h has not been created from the template.  Release binaries ship
# credential-free; provide WiFi/API values at runtime or reflash locally.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h")
    message(STATUS "credentials.h not found — generating credential-free stub for CI build")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h"
"/**\n"
" * Auto-generated stub credentials.h for credential-free CI / web-flasher builds.\n"
" * DO NOT add real credentials here — use main/credentials.h.example as a template.\n"
" */\n"
"#pragma once\n"
"#define WIFI_SSID \"\"\n"
"#define WIFI_PASS \"\"\n"
"#define CLAUDE_API_KEY \"\"\n"
"#define CLAUDE_MODEL \"\"\n")
endif()

set(EXTRA_COMPONENT_DIRS "components")

# Read version from release-please managed version.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(it-troubleshooter)
