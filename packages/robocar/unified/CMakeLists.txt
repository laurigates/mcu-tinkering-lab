cmake_minimum_required(VERSION 3.16)

# Generate a stub credentials.h for CI / web-flasher builds where the real
# credentials.h has not been created from the template.  Real credentials are
# supplied at runtime via Improv WiFi and stored in NVS.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h")
    message(STATUS "credentials.h not found — generating stub for Improv WiFi build")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h"
"/**\n"
" * Auto-generated stub credentials.h for Improv WiFi (no-credential) builds.\n"
" * Real WiFi credentials are provisioned at runtime via the Improv WiFi protocol.\n"
" * DO NOT add real credentials here — use main/credentials.h.example as a template.\n"
" */\n"
"#ifndef CREDENTIALS_H\n"
"#define CREDENTIALS_H\n"
"#define WIFI_SSID \"\"\n"
"#define WIFI_PASSWORD \"\"\n"
"#endif  // CREDENTIALS_H\n")
endif()

# Add esp-idf-lib to component directories
set(EXTRA_COMPONENT_DIRS "components/esp-idf-lib/components")
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

# Partition table + flash size are configured in sdkconfig.defaults
# (CONFIG_PARTITION_TABLE_CUSTOM / CONFIG_ESPTOOLPY_FLASHSIZE_8MB). ESP-IDF
# ignores set(CONFIG_* ...) here, so it is not repeated in this file.

# Read version from release-please managed manifest for OTA version tracking
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robocar-unified)
