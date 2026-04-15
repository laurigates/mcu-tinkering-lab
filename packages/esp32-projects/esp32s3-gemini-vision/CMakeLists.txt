cmake_minimum_required(VERSION 3.16)

# Generate a stub credentials.h for CI / web-flasher builds where the real
# credentials.h has not been created from the template. Real credentials are
# expected at runtime (or edited into credentials.h before a local build).
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h")
    message(STATUS "credentials.h not found — generating empty stub (edit main/credentials.h before flashing)")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h"
"/* Auto-generated stub. Copy credentials.h.example -> credentials.h and edit. */\n"
"#ifndef CREDENTIALS_H\n"
"#define CREDENTIALS_H\n"
"#define WIFI_SSID \"\"\n"
"#define WIFI_PASSWORD \"\"\n"
"#define GEMINI_API_KEY \"\"\n"
"#endif\n")
endif()

set(CONFIG_PARTITION_TABLE_CUSTOM ON)
set(CONFIG_PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32s3-gemini-vision)
