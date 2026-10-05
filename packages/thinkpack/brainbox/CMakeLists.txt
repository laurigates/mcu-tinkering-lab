cmake_minimum_required(VERSION 3.16)

# CI (build.yml) runs idf.py directly with no credentials.h; generate an empty
# stub so the build compiles. `just build` still requires a real file first.
# The stub defines no API keys, so credentials_loader.c's #ifdef guards skip them.
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h")
    message(STATUS "credentials.h not found — generating stub for a no-credential build")
    file(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/main/credentials.h"
"/**\n"
" * Auto-generated stub credentials.h for no-credential (CI) builds.\n"
" * DO NOT add real credentials here — use main/credentials.h.example as a template.\n"
" */\n"
"#ifndef CREDENTIALS_H\n"
"#define CREDENTIALS_H\n"
"#define WIFI_SSID \"\"\n"
"#define WIFI_PASSWORD \"\"\n"
"#endif  // CREDENTIALS_H\n")
endif()

# Read version from release-please managed version.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

# Pick up shared ThinkPack components from packages/components/.
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(thinkpack-brainbox)
