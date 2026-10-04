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
"/* Empty: MQTT accepts only read-only commands until broker credentials are\n"
" * provisioned (`mqtt auth` on the serial console stores them in NVS). */\n"
"#define MQTT_USERNAME \"\"\n"
"#define MQTT_PASSWORD \"\"\n"
"#endif  // CREDENTIALS_H\n")
endif()

# esp-idf-lib drivers come from the component manager (main/idf_component.yml);
# this list carries only the monorepo's own shared components.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
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

# Build commit SHA, compared on-device against the OTA manifest's "buildSha"
# (issue #627; see main/ota_update_decision.h). Every release rebuilds this
# project at whichever commit fired it, so the same PROJECT_VER can name
# different contents — only the commit tells them apart.
#
# "-c safe.directory=*" is load-bearing: builds run in the espressif/idf
# container as root against a checkout owned by another uid, and git there
# refuses the repository outright ("detected dubious ownership"), which would
# silently embed "unknown" in every CI binary. A tree with uncommitted changes
# to tracked files gets "-dirty" appended, so it never claims to be the commit
# it was built from; the OTA check reads either fallback as "no usable SHA".
# Like PROJECT_VER this is read at configure time; a new commit re-runs
# configure through the CMAKE_CONFIGURE_DEPENDS below, an edit does not.
set(ROBOCAR_BUILD_SHA "unknown")
find_program(ROBOCAR_GIT git)
if(ROBOCAR_GIT)
    execute_process(
        COMMAND "${ROBOCAR_GIT}" -c safe.directory=* rev-parse HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        OUTPUT_VARIABLE _robocar_sha
        RESULT_VARIABLE _robocar_sha_rc
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    string(LENGTH "${_robocar_sha}" _robocar_sha_len)
    if(_robocar_sha_rc EQUAL 0 AND _robocar_sha_len EQUAL 40 AND _robocar_sha MATCHES "^[0-9a-f]+$")
        set(ROBOCAR_BUILD_SHA "${_robocar_sha}")
        execute_process(
            COMMAND "${ROBOCAR_GIT}" -c safe.directory=* diff --quiet HEAD --
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            RESULT_VARIABLE _robocar_dirty_rc
            OUTPUT_QUIET
            ERROR_QUIET
        )
        if(NOT _robocar_dirty_rc EQUAL 0)
            string(APPEND ROBOCAR_BUILD_SHA "-dirty")
        endif()

        # Re-run configure when HEAD moves: the HEAD file itself (a detached
        # checkout writes the SHA there) and the branch ref it points to.
        execute_process(
            COMMAND "${ROBOCAR_GIT}" -c safe.directory=* rev-parse --git-path HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _robocar_head_file
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        execute_process(
            COMMAND "${ROBOCAR_GIT}" -c safe.directory=* symbolic-ref -q HEAD
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            OUTPUT_VARIABLE _robocar_head_ref
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        set(_robocar_git_watch "${_robocar_head_file}")
        if(_robocar_head_ref)
            execute_process(
                COMMAND "${ROBOCAR_GIT}" -c safe.directory=* rev-parse --git-path "${_robocar_head_ref}"
                WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                OUTPUT_VARIABLE _robocar_ref_file
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
            )
            list(APPEND _robocar_git_watch "${_robocar_ref_file}")
        endif()
        foreach(_robocar_watch IN LISTS _robocar_git_watch)
            get_filename_component(_robocar_watch_abs "${_robocar_watch}" ABSOLUTE
                                   BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
            if(EXISTS "${_robocar_watch_abs}")
                set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_robocar_watch_abs}")
            endif()
        endforeach()
    endif()
endif()
message(STATUS "robocar-unified build SHA: ${ROBOCAR_BUILD_SHA}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robocar-unified)
