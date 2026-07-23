cmake_minimum_required(VERSION 3.16)

# Two component roots: our components/ and the nested tinyusb_port (a DCD-only
# vendored TinyUSB port; ADR-0006). Nested components aren't auto-discovered, so
# they must appear in EXTRA_COMPONENT_DIRS.
set(EXTRA_COMPONENT_DIRS
    "components"
    "components/raw_usb/tinyusb_port"
)

# Read version from version.txt (release-please managed)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(facedancer-espdancer-fw)

# Warn if any single function in our own components uses > 512 bytes of stack.
set(OUR_COMPONENTS main status_led log_udp usb_rpc raw_usb)
foreach(comp IN LISTS OUR_COMPONENTS)
    if(TARGET __idf_${comp})
        target_compile_options(__idf_${comp} PRIVATE -Wstack-usage=512)
    endif()
endforeach()