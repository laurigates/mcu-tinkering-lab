cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "components"
    "external/bluepad32/src/components"
    "external/bluepad32/external/btstack/port/esp32/components"
)

# Read version from release-please managed version.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/version.txt")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/version.txt" PROJECT_VER)
    string(STRIP "${PROJECT_VER}" PROJECT_VER)
else()
    set(PROJECT_VER "0.1.0")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xbox-switch-bridge)

# Warn if any single function in our own components uses > 512 bytes of stack.
# Applied only to our code (not vendored bluepad32/btstack).
set(OUR_COMPONENTS main bluepad32_host button_mapper log_udp status_led switch_pro_usb)
foreach(comp IN LISTS OUR_COMPONENTS)
    if(TARGET __idf_${comp})
        target_compile_options(__idf_${comp} PRIVATE -Wstack-usage=512)
    endif()
endforeach()
