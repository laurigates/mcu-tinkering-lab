cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "components"
    "external/bluepad32/src/components"
    "external/bluepad32/external/btstack/port/esp32/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(xbox-switch-bridge)
