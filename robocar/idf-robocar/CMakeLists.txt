# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Add esp-idf-lib to component directories
set(EXTRA_COMPONENT_DIRS "components/esp-idf-lib/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(idf-robocar)
