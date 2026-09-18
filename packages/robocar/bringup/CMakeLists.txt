cmake_minimum_required(VERSION 3.16)

# esp-idf-lib (i2cdev / pca9685 / tca9548 / mcp23x17) used to be a vendored
# snapshot under robocar/main, reached from here through robocar-unified's
# relative symlink. #529 replaced it with managed components, deleting that
# tree — and this line kept naming it, so CMake refused to configure at all
# ("Directory specified in EXTRA_COMPONENT_DIRS doesn't exist"). The drivers
# now come from main/idf_component.yml.
#
# Relative paths only, still: absolute ones do not survive the container's
# /workspace bind mount — see .claude/rules/containerized-builds.md.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

# No version.txt and no release-please entry: this firmware is a bench tool, not
# something that ships or self-updates. It has no OTA path to carry a version to.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robocar-bringup)
