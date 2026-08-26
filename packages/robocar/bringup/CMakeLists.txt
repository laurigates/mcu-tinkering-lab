cmake_minimum_required(VERSION 3.16)

# esp-idf-lib (i2cdev / pca9685 / tca9548 / mcp23x17) is vendored under
# robocar/main and reached through robocar-unified's relative symlink. Absolute
# paths do not survive the container's /workspace bind mount, so this stays
# relative — see .claude/rules/containerized-builds.md.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../unified/components/esp-idf-lib/components")
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

# No version.txt and no release-please entry: this firmware is a bench tool, not
# something that ships or self-updates. It has no OTA path to carry a version to.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robocar-bringup)
