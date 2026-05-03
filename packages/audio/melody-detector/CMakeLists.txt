cmake_minimum_required(VERSION 3.16)

set(CONFIG_PARTITION_TABLE_CUSTOM ON)
set(CONFIG_PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(melody-detector)
