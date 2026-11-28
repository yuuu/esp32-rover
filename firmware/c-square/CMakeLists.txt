cmake_minimum_required(VERSION 3.16)
# The rover hardware component shared by the C projects
set(EXTRA_COMPONENT_DIRS ../components)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c-square)
