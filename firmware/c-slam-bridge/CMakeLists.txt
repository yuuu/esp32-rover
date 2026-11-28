cmake_minimum_required(VERSION 3.16)
# The rover hardware component shared by the C projects
set(EXTRA_COMPONENT_DIRS ../components)
# Wi-Fi credentials live in sdkconfig.local (ignored by git); see README.md
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/sdkconfig.local)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.local")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(c-slam-bridge)
