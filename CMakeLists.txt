# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(traversability_generator3d
        VERSION 0.1
        DESCRIPTION "GF")
find_package(Rock)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(ENABLE_DEBUG_DRAWINGS)
    message(STATUS "ENABLE_DEBUG_DRAWINGS is defined with value: ${ENABLE_DEBUG_DRAWINGS}")
    add_definitions(-DENABLE_DEBUG_DRAWINGS)
else()
    message(STATUS "ENABLE_DEBUG_DRAWINGS is set to OFF. Skipped!")
endif()

rock_init()
rock_standard_layout()
