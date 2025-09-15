# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(traversability_generator3d
        VERSION 1.0
        DESCRIPTION "C++ library for generating 3D traversability maps from MLS terrain data"
)

if(INSTALL_DEPS)

  execute_process(COMMAND bash install_os_dependencies.bash
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/source_dependencies)
  execute_process(COMMAND bash build.bash ${CMAKE_INSTALL_PREFIX}
                  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/source_dependencies)
  set(ENV{PKG_CONFIG_PATH} ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig/)
  set(ENV{CMAKE_PREFIX_PATH} ${CMAKE_INSTALL_PREFIX}/share/rock/cmake/)
endif()

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
rock_feature(NOCURDIR)
rock_standard_layout()

add_subdirectory(test)
add_subdirectory(gui)