cmake_minimum_required(VERSION 3.14)

project(travgen_test)

set(CMAKE_CXX_STANDARD 17)

find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(base REQUIRED IMPORTED_TARGET base-types)
pkg_check_modules(maps REQUIRED IMPORTED_TARGET maps)
pkg_check_modules(traversability_generator3d REQUIRED IMPORTED_TARGET traversability_generator3d)

add_executable(test_travgen test_travgen.cpp)

target_link_libraries(test_travgen
  ${PCL_LIBRARIES}
  PkgConfig::base
  PkgConfig::maps
  PkgConfig::traversability_generator3d
)

target_include_directories(test_travgen PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)