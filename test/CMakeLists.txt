find_package(Boost REQUIRED COMPONENTS filesystem serialization)

add_executable(test_TraversabilityGenerator3d test_TraversabilityGenerator3d.cpp)
target_link_libraries(test_TraversabilityGenerator3d
  traversability_generator3d
  Boost::filesystem 
  Boost::serialization
)

add_executable(travgen3d_walkthrough travgen3d_walkthrough.cpp)
target_link_libraries(travgen3d_walkthrough
  traversability_generator3d
)

# Thread-count invariance / repeatability tests of the wave-parallel expansion.
# OpenMP is linked directly so the test can set the thread count itself.
add_executable(test_ParallelExpansion test_ParallelExpansion.cpp)
target_link_libraries(test_ParallelExpansion
  traversability_generator3d
)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(test_ParallelExpansion OpenMP::OpenMP_CXX)
endif()