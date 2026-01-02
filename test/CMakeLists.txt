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