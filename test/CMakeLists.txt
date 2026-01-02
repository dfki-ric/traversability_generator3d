find_package(Boost REQUIRED COMPONENTS filesystem serialization)

rock_executable(test_TraversabilityGenerator3d
    SOURCES test_TraversabilityGenerator3d.cpp
	DEPS Boost::filesystem Boost::serialization
    DEPS_PKGCONFIG traversability_generator3d
)

add_executable(travgen3d_walkthrough travgen3d_walkthrough.cpp)

target_link_libraries(travgen3d_walkthrough
  traversability_generator3d
)