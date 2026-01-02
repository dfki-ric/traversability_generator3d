find_package(Boost REQUIRED COMPONENTS filesystem serialization)

rock_executable(test_TraversabilityGenerator3d
    SOURCES test_TraversabilityGenerator3d.cpp
	  DEPS Boost::filesystem Boost::serialization
    DEPS_PKGCONFIG traversability_generator3d
)

rock_executable(travgen3d_walkthrough
    SOURCES travgen3d_walkthrough.cpp
	  DEPS Boost::filesystem Boost::serialization
    DEPS_PKGCONFIG traversability_generator3d
)