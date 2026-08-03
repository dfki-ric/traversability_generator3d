find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Boost REQUIRED COMPONENTS serialization)

IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

rock_library(traversability_generator3d
    SOURCES TraversabilityGenerator3d.cpp
    HEADERS TraversabilityGenerator3d.hpp
            TravGenNode.hpp
            SoilNode.hpp
            SoilSample.hpp
            TraversabilityConfig.hpp
            TravMapBfsVisitor.hpp         
    DEPS_PKGCONFIG 
        base-types 
        maps
        pcl_segmentation${PCL_VERSION_SUFFIX}
        vizkit3d_debug_drawings-commands
    LIBS
        CGAL::CGAL
        Boost::serialization
)

# Multi-threaded map expansion / obstacle inflation (wave-parallel OpenMP pragmas
# in TraversabilityGenerator3d.cpp). Without OpenMP the pragmas are ignored and
# everything runs single-threaded, exactly as before.
# NOTE: plain signature — Rock.cmake already links this target with the plain
# signature, and CMake forbids mixing plain and keyword forms on one target.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(traversability_generator3d OpenMP::OpenMP_CXX)
endif()
