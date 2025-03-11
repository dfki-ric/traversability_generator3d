find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)
find_package(CGAL REQUIRED COMPONENTS Core)

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
)
