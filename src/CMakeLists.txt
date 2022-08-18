find_package(PCL 1.7 REQUIRED COMPONENTS segmentation)

rock_library(traversability_generator3d
    SOURCES TraversabilityGenerator3d.cpp
    HEADERS TraversabilityGenerator3d.hpp
            TravGenNode.hpp
            TraversabilityConfig.hpp
            TravMapBfsVisitor.hpp         
    DEPS_PKGCONFIG 
        base-types 
        maps
        pcl_segmentation-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
        vizkit3d_debug_drawings
)
