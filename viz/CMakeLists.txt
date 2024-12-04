pkg_check_modules(OSGVIZ_PRIMITIVES PrimitivesFactory)
pkg_check_modules(OSG REQUIRED openscenegraph)

if(${OSG_VERSION} VERSION_GREATER_EQUAL 3.6.4)
    add_compile_definitions(NEW_OSG)
endif()

if (vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
    rock_find_qt4()
    rock_vizkit_plugin(traversability_generator3d-viz
            PluginLoader.cpp
            PatchesGeode.cpp
        MOC
            SoilMap3dVisualization.cpp
        HEADERS
            SoilMap3dVisualization.hpp
            PatchesGeode.hpp
        DEPS 
            traversability_generator3d
        DEPS_PKGCONFIG 
            maps
            base-logging
            base-viz 
            maps-viz
            vizkit3d  
            vizkit3d-viz
            PrimitivesFactory
    )
else()
    message(STATUS "osgviz not found ... NOT building the traversability_generator3d-viz plugins")
endif()

