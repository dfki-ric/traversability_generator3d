rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

pkg_check_modules(OSGVIZ_PRIMITIVES PrimitivesFactory)
pkg_check_modules(OSG REQUIRED openscenegraph)

if(${OSG_VERSION} VERSION_GREATER_EQUAL 3.6.4)
    add_compile_definitions(NEW_OSG)
endif()

if (ROCK_QT_VERSION_4 AND vizkit3d_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
    rock_vizkit_plugin(traversability_generator3d-viz
            PluginLoader.cpp
            PatchesGeode.cpp
        MOC
            TravMap3dVisualization.cpp
        HEADERS
            TravMap3dVisualization.hpp
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
    message(STATUS "Required library(s) not found ... NOT building the traversability_generator3d-viz plugins")
endif()

if (ROCK_QT_VERSION_5 AND vizkit3d-qt5_FOUND AND OSGVIZ_PRIMITIVES_FOUND)
    rock_vizkit_plugin(traversability_generator3d-viz-qt5
            PluginLoader.cpp
            PatchesGeode.cpp
        MOC
            TravMap3dVisualization.cpp
        HEADERS
            TravMap3dVisualization.hpp
            PatchesGeode.hpp
        DEPS 
            traversability_generator3d
        DEPS_PKGCONFIG 
            maps
            base-logging
            base-viz-qt5
            maps-viz-qt5
            vizkit3d-qt5  
            vizkit3d-viz-qt5
            PrimitivesFactory
    )
else()
    message(STATUS "Required library(s) not found ... NOT building the traversability_generator3d-viz-qt5 plugins")
endif()