rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

pkg_check_modules(OSGVIZ_PRIMITIVES PrimitivesFactory)
pkg_check_modules(OSG REQUIRED openscenegraph)

if(${OSG_VERSION} VERSION_GREATER_EQUAL 3.6.4)
    add_compile_definitions(NEW_OSG)
endif()

if (OSGVIZ_PRIMITIVES_FOUND)
    rock_qt_vizkit_plugin(
        TARGETPREFIX traversability_generator3d-viz
        QT4_SUFFIX ""
        MISSINGQTDEPS_NOBUILD
        SOURCES
            PluginLoader.cpp
            PatchesGeode.cpp
            TravMap3dVisualization.cpp
            SoilMap3dVisualization.cpp
        MOC
            TravMap3dVisualization.hpp
            SoilMap3dVisualization.hpp
            PluginLoader.hpp
        # installed headers, used when subclassing the visualizations here.
        HEADERS
            TravMap3dVisualization.hpp
            SoilMap3dVisualization.hpp
            PatchesGeode.hpp
        DEPS
            traversability_generator3d
        DEPS_PKGCONFIG
            maps
            base-logging
            PrimitivesFactory
        DEPS_PKGCONFIG_QT
            base-viz
            maps-viz
            vizkit3d-viz
    )
    if (TARGET traversability_generator3d-viz)
        target_compile_definitions(traversability_generator3d-viz PRIVATE TRAVGEN3D_PLUGIN_QT_4)
    endif()
    if (TARGET traversability_generator3d-viz-qt5)
        target_compile_definitions(traversability_generator3d-viz-qt5 PRIVATE TRAVGEN3D_PLUGIN_QT_5)
    endif()
endif()

if(NOT TARGET traversability_generator3d-viz)
    message(STATUS "Required library(s) not found ... NOT building the traversability_generator3d-viz plugins")
endif()

if(NOT TARGET traversability_generator3d-viz-qt5)
    message(STATUS "Required library(s) not found ... NOT building the traversability_generator3d-viz-qt5 plugins")
endif()
