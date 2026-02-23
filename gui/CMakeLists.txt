rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)
find_package(yaml-cpp REQUIRED)

add_definitions(
    -DTRAV3D_CONFIG_DIR=\"${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/config\"
)

include(RockQt)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
IF("${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}" VERSION_LESS 1.14)
    SET(PCL_VERSION_SUFFIX "-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")
ENDIF()

if(ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND)
	# These are (only) required for building the GUIs:
	list(APPEND DEPS_PKGCONFIG_QT vizkit3d_debug_drawings)
endif()

rock_qt_library(
    TARGETPREFIX traversability_generator3d_gui
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        TraversabilityGenerator3dGui.cpp
    HEADERS
        TraversabilityGenerator3dGui.hpp
    MOC
        TraversabilityGenerator3dGui.hpp
    DEPS traversability_generator3d
    LIBS yaml-cpp
    DEPS_QT4 Qt4::QtCore
    DEPS_QT5 Qt5::Core
    DEPS_PKGCONFIG
                       base-types
                       pcl_common${PCL_VERSION_SUFFIX}
                       pcl_io${PCL_VERSION_SUFFIX}
    DEPS_PKGCONFIG_QT  vizkit3d
                       vizkit3d-viz
                       maps-viz
                       base-viz
                       ${DEPS_PKGCONFIG_QT}
)

rock_qt_executable(
    TARGETPREFIX traversability_generator3d_bin
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES
        Main.cpp
    DEPS traversability_generator3d
    LIBS yaml-cpp
    DEPS_QT traversability_generator3d_gui traversability_generator3d-viz
)

if(TARGET traversability_generator3d_gui)
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings_FOUND)
        message(STATUS "Gui using Qt4: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt4: enabled, without debug drawings")
    endif()
else(ROCK_QT_VERSION_4 AND vizkit3d_FOUND)
    message(STATUS "Gui using Qt4: disabled")
endif(ROCK_QT_VERSION_4 AND vizkit3d_FOUND)

if(TARGET traversability_generator3d_gui-qt5)
    if (ENABLE_DEBUG_DRAWINGS AND vizkit3d_debug_drawings-commands_FOUND AND vizkit3d_debug_drawings-qt5_FOUND)
        message(STATUS "Gui using Qt5: enabled, with debug drawings")
    else()
        message(STATUS "Gui using Qt5: enabled, without debug drawings")
    endif()
else()
    message(STATUS "Gui using Qt5: disabled")
endif(ROCK_QT_VERSION_5 AND vizkit3d-qt5_FOUND)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
    FILES_MATCHING PATTERN "*.yaml"
)