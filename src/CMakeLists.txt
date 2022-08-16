rock_library(traversability_generator3d
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(traversability_generator3d_bin Main.cpp
    DEPS traversability_generator3d)

