cmake_minimum_required(VERSION 3.5)
project(RUSLibrariesExample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the installed RUS libraries
find_package(GeometryLib REQUIRED)
find_package(Hauser10 REQUIRED)

# Create example executable
add_executable(geometry_example geometry_example.cpp)
target_link_libraries(geometry_example GeometryLib::GeometryLib)

add_executable(trajectory_example trajectory_example.cpp)
target_link_libraries(trajectory_example Hauser10::Hauser10)

# For unified usage (if available)
find_package(RUSLibraries QUIET)
if(RUSLibraries_FOUND)
    add_executable(unified_example unified_example.cpp)
    target_link_libraries(unified_example GeometryLib Hauser10)
endif()