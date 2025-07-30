cmake_minimum_required(VERSION 3.5)
project(RUSLibrariesExample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(Eigen3 REQUIRED)

# Set paths to the installed libraries
set(RUS_INSTALL_DIR "${CMAKE_SOURCE_DIR}/../build/install")

# Include directories
include_directories(${RUS_INSTALL_DIR}/include)

# Create example executables
add_executable(geometry_example geometry_example.cpp)
target_link_libraries(geometry_example 
    ${RUS_INSTALL_DIR}/lib/libGeometryLib.a
    Eigen3::Eigen
)

add_executable(trajectory_example trajectory_example.cpp)
target_link_libraries(trajectory_example 
    ${RUS_INSTALL_DIR}/lib/libHauser10.a
    Eigen3::Eigen
    m  # math library for Hauser10
)

add_executable(unified_example unified_example.cpp)
target_link_libraries(unified_example 
    ${RUS_INSTALL_DIR}/lib/libGeometryLib.a
    ${RUS_INSTALL_DIR}/lib/libHauser10.a
    Eigen3::Eigen
    m  # math library
)