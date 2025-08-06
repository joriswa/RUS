cmake_minimum_required(VERSION 3.5)
project(GeometryLib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)

# Source files
set(GEOMETRYLIB_SOURCES
    src/Obstacle.cpp
    src/BVHTree.cpp
    src/ObstacleTree.cpp
)

# Create static library
add_library(GeometryLib STATIC ${GEOMETRYLIB_SOURCES})

# Include directories
target_include_directories(GeometryLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(GeometryLib PUBLIC 
    Eigen3::Eigen
)

# Install targets
install(TARGETS GeometryLib
    EXPORT GeometryLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT GeometryLibTargets
    FILE GeometryLibTargets.cmake
    DESTINATION lib/cmake/GeometryLib
)
