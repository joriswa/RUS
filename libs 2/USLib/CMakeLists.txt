cmake_minimum_required(VERSION 3.5)
project(USLib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Source files
set(USLIB_SOURCES
    src/USTrajectoryPlanner.cpp
)

# Create static library
add_library(USLib STATIC ${USLIB_SOURCES})

# Include directories
target_include_directories(USLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(USLib PUBLIC
    TrajectoryLib
)

# Install targets
install(TARGETS USLib
    EXPORT USLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT USLibTargets
    FILE USLibTargets.cmake
    DESTINATION lib/cmake/USLib
)
