cmake_minimum_required(VERSION 3.5)
project(Hauser10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)

# Source files
set(HAUSER10_SOURCES
    src/DynamicPath.cpp
    src/ParabolicRamp.cpp
)

# Create static library
add_library(Hauser10 STATIC ${HAUSER10_SOURCES})

# Include directories
target_include_directories(Hauser10 PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Compiler optimizations for Hauser10
target_compile_options(Hauser10 PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-O3 -DNDEBUG>
    $<$<CXX_COMPILER_ID:MSVC>:/O2 /DNDEBUG>
)

target_compile_definitions(Hauser10 PRIVATE
    $<$<NOT:$<CONFIG:Release>>:NDEBUG>
)

# Link dependencies
target_link_libraries(Hauser10 PUBLIC 
    Eigen3::Eigen 
    m
)

# Install targets
install(TARGETS Hauser10
    EXPORT Hauser10Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT Hauser10Targets
    FILE Hauser10Targets.cmake
    DESTINATION lib/cmake/Hauser10
)

# Install the config file
install(FILES Hauser10Config.cmake
    DESTINATION lib/cmake/Hauser10
)
