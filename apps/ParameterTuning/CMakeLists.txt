cmake_minimum_required(VERSION 3.10)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Qt6 REQUIRED COMPONENTS Core)

# Create parameter_evaluator executable
add_executable(parameter_evaluator
    parameter_evaluator.cpp
)

# Include directories
target_include_directories(parameter_evaluator PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include
    ${CMAKE_SOURCE_DIR}/libs/GeometryLib/include
    ${CMAKE_SOURCE_DIR}/libs/Hauser10/include
    ${CMAKE_SOURCE_DIR}/libs/USLib/include
    ${yaml-cpp_INCLUDE_DIRS}
    ${tinyxml2_INCLUDE_DIRS}
    ${jsoncpp_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(parameter_evaluator
    TrajectoryLib
    GeometryLib
    Hauser10
    USLib
    ${yaml-cpp_LIBRARIES}
    ${tinyxml2_LIBRARIES}
    ${jsoncpp_LIBRARIES}
    Boost::system
    Boost::thread
    Qt6::Core
    pthread
)

# Add link directories
target_link_directories(parameter_evaluator PRIVATE
    ${yaml-cpp_LIBRARY_DIRS}
    ${tinyxml2_LIBRARY_DIRS}
    ${jsoncpp_LIBRARY_DIRS}
)

# Compiler flags
target_compile_options(parameter_evaluator PRIVATE
    -std=c++17
    -O3
    ${yaml-cpp_CFLAGS_OTHER}
    ${tinyxml2_CFLAGS_OTHER}
    ${jsoncpp_CFLAGS_OTHER}
)
