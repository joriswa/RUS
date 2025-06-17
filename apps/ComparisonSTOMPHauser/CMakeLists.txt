cmake_minimum_required(VERSION 3.16)

# Set the project name
project(ComparisonSTOMPHauser)

# Find required packages
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/libs/GeometryLib/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/USLib/include)
include_directories(${CMAKE_SOURCE_DIR}/libs/Hauser10/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add executable for STOMP vs Hauser trajectory planning comparison
add_executable(STOMPHauserComparison
    stomp_hauser_comparison.cpp
)

# Add executable for clearance calculation utility
add_executable(ClearanceCalculator
    clearance_calculator.cpp
)

# Link libraries for main comparison
target_link_libraries(STOMPHauserComparison
    GeometryLib
    TrajectoryLib
    USLib
    Hauser10
    Qt${QT_VERSION_MAJOR}::Core
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    pthread
)

# Link libraries for clearance calculator
target_link_libraries(ClearanceCalculator
    GeometryLib
    TrajectoryLib
    Eigen3::Eigen
    pthread
)

# Set output directory
set_target_properties(STOMPHauserComparison PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps/ComparisonSTOMPHauser
)

set_target_properties(ClearanceCalculator PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/apps/ComparisonSTOMPHauser
)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(STOMPHauserComparison PRIVATE -Wall -Wextra -O3)
    target_compile_options(ClearanceCalculator PRIVATE -Wall -Wextra -O3)
endif()

# Add definitions for Qt
target_compile_definitions(STOMPHauserComparison PRIVATE QT_NO_KEYWORDS)
add_definitions(-DQT_NO_VERSION_TAGGING)