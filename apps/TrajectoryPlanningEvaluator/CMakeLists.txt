cmake_minimum_required(VERSION 3.16)

# Trajectory Planning Evaluator Application
project(TrajectoryPlanningEvaluator)

# Set Release mode by default
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Source files
set(EVALUATOR_SOURCES
    src/trajectory_planning_evaluator.cpp
    src/main.cpp
)

# Create executable
add_executable(trajectory_planning_evaluator ${EVALUATOR_SOURCES})

# Set C++ standard
set_target_properties(trajectory_planning_evaluator PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Try to find Eigen3, but make it optional for now
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    target_link_libraries(trajectory_planning_evaluator Eigen3::Eigen)
    message(STATUS "Found Eigen3, linking to target")
else()
    message(STATUS "Eigen3 not found, using manual inclusion")
    # Try common Eigen3 locations
    target_include_directories(trajectory_planning_evaluator PRIVATE 
        /opt/homebrew/include/eigen3
        /usr/local/include/eigen3
        /usr/include/eigen3
    )
endif()

# Link required libraries
target_link_libraries(trajectory_planning_evaluator 
    TrajectoryLib
    USLib
    GeometryLib
    Hauser10
)

# Include directories for dependencies
target_include_directories(trajectory_planning_evaluator PRIVATE
    ${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include
    ${CMAKE_SOURCE_DIR}/libs/USLib/include
    ${CMAKE_SOURCE_DIR}/libs/GeometryLib/include
    ${CMAKE_SOURCE_DIR}/libs/Hauser10/include
)
