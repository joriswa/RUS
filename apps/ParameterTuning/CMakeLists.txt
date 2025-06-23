cmake_minimum_required(VERSION 3.5)
project(ParameterTuningApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)
pkg_check_modules(JSONCPP jsoncpp)

# Add executables
add_executable(ParameterTuningMain parameter_tuning_main.cpp)
add_executable(EnhancedParameterEvaluator enhanced_parameter_evaluator.cpp)

# Link libraries for main parameter tuning
target_link_libraries(ParameterTuningMain PUBLIC 
    TrajectoryLib
    GeometryLib
    Eigen3::Eigen
)

# Link libraries for enhanced evaluator
target_link_libraries(EnhancedParameterEvaluator PUBLIC 
    TrajectoryLib
    GeometryLib
    USLib
    Hauser10
    Eigen3::Eigen
    ${YAMLCPP_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

# Add library directories
target_link_directories(EnhancedParameterEvaluator PUBLIC 
    ${YAMLCPP_LIBRARY_DIRS}
    ${JSONCPP_LIBRARY_DIRS}
)

# Include directories
target_include_directories(ParameterTuningMain PUBLIC
    ${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include
    ${CMAKE_SOURCE_DIR}/libs/GeometryLib/include
)

target_include_directories(EnhancedParameterEvaluator PUBLIC
    ${CMAKE_SOURCE_DIR}/libs/TrajectoryLib/include
    ${CMAKE_SOURCE_DIR}/libs/GeometryLib/include
    ${CMAKE_SOURCE_DIR}/libs/USLib/include
    ${CMAKE_SOURCE_DIR}/libs/Hauser10/include
    ${YAMLCPP_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# Compiler definitions
target_compile_definitions(EnhancedParameterEvaluator PRIVATE 
    ${YAMLCPP_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)