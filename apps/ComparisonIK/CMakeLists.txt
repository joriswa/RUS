cmake_minimum_required(VERSION 3.5)
project(ComparisonIKApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)

# Include directories for the organized structure
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Core IK library
add_library(IKCore 
    core/newton_raphson_ik.cpp
)
target_link_libraries(IKCore PUBLIC 
    Eigen3::Eigen 
    TrajectoryLib
)

# Main comparison executable - uses real CSV poses
add_executable(TwoMethodComparison three_method_comparison.cpp)
set_target_properties(TwoMethodComparison PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TwoMethodComparison PUBLIC 
    IKCore
    Eigen3::Eigen 
    USLib
    TrajectoryLib
)

# Underlying cost function generator (using evaluateSelectGoalPoseCost)
add_executable(UnderlyingCostFunctionGenerator underlying_cost_function_generator.cpp)
set_target_properties(UnderlyingCostFunctionGenerator PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(UnderlyingCostFunctionGenerator PUBLIC
    USLib
    TrajectoryLib
    Eigen3::Eigen
)
