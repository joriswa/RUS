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
    core/grid_search_ik.cpp
    core/ik_cost_functions.cpp
    core/simulated_annealing_ik.cpp
    core/constraint_projected_newton_ik.cpp
)
target_link_libraries(IKCore PUBLIC 
    Eigen3::Eigen 
    TrajectoryLib
)

# Main comparison executable - uses real CSV poses
add_executable(ThreeMethodComparison three_method_comparison.cpp)
set_target_properties(ThreeMethodComparison PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(ThreeMethodComparison PRIVATE IKCore TrajectoryLib USLib)

# Comprehensive constraint projected Newton test - uses same CSV poses as comparison
add_executable(TestConstraintProjectedNewtonComprehensive test_constraint_projected_newton_comprehensive.cpp)
set_target_properties(TestConstraintProjectedNewtonComprehensive PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TestConstraintProjectedNewtonComprehensive PUBLIC 
    IKCore
    Eigen3::Eigen 
    USLib
    TrajectoryLib
)

# Feasibility check for CSV poses
add_executable(TestFeasibilityCheck test_feasibility_check.cpp)
set_target_properties(TestFeasibilityCheck PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TestFeasibilityCheck PUBLIC 
    IKCore
    Eigen3::Eigen 
    USLib
    TrajectoryLib
)

# Pose analysis for CSV poses
add_executable(TestPoseAnalysis test_pose_analysis.cpp)
set_target_properties(TestPoseAnalysis PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TestPoseAnalysis PUBLIC 
    Eigen3::Eigen
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

# Test for constraint projected Newton's method
add_executable(TestConstraintProjectedNewton test_constraint_projected_newton.cpp)
set_target_properties(TestConstraintProjectedNewton PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TestConstraintProjectedNewton PUBLIC 
    IKCore
    Eigen3::Eigen 
    USLib
    TrajectoryLib
)

# Debug collision penalty test
add_executable(DebugCollisionPenalty debug_collision_penalty.cpp)
set_target_properties(DebugCollisionPenalty PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(DebugCollisionPenalty PRIVATE IKCore TrajectoryLib USLib)

# Test adaptive damping vs traditional damping
add_executable(TestAdaptiveDamping test_adaptive_damping.cpp)
set_target_properties(TestAdaptiveDamping PROPERTIES
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
target_link_libraries(TestAdaptiveDamping PUBLIC 
    IKCore
    Eigen3::Eigen 
    USLib
    TrajectoryLib
)
