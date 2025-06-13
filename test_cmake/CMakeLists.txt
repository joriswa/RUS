cmake_minimum_required(VERSION 3.16)

# Add executable
add_executable(TestPlanCheckpointsAngleHandling test_plan_checkpoints_angle_handling.cpp)

# Link libraries
target_link_libraries(TestPlanCheckpointsAngleHandling
    USLib
    TrajectoryLib
    GeometryLib
    Hauser10
)
