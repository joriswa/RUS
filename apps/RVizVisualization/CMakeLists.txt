cmake_minimum_required(VERSION 3.16)
project(RVizVisualization)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    geometry_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)

# Catkin package configuration
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES obstacle_visualizer
    CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs std_msgs tf2 tf2_ros
    DEPENDS EIGEN3
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../../libs/GeometryLib/include
    ../../libs/USLib/include
    ../../libs/TrajectoryLib/include
)

# Create obstacle visualizer library
add_library(obstacle_visualizer
    obstacle_visualizer.cpp
)

target_link_libraries(obstacle_visualizer
    ${catkin_LIBRARIES}
    GeometryLib
    USLib
    TrajectoryLib
)

# Create standalone obstacle visualizer node
add_executable(obstacle_visualizer_node
    obstacle_visualizer.cpp
)

target_link_libraries(obstacle_visualizer_node
    ${catkin_LIBRARIES}
    GeometryLib
    USLib
    TrajectoryLib
)

# Create example trajectory planner with visualization
add_executable(trajectory_planner_with_viz
    trajectory_planner_visualization_example.cpp
    obstacle_visualizer.cpp
)

target_link_libraries(trajectory_planner_with_viz
    ${catkin_LIBRARIES}
    GeometryLib
    USLib
    TrajectoryLib
)

# Create integrated planning node with visualization
add_executable(planning_node_with_visualization
    planning_node_with_visualization.cpp
)

target_link_libraries(planning_node_with_visualization
    ${catkin_LIBRARIES}
    GeometryLib
    USLib
    TrajectoryLib
)

# Installation
install(TARGETS obstacle_visualizer obstacle_visualizer_node trajectory_planner_with_viz planning_node_with_visualization
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
