cmake_minimum_required(VERSION 3.5)
project(TrajectoryLib)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS 3DCore 3DExtras 3DInput 3DLogic 3DRender Concurrent)
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS math_c99 system)
find_package(orocos_kdl REQUIRED)

# Source files organized by module
set(TRAJECTORYLIB_SOURCES
    # Core module
    src/Core/Util.cpp
    
    # Robot module  
    src/Robot/Robot.cpp
    src/Robot/RobotArm.cpp
    src/Robot/RobotManager.cpp
    src/Robot/franka_ik_He.cpp
    
    # Planning module
    src/Planning/PathPlanner.cpp
    
    # Motion module
    src/Motion/MotionGenerator.cpp
    
    # Visualization module
    src/Visualization/trackballcameracontroller.cpp
)

# Create static library
add_library(TrajectoryLib STATIC ${TRAJECTORYLIB_SOURCES})

# Include directories
target_include_directories(TrajectoryLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(TrajectoryLib PUBLIC
    GeometryLib
    Hauser10
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::Concurrent
    Qt${QT_VERSION_MAJOR}::3DCore
    Qt${QT_VERSION_MAJOR}::3DExtras
    Qt${QT_VERSION_MAJOR}::3DInput
    Qt${QT_VERSION_MAJOR}::3DLogic
    Qt${QT_VERSION_MAJOR}::3DRender
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)

# Install targets
install(TARGETS TrajectoryLib
    EXPORT TrajectoryLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT TrajectoryLibTargets
    FILE TrajectoryLibTargets.cmake
    DESTINATION lib/cmake/TrajectoryLib
)
