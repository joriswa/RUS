cmake_minimum_required(VERSION 3.5)
project(PathPlanner VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find common dependencies
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets)
find_package(Eigen3 3.3 REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS 3DCore 3DExtras 3DInput 3DLogic 3DRender Concurrent)
find_package(Boost REQUIRED COMPONENTS math_c99 system log log_setup thread filesystem)
find_package(orocos_kdl REQUIRED)

enable_testing()

if(APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -arch arm64")
endif()

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Could not find Boost!")
endif()

include_directories(${orocos_kdl_INCLUDE_DIRS})

# Add external dependencies
add_subdirectory(tinyurdfparser)

# Add library modules in dependency order
add_subdirectory(libs/GeometryLib)
add_subdirectory(libs/Hauser10)
add_subdirectory(libs/TrajectoryLib)
add_subdirectory(libs/USLib)

# Add applications
add_subdirectory(apps/PathPlanner)
add_subdirectory(apps/Evaluator)
# add_subdirectory(apps/ComparisonIK)
add_subdirectory(apps/ComparisonSTOMPHauser)
add_subdirectory(apps/UltrasoundRepositioningEvaluator)
add_subdirectory(apps/TrajectoryPlanningEvaluator)
 # add_subdirectory(apps/ParameterTuning)

# Install configuration
include(GNUInstallDirs)
install(TARGETS PathPlanner
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
