cmake_minimum_required(VERSION 3.5)
project(PathPlanner VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find common dependencies
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL)

# Try to find OpenGLWidgets if available (Qt6 feature)
find_package(Qt${QT_VERSION_MAJOR} QUIET COMPONENTS OpenGLWidgets)
if(Qt${QT_VERSION_MAJOR}OpenGLWidgets_FOUND)
    set(QT_OPENGL_WIDGETS_AVAILABLE TRUE)
else()
    set(QT_OPENGL_WIDGETS_AVAILABLE FALSE)
    message(STATUS "Qt${QT_VERSION_MAJOR}OpenGLWidgets not found, using OpenGL instead")
endif()
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

# Add library modules in dependency order
# Note: These libraries build successfully with basic dependencies
add_subdirectory(libs/GeometryLib)
add_subdirectory(libs/Hauser10)

# Try to build more complex libraries if Qt3D is fully available
option(BUILD_ADVANCED_LIBS "Build libraries that require full Qt3D support" ON)
if(BUILD_ADVANCED_LIBS)
    message(STATUS "Attempting to build advanced libraries with Qt3D support...")
    add_subdirectory(libs/TrajectoryLib)
    add_subdirectory(libs/USLib)
endif()

# Add applications only if dependencies are available
option(BUILD_APPS "Build applications" ON)
if(BUILD_APPS AND BUILD_ADVANCED_LIBS)
    add_subdirectory(apps/PathPlanner)
    add_subdirectory(apps/ParameterTuning)
    # add_subdirectory(apps/ComparisonIK)
    # add_subdirectory(apps/ComparisonSTOMPHauser)
    # add_subdirectory(apps/UltrasoundRepositioningEvaluator)
    # add_subdirectory(apps/TrajectoryPlanningEvaluator)
endif()

# Install configuration
include(GNUInstallDirs)

# Always install the basic libraries
install(TARGETS GeometryLib Hauser10
    EXPORT RUSLibrariesTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install advanced libraries if they were built
if(BUILD_ADVANCED_LIBS)
    if(TARGET TrajectoryLib)
        install(TARGETS TrajectoryLib
            EXPORT RUSLibrariesTargets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
    endif()
    
    if(TARGET USLib)
        install(TARGETS USLib
            EXPORT RUSLibrariesTargets
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
            INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        )
    endif()
endif()

# Install applications if they were built
if(BUILD_APPS AND BUILD_ADVANCED_LIBS AND TARGET PathPlanner)
    install(TARGETS PathPlanner
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
endif()

# Install library configuration
install(EXPORT RUSLibrariesTargets
    FILE RUSLibrariesTargets.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RUSLibraries
)
