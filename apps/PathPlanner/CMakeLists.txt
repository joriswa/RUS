cmake_minimum_required(VERSION 3.5)
project(PathPlannerApp)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets OpenGL 3DCore 3DRender 3DInput 3DExtras)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets OpenGL 3DCore 3DRender 3DInput 3DExtras)

# Try to find OpenGLWidgets if available (Qt6 feature)
find_package(Qt${QT_VERSION_MAJOR} QUIET COMPONENTS OpenGLWidgets)
if(Qt${QT_VERSION_MAJOR}OpenGLWidgets_FOUND)
    set(QT_OPENGL_WIDGETS_AVAILABLE TRUE)
else()
    set(QT_OPENGL_WIDGETS_AVAILABLE FALSE)
    message(STATUS "Qt${QT_VERSION_MAJOR}OpenGLWidgets not found, using OpenGL instead")
endif()

add_definitions(-DQT_NO_VERSION_TAGGING)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    customcameracontroller.h
    customcameracontroller.cpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(PathPlanner
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(PathPlanner
        ${PROJECT_SOURCES})
endif()

# Link dependencies
set(QT_LIBRARIES
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::OpenGL
    Qt${QT_VERSION_MAJOR}::3DCore
    Qt${QT_VERSION_MAJOR}::3DRender
    Qt${QT_VERSION_MAJOR}::3DInput
    Qt${QT_VERSION_MAJOR}::3DExtras
)

# Add OpenGLWidgets if available
if(QT_OPENGL_WIDGETS_AVAILABLE)
    list(APPEND QT_LIBRARIES Qt${QT_VERSION_MAJOR}::OpenGLWidgets)
endif()

target_link_libraries(PathPlanner PRIVATE
    USLib
    ${QT_LIBRARIES}
)

if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.PathPlanner)
endif()

set_target_properties(PathPlanner PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(PathPlanner)
endif()
