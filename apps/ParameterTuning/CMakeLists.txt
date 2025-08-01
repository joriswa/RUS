cmake_minimum_required(VERSION 3.5)
project(ParameterTuningApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PkgConfig REQUIRED)

# Find yaml-cpp
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp)

# Find jsoncpp
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

# Add the parameter evaluator executable
add_executable(parameter_evaluator parameter_evaluator.cpp)

# Link libraries (using the same pattern as PathPlanner)
target_link_libraries(parameter_evaluator PRIVATE
    USLib
    /opt/homebrew/opt/yaml-cpp/lib/libyaml-cpp.dylib
    /opt/homebrew/opt/jsoncpp/lib/libjsoncpp.dylib
)

# Include yaml-cpp and jsoncpp
target_include_directories(parameter_evaluator PRIVATE 
    ${YAMLCPP_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

# Compiler flags
target_compile_options(parameter_evaluator PRIVATE
    ${YAMLCPP_CFLAGS_OTHER}
    ${JSONCPP_CFLAGS_OTHER}
)

# Add the hard case analyzer executable
add_executable(hard_case_analyzer hard_case_analyzer.cpp)

# Link libraries for hard case analyzer
target_link_libraries(hard_case_analyzer PRIVATE
    USLib
    /opt/homebrew/opt/yaml-cpp/lib/libyaml-cpp.dylib
)

# Include yaml-cpp for hard case analyzer
target_include_directories(hard_case_analyzer PRIVATE 
    ${YAMLCPP_INCLUDE_DIRS}
)

# Compiler flags for hard case analyzer
target_compile_options(hard_case_analyzer PRIVATE
    ${YAMLCPP_CFLAGS_OTHER}
)