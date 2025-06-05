cmake_minimum_required(VERSION 3.5)
project(ComparisonIKApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(orocos_kdl REQUIRED)

add_executable(ComparisonIK comparison_ik.cpp)

target_link_libraries(ComparisonIK PUBLIC 
    Eigen3::Eigen 
    ${orocos_kdl_LIBRARIES} 
    TinyURDFParser
    USLib
)
