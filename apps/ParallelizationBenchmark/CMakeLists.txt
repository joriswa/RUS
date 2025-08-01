cmake_minimum_required(VERSION 3.5)
project(ParallelizationBenchmark)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(ParallelizationBenchmark main.cpp)

target_link_libraries(ParallelizationBenchmark 
    USLib
    TrajectoryLib
    GeometryLib
    Hauser10
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

target_include_directories(ParallelizationBenchmark PRIVATE
    ${CMAKE_SOURCE_DIR}/libs
)