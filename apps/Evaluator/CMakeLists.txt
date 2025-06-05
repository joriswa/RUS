cmake_minimum_required(VERSION 3.5)
project(EvaluatorApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Evaluator evaluator.cpp)

target_link_libraries(Evaluator PUBLIC 
    TrajectoryLib
)
