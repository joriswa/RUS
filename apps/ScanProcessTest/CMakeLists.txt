# CMake configuration for ScanProcessTest
cmake_minimum_required(VERSION 3.10)
project(ScanProcessTest)

set(CMAKE_CXX_STANDARD 14)

# Add main scan process test executable
add_executable(scan_process_test scan_process_test.cpp)
target_link_libraries(scan_process_test PRIVATE USLib)

# Add debug test executable
add_executable(debug_test debug_test.cpp)
target_link_libraries(debug_test PRIVATE USLib)
