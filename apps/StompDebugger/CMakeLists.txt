cmake_minimum_required(VERSION 3.5)
project(StompDebuggerApp)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the STOMP debugger executable
add_executable(stomp_debugger stomp_debugger.cpp)

# Link libraries (using the same pattern as other apps)
target_link_libraries(stomp_debugger PRIVATE
    USLib
)

# Set output directory to make it easier to run
set_target_properties(stomp_debugger PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
