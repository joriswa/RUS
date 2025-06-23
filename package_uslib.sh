#!/bin/bash
# Script to package USLib for standalone distribution

echo "Packaging USLib for standalone use..."

# Create package directory
PACKAGE_DIR="uslib_standalone_package"
mkdir -p $PACKAGE_DIR
cd $PACKAGE_DIR

# Create directory structure
mkdir -p {include,src,cmake,examples}

echo "Copying USLib files..."

# Copy USLib headers and sources
cp -r ../libs/USLib/include/* include/
cp -r ../libs/USLib/src/* src/

# Copy dependencies (headers only for interface)
mkdir -p include/TrajectoryLib include/GeometryLib include/Hauser10
cp -r ../libs/TrajectoryLib/include/* include/TrajectoryLib/
cp -r ../libs/GeometryLib/include/* include/GeometryLib/
cp -r ../libs/Hauser10/include/* include/Hauser10/

# Copy dependency sources (needed for compilation)
mkdir -p src/TrajectoryLib src/GeometryLib src/Hauser10 src/tinyurdfparser
cp -r ../libs/TrajectoryLib/src/* src/TrajectoryLib/
cp -r ../libs/GeometryLib/src/* src/GeometryLib/
cp -r ../libs/Hauser10/src/* src/Hauser10/
cp -r ../tinyurdfparser/* src/tinyurdfparser/

# Create a comprehensive CMakeLists.txt for the package
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(USLibStandalone VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS log log_setup thread filesystem system math_c99)
find_package(orocos_kdl REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
)

# TinyURDFParser library
file(GLOB_RECURSE TINYURDF_SOURCES "src/tinyurdfparser/*.cpp")
add_library(tinyurdfparser STATIC ${TINYURDF_SOURCES})
target_include_directories(tinyurdfparser PUBLIC src/tinyurdfparser)

# GeometryLib
file(GLOB_RECURSE GEOMETRY_SOURCES "src/GeometryLib/*.cpp")
add_library(GeometryLib STATIC ${GEOMETRY_SOURCES})
target_link_libraries(GeometryLib PUBLIC 
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

# Hauser10 library (trajectory optimization)
file(GLOB_RECURSE HAUSER_SOURCES "src/Hauser10/*.cpp")
add_library(Hauser10 STATIC ${HAUSER_SOURCES})
target_link_libraries(Hauser10 PUBLIC 
    GeometryLib
    Eigen3::Eigen
    ${Boost_LIBRARIES}
)

# TrajectoryLib
file(GLOB_RECURSE TRAJECTORY_SOURCES "src/TrajectoryLib/*.cpp")
add_library(TrajectoryLib STATIC ${TRAJECTORY_SOURCES})
target_link_libraries(TrajectoryLib PUBLIC 
    GeometryLib
    Hauser10
    tinyurdfparser
    ${orocos_kdl_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
)

# USLib
file(GLOB_RECURSE USLIB_SOURCES "src/USTrajectoryPlanner.cpp" "src/LogConfig.cpp")
add_library(USLib STATIC ${USLIB_SOURCES})
target_link_libraries(USLib PUBLIC 
    TrajectoryLib
    ${Boost_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    Eigen3::Eigen
)

# Export targets for find_package
install(TARGETS USLib TrajectoryLib GeometryLib Hauser10 tinyurdfparser
    EXPORT USLibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT USLibTargets
    FILE USLibTargets.cmake
    DESTINATION lib/cmake/USLib
)

# Create USLibConfig.cmake
configure_file(cmake/USLibConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/USLibConfig.cmake
    @ONLY
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/USLibConfig.cmake
    DESTINATION lib/cmake/USLib
)
EOF

# Create CMake config file
mkdir -p cmake
cat > cmake/USLibConfig.cmake.in << 'EOF'
# USLibConfig.cmake
include(CMakeFindDependencyMacro)

# Find required dependencies
find_dependency(Eigen3 3.3 REQUIRED)
find_dependency(Boost REQUIRED COMPONENTS log log_setup thread filesystem system math_c99)
find_dependency(orocos_kdl REQUIRED)

# Include the targets
include("${CMAKE_CURRENT_LIST_DIR}/USLibTargets.cmake")
EOF

# Create example application
cat > examples/simple_example.cpp << 'EOF'
#include <USLib/USTrajectoryPlanner.h>
#include <USLib/LogConfig.h>
#include <iostream>

int main() {
    std::cout << "USLib Standalone Example" << std::endl;
    
    try {
        // Initialize logging
        USLib::initializeLogging("example.log", USLib::info);
        
        // Note: You'll need to provide a valid URDF file
        // UltrasoundScanTrajectoryPlanner planner("robot.urdf");
        
        std::cout << "USLib initialized successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
EOF

# Create build script for the package
cat > build_package.sh << 'EOF'
#!/bin/bash
echo "Building USLib standalone package..."

mkdir -p build
cd build

cmake ..
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ $? -eq 0 ]; then
    echo "Package built successfully!"
    echo "To install system-wide: sudo make install"
else
    echo "Build failed!"
    exit 1
fi
EOF

chmod +x build_package.sh

# Create README for the package
cat > README.md << 'EOF'
# USLib Standalone Package

This package contains USLib and all its dependencies for standalone use.

## Contents

- **USLib**: Ultrasound trajectory planning library
- **TrajectoryLib**: Core trajectory planning functionality  
- **GeometryLib**: Geometric utilities and collision detection
- **Hauser10**: Trajectory optimization algorithms
- **tinyurdfparser**: URDF file parsing

## Dependencies

Before building, ensure you have:

- CMake 3.5+
- C++20 compatible compiler
- Eigen3 (≥ 3.3)
- Boost libraries (log, log_setup, thread, filesystem, system, math_c99)
- orocos_kdl

### Installing Dependencies on Ubuntu/Debian

```bash
sudo apt update
sudo apt install cmake build-essential libeigen3-dev libboost-all-dev liborocos-kdl-dev
```

### Installing Dependencies on macOS

```bash
brew install cmake eigen boost orocos-kdl
```

## Building

```bash
./build_package.sh
```

## Installing System-Wide

```bash
cd build
sudo make install
```

## Using in Your Project

After installation, you can use USLib in your CMake projects:

```cmake
find_package(USLib REQUIRED)
target_link_libraries(your_target USLib)
```

## Example Usage

See `examples/simple_example.cpp` for basic usage patterns.

## License

[Add your license information here]
EOF

cd ..

echo "USLib standalone package created in '$PACKAGE_DIR'"
echo ""
echo "To build the package:"
echo "  cd $PACKAGE_DIR"
echo "  ./build_package.sh"
echo ""
echo "To install system-wide after building:"
echo "  cd $PACKAGE_DIR/build"
echo "  sudo make install"
