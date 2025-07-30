# RUS Libraries Usage Examples

This directory contains examples demonstrating how to use the RUS libraries in your own projects.

## Prerequisites

Build and install the RUS libraries first:

```bash
# From the repository root
./build.sh basic
```

This will install the libraries to `build/install/`. To use them system-wide:

```bash
# From the repository root
cd build && sudo make install
```

## Building the Examples

### Using the Local Install

```bash
cd examples
mkdir build && cd build

# Point CMake to the local installation
cmake .. -DCMAKE_PREFIX_PATH=../../build/install

make
```

### Using System-Wide Install

```bash
cd examples
mkdir build && cd build
cmake ..
make
```

## Running the Examples

### GeometryLib Example
```bash
./geometry_example
```

Demonstrates:
- Creating obstacles
- Using the BVH tree for collision detection
- Basic geometry operations

### Trajectory Planning Example  
```bash
./trajectory_example
```

Demonstrates:
- Creating parabolic ramp trajectories
- Setting motion constraints
- Evaluating trajectories at specific times

### Unified Example
```bash
./unified_example
```

Demonstrates:
- Integration of geometry and trajectory planning
- Obstacle avoidance concepts
- Combined usage of multiple libraries

## Integration in Your Project

### Method 1: CMake find_package

```cmake
find_package(GeometryLib REQUIRED)
find_package(Hauser10 REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app GeometryLib::GeometryLib Hauser10::Hauser10)
```

### Method 2: Direct Library Linking

```cmake
target_include_directories(my_app PRIVATE /path/to/install/include)
target_link_libraries(my_app /path/to/install/lib/libGeometryLib.a /path/to/install/lib/libHauser10.a)
```

### Method 3: Using pkg-config

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(RUS_LIBS REQUIRED geometrylib hauser10)

target_include_directories(my_app PRIVATE ${RUS_LIBS_INCLUDE_DIRS})
target_link_libraries(my_app ${RUS_LIBS_LIBRARIES})
```

## Library Features

### GeometryLib
- 3D obstacle representation
- Bounding Volume Hierarchy (BVH) trees for fast collision detection
- Spatial data structures
- STL file processing

### Hauser10  
- Parabolic ramp trajectory generation
- Multi-dimensional trajectory planning
- Velocity and acceleration constraints
- Minimum-time trajectory optimization

## Advanced Usage

For more complex applications, consider:
- Combining obstacle detection with trajectory planning
- Using the BVH tree for real-time collision checking
- Implementing custom trajectory constraints
- Extending the libraries with your own algorithms