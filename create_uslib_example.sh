#!/bin/bash
# Script to create a standalone example project using USLib

echo "Creating standalone USLib example project..."

# Create project directory
mkdir -p uslib_example
cd uslib_example

# Create source files
cat > main.cpp << 'EOF'
#include <USLib/USTrajectoryPlanner.h>
#include <USLib/LogConfig.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

int main() {
    try {
        // Initialize logging
        USLib::initializeLogging("uslib_example.log", USLib::info);
        
        std::cout << "USLib Example Application" << std::endl;
        std::cout << "=========================" << std::endl;
        
        // Create trajectory planner with robot URDF
        // Note: You need to provide a valid URDF file path
        std::string robot_urdf = "path/to/your/robot.urdf";
        UltrasoundScanTrajectoryPlanner planner(robot_urdf);
        
        // Set initial joint configuration (7-DOF robot example)
        Eigen::VectorXd initial_joints(7);
        initial_joints << 0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0;
        planner.setCurrentJoints(initial_joints);
        
        std::cout << "Set initial joint configuration: " 
                  << initial_joints.transpose() << std::endl;
        
        // Set environment (obstacles)
        // planner.setEnvironment("path/to/environment.stl");
        
        // Create sample scan poses for ultrasound examination
        std::vector<Eigen::Affine3d> scan_poses;
        
        // Example: Create 3 poses for ultrasound scanning
        for (int i = 0; i < 3; ++i) {
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            
            // Position: Move along a line
            pose.translation() = Eigen::Vector3d(0.4 + i * 0.05, 0.2, 0.3);
            
            // Orientation: Point ultrasound probe towards patient
            Eigen::Matrix3d rotation;
            rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
            pose.linear() = rotation;
            
            scan_poses.push_back(pose);
        }
        
        planner.setPoses(scan_poses);
        std::cout << "Created " << scan_poses.size() << " scan poses" << std::endl;
        
        // Plan trajectories between poses
        std::cout << "Planning trajectories..." << std::endl;
        if (planner.planTrajectories()) {
            auto trajectories = planner.getTrajectories();
            std::cout << "Successfully planned " << trajectories.size() 
                      << " trajectories" << std::endl;
            
            // Analyze trajectory results
            for (size_t i = 0; i < trajectories.size(); ++i) {
                const auto& trajectory = trajectories[i].first;
                bool is_contact_motion = trajectories[i].second;
                
                std::cout << "Trajectory " << i + 1 << ":" << std::endl;
                std::cout << "  - Points: " << trajectory.size() << std::endl;
                std::cout << "  - Contact motion: " << (is_contact_motion ? "Yes" : "No") << std::endl;
                
                if (!trajectory.empty()) {
                    std::cout << "  - Duration: " << trajectory.back().time << " seconds" << std::endl;
                }
            }
            
        } else {
            std::cout << "Failed to plan trajectories" << std::endl;
            return 1;
        }
        
        std::cout << "\nExample completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
EOF

# Create CMakeLists.txt
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.5)
project(USLibExample)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required dependencies
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS log log_setup thread filesystem system)
find_package(orocos_kdl REQUIRED)

# Option 1: If USLib is installed system-wide
# find_package(USLib REQUIRED)

# Option 2: If using as subdirectory (adjust path as needed)
# Assuming PathPlanner_US_wip is at same level as this project
set(PATHPLANNER_DIR "../PathPlanner_US_wip")
if(EXISTS ${PATHPLANNER_DIR})
    message(STATUS "Using PathPlanner from: ${PATHPLANNER_DIR}")
    
    # Add subdirectories in dependency order
    add_subdirectory(${PATHPLANNER_DIR}/tinyurdfparser tinyurdfparser)
    add_subdirectory(${PATHPLANNER_DIR}/libs/GeometryLib GeometryLib)
    add_subdirectory(${PATHPLANNER_DIR}/libs/Hauser10 Hauser10)
    add_subdirectory(${PATHPLANNER_DIR}/libs/TrajectoryLib TrajectoryLib)
    add_subdirectory(${PATHPLANNER_DIR}/libs/USLib USLib)
    
    set(USLIB_AVAILABLE TRUE)
else()
    message(WARNING "PathPlanner directory not found at ${PATHPLANNER_DIR}")
    message(STATUS "Please adjust PATHPLANNER_DIR or install USLib system-wide")
    set(USLIB_AVAILABLE FALSE)
endif()

# Create executable
add_executable(uslib_example main.cpp)

if(USLIB_AVAILABLE)
    # Link USLib and its dependencies
    target_link_libraries(uslib_example 
        USLib
        ${Boost_LIBRARIES}
        ${orocos_kdl_LIBRARIES}
        Eigen3::Eigen
    )
    
    # Include directories
    target_include_directories(uslib_example PRIVATE
        ${PATHPLANNER_DIR}/libs/USLib/include
        ${Boost_INCLUDE_DIRS}
        ${orocos_kdl_INCLUDE_DIRS}
    )
else()
    # Dummy target if USLib not available
    target_compile_definitions(uslib_example PRIVATE USLIB_NOT_AVAILABLE)
endif()

# Set output directory
set_target_properties(uslib_example PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
EOF

# Create build script
cat > build.sh << 'EOF'
#!/bin/bash
echo "Building USLib example..."

# Create build directory
mkdir -p build
cd build

# Configure and build
cmake ..
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Run the example with: ./bin/uslib_example"
else
    echo "Build failed!"
    exit 1
fi
EOF

chmod +x build.sh

# Create README
cat > README.md << 'EOF'
# USLib Example Project

This is a minimal example showing how to use USLib for ultrasound trajectory planning.

## Prerequisites

- CMake 3.5+
- C++20 compiler
- Eigen3
- Boost (log, log_setup, thread, filesystem, system)
- orocos_kdl
- Access to PathPlanner_US_wip project

## Building

1. Ensure PathPlanner_US_wip is at the same level as this project directory
2. Run the build script:
   ```bash
   ./build.sh
   ```

## Running

```bash
cd build
./bin/uslib_example
```

## Customization

- Modify `main.cpp` to use your specific robot URDF file
- Add environment files for obstacle avoidance
- Customize scan poses for your specific ultrasound application
- Adjust trajectory planning parameters as needed

## Notes

This example demonstrates:
- USLib initialization and configuration
- Setting up robot kinematics
- Defining ultrasound scan poses
- Planning trajectories between poses
- Analyzing trajectory results
EOF

echo "Created USLib example project in 'uslib_example' directory"
echo "To build and run:"
echo "  cd uslib_example"
echo "  ./build.sh"
echo "  cd build && ./bin/uslib_example"
