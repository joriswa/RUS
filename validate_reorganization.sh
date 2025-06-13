#!/bin/bash
# TrajectoryLib Reorganization Validation Script

echo "=== TrajectoryLib Modular Reorganization Validation ==="
echo ""

# Check directory structure
echo "1. Checking modular directory structure..."
if [ -d "libs/TrajectoryLib/include/TrajectoryLib/Planning" ] && 
   [ -d "libs/TrajectoryLib/include/TrajectoryLib/Robot" ] && 
   [ -d "libs/TrajectoryLib/include/TrajectoryLib/Motion" ] && 
   [ -d "libs/TrajectoryLib/include/TrajectoryLib/Core" ] && 
   [ -d "libs/TrajectoryLib/include/TrajectoryLib/Utils" ] && 
   [ -d "libs/TrajectoryLib/include/TrajectoryLib/Visualization" ]; then
    echo "✓ All header module directories exist"
else
    echo "✗ Missing header module directories"
fi

if [ -d "libs/TrajectoryLib/src/Planning" ] && 
   [ -d "libs/TrajectoryLib/src/Robot" ] && 
   [ -d "libs/TrajectoryLib/src/Motion" ] && 
   [ -d "libs/TrajectoryLib/src/Core" ] && 
   [ -d "libs/TrajectoryLib/src/Utils" ] && 
   [ -d "libs/TrajectoryLib/src/Visualization" ]; then
    echo "✓ All source module directories exist"
else
    echo "✗ Missing source module directories"
fi

# Check file organization
echo ""
echo "2. Checking file organization..."

# Headers
planning_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Planning -name "*.h" | wc -l)
robot_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Robot -name "*.h" | wc -l)
motion_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Motion -name "*.h" | wc -l)
core_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Core -name "*.h" | wc -l)
utils_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Utils -name "*.h" | wc -l)
viz_headers=$(find libs/TrajectoryLib/include/TrajectoryLib/Visualization -name "*.h" | wc -l)

echo "✓ Planning module: $planning_headers headers"
echo "✓ Robot module: $robot_headers headers"  
echo "✓ Motion module: $motion_headers headers"
echo "✓ Core module: $core_headers headers"
echo "✓ Utils module: $utils_headers headers"
echo "✓ Visualization module: $viz_headers headers"

# Sources
planning_sources=$(find libs/TrajectoryLib/src/Planning -name "*.cpp" | wc -l)
robot_sources=$(find libs/TrajectoryLib/src/Robot -name "*.cpp" | wc -l)
motion_sources=$(find libs/TrajectoryLib/src/Motion -name "*.cpp" | wc -l)
core_sources=$(find libs/TrajectoryLib/src/Core -name "*.cpp" | wc -l)
utils_sources=$(find libs/TrajectoryLib/src/Utils -name "*.cpp" | wc -l)
viz_sources=$(find libs/TrajectoryLib/src/Visualization -name "*.cpp" | wc -l)

echo "✓ Planning module: $planning_sources sources"
echo "✓ Robot module: $robot_sources sources"
echo "✓ Motion module: $motion_sources sources"
echo "✓ Core module: $core_sources sources"
echo "✓ Utils module: $utils_sources sources"
echo "✓ Visualization module: $viz_sources sources"

# Check include paths
echo ""
echo "3. Checking include path consistency..."

# Check for old-style includes in TrajectoryLib
old_includes=$(grep -r '#include "TrajectoryLib/' libs/TrajectoryLib/ | grep -v '/Planning/\|/Robot/\|/Motion/\|/Core/\|/Utils/\|/Visualization/' | wc -l)
if [ "$old_includes" -eq 0 ]; then
    echo "✓ All TrajectoryLib includes use modular paths"
else
    echo "✗ Found $old_includes old-style includes in TrajectoryLib"
fi

# Check USLib includes
us_old_includes=$(grep -r '#include "TrajectoryLib/' libs/USLib/ | grep -v '/Planning/\|/Robot/\|/Motion/\|/Core/\|/Utils/\|/Visualization/' | wc -l)
if [ "$us_old_includes" -eq 0 ]; then
    echo "✓ USLib uses modular include paths"
else
    echo "✗ Found $us_old_includes old-style includes in USLib"
fi

# Check for orphaned files
echo ""
echo "4. Checking for orphaned files..."

top_level_headers=$(find libs/TrajectoryLib/include/TrajectoryLib -maxdepth 1 -name "*.h" | wc -l)
top_level_sources=$(find libs/TrajectoryLib/src -maxdepth 1 -name "*.cpp" | wc -l)

if [ "$top_level_headers" -eq 0 ]; then
    echo "✓ No orphaned headers in top-level include directory"
else
    echo "✗ Found $top_level_headers orphaned headers"
fi

if [ "$top_level_sources" -eq 0 ]; then
    echo "✓ No orphaned sources in top-level src directory"
else
    echo "✗ Found $top_level_sources orphaned sources"
fi

# CMakeLists.txt validation
echo ""
echo "5. Checking CMakeLists.txt organization..."

if grep -q "src/Planning/PathPlanner.cpp" libs/TrajectoryLib/CMakeLists.txt && 
   grep -q "src/Robot/RobotArm.cpp" libs/TrajectoryLib/CMakeLists.txt &&
   grep -q "src/Motion/MotionGenerator.cpp" libs/TrajectoryLib/CMakeLists.txt; then
    echo "✓ CMakeLists.txt uses modular source paths"
else
    echo "✗ CMakeLists.txt needs to be updated for modular paths"
fi

echo ""
echo "=== Reorganization Validation Complete ==="

# Module summary
echo ""
echo "Module Summary:"
echo "📁 Planning/     - Path and trajectory planning algorithms"
echo "📁 Robot/        - Robot models, kinematics, and management"  
echo "📁 Motion/       - Motion generation and optimization"
echo "📁 Core/         - Core utilities and mathematical functions"
echo "📁 Utils/        - Helper utilities and evaluation tools"
echo "📁 Visualization/ - 3D visualization and camera controls"
echo ""
echo "The modular structure provides clear separation of concerns"
echo "and improved maintainability for the TrajectoryLib library."
