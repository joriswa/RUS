# Use CMakeFindDependencyMacro for proper dependency handling
include(CMakeFindDependencyMacro)

# Find Eigen3 (required dependency)
find_dependency(Eigen3 3.3 REQUIRED)

# Include the exported targets
include("${CMAKE_CURRENT_LIST_DIR}/GeometryLibTargets.cmake")

# Set up the target alias for easier usage
if(NOT TARGET GeometryLib::GeometryLib)
    add_library(GeometryLib::GeometryLib ALIAS GeometryLib)
endif()

# Set found flag
set(GeometryLib_FOUND TRUE)