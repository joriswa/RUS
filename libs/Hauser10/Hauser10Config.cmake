# Use CMakeFindDependencyMacro for proper dependency handling
include(CMakeFindDependencyMacro)

# Find Eigen3 (required dependency)
find_dependency(Eigen3 3.3 REQUIRED)

# Include the exported targets
include("${CMAKE_CURRENT_LIST_DIR}/Hauser10Targets.cmake")

# Set up the target alias for easier usage
if(NOT TARGET Hauser10::Hauser10)
    add_library(Hauser10::Hauser10 ALIAS Hauser10)
endif()

# Set found flag
set(Hauser10_FOUND TRUE)