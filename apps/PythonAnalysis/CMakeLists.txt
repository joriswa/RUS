# Python Analysis Application
# This directory contains Python-based analysis tools for trajectory planning evaluation

cmake_minimum_required(VERSION 3.16)

# Define the Python Analysis application
set(PYTHON_ANALYSIS_SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
set(PYTHON_ANALYSIS_RESULTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/results")
set(PYTHON_ANALYSIS_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")

# Create custom targets for running Python analysis scripts
# These are convenience targets that can be run with 'make <target_name>'

# IK Execution Time Analysis
add_custom_target(run_ik_analysis
    COMMAND python3 ${PYTHON_ANALYSIS_SCRIPTS_DIR}/ik_execution_time_analysis.py ${PYTHON_ANALYSIS_DATA_DIR}/four_method_comparison_results.csv
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running IK execution time analysis"
)

# SinglePose Execution Analysis  
add_custom_target(run_singlepose_analysis
    COMMAND python3 ${PYTHON_ANALYSIS_SCRIPTS_DIR}/single_pose_execution_analysis.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running SinglePose execution time analysis"
)

# Simple SinglePose Analysis
add_custom_target(run_simple_singlepose_analysis
    COMMAND python3 ${PYTHON_ANALYSIS_SCRIPTS_DIR}/simple_single_pose_analysis.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running simple SinglePose analysis"
)

# Trajectory Visualization
add_custom_target(run_trajectory_visualization
    COMMAND python3 ${PYTHON_ANALYSIS_SCRIPTS_DIR}/visualize_trajectory_analysis.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running trajectory visualization analysis"
)

# Quick Dashboard (requires CSV file parameter)
add_custom_target(run_quick_dashboard
    COMMAND echo "Usage: python3 ${PYTHON_ANALYSIS_SCRIPTS_DIR}/quick_dashboard.py <csv_file>"
    COMMENT "Display usage for quick dashboard"
)

# Meta target to run all automated analyses
add_custom_target(run_all_analysis
    DEPENDS run_ik_analysis run_singlepose_analysis run_simple_singlepose_analysis run_trajectory_visualization
    COMMENT "Running all automated Python analyses"
)

# Install Python scripts to build directory for easier access
install(DIRECTORY scripts/
    DESTINATION ${CMAKE_BINARY_DIR}/python_analysis/scripts
    FILES_MATCHING PATTERN "*.py"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(DIRECTORY data/
    DESTINATION ${CMAKE_BINARY_DIR}/python_analysis/data
    FILES_MATCHING PATTERN "*.csv"
)

install(FILES README.md
    DESTINATION ${CMAKE_BINARY_DIR}/python_analysis/
)

# Create results directory in build folder
install(CODE "file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/python_analysis/results)")

message(STATUS "Python Analysis Application configured")
message(STATUS "  Scripts directory: ${PYTHON_ANALYSIS_SCRIPTS_DIR}")
message(STATUS "  Results directory: ${PYTHON_ANALYSIS_RESULTS_DIR}")
message(STATUS "  Data directory: ${PYTHON_ANALYSIS_DATA_DIR}")
message(STATUS "Available targets:")
message(STATUS "  - run_ik_analysis: Analyze IK method execution times")
message(STATUS "  - run_singlepose_analysis: Analyze SinglePose evaluator performance")
message(STATUS "  - run_simple_singlepose_analysis: Run simple SinglePose analysis")
message(STATUS "  - run_trajectory_visualization: Generate trajectory visualizations")
message(STATUS "  - run_all_analysis: Run all automated analyses")
