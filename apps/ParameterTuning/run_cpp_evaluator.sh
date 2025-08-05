#!/bin/bash
# Script to run the C++ parameter evaluator with correct paths

# Change to the correct directory
cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning

# Check if config file argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <config_file>"
    echo "Example: $0 test_ext_ant_config.yaml"
    echo "Example: $0 output/configs/temp_config_1234567890.yaml"
    exit 1
fi

CONFIG_FILE="$1"

# Check if config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file '$CONFIG_FILE' not found"
    exit 1
fi

# Check if parameter_evaluator executable exists
EVALUATOR_PATH="../../build/apps/ParameterTuning/parameter_evaluator"
if [ ! -f "$EVALUATOR_PATH" ]; then
    # Try alternative Qt build path
    EVALUATOR_PATH="../../build/Qt_6_6_0_for_macOS-Release/apps/ParameterTuning/parameter_evaluator"
    if [ ! -f "$EVALUATOR_PATH" ]; then
        echo "Error: parameter_evaluator executable not found"
        echo "Make sure to build the project first with CMake"
        exit 1
    fi
fi

echo "🚀 Running C++ Parameter Evaluator..."
echo "📁 Working directory: $(pwd)"
echo "📄 Config file: $CONFIG_FILE"
echo "🔧 Executable: $EVALUATOR_PATH"
echo ""

# Run the evaluator
"$EVALUATOR_PATH" "$CONFIG_FILE"

echo ""
echo "✅ C++ Parameter Evaluator completed!"
